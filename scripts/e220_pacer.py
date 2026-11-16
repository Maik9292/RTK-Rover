#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: e220_pacer.py
Description: Asynchronous Ping-Pong Mailbox & Stage-1 Calibrated Ebyte E220-900T30S LoRa Driver for RTKBase.
             Consumes raw RTCM3 streams on an asynchronous ingress thread, verifies CRC24Q,
             filters empty MSM frames, and maintains latest-value ping-pong mailboxes per
             RTCM message type (zero epoch queue buildup). Transmits intact or <= 200B
             coalesced batches across a consolidated content-independent N-second macro-cycle
             (default 5s) anchored to GPS/NTP Second 0 uplink & RSSI windows, using Ebyte
             E220 Manual Section 5.6.2 two-regime AUX + Stage-1-calibrated Time-on-Air pacing:
               - Single-subpacket frames (<= 200B): Transfer directly into an idle
                 LLCC68 FIFO without holding AUX LOW; paced deterministically by
                 calibrated LoRa Time-on-Air + receiver UART drain guard delay.
               - Two-subpacket frames (201..400B): Hold AUX LOW while Subpacket 1
                 (200B) transmits over the air, then raise AUX HIGH the instant
                 Subpacket 2 (len - 200) enters the LLCC68 FIFO and begins RF TX;
                 paced by total calibrated Time-on-Air + guard delay.
             Includes GPIO 22 hardware reset watchdog if AUX ever locks LOW (>1.5s),
             POSIX exclusive UART locking (TIOCEXCL), startup EEPROM verification,
             and Mode 0 ambient RF noise floor probing (REG1 Bit 5 = 0x20) on CH 75
             (925.125 MHz, Rank #1 cleanest US ISM channel clear of 915 MHz Ecowitt).

Hardware Connections (Raspberry Pi BCM Pins, matching setup_e220.py):
  - E220 M0:    GPIO 18 (BCM) -> Pin 12
  - E220 M1:    GPIO 17 (BCM) -> Pin 11
  - E220 AUX:   GPIO 27 (BCM) -> Pin 13 (Active HIGH = Buffer Empty, LOW = Busy)
  - E220 RESET: GPIO 22 (BCM) -> Pin 15 (Active LOW hardware reset)
  - E220 TX/RX: /dev/ttyAMA0 (UART0 GPIO 14/15)

Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import errno
import gzip
import json
import os
import select
import shutil
import signal
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    import pty
except ImportError:
    pty = None

try:
    import termios
    _TERMIOS_ERRORS: tuple[type[BaseException], ...] = (termios.error,)
except ImportError:
    _TERMIOS_ERRORS = ()


__filename__ = "e220_pacer.py"
__app_name__ = "e220_pacer"
__version__ = "3.7.0"
__date__ = "2026-11-16"

# Force unbuffered output so systemd journalctl displays entries in real time
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(line_buffering=True)

# Default Pin Mapping (matches setup_e220.py)
DEFAULT_PIN_M0 = 18
DEFAULT_PIN_M1 = 17
DEFAULT_PIN_AUX = 27
DEFAULT_PIN_RST = 22
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200
DEFAULT_PACKET_SIZE = 200
DEFAULT_MACRO_CYCLE_S = 5

# Inter-frame guard delay after RF completion (calibrated in Stage 1 for UART drain + turnaround)
DEFAULT_GUARD_DELAY_S = 0.030  # 30 ms

# Architecture 2B Ping-Pong Mailbox TTL & Static RTCM Metadata Classifications
STALE_FRAME_TTL_S = 2.5
STALE_STATIC_TTL_S = 15.0
STATIC_RTCM_TYPES: frozenset[int] = frozenset((1005, 1006, 1033, 1230))

# Hardware Flow Control & Watchdog Thresholds
AUX_SINGLE_PROBE_S = 0.008     # 8 ms probe for <=200B frames (normally stays HIGH per Sec 5.6.2)
AUX_MULTI_FALL_TIMEOUT_S = 0.050  # 50 ms max wait for AUX falling edge on >200B frames (typical: 2.0 ms)
AUX_RISE_WATCHDOG_S = 1.500    # 1.5 s max AUX LOW before triggering GPIO 22 hardware reset

# Lookups for Auto-Config / Register Decode
BAUD_RATES = {1200: 0b000, 2400: 0b001, 4800: 0b010, 9600: 0b011, 19200: 0b100, 38400: 0b101, 57600: 0b110, 115200: 0b111}
AIR_DATA_RATES = {2.4: 0b010, 4.8: 0b011, 9.6: 0b100, 19.2: 0b101, 38.4: 0b110, 62.5: 0b111}
SUBPACKET_SIZES = {200: 0b00, 128: 0b01, 64: 0b10, 32: 0b11}
POWER_LEVELS = {30: 0b00, 27: 0b01, 24: 0b10, 21: 0b11}

INV_BAUD = {v: k for k, v in BAUD_RATES.items()}
INV_AIR = {0b000: 2.4, 0b001: 2.4, **{v: k for k, v in AIR_DATA_RATES.items()}}
INV_PKT = {v: k for k, v in SUBPACKET_SIZES.items()}
INV_PWR = {v: k for k, v in POWER_LEVELS.items()}

running = True


def sig_handler(signum, frame):
    global running
    running = False


def rtcm_crc24q(data: bytes) -> int:
    """Compute 24-bit RTCM3 CRC24Q over full frame (header + payload + 3-byte CRC). Returns 0 if valid."""
    crc = 0
    for b in data:
        crc ^= (b << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc


def is_empty_msm_frame(frame: bytes, msg_type: int) -> bool:
    """Detects empty RTCM3 MSM frames (0 satellites / 0 signals, <= 28 bytes total)."""
    if 1071 <= msg_type <= 1130:
        return len(frame) <= 28
    return False


# =============================================================================
# Bi-Directional LoRa Protocol Definitions (0xAA 0x55 Framing)
# =============================================================================
ROVER_SYNC = b"\xAA\x55"
MSG_ID_TELEMETRY = 0x01
MSG_ID_REQ_SWITCH = 0x02
MSG_ID_ACK_SWITCH = 0x82

# 39-Byte Legacy Uplink Telemetry Frame Format (plen=33):
# sync(2s), mid(B), plen(B), lat(d), lon(d), height(f), h_acc_mm(H), fix(B),
# sats_m(B), sats_s(B), hdg_cdeg(H), bat_pct(B), bat_mv(H), rssi(b), noise(b), seq(B), crc(H)
FMT_TELEMETRY_LEGACY = "<2sBBddfHBBBHBHbbBH"

# 45-Byte V2 Uplink Telemetry Frame Format (plen=39, without firmware version):
# sync(2s), mid(B), plen(B), lat(d), lon(d), height(f), h_acc_mm(H), fix(B),
# ant2_fix(B), baseline_mm(H), baseline_pitch_cdeg(h), hdg_source(B),
# sats_m(B), sats_s(B), hdg_cdeg(H), bat_pct(B), bat_mv(H), rssi(b), noise(b), seq(B), crc(H)
FMT_TELEMETRY_V2 = "<2sBBddfHBBHhBBBHBHbbBH"

# 48-Byte V3 Uplink Telemetry Frame Format (plen=42, with fw_major, fw_minor, fw_patch):
# sync(2s), mid(B), plen(B), lat(d), lon(d), height(f), h_acc_mm(H), fix(B),
# ant2_fix(B), baseline_mm(H), baseline_pitch_cdeg(h), hdg_source(B),
# sats_m(B), sats_s(B), hdg_cdeg(H), bat_pct(B), bat_mv(H), rssi(b), noise(b), seq(B),
# fw_major(B), fw_minor(B), fw_patch(B), crc(H)
FMT_TELEMETRY = "<2sBBddfHBBHhBBBHBHbbBBBBH"

# 10-Byte Remote Air-Rate Switch Request Format:
# sync(2s), mid(B), plen(B), target_code(B), target_chan(B), req_id(H), crc(H)
FMT_REQ_SWITCH = "<2sBBBBHH"

# 11-Byte Remote Air-Rate Switch Acknowledge Format:
# sync(2s), mid(B), plen(B), req_id(H), status(B), delay_ms(H), crc(H)
FMT_ACK_SWITCH = "<2sBBHBHH"

# Spaced Second 1 ACK train offsets (ms) avoiding 0ms Base-1PPS/Rover-NMEA clock edge collision
SWITCH_ACK_SLOTS_MS: tuple[float, ...] = (80.0, 260.0, 440.0, 620.0)


def crc16_ccitt(data: bytes) -> int:
    """Standard CRC16-CCITT (False: poly 0x1021, init 0xFFFF, no final XOR)."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class MailboxSlot:
    """Latest-value ping-pong mailbox entry for a single RTCM3 message type."""
    msg_type: int
    frame: Optional[bytes]
    rx_monotonic: float
    last_tx_monotonic: float = 0.0
    superseded_count: int = 0


class E220Pacer:
    def __init__(
        self,
        port: str = DEFAULT_SERIAL_PORT,
        baud: int = DEFAULT_BAUD,
        packet_size: int = DEFAULT_PACKET_SIZE,
        guard_delay_s: float = DEFAULT_GUARD_DELAY_S,
        pin_m0: int = DEFAULT_PIN_M0,
        pin_m1: int = DEFAULT_PIN_M1,
        pin_aux: int = DEFAULT_PIN_AUX,
        pin_rst: int = DEFAULT_PIN_RST,
        auto_config: bool = False,
        air_rate_kbps: float = 19.2,
        power_dbm: int = 21,
        channel: int = 75,
        verbose: bool = False,
        quiet: bool = False,
        log_dir: str = "~/rover_logs",
        log_retention_days: int = 30,
        max_log_mb: int = 50,
        compress_logs: bool = True,
        no_csv: bool = False,
        no_interleave: bool = False,
        macro_cycle_s: int = DEFAULT_MACRO_CYCLE_S,
    ):
        self.port_name = port
        self.baud = baud
        self.packet_size = packet_size
        self.guard_delay_s = guard_delay_s
        self.pin_m0 = pin_m0
        self.pin_m1 = pin_m1
        self.pin_aux = pin_aux
        self.pin_rst = pin_rst
        self.auto_config = auto_config
        self.air_rate_kbps = air_rate_kbps
        self.power_dbm = power_dbm
        self.channel = channel
        self.verbose = verbose
        self.quiet = quiet
        self.macro_cycle_s = max(2, int(macro_cycle_s))
        self.ser: Optional[serial.Serial] = None
        self.gpio_available = GPIO is not None
        self.rssi_reg_enabled = False

        # Bi-Directional LoRa Uplink State
        self.uplink_buf = bytearray()
        self.last_rover_telemetry: dict = {}
        self.last_rover_rx_time = 0.0
        self.uplink_packets_received = 0
        self.uplink_crc_errors = 0
        self.rate_switch_count = 0
        self.watchdog_rate_switch_time = 0.0
        self.watchdog_rate_revert_time = 0.0
        self.fallback_air_rate = air_rate_kbps
        self.active_air_rate = air_rate_kbps

        # Log Retention & Storage
        resolved_log_dir = os.path.expanduser(log_dir)
        if resolved_log_dir.startswith("/root") or resolved_log_dir == "/root":
            if os.path.isdir("/home/pi"):
                resolved_log_dir = "/home/pi/rover_logs"
            elif os.environ.get("SUDO_USER") and os.path.isdir(f"/home/{os.environ.get('SUDO_USER')}"):
                resolved_log_dir = f"/home/{os.environ.get('SUDO_USER')}/rover_logs"
            else:
                # Find the first non-root interactive user directory in /home
                try:
                    candidates = [
                        os.path.join("/home", u) for u in os.listdir("/home")
                        if os.path.isdir(os.path.join("/home", u)) and not u.startswith(".")
                    ]
                    if candidates:
                        resolved_log_dir = os.path.join(candidates[0], "rover_logs")
                except Exception:
                    pass
        self.log_dir = resolved_log_dir
        self.log_retention_days = log_retention_days
        self.max_log_mb = max_log_mb
        self.compress_logs = compress_logs
        self.enable_csv = not no_csv
        self.current_log_date = ""
        self.current_log_file = None

        # Pre-create log directory immediately on startup with full access
        if self.enable_csv:
            try:
                os.makedirs(self.log_dir, exist_ok=True)
                os.chmod(self.log_dir, 0o777)
                # If created in /root, also attempt to symlink into /home candidate for convenience
                if self.log_dir.startswith("/root"):
                    for u in os.listdir("/home"):
                        uhome = os.path.join("/home", u)
                        if os.path.isdir(uhome) and not u.startswith("."):
                            ulink = os.path.join(uhome, "rover_logs")
                            if not os.path.exists(ulink):
                                try:
                                    os.symlink(self.log_dir, ulink)
                                except OSError:
                                    pass
            except OSError:
                pass

        # Telemetry counters
        self.total_bytes_in = 0
        self.total_bytes_out = 0
        self.total_packets_sent = 0
        self.total_subpackets_sent = 0
        self.total_wait_time_s = 0.0
        self.watchdog_resets = 0
        self.src_crc_errors = 0
        self.src_orphan_bytes = 0
        self.backlog_shed_frames = 0
        self.rtcm_stats: dict[int, int] = {}

        # Architecture 2B Ping-Pong Latest-Value Mailboxes & Coalescing Telemetry
        self.enable_interleave = not no_interleave
        self._mailbox_lock = threading.Lock()
        self._mailbox_event = threading.Event()
        self._mailboxes: dict[int, MailboxSlot] = {}
        self.mailbox_superseded = 0
        self.mailbox_stale_dropped = 0
        self.coalesced_batches = 0
        self.coalesced_frames = 0
        self.mailbox_max_age_ms = 0.0
        self.seen_secondary_types: set[int] = set()
        self.interleave_dropped_frames = 0
        self.sec_tx_utc: int = 0
        self.sec_tx_count: int = 0

        # Empty MSM Dropper & GPS-Disciplined Synchronous Switch State
        self.empty_msm_dropped = 0
        self.pending_switch_ack_frame: Optional[bytes] = None
        self.pending_switch_ack_sent: bool = False
        self.pending_switch_ack_repeat_sent: bool = False
        self.pending_switch_acks_sent: int = 0
        self.pending_switch_req_id: int = 0
        self.pending_switch_target_rate: float = 19.2
        self.pending_switch_ack_sec: int = 0
        self.pending_switch_exec_sec: int = 0

    def estimate_subpacket_air_s(self, n_bytes: int) -> float:
        """
        Estimate single-subpacket LoRa Time-on-Air (seconds) for E220-900T30S (LLCC68).
        Calibrated against Stage 1 hardware bench measurements (docs/benchmarks/e220_stage1_20261116/):
          - <= 3.0 kbps (2.4k):   0.095 + 0.00360 * n
          - <= 5.0 kbps (4.8k):   0.070 + 0.00187 * n  (200B -> 444.0ms, real 442.8ms)
          - <= 10.0 kbps (9.6k):  0.045 + 0.00119 * n  (200B -> 283.0ms)
          - <= 20.0 kbps (19.2k): 0.029 + 0.00080 * n  (200B -> 189.0ms)
          - >  20.0 kbps (38.4k): 0.022 + 0.00050 * n  (200B -> 122.0ms)
        """
        if n_bytes <= 0:
            return 0.0
        n = float(n_bytes)
        if self.air_rate_kbps <= 3.0:
            return 0.095 + 0.00360 * n
        if self.air_rate_kbps <= 5.0:
            return 0.070 + 0.00187 * n
        if self.air_rate_kbps <= 10.0:
            return 0.045 + 0.00119 * n
        if self.air_rate_kbps <= 20.0:
            return 0.029 + 0.00080 * n
        return 0.022 + 0.00050 * n

    def estimate_frame_air_s(self, total_len: int) -> float:
        """Estimate total multi-subpacket LoRa airtime (seconds) across all subpackets."""
        if total_len <= 0:
            return 0.0
        pkt_sz = max(int(self.packet_size), 32)
        rem = total_len
        total_air_s = 0.0
        while rem > 0:
            chunk = min(rem, pkt_sz)
            total_air_s += self.estimate_subpacket_air_s(chunk)
            rem -= chunk
        return total_air_s

    def _get_effective_guard_delay_s(self) -> float:
        """Enforces >= 35ms inter-frame guard at <= 5.0 kbps, default 30ms at >= 9.6 kbps."""
        if self.air_rate_kbps <= 5.0:
            return max(self.guard_delay_s, 0.035)
        return self.guard_delay_s

    @staticmethod
    def _safe_flush(ser_obj: "serial.Serial") -> bool:
        """Drain UART TX buffer via tcdrain(), returning False cleanly if interrupted by SIGTERM/SIGINT (EINTR)."""
        while True:
            try:
                ser_obj.flush()
                return True
            except _TERMIOS_ERRORS + (OSError,) as e:
                err_no = e.args[0] if getattr(e, "args", None) else None
                if err_no == errno.EINTR:
                    if not running:
                        return False
                    continue
                if not running:
                    return False
                raise

    def _open_operational_uart(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed.")
        if self.ser and self.ser.is_open:
            self.ser.close()
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05, exclusive=True)
            excl_tag = "exclusive=True (TIOCEXCL locked)"
        except TypeError:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
            excl_tag = "exclusive=N/A"
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"[UART] Opened {self.port_name} @ {self.baud} bps ({excl_tag})", flush=True)

    @staticmethod
    def _decode_registers_str(regs: bytes) -> str:
        if len(regs) < 6:
            return "INVALID"
        addh, addl, r0, r1, ch, r3 = regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]
        u_baud = INV_BAUD.get((r0 >> 5) & 0x07, 0)
        a_rate = INV_AIR.get(r0 & 0x07, 0.0)
        pkt_sz = INV_PKT.get((r1 >> 6) & 0x03, 0)
        tx_pwr = INV_PWR.get(r1 & 0x03, 0)
        rssi_reg = "ON" if (r1 & 0x20) else "OFF"
        mode_s = "Fixed" if (r3 & 0x40) else "Transparent"
        lbt_s = "ON" if (r3 & 0x10) else "OFF"
        rssi_byte = "ON" if (r3 & 0x80) else "OFF"
        freq = 850.125 + ch
        return (
            f"ADD=0x{addh:02X}{addl:02X} REG0=0x{r0:02X} REG1=0x{r1:02X} CHAN=0x{ch:02X}({ch}) REG3=0x{r3:02X} -> "
            f"UART={u_baud} | Air={a_rate}k | Pkt={pkt_sz}B | Pwr={tx_pwr}dBm | "
            f"RSSI_Reg={rssi_reg} | Freq={freq:.3f}MHz | Mode={mode_s} | LBT={lbt_s} | RSSI_Byte={rssi_byte}"
        )

    def inspect_and_configure_eeprom(self) -> bool:
        """Always reads Base E220 EEPROM in Mode 3 on startup; programs target profile if --auto-config is enabled."""
        if not self.gpio_available or serial is None:
            print("[EEPROM] Skipped (GPIO or pyserial not available).", flush=True)
            return False

        baud_bits = BAUD_RATES.get(self.baud, 0b111)
        air_bits = AIR_DATA_RATES.get(self.air_rate_kbps, 0b110)
        pkt_bits = SUBPACKET_SIZES.get(self.packet_size, 0b00)
        pwr_bits = POWER_LEVELS.get(self.power_dbm, 0b11)

        reg0 = (baud_bits << 5) | (0b00 << 3) | air_bits
        # REG1 Bit 5 (0x20) = Enable Mode 0 Ambient Noise & Last Packet RSSI Register (C0 C1 C2 C3)
        reg1 = (pkt_bits << 6) | 0x20 | pwr_bits
        chan = self.channel & 0xFF
        # REG3: Bit 7=0 (No extra byte appended to RX), Bit 6=0 (Transparent Mode), Bit 4=0 (LBT off)
        reg3 = 0x00
        target_regs = bytes([0x00, 0x00, reg0, reg1, chan, reg3, 0x00, 0x00])

        print(f"[EEPROM] Target Profile: {self._decode_registers_str(target_regs)}", flush=True)

        # 1. Enter Configuration Mode 3 (M0=1, M1=1 @ 9600 8N1)
        GPIO.output(self.pin_m0, GPIO.HIGH)
        GPIO.output(self.pin_m1, GPIO.HIGH)
        time.sleep(0.06)
        self.wait_aux(2.0)

        try:
            cfg_ser = serial.Serial(self.port_name, 9600, timeout=0.8, exclusive=True)
        except TypeError:
            cfg_ser = serial.Serial(self.port_name, 9600, timeout=0.8)

        try:
            current_regs = b""
            for _ in range(2):
                cfg_ser.reset_input_buffer()
                cfg_ser.write(bytes([0xC1, 0x00, 0x08]))
                self._safe_flush(cfg_ser)
                time.sleep(0.08)
                res = cfg_ser.read(11)
                if len(res) == 11 and res[0] == 0xC1:
                    current_regs = res[3:]
                    break
                time.sleep(0.08)

            if current_regs:
                print(f"[EEPROM] Current Radio : {self._decode_registers_str(current_regs)}", flush=True)
                self.rssi_reg_enabled = bool(current_regs[3] & 0x20)
            else:
                print("[WARN] Could not read Base E220 registers in Mode 3 (check UART/power/M0/M1).", flush=True)

            eeprom_ok = True
            needs_update = (current_regs != target_regs)
            if needs_update and self.auto_config:
                write_ok = False
                for attempt in range(1, 4):
                    print(f"[AUTO-CONFIG] Programming target EEPROM profile (attempt {attempt}/3)...", flush=True)
                    cfg_ser.reset_input_buffer()
                    cfg_ser.reset_output_buffer()
                    cfg_ser.write(bytes([0xC0, 0x00, 0x08]) + target_regs)
                    self._safe_flush(cfg_ser)
                    time.sleep(0.10)
                    ack = cfg_ser.read(11)
                    if len(ack) == 11 and ack[0] in (0xC0, 0xC1):
                        write_ok = True
                        break
                    time.sleep(0.15)

                time.sleep(0.05)
                cfg_ser.reset_input_buffer()
                cfg_ser.write(bytes([0xC1, 0x00, 0x08]))
                self._safe_flush(cfg_ser)
                time.sleep(0.08)
                verify = cfg_ser.read(11)
                if len(verify) == 11 and verify[0] == 0xC1 and verify[3:] == target_regs:
                    self.rssi_reg_enabled = bool(verify[6] & 0x20)
                    print(f"[AUTO-CONFIG] VERIFIED! EEPROM matches target: {verify[3:].hex().upper()}", flush=True)
                else:
                    eeprom_ok = False
                    actual = verify[3:].hex().upper() if len(verify) == 11 else f"LEN_{len(verify)}"
                    print(
                        f"[ERROR] EEPROM readback mismatch (write_ack={write_ok}): "
                        f"Actual={actual} vs Target={target_regs.hex().upper()}",
                        flush=True,
                    )
            elif needs_update and not self.auto_config:
                print("[WARN] Base E220 EEPROM differs from CLI target and --auto-config was NOT set!", flush=True)
            else:
                print("[EEPROM] Base E220 EEPROM already matches target profile.", flush=True)
        finally:
            cfg_ser.close()

        # 2. Return to Transparent Mode 0 (M0=0, M1=0) and allow Mode 3->Mode 0 settle (>= 55ms + AUX HIGH)
        GPIO.output(self.pin_m0, GPIO.LOW)
        GPIO.output(self.pin_m1, GPIO.LOW)
        time.sleep(0.025)
        self.wait_aux(2.0)
        time.sleep(0.055)
        return eeprom_ok

    def probe_ambient_noise(self) -> None:
        """Query Channel ambient RF noise floor in Mode 0 via C0 C1 C2 C3 00 01 (requires REG1 Bit 5 = 0x20)."""
        if not self.ser or not self.ser.is_open or not self.rssi_reg_enabled:
            return

        samples_dbm: list[int] = []
        for _ in range(3):
            self.wait_aux(0.5)
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01]))
            self._safe_flush(self.ser)
            time.sleep(0.030)
            resp = self.ser.read(4)
            if len(resp) == 4 and resp[0] == 0xC1 and resp[1] == 0x00 and resp[2] == 0x01:
                samples_dbm.append(int(resp[3]) - 256)
            elif len(resp) == 3 and resp[0] == 0xC1 and resp[1] == 0x00:
                samples_dbm.append(int(resp[2]) - 256)
            time.sleep(0.035)

        self.ser.reset_input_buffer()
        freq_mhz = 850.125 + self.channel
        if samples_dbm:
            avg_dbm = sum(samples_dbm) / len(samples_dbm)
            print(
                f"[RF PROBE] Base CH {self.channel} ({freq_mhz:.3f}MHz) Ambient Noise Floor: "
                f"avg={avg_dbm:.1f} dBm (samples={samples_dbm} dBm)",
                flush=True,
            )
            if avg_dbm > -88.0:
                print(
                    f"[WARN] High ambient 900MHz RF noise ({avg_dbm:.1f} dBm > -88 dBm) at Base! "
                    f"Check antenna coax/separation from Pi 4B / USB3 / switching PSU.",
                    flush=True,
                )
        else:
            print("[RF PROBE] Mode 0 RSSI register probe returned no C1 reply.", flush=True)

    def hardware_reset_e220(self, reason: str) -> None:
        """Pulse E220 RESET (GPIO 22) LOW to recover from a wedged LLCC68 / stuck-LOW AUX state."""
        self.watchdog_resets += 1
        print(
            f"[ERROR] WATCHDOG #{self.watchdog_resets}: {reason} -> Pulsing E220 RESET (GPIO {self.pin_rst}) LOW for 50ms...",
            flush=True,
        )
        if self.gpio_available:
            GPIO.output(self.pin_m0, GPIO.LOW)
            GPIO.output(self.pin_m1, GPIO.LOW)
            GPIO.output(self.pin_rst, GPIO.LOW)
            time.sleep(0.050)
            GPIO.output(self.pin_rst, GPIO.HIGH)
            time.sleep(0.100)
            ok = self.wait_aux(timeout=2.0)
            aux_state = "HIGH" if (GPIO.input(self.pin_aux) == GPIO.HIGH) else "STILL_LOW"
            print(f"[RECOVERY] E220 hardware reset complete (wait_ok={ok}, AUX={aux_state})", flush=True)
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def setup_hardware(self) -> None:
        print(f"[INIT] {__app_name__} v{__version__} ({__date__}) initializing hardware...", flush=True)
        print(f"[LOG] Daily CSV logging directory: {self.log_dir}", flush=True)

        if self.gpio_available:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for pin in (self.pin_m0, self.pin_m1, self.pin_rst):
                GPIO.setup(pin, GPIO.OUT)
            GPIO.setup(self.pin_aux, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.output(self.pin_rst, GPIO.HIGH)
            GPIO.output(self.pin_m0, GPIO.LOW)
            GPIO.output(self.pin_m1, GPIO.LOW)
            time.sleep(0.04)
            if not self.wait_aux(timeout=1.5):
                self.hardware_reset_e220("AUX stuck LOW at startup")

            self.inspect_and_configure_eeprom()

            if self.verbose:
                print(
                    f"[HW] GPIO ready: M0={self.pin_m0}, M1={self.pin_m1}, AUX={self.pin_aux}, RST={self.pin_rst}",
                    flush=True,
                )
        else:
            print("[WARN] RPi.GPIO not available - running in calculated Time-on-Air pacing mode.", flush=True)

        self._open_operational_uart()
        self.probe_ambient_noise()

    def cleanup_hardware(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.gpio_available:
            GPIO.cleanup()
        if self.current_log_file:
            try:
                self.current_log_file.close()
            except Exception:
                pass
        print("[HW] Hardware cleaned up.", flush=True)

    def wait_aux(self, timeout: float = AUX_RISE_WATCHDOG_S) -> bool:
        """Wait until E220 AUX pin is HIGH (UART FIFO empty) plus 2ms datasheet guard."""
        if not self.gpio_available:
            return True
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            if GPIO.input(self.pin_aux) == GPIO.HIGH:
                time.sleep(0.002)
                return True
            time.sleep(0.0005)
        return False

    def send_packet(self, chunk: bytes) -> tuple[int, str, float]:
        """
        Feed an intact RTCM3 frame into the E220 and pace strictly per Ebyte E220 Section 5.6.2.
        Returns (num_subpackets, timing_detail_str, total_elapsed_s).
        """
        if not self.ser or not self.ser.is_open or not chunk:
            return (0, "SKIP", 0.0)

        t0 = time.monotonic()
        total_len = len(chunk)

        # Fallback safety for any oversized (> 2 * packet_size = >400B) frame so 400B FIFO never overflows
        if total_len > (self.packet_size * 2):
            sub_cnt = 0
            offset = 0
            while offset < total_len:
                part = chunk[offset : offset + self.packet_size]
                offset += len(part)
                c, _, _ = self.send_packet(part)
                sub_cnt += c
            dt = time.monotonic() - t0
            return (sub_cnt, f"chunked>{self.packet_size * 2}B", dt)

        # 1. Ensure E220 buffer is idle before writing; reset module if wedged
        if not self.wait_aux(timeout=AUX_RISE_WATCHDOG_S):
            self.hardware_reset_e220(f"AUX stuck LOW pre-TX before {total_len}B")

        # 2. Write complete RTCM3 frame atomically to /dev/ttyAMA0
        self.ser.write(chunk)
        if not self._safe_flush(self.ser) and not running:
            return (0, "STOP", time.monotonic() - t0)
        t_after_flush = time.monotonic()

        # 3. Pace over-the-air completion according to subpacket count
        if total_len <= self.packet_size:
            # -----------------------------------------------------------------
            # REGIME A: Single-Subpacket Frame (<= 200B, e.g. 1006, 1230, 1084)
            # Per E220 Sec 5.6.2, <200B transfers immediately into an idle LLCC68
            # FIFO without holding AUX LOW. Do NOT block 500ms waiting for AUX=0!
            # -----------------------------------------------------------------
            num_subpkts = 1
            est_air_s = self.estimate_subpacket_air_s(total_len)
            aux_dropped = False
            aux_low_s = 0.0

            if self.gpio_available:
                while (time.monotonic() - t_after_flush) < AUX_SINGLE_PROBE_S:
                    if GPIO.input(self.pin_aux) == GPIO.LOW:
                        aux_dropped = True
                        break
                    time.sleep(0.0005)

                if aux_dropped:
                    t_low = time.monotonic()
                    if not self.wait_aux(timeout=AUX_RISE_WATCHDOG_S):
                        self.hardware_reset_e220(f"AUX stuck LOW >{AUX_RISE_WATCHDOG_S}s on 1-pkt {total_len}B")
                    aux_low_s = time.monotonic() - t_low

            elapsed_since_flush = time.monotonic() - t_after_flush
            rem_air_s = est_air_s - elapsed_since_flush
            if rem_air_s > 0.0:
                time.sleep(rem_air_s)

            eff_guard_s = self._get_effective_guard_delay_s()
            time.sleep(eff_guard_s)
            detail = (
                f"1 pkt | EstAir={est_air_s*1000:.1f}ms"
                + (f"(AUX_low={aux_low_s*1000:.1f}ms)" if aux_dropped else "")
                + f" + Guard={eff_guard_s*1000:.0f}ms"
            )
        else:
            # -----------------------------------------------------------------
            # REGIME B: Multi-Subpacket Frame (> 200B, e.g. 1074, 1094, 1124)
            # Supports 2 subpackets (201..400B) or 3+ subpackets (>400B) cleanly.
            # Subpacket 1 (200B) starts RF TX while trailing subpacket(s) drain
            # through UART RAM holding AUX LOW. Enforce both tail wait and full
            # multi-subpacket calibrated ToA floor + effective guard delay.
            # -----------------------------------------------------------------
            num_subpkts = (total_len + self.packet_size - 1) // self.packet_size
            full_heads = num_subpkts - 1
            tail_len = total_len - (full_heads * self.packet_size)
            total_air_est_s = self.estimate_frame_air_s(total_len)
            tail_air_s = self.estimate_subpacket_air_s(tail_len)
            aux_dropped = False
            drop_ms = 0.0
            aux_low_s = 0.0

            if self.gpio_available:
                while (time.monotonic() - t_after_flush) < AUX_MULTI_FALL_TIMEOUT_S:
                    if GPIO.input(self.pin_aux) == GPIO.LOW:
                        aux_dropped = True
                        drop_ms = (time.monotonic() - t_after_flush) * 1000.0
                        break
                    time.sleep(0.0005)

                if aux_dropped:
                    t_low = time.monotonic()
                    if not self.wait_aux(timeout=AUX_RISE_WATCHDOG_S):
                        self.hardware_reset_e220(f"AUX stuck LOW >{AUX_RISE_WATCHDOG_S}s on {num_subpkts}-pkt {total_len}B")
                    aux_low_s = time.monotonic() - t_low
                    # At <=9.6 kbps, the E220 MCU empties its UART RAM buffer and raises AUX HIGH
                    # ~100-147ms BEFORE Subpacket 1 (200B) finishes transmitting on RF!
                    # Enforce both the final subpacket tail wait and full multi-subpacket ToA floor.
                    elapsed_since_flush = time.monotonic() - t_after_flush
                    rem_air_s = max(tail_air_s, total_air_est_s - elapsed_since_flush)
                    time.sleep(rem_air_s)
                else:
                    # Fallback if AUX edge was missed: sleep full calculated multi-subpacket air time
                    elapsed_since_flush = time.monotonic() - t_after_flush
                    rem_air_s = max(0.0, total_air_est_s - elapsed_since_flush)
                    if rem_air_s > 0.0:
                        time.sleep(rem_air_s)
            else:
                rem_air_s = tail_air_s
                time.sleep(total_air_est_s)

            eff_guard_s = self._get_effective_guard_delay_s()
            time.sleep(eff_guard_s)
            pkt_desc = f"2 pkts ({self.packet_size}+{tail_len}B)" if num_subpkts == 2 else f"{num_subpkts} pkts ({full_heads}x{self.packet_size}+{tail_len}B)"
            if aux_dropped:
                tail_note = (
                    f"TailWait={rem_air_s*1000:.1f}ms(Tail={tail_air_s*1000:.1f}ms)"
                    if rem_air_s > (tail_air_s + 0.002)
                    else f"TailAir={tail_air_s*1000:.1f}ms"
                )
                detail = (
                    f"{pkt_desc} | "
                    f"AUX_low={aux_low_s*1000:.1f}ms(drop={drop_ms:.1f}ms) + "
                    f"{tail_note} + Guard={eff_guard_s*1000:.0f}ms"
                )
            else:
                detail = (
                    f"{pkt_desc} | "
                    f"NO-AUX-DROP EstAir={total_air_est_s*1000:.1f}ms + Guard={eff_guard_s*1000:.0f}ms"
                )

        total_dt = time.monotonic() - t0
        self.total_wait_time_s += total_dt
        self.total_bytes_out += total_len
        self.total_packets_sent += 1
        self.total_subpackets_sent += num_subpkts

        if self.verbose:
            print(f"[TX PKT] {total_len:3d}B | {detail} | Total={total_dt*1000:.1f}ms", flush=True)

        return (num_subpkts, detail, total_dt)

    def send_rtcm_batch(self, items: list[tuple[int, bytes, float]]) -> None:
        """Transmit 1 or more packed RTCM3 frames (<= packet_size when coalesced) and record telemetry."""
        if not items:
            return
        payload = b"".join(frame for _, frame, _ in items)
        tot_len = len(payload)
        _, detail, total_dt = self.send_packet(payload)
        with self._mailbox_lock:
            for msg_type, _, _ in items:
                self.rtcm_stats[msg_type] = self.rtcm_stats.get(msg_type, 0) + 1

        if not self.quiet:
            if len(items) == 1:
                msg_type, _, age_ms = items[0]
                print(
                    f"[TX RTCM] Type:{msg_type:<4d} ({tot_len:3d}B, age:{age_ms:4.0f}ms) -> {detail} | Total={total_dt*1000:5.1f}ms",
                    flush=True,
                )
            else:
                desc = "+".join(f"{m}({len(f)}B)" for m, f, _ in items)
                max_age = max(age_ms for _, _, age_ms in items)
                print(
                    f"[TX RTCM] Coalesced[{desc}] ({tot_len:3d}B, age:{max_age:4.0f}ms) -> {detail} | Total={total_dt*1000:5.1f}ms",
                    flush=True,
                )

    def send_rtcm_frame(self, frame: bytes, msg_type: int) -> None:
        """Transmit a verified, CRC24Q-valid RTCM3 frame and record telemetry."""
        self.send_rtcm_batch([(msg_type, frame, 0.0)])

    def _deposit_mailbox_frame(self, msg_type: int, frame: bytes, rx_now: float) -> None:
        """Deposit a CRC24Q-verified RTCM3 frame into its latest-value ping-pong mailbox."""
        with self._mailbox_lock:
            if msg_type in (1084, 1094, 1124):
                self.seen_secondary_types.add(msg_type)
            slot = self._mailboxes.get(msg_type)
            if slot is None:
                self._mailboxes[msg_type] = MailboxSlot(
                    msg_type=msg_type,
                    frame=frame,
                    rx_monotonic=rx_now,
                )
            else:
                if slot.frame is not None:
                    slot.superseded_count += 1
                    self.mailbox_superseded += 1
                slot.frame = frame
                slot.rx_monotonic = rx_now
        self._mailbox_event.set()

    def _clear_pending_mailboxes(self) -> None:
        """Discard untransmitted mailbox frames on rate transition while preserving starvation history."""
        with self._mailbox_lock:
            dropped = 0
            for slot in self._mailboxes.values():
                if slot.frame is not None:
                    slot.frame = None
                    dropped += 1
            if dropped > 0:
                self.backlog_shed_frames += dropped

    def _pop_next_tx_batch(self, remaining_window_ms: float) -> list[tuple[int, bytes, float]]:
        """
        Pop the highest-priority pending mailbox frame that fits inside remaining_window_ms,
        coalescing additional <= packet_size frames while combined length <= packet_size and
        combined ToA fits inside remaining_window_ms.
        """
        if remaining_window_ms <= 0.0:
            return []

        max_window_ms = (
            float(max(1, self.macro_cycle_s - 1)) * 1000.0
            + self._get_sec0_downlink_cutoff_ms()
        )

        with self._mailbox_lock:
            now = time.monotonic()

            # 1. Expire stale untransmitted payloads (2.5s dynamic MSM, 15s static station metadata)
            for slot in self._mailboxes.values():
                if slot.frame is None:
                    continue
                ttl = STALE_STATIC_TTL_S if slot.msg_type in STATIC_RTCM_TYPES else STALE_FRAME_TTL_S
                if (now - slot.rx_monotonic) > ttl:
                    slot.frame = None
                    self.mailbox_stale_dropped += 1

            # 2. Gather fitting pending slots ordered cleanly by tier, starvation age, and message type
            candidates: list[tuple[tuple[int, float, int], MailboxSlot]] = []
            for slot in self._mailboxes.values():
                if slot.frame is None:
                    continue
                est_toa_ms = self._estimate_frame_toa_ms(len(slot.frame))
                if est_toa_ms > max_window_ms:
                    slot.frame = None
                    self.mailbox_stale_dropped += 1
                    continue
                if est_toa_ms > remaining_window_ms:
                    continue

                is_static = 0 if slot.msg_type in STATIC_RTCM_TYPES else 1
                if is_static == 0:
                    # Oldest pending static metadata frame first
                    rank_val = -(now - slot.rx_monotonic)
                elif self.enable_interleave:
                    # Starvation-fair round robin anchored toward GPS (1074)
                    starve_s = (now - slot.last_tx_monotonic) if slot.last_tx_monotonic > 0.0 else 9999.0
                    if slot.msg_type == 1074:
                        starve_s += 0.35
                    rank_val = -starve_s
                else:
                    rank_val = slot.rx_monotonic
                candidates.append(((is_static, rank_val, slot.msg_type), slot))

            if not candidates:
                return []

            candidates.sort(key=lambda pair: pair[0])
            head = candidates[0][1]
            assert head.frame is not None

            batch_slots: list[MailboxSlot] = [head]
            batch_len = len(head.frame)

            # 3. Smart <= packet_size (200B) Coalescing into a single Regime-A UART write
            if batch_len <= self.packet_size:
                for _, cand in candidates[1:]:
                    assert cand.frame is not None
                    cand_len = batch_len + len(cand.frame)
                    if cand_len > self.packet_size:
                        continue
                    if self._estimate_frame_toa_ms(cand_len) > remaining_window_ms:
                        continue
                    batch_slots.append(cand)
                    batch_len = cand_len

            result: list[tuple[int, bytes, float]] = []
            for slot in batch_slots:
                assert slot.frame is not None
                age_ms = max(0.0, (now - slot.rx_monotonic) * 1000.0)
                if age_ms > self.mailbox_max_age_ms:
                    self.mailbox_max_age_ms = age_ms
                result.append((slot.msg_type, slot.frame, age_ms))
                slot.frame = None
                slot.last_tx_monotonic = now

            if len(result) > 1:
                self.coalesced_batches += 1
                self.coalesced_frames += len(result)

            return result

    def _get_superframe_timing(self) -> tuple[int, int, float]:
        """Returns (utc_sec, macro_sec [0..macro_cycle_s-1], ms_in_sec [0..1000)) anchored to wall-clock."""
        now_wall = time.time()
        utc_sec = int(now_wall)
        superframe_sec = utc_sec % self.macro_cycle_s
        ms_in_sec = (now_wall - float(utc_sec)) * 1000.0
        return (utc_sec, superframe_sec, ms_in_sec)

    def _get_sec0_downlink_cutoff_ms(self) -> float:
        """Returns the Second 0 Base Downlink TX cutoff (ms) leaving >=50ms before Rover uplink."""
        if self.air_rate_kbps <= 5.0:
            return 450.0
        if self.air_rate_kbps <= 10.0:
            return 620.0
        return 680.0

    def _get_remaining_downlink_ms(self) -> float:
        """
        Compute continuous remaining downlink TX budget (ms) before the consolidated Second 0
        pause cutoff or an impending synchronous air-rate switch (Second 1 ACK / Second 2 Mode 3).
        Seconds 1..(macro_cycle_s - 1) stay 100% unbroken downlink with zero end-of-second pauses.
        """
        utc_sec, sf_sec, ms_in_sec = self._get_superframe_timing()
        if self.pending_switch_exec_sec > 0:
            if utc_sec >= self.pending_switch_ack_sec:
                return 0.0
            return max(0.0, float(self.pending_switch_ack_sec - utc_sec) * 1000.0 - ms_in_sec)

        cutoff_ms = self._get_sec0_downlink_cutoff_ms()
        if sf_sec == 0:
            return max(0.0, cutoff_ms - ms_in_sec)
        secs_until_sec0 = self.macro_cycle_s - sf_sec
        return (float(secs_until_sec0) * 1000.0 - ms_in_sec) + cutoff_ms

    def _estimate_frame_toa_ms(self, frame_len: int) -> float:
        """Estimate total UART + Stage-1-calibrated RF Time-on-Air + effective guard delay in ms."""
        if frame_len <= 0:
            return 0.0
        air_s = self.estimate_frame_air_s(frame_len)
        uart_s = (float(frame_len) * 10.0) / max(float(self.baud), 9600.0)
        guard_s = self._get_effective_guard_delay_s()
        return (air_s + uart_s + guard_s) * 1000.0

    @staticmethod
    def _build_switch_ack_frame(req_id: int, delay_ms: int) -> bytes:
        """Builds a CRC16-protected 11-byte ACK_RADIO_SWITCH frame with remaining delay to Second 2."""
        clamped_delay = max(100, min(1500, int(delay_ms)))
        ack_body = struct.pack(FMT_ACK_SWITCH, ROVER_SYNC, MSG_ID_ACK_SWITCH, 5, req_id, 0, clamped_delay, 0)
        ack_crc = crc16_ccitt(ack_body[:-2])
        return ack_body[:-2] + struct.pack("<H", ack_crc)

    def _transmit_pending_switch_ack_if_due(self, utc_sec: int, ms_in_sec: float) -> None:
        if self.pending_switch_ack_frame is None or self.ser is None or not self.ser.is_open:
            return
        if utc_sec != self.pending_switch_ack_sec:
            return
        total_slots = len(SWITCH_ACK_SLOTS_MS)
        if self.pending_switch_acks_sent >= total_slots:
            return
        if ms_in_sec < SWITCH_ACK_SLOTS_MS[self.pending_switch_acks_sent]:
            return

        req_id = self.pending_switch_req_id
        slot_idx = self.pending_switch_acks_sent + 1
        rem_ms = max(100, int(round(1000.0 - ms_in_sec)))
        ack_frame = self._build_switch_ack_frame(req_id, rem_ms)
        self.pending_switch_acks_sent = slot_idx
        self.pending_switch_ack_sent = True
        self.pending_switch_ack_repeat_sent = (slot_idx >= total_slots)

        _, detail, total_dt = self.send_packet(ack_frame)
        print(
            f"[LORA SWITCH] Sent synchronous ACK #{slot_idx}/{total_slots} in Second 1 "
            f"(Req #{req_id} @ {ms_in_sec:.0f}ms, delay {rem_ms}ms | {detail}, {total_dt*1000:.1f}ms)",
            flush=True,
        )

    def _drive_pending_rate_switch(self, buffer: Optional[bytearray] = None) -> bool:
        """Drives the pending Second 1 4-ACK spaced train and executes Second 2 E220 reconfiguration."""
        if self.pending_switch_exec_sec <= 0:
            return False

        utc_sec, _, _ = self._get_superframe_timing()
        if utc_sec < self.pending_switch_exec_sec:
            while running:
                utc_sec, _, cur_ms = self._get_superframe_timing()
                if utc_sec >= self.pending_switch_exec_sec:
                    break
                if utc_sec < self.pending_switch_ack_sec:
                    self.poll_uplink()
                else:
                    self._transmit_pending_switch_ack_if_due(utc_sec, cur_ms)
                time.sleep(0.005)

        if self.pending_switch_exec_sec > 0 and utc_sec >= self.pending_switch_exec_sec:
            target_rate = self.pending_switch_target_rate
            prev_rate = self.air_rate_kbps
            self.pending_switch_exec_sec = 0
            self.pending_switch_ack_frame = None
            self.pending_switch_ack_sent = False
            self.pending_switch_ack_repeat_sent = False
            self.pending_switch_acks_sent = 0
            ok = self.reconfigure_air_rate(target_rate)
            if ok:
                self.fallback_air_rate = prev_rate
                self.active_air_rate = target_rate
                self.rate_switch_count += 1
                self.watchdog_rate_switch_time = time.monotonic()
                self.watchdog_rate_revert_time = self.watchdog_rate_switch_time + 60.0
                print(
                    f"[LORA SWITCH] Synchronously reconfigured Base E220 to {target_rate}k at Second 2 boundary! "
                    f"Rollback watchdog armed (60s -> fallback {self.fallback_air_rate}k).",
                    flush=True,
                )
            else:
                print(f"[ERROR] Failed to reconfigure Base E220 to {target_rate}k! Staying on {prev_rate}k.", flush=True)
            self._clear_pending_mailboxes()

        return self.pending_switch_exec_sec > 0 and utc_sec < self.pending_switch_exec_sec

    def _service_superframe_events(self, buffer: Optional[bytearray] = None) -> bool:
        """Service pending rate switches during synchronous Second 1/Second 2 windows."""
        return self._drive_pending_rate_switch(buffer)

    def _enforce_quiet_window(self, buffer: bytearray) -> None:
        """Legacy helper retained cleanly for callers expecting rate switch servicing."""
        self._service_superframe_events(buffer)

    def _write_atomic_json(self, data: dict) -> None:
        """Writes live rover telemetry atomically to RAM tmpfs (/run or /tmp) with zero SD card wear."""
        for target_dir in ("/run", "/tmp"):
            path = os.path.join(target_dir, "rtk_rover_status.json")
            tmp_path = path + ".tmp"
            try:
                with open(tmp_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2)
                os.replace(tmp_path, path)
                break
            except (OSError, PermissionError):
                continue

    def _write_csv_record(self, data: dict) -> None:
        """Appends telemetry to daily CSV with rollover, gzip compression of yesterday's log, and retention pruning."""
        if not self.enable_csv:
            return
        today_str = time.strftime("%Y-%m-%d")
        if today_str != self.current_log_date:
            if self.current_log_file:
                try:
                    self.current_log_file.close()
                except Exception:
                    pass
                if self.compress_logs and self.current_log_date:
                    self._compress_log(self.current_log_date)
            self._prune_old_logs()
            self.current_log_date = today_str
            os.makedirs(self.log_dir, exist_ok=True)
            try:
                os.chmod(self.log_dir, 0o777)
            except OSError:
                pass
            csv_path = os.path.join(self.log_dir, f"rover_{today_str}.csv")
            file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
            self.current_log_file = open(csv_path, "a", encoding="utf-8", buffering=1)
            if not file_exists:
                try:
                    os.chmod(csv_path, 0o666)
                except OSError:
                    pass
                headers = [
                    "timestamp_utc", "seq", "fix_type", "fix_name",
                    "ant2_fix", "baseline_mm", "pitch_deg", "hdg_source",
                    "latitude", "longitude", "height_m", "h_acc_mm",
                    "sats_master", "sats_slave", "heading_deg",
                    "battery_pct", "battery_mv",
                    "rover_rssi_dbm", "rover_noise_dbm",
                    "air_rate_kbps", "channel", "rover_fw"
                ]
                self.current_log_file.write(",".join(headers) + "\n")

        row = [
            data["timestamp_utc"],
            str(data["seq"]),
            str(data["fix_type"]),
            data["fix_name"],
            data.get("ant2_fix_name", "NONE"),
            str(int(data.get("baseline_length_cm", 0.0) * 10)),
            f"{data.get('baseline_pitch_deg', 0.0):.2f}",
            data.get("heading_source", "NONE"),
            f"{data['latitude']:.8f}",
            f"{data['longitude']:.8f}",
            f"{data['height_m']:.3f}",
            str(data["h_acc_mm"]),
            str(data["sats_master"]),
            str(data["sats_slave"]),
            f"{data['heading_deg']:.2f}",
            str(data["battery_pct"]),
            str(data["battery_mv"]),
            str(data["rover_rssi_dbm"]),
            str(data["rover_noise_dbm"]),
            f"{data['base_air_rate_kbps']:.1f}",
            str(data["channel"]),
            data.get("rover_fw_version", "unknown"),
        ]
        if self.current_log_file:
            self.current_log_file.write(",".join(row) + "\n")

    def _prune_old_logs(self) -> None:
        """Enforces retention policy (deletes files older than N days) and max total directory size (FIFO cap)."""
        if not os.path.exists(self.log_dir):
            return
        now = time.time()
        cutoff_s = self.log_retention_days * 86400.0
        files = []
        for fname in os.listdir(self.log_dir):
            if fname.startswith("rover_") and (fname.endswith(".csv") or fname.endswith(".csv.gz")):
                fpath = os.path.join(self.log_dir, fname)
                try:
                    mtime = os.path.getmtime(fpath)
                    sz = os.path.getsize(fpath)
                    files.append((fpath, mtime, sz))
                    if (now - mtime) > cutoff_s:
                        os.remove(fpath)
                        print(f"[LOG RETENTION] Pruned expired log (> {self.log_retention_days}d): {fname}", flush=True)
                except OSError:
                    pass

        if self.max_log_mb > 0:
            total_sz = sum(sz for p, _, sz in files if os.path.exists(p))
            max_bytes = self.max_log_mb * 1024 * 1024
            if total_sz > max_bytes:
                files.sort(key=lambda x: x[1])
                for fpath, _, sz in files:
                    if total_sz <= max_bytes:
                        break
                    try:
                        if os.path.exists(fpath):
                            os.remove(fpath)
                            total_sz -= sz
                            print(f"[LOG CEILING] Pruned oldest log to keep total < {self.max_log_mb}MB: {os.path.basename(fpath)}", flush=True)
                    except OSError:
                        pass

    def _compress_log(self, date_str: str) -> None:
        """Compresses closed daily CSV using standard library gzip."""
        raw_path = os.path.join(self.log_dir, f"rover_{date_str}.csv")
        gz_path = raw_path + ".gz"
        if os.path.exists(raw_path) and not os.path.exists(gz_path):
            try:
                with open(raw_path, "rb") as f_in, gzip.open(gz_path, "wb") as f_out:
                    shutil.copyfileobj(f_in, f_out)
                os.remove(raw_path)
                print(f"[LOG GZIP] Compressed {os.path.basename(raw_path)} -> {os.path.basename(gz_path)}", flush=True)
            except Exception as e:
                print(f"[WARN] Failed to compress {raw_path}: {e}", flush=True)

    def reconfigure_air_rate(self, new_air_rate: float) -> bool:
        """Dynamically reconfigures Base E220 EEPROM to new air-rate in Mode 3 without restarting service."""
        if not self.gpio_available or serial is None:
            return False
        old_rate = self.air_rate_kbps
        self.air_rate_kbps = new_air_rate
        prev_auto = self.auto_config
        self.auto_config = True

        if self.ser and self.ser.is_open:
            self._safe_flush(self.ser)
            self.wait_aux(1.0)
            self.ser.close()

        ok = self.inspect_and_configure_eeprom()
        self.auto_config = prev_auto
        self._open_operational_uart()
        if ok:
            self.active_air_rate = new_air_rate
            print(f"[RECONFIG] Base E220 air-rate switched {old_rate}k -> {new_air_rate}k successfully!", flush=True)
        else:
            self.air_rate_kbps = old_rate
            self.active_air_rate = old_rate
            print(f"[RECONFIG] Base E220 air-rate switch to {new_air_rate}k failed! Restored {old_rate}k.", flush=True)
        return ok

    def _check_watchdog_revert(self) -> None:
        """Rolls back to previous working fallback rate if 60 seconds elapse after a switch without rover traffic."""
        if self.watchdog_rate_revert_time > 0:
            now = time.monotonic()
            if self.last_rover_rx_time > self.watchdog_rate_switch_time:
                self.watchdog_rate_revert_time = 0.0
                self.fallback_air_rate = self.air_rate_kbps
                print(
                    f"[LORA WATCHDOG] Verified uplink at {self.air_rate_kbps}k! Rollback watchdog disarmed.",
                    flush=True,
                )
            elif now >= self.watchdog_rate_revert_time:
                self.watchdog_rate_revert_time = 0.0
                revert_rate = self.fallback_air_rate if 2.0 <= self.fallback_air_rate <= 65.0 else 19.2
                print(
                    f"[LORA WATCHDOG] 60s silence after switch to {self.air_rate_kbps}k! Reverting to {revert_rate}k fallback...",
                    flush=True,
                )
                if abs(self.air_rate_kbps - revert_rate) >= 0.2:
                    self.reconfigure_air_rate(revert_rate)

    def _handle_telemetry_packet(self, cand: bytes) -> None:
        self.uplink_packets_received += 1
        fix_names = {0: "NONE", 1: "SINGLE", 2: "DGPS", 3: "PPS", 4: "RTK_FIX", 5: "RTK_FLOAT", 6: "DR"}
        slv_fix_names = {0: "NONE", 1: "SLV_SGL", 2: "SLV_DGPS", 3: "PPS", 4: "SLV_FIX", 5: "SLV_FLT", 6: "DR"}
        hdg_source_names = {
            0: "NONE",
            1: "DUAL_FIX",
            2: "DUAL_FLT",
            3: "WALK",
            4: "GYRO",
            5: "COG",
            6: "HOLD",
            7: "COMP",
        }

        if len(cand) == 48:
            (sync, mid, plen, lat, lon, height, h_acc_mm, fix_type,
             ant2_fix_type, baseline_mm, baseline_pitch_cdeg, heading_source,
             sats_m, sats_s, hdg_cdeg, bat_pct, bat_mv, r_rssi, r_noise, seq,
             fw_major, fw_minor, fw_patch, crc) = struct.unpack(FMT_TELEMETRY, cand)
            bl_cm = baseline_mm / 10.0
            pitch_deg = baseline_pitch_cdeg / 100.0
            slv_fix_str = slv_fix_names.get(ant2_fix_type, f"TYPE_{ant2_fix_type}")
            hdg_tag = hdg_source_names.get(heading_source, f"SRC_{heading_source}")
            rover_fw_str = f"v{fw_major}.{fw_minor}.{fw_patch}"
        elif len(cand) == 45:
            (sync, mid, plen, lat, lon, height, h_acc_mm, fix_type,
             ant2_fix_type, baseline_mm, baseline_pitch_cdeg, heading_source,
             sats_m, sats_s, hdg_cdeg, bat_pct, bat_mv, r_rssi, r_noise, seq, crc) = struct.unpack(FMT_TELEMETRY_V2, cand)
            bl_cm = baseline_mm / 10.0
            pitch_deg = baseline_pitch_cdeg / 100.0
            slv_fix_str = slv_fix_names.get(ant2_fix_type, f"TYPE_{ant2_fix_type}")
            hdg_tag = hdg_source_names.get(heading_source, f"SRC_{heading_source}")
            rover_fw_str = "unknown"
        else:
            (sync, mid, plen, lat, lon, height, h_acc_mm, fix_type, sats_m, sats_s,
             hdg_cdeg, bat_pct, bat_mv, r_rssi, r_noise, seq, crc) = struct.unpack(FMT_TELEMETRY_LEGACY, cand)
            bl_cm = 55.5
            pitch_deg = 0.0
            ant2_fix_type = 4 if (sats_s & 0x80) else (5 if (sats_s & 0x40) else 0)
            slv_fix_str = slv_fix_names.get(ant2_fix_type, "NONE")
            heading_source = 1 if (sats_s & 0x80) else (2 if (sats_s & 0x40) else 0)
            hdg_tag = hdg_source_names.get(heading_source, "NONE")
            rover_fw_str = "unknown"

        fix_str = fix_names.get(fix_type, f"TYPE_{fix_type}")
        hdg_deg = hdg_cdeg / 100.0

        # High 2 bits of sats_s: bit 6 = heading_valid, bit 7 = heading_is_fixed
        sats_s_clean = sats_s & 0x3F
        hdg_valid = bool(sats_s & 0x40) or (heading_source in (1, 2, 3, 4, 5, 6, 7))
        hdg_fixed = bool(sats_s & 0x80) or (heading_source == 1)

        bat_str = f"PWR:USB ({bat_mv/1000.0:.2f}V)" if bat_pct == 255 else f"Bat:{bat_pct}% ({bat_mv/1000.0:.2f}V)"

        print(
            f"[ROVER TEL #{seq:03d}] FW:{rover_fw_str} | ANT1:{fix_str} ({sats_m}M) | "
            f"ANT2:{slv_fix_str} ({sats_s_clean}S, BL:{bl_cm:.1f}cm, P:{pitch_deg:+.1f}deg) | "
            f"Hdg:{hdg_deg:5.1f}deg [{hdg_tag}] | {bat_str} | "
            f"RSSI:{r_rssi}dBm | Pos:{lat:.7f}, {lon:.7f}, {height:.2f}m",
            flush=True,
        )

        status_data = {
            "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "seq": seq,
            "rover_fw_version": rover_fw_str,
            "fix_type": fix_type,
            "fix_name": fix_str,
            "ant2_fix_type": ant2_fix_type,
            "ant2_fix_name": slv_fix_str,
            "latitude": round(lat, 8),
            "longitude": round(lon, 8),
            "height_m": round(height, 3),
            "h_acc_mm": h_acc_mm,
            "sats_master": sats_m,
            "sats_slave": sats_s_clean,
            "baseline_length_cm": round(bl_cm, 1),
            "baseline_pitch_deg": round(pitch_deg, 2),
            "heading_source": hdg_tag,
            "heading_deg": round(hdg_deg, 2),
            "heading_valid": hdg_valid,
            "heading_fixed": hdg_fixed,
            "battery_pct": bat_pct,
            "battery_mv": bat_mv,
            "rover_rssi_dbm": r_rssi,
            "rover_noise_dbm": r_noise,
            "base_air_rate_kbps": self.air_rate_kbps,
            "channel": self.channel,
        }
        self.last_rover_telemetry = status_data
        self._write_atomic_json(status_data)
        self._write_csv_record(status_data)

    def _handle_rate_switch_request(self, cand: bytes) -> None:
        (sync, mid, plen, target_code, target_chan, req_id, crc) = struct.unpack(FMT_REQ_SWITCH, cand)
        code_to_air = {0x02: 2.4, 0x03: 4.8, 0x04: 9.6, 0x05: 19.2, 0x06: 38.4, 0x07: 62.5}
        target_rate = code_to_air.get(target_code, 19.2)

        t0_sec = int(time.time())
        ack_sec = t0_sec + 1
        exec_sec = t0_sec + 2

        print(
            f"[LORA SWITCH] Received switch request in T0={t0_sec} (MC={t0_sec % self.macro_cycle_s}): "
            f"{self.air_rate_kbps}k -> {target_rate}k (Req #{req_id}, CH{target_chan}) | "
            f"ACK@T0+1, Switch@T0+2",
            flush=True,
        )

        # Build default ACK frame: sync(2), msg_id=0x82(1), plen=5(1), req_id(2), status=0(1), delay=1000ms(2), crc(2) = 11B
        ack_frame = self._build_switch_ack_frame(req_id, 1000)

        # Schedule synchronous 4-ACK train in Second 1 (T0 + 1) and hardware switch at top of Second 2 (T0 + 2)
        self.pending_switch_ack_frame = ack_frame
        self.pending_switch_ack_sent = False
        self.pending_switch_ack_repeat_sent = False
        self.pending_switch_acks_sent = 0
        self.pending_switch_req_id = req_id
        self.pending_switch_target_rate = target_rate
        self.pending_switch_ack_sec = ack_sec
        self.pending_switch_exec_sec = exec_sec

    def poll_uplink(self) -> None:
        """Polls Base E220 serial port for 0xAA 0x55 uplink telemetry or rate switch commands from Rover."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            avail = self.ser.in_waiting
        except Exception:
            return

        if avail > 0:
            try:
                raw = self.ser.read(avail)
                if raw:
                    self.uplink_buf.extend(raw)
            except Exception:
                return

        while len(self.uplink_buf) >= 2:
            if self.uplink_buf[0] != 0xAA or self.uplink_buf[1] != 0x55:
                idx = self.uplink_buf.find(ROVER_SYNC)
                if idx == -1:
                    self.uplink_buf.clear()
                    break
                del self.uplink_buf[:idx]
                if len(self.uplink_buf) < 2:
                    break

            if len(self.uplink_buf) < 4:
                break

            msg_id = self.uplink_buf[2]
            plen = self.uplink_buf[3]
            tot_len = plen + 6
            if len(self.uplink_buf) < tot_len:
                break

            cand = bytes(self.uplink_buf[:tot_len])
            calc_crc = crc16_ccitt(cand[:-2])
            pkt_crc = cand[-2] | (cand[-1] << 8)

            if calc_crc != pkt_crc:
                self.uplink_crc_errors += 1
                del self.uplink_buf[:1]
                continue

            del self.uplink_buf[:tot_len]
            self.last_rover_rx_time = time.monotonic()

            if msg_id == MSG_ID_TELEMETRY and tot_len in (48, 45, 39):
                self._handle_telemetry_packet(cand)
            elif msg_id == MSG_ID_REQ_SWITCH and tot_len == 10:
                self._handle_rate_switch_request(cand)

    def _ingest_rtcm_bytes(self, buffer: bytearray, data: bytes, last_rx_data_time: float) -> float:
        """
        Parse incoming RTCM3 bytes cleanly on the ingress thread, verify CRC24Q, drop empty
        MSMs, and deposit verified frames directly into latest-value ping-pong mailboxes.
        """
        now = time.monotonic()
        if data:
            with self._mailbox_lock:
                self.total_bytes_in += len(data)
            buffer.extend(data)
            last_rx_data_time = now

        while len(buffer) >= 3:
            # 1. Sync strictly to RTCM3 preamble 0xD3 (never transmit non-RTCM orphan bytes over LoRa)
            if buffer[0] != 0xD3:
                d3_idx = buffer.find(b"\xd3")
                if d3_idx == -1:
                    with self._mailbox_lock:
                        self.src_orphan_bytes += len(buffer)
                    buffer.clear()
                    break
                with self._mailbox_lock:
                    self.src_orphan_bytes += d3_idx
                del buffer[:d3_idx]
                if len(buffer) < 3:
                    break

            # 2. Validate 6 zero reserved bits and 10-bit payload length [6..1023]
            if (buffer[1] & 0xFC) != 0x00:
                with self._mailbox_lock:
                    self.src_orphan_bytes += 1
                del buffer[:1]
                continue

            payload_len = ((buffer[1] & 0x03) << 8) | buffer[2]
            if payload_len < 6 or payload_len > 1023:
                with self._mailbox_lock:
                    self.src_orphan_bytes += 1
                del buffer[:1]
                continue

            total_frame_len = payload_len + 6

            # 3. Wait for complete frame; if incomplete for >350ms, step past false 0xD3
            if len(buffer) < total_frame_len:
                if (now - last_rx_data_time) >= 0.350:
                    with self._mailbox_lock:
                        self.src_orphan_bytes += 1
                    del buffer[:1]
                    continue
                break

            # 4. Verify 24-bit CRC24Q before depositing into TX mailbox
            frame = bytes(buffer[:total_frame_len])
            msg_type = (frame[3] << 4) | (frame[4] >> 4)
            if rtcm_crc24q(frame) != 0:
                with self._mailbox_lock:
                    self.src_crc_errors += 1
                print(
                    f"[WARN] Dropped corrupt source RTCM candidate Type:{msg_type} ({total_frame_len}B, bad CRC24Q)",
                    flush=True,
                )
                del buffer[:1]
                continue

            del buffer[:total_frame_len]

            # 4b. Filter empty MSM frames (<= 28B) before spending LoRa airtime
            if is_empty_msm_frame(frame, msg_type):
                with self._mailbox_lock:
                    self.empty_msm_dropped += 1
                if self.verbose:
                    print(
                        f"[EMPTY MSM] Dropped empty Type:{msg_type} ({total_frame_len}B <= 28B)",
                        flush=True,
                    )
                continue

            self._deposit_mailbox_frame(msg_type, frame, now)

        return last_rx_data_time

    def process_stream(self, read_fn) -> None:
        """
        Main Arch 2B loop: consumes RTKBase stream asynchronously via ping-pong mailboxes,
        coalesces small <= packet_size frames, and paces TX across the consolidated N-second
        macro-cycle while keeping Second 0 clear for Rover uplink and Mode 0 RSSI polling.
        """
        last_report = time.monotonic()
        self._ingress_stop = threading.Event()
        self._ingress_exc: Optional[BaseException] = None
        self._mailbox_event.clear()

        def _ingress_worker() -> None:
            ingress_buf = bytearray()
            last_rx_data_time = time.monotonic()
            while running and not self._ingress_stop.is_set():
                t_read0 = time.monotonic()
                try:
                    chunk = read_fn()
                except Exception as exc:
                    self._ingress_exc = exc
                    self._mailbox_event.set()
                    break
                now_rx = time.monotonic()
                if chunk:
                    last_rx_data_time = self._ingest_rtcm_bytes(ingress_buf, chunk, last_rx_data_time)
                else:
                    if ingress_buf and (now_rx - last_rx_data_time) >= 0.350:
                        last_rx_data_time = self._ingest_rtcm_bytes(ingress_buf, b"", last_rx_data_time)
                    if (now_rx - t_read0) < 0.002:
                        time.sleep(0.005)

        ingress_thread = threading.Thread(
            target=_ingress_worker,
            name="e220_rtcm_ingress",
            daemon=True,
        )
        ingress_thread.start()

        try:
            while running:
                if self._ingress_exc is not None:
                    raise self._ingress_exc

                if self._drive_pending_rate_switch():
                    continue

                self.poll_uplink()
                self._check_watchdog_revert()

                rem_ms = self._get_remaining_downlink_ms()
                if rem_ms <= 0.0:
                    # Consolidated Second 0 quiet window: zero downlink TX, fast uplink/watchdog polling
                    time.sleep(0.005)
                else:
                    batch = self._pop_next_tx_batch(rem_ms)
                    if batch:
                        self.send_rtcm_batch(batch)
                        self.sec_tx_count += len(batch)
                        self.poll_uplink()
                        self._check_watchdog_revert()
                    else:
                        self._mailbox_event.wait(timeout=0.015)
                        self._mailbox_event.clear()

                now = time.monotonic()
                if (now - last_report) >= 5.0:
                    dt = now - last_report
                    with self._mailbox_lock:
                        bytes_in = self.total_bytes_in
                        self.total_bytes_in = 0
                        rtcm_snapshot = dict(self.rtcm_stats)
                        self.rtcm_stats.clear()
                        sup_cnt = self.mailbox_superseded
                        coal_cnt = self.coalesced_frames
                        stale_cnt = self.mailbox_stale_dropped
                        max_age_ms = self.mailbox_max_age_ms
                        self.mailbox_max_age_ms = 0.0
                        empty_cnt = self.empty_msm_dropped
                        src_err = self.src_crc_errors

                    rate_in = bytes_in / dt
                    rate_out = self.total_bytes_out / dt
                    duty_pct = min(100.0, (self.total_wait_time_s / dt) * 100.0)
                    t_str = time.strftime("%H:%M:%S", time.gmtime())
                    frames_summary = ", ".join(f"{t}:{c}" for t, c in sorted(rtcm_snapshot.items())) or "None"
                    uplink_str = f"Uplink:[{self.uplink_packets_received} pkts, {self.rate_switch_count} sw]"
                    mbox_str = f"Mbox:[Sup:{sup_cnt} Coal:{coal_cnt} Stale:{stale_cnt} MaxAge:{max_age_ms:.0f}ms]"
                    print(
                        f"[{t_str}] [PACER HEARTBEAT] In:{bytes_in}B ({rate_in:.1f}B/s) | "
                        f"Out:{self.total_bytes_out}B ({rate_out:.1f}B/s) | "
                        f"Duty:{duty_pct:.1f}% | RTCM:[{frames_summary}] | "
                        f"{uplink_str} | {mbox_str} | EmptyMSM:{empty_cnt} | "
                        f"Frames:{self.total_packets_sent} (Subpkts:{self.total_subpackets_sent} "
                        f"Rst:{self.watchdog_resets} SrcErr:{src_err})",
                        flush=True,
                    )
                    self.total_bytes_out = 0
                    self.total_wait_time_s = 0.0
                    last_report = now
        finally:
            self._ingress_stop.set()
            self._mailbox_event.set()
            ingress_thread.join(timeout=1.0)


def _drain_initial_socket_backlog(sock: socket.socket) -> int:
    """Drain any stale buffered bytes queued on TCP server port prior to pacer startup."""
    drained = 0
    sock.setblocking(False)
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            drained += len(chunk)
            if drained >= 65536:
                break
    except BlockingIOError:
        pass
    except Exception:
        pass
    return drained


def run_tcp_client_mode(pacer: E220Pacer, host: str, port: int) -> None:
    """Connects to RTKBase RTCM TCP Service."""
    pacer.setup_hardware()
    try:
        while running:
            print(f"[TCP] Connecting to RTKBase at {host}:{port}...", flush=True)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.connect((host, port))
                drained = _drain_initial_socket_backlog(s)
                s.settimeout(0.015)
                print(
                    f"[TCP] Connected to RTKBase (drained {drained}B stale backlog). "
                    f"Streaming verified RTCM3 frames with Section 5.6.2 pacing...",
                    flush=True,
                )

                def read_socket() -> bytes:
                    try:
                        chunk = s.recv(4096)
                        if chunk == b"":
                            raise ConnectionResetError("RTKBase closed TCP socket")
                        return chunk
                    except socket.timeout:
                        return b""

                pacer.process_stream(read_socket)
            except _TERMIOS_ERRORS + (ConnectionRefusedError, ConnectionResetError, OSError) as e:
                if not running:
                    break
                print(f"[TCP] Connection interrupted: {e}. Retrying in 2.0s...", flush=True)
                time.sleep(2.0)
            finally:
                try:
                    s.close()
                except Exception:
                    pass
    finally:
        pacer.cleanup_hardware()


def run_tcp_server_mode(pacer: E220Pacer, listen_port: int) -> None:
    """Listens on a local port for RTKBase to stream to."""
    pacer.setup_hardware()
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", listen_port))
    server_sock.listen(1)
    server_sock.settimeout(1.0)
    print(f"[TCP] Listening on port {listen_port} for RTKBase connection...", flush=True)

    try:
        while running:
            try:
                client_sock, addr = server_sock.accept()
                client_sock.settimeout(0.02)
                print(f"[TCP] RTKBase connected from {addr}! Streaming verified RTCM3 frames...", flush=True)

                def read_client() -> bytes:
                    try:
                        chunk = client_sock.recv(4096)
                        if chunk == b"":
                            raise ConnectionResetError("Client disconnected")
                        return chunk
                    except socket.timeout:
                        return b""

                try:
                    pacer.process_stream(read_client)
                except ConnectionResetError:
                    print("[TCP] RTKBase client disconnected.", flush=True)
                finally:
                    client_sock.close()
            except socket.timeout:
                continue
    finally:
        server_sock.close()
        pacer.cleanup_hardware()


def run_pty_mode(pacer: E220Pacer, pty_symlink: str) -> None:
    """Creates a virtual serial port (PTY) for RTKBase."""
    if pty is None or not hasattr(os, "openpty"):
        print("[ERROR] PTY mode is only supported on Linux/Unix.", flush=True)
        sys.exit(1)

    master, slave = os.openpty()
    slave_name = os.ttyname(slave)

    if os.path.islink(pty_symlink) or os.path.exists(pty_symlink):
        os.unlink(pty_symlink)
    os.symlink(slave_name, pty_symlink)
    print(f"[PTY] Created virtual serial device: {pty_symlink} -> {slave_name}", flush=True)

    pacer.setup_hardware()
    try:
        def read_pty() -> bytes:
            r, _, _ = select.select([master], [], [], 0.02)
            if r:
                return os.read(master, 4096)
            return b""

        pacer.process_stream(read_pty)
    finally:
        if os.path.islink(pty_symlink):
            os.unlink(pty_symlink)
        os.close(master)
        os.close(slave)
        pacer.cleanup_hardware()


def run_pipe_mode(pacer: E220Pacer) -> None:
    """Reads directly from STDIN."""
    pacer.setup_hardware()
    print("[PIPE] Reading RTCM from standard input...", flush=True)
    try:
        def read_stdin() -> bytes:
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                return sys.stdin.buffer.read(4096)
            return b""

        pacer.process_stream(read_stdin)
    finally:
        pacer.cleanup_hardware()


def main() -> int:
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    parser = argparse.ArgumentParser(
        description=f"{__app_name__} v{__version__} - Hardware-Paced Ebyte E220 LoRa Driver for RTKBase",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--tcp-connect", metavar="HOST:PORT", help="Connect as client to RTKBase TCP Caster")
    group.add_argument("--tcp-listen", metavar="PORT", type=int, help="Listen as TCP server for RTKBase stream")
    group.add_argument("--pty", metavar="SYMLINK_PATH", help="Create virtual serial port for RTKBase")
    group.add_argument("--pipe", action="store_true", help="Read directly from STDIN")

    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT, help="E220 hardware serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="E220 hardware UART baud rate")
    parser.add_argument("--packet-size", type=int, default=DEFAULT_PACKET_SIZE, help="E220 EEPROM Subpacket Size")
    parser.add_argument("--guard-delay", type=float, default=DEFAULT_GUARD_DELAY_S, help="Inter-frame guard delay in seconds")

    parser.add_argument("--auto-config", action="store_true", help="Verify and program E220 EEPROM to target profile")
    parser.add_argument("--power", type=int, choices=[21, 24, 27, 30], default=21, help="Target TX power in dBm")
    parser.add_argument("--air-rate", type=float, choices=[2.4, 4.8, 9.6, 19.2, 38.4, 62.5], default=38.4, help="Target Air Data Rate in kbps")
    parser.add_argument("--channel", type=int, default=75, help="Target RF channel (75 = 925.125 MHz)")

    parser.add_argument("--pin-m0", type=int, default=DEFAULT_PIN_M0, help="BCM GPIO pin for E220 M0")
    parser.add_argument("--pin-m1", type=int, default=DEFAULT_PIN_M1, help="BCM GPIO pin for E220 M1")
    parser.add_argument("--pin-aux", type=int, default=DEFAULT_PIN_AUX, help="BCM GPIO pin for E220 AUX (Pacing line)")
    parser.add_argument("--pin-rst", type=int, default=DEFAULT_PIN_RST, help="BCM GPIO pin for E220 RESET")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose pacing logs")
    parser.add_argument("-q", "--quiet", action="store_true", help="Suppress [TX RTCM] logs")
    parser.add_argument("--log-dir", default="~/rover_logs", help="Directory for daily rolling CSV rover logs")
    parser.add_argument("--log-retention-days", type=int, default=30, help="Days to retain daily CSV logs before auto-pruning")
    parser.add_argument("--max-log-mb", type=int, default=50, help="Maximum total megabytes for rover log directory")
    parser.add_argument("--no-compress", action="store_true", help="Disable automatic gzip compression of yesterday's CSV")
    parser.add_argument("--no-csv", action="store_true", help="Disable CSV disk logging (RAM JSON only)")
    parser.add_argument("--no-interleave", action="store_true", help="Disable starvation-fair MSM interleaving")
    parser.add_argument(
        "--macro-cycle-s",
        type=int,
        choices=[5, 10],
        default=DEFAULT_MACRO_CYCLE_S,
        help="Base TX macro-cycle length in seconds (5s default matches Rover firmware superframe)",
    )

    args = parser.parse_args()
    freq_mhz = 850.125 + args.channel

    print(
        f"[INIT] {__app_name__} v{__version__} ({__date__}) starting | Port:{args.port} @ {args.baud} bps | "
        f"Air:{args.air_rate}k | CH:{args.channel} ({freq_mhz:.3f}MHz) | Pwr:{args.power}dBm | "
        f"Subpkt:{args.packet_size}B | Guard:{int(args.guard_delay * 1000)}ms | MacroCycle:{args.macro_cycle_s}s",
        flush=True,
    )

    pacer = E220Pacer(
        port=args.port,
        baud=args.baud,
        packet_size=args.packet_size,
        guard_delay_s=args.guard_delay,
        pin_m0=args.pin_m0,
        pin_m1=args.pin_m1,
        pin_aux=args.pin_aux,
        pin_rst=args.pin_rst,
        auto_config=args.auto_config,
        air_rate_kbps=args.air_rate,
        power_dbm=args.power,
        channel=args.channel,
        verbose=args.verbose,
        quiet=args.quiet,
        log_dir=args.log_dir,
        log_retention_days=args.log_retention_days,
        max_log_mb=args.max_log_mb,
        compress_logs=not args.no_compress,
        no_csv=args.no_csv,
        no_interleave=args.no_interleave,
        macro_cycle_s=args.macro_cycle_s,
    )

    if args.tcp_connect:
        parts = args.tcp_connect.split(":")
        host = parts[0]
        port = int(parts[1]) if len(parts) > 1 else 5016
        run_tcp_client_mode(pacer, host, port)
    elif args.tcp_listen:
        run_tcp_server_mode(pacer, args.tcp_listen)
    elif args.pty:
        run_pty_mode(pacer, args.pty)
    elif args.pipe:
        run_pipe_mode(pacer)

    return 0


if __name__ == "__main__":
    sys.exit(main())
