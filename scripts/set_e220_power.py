#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: set_e220_power.py
Description: Fast runtime utility to switch E220 transmit power levels (21, 24, 27, 30 dBm)
             on an active RTKBase station. Safely stops e220-pacer.service, programs
             E220 EEPROM registers via GPIO M0/M1 with Mode 3 settling & readback verification,
             persists E220_POWER to /etc/default/e220-pacer (and syncs legacy hardcoded --power
             flags in /etc/systemd/system/e220-pacer.service if present) so service restarts
             never overwrite the selected power, and restarts e220-pacer.
Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"
__date__ = "2026-11-16"

# Hardware Pins (matches setup_e220.py and e220_pacer.py)
PIN_M0 = 18
PIN_M1 = 17
PIN_AUX = 27
PIN_RST = 22
PORT = "/dev/ttyAMA0"

ENV_FILE_PATH = "/etc/default/e220-pacer"
SERVICE_UNIT_PATH = "/etc/systemd/system/e220-pacer.service"

POWER_LEVELS = {30: 0b00, 27: 0b01, 24: 0b10, 21: 0b11}
INV_POWER = {v: k for k, v in POWER_LEVELS.items()}


def wait_aux(timeout: float = 3.0) -> bool:
    """Wait until E220 AUX pin reads HIGH plus a short post-idle guard."""
    if GPIO is None:
        return True
    start = time.monotonic()
    while (time.monotonic() - start) < timeout:
        if GPIO.input(PIN_AUX) == GPIO.HIGH:
            time.sleep(0.04)
            return True
        time.sleep(0.005)
    return False


def _persist_systemd_power(target_power_dbm: int) -> bool:
    """
    Persist E220_POWER into /etc/default/e220-pacer and sync any hardcoded
    --power <N> flag in /etc/systemd/system/e220-pacer.service so service
    restarts with --auto-config never revert TX power.
    """
    persisted = False

    # 1. Update or create /etc/default/e220-pacer
    try:
        lines: list[str] = []
        found = False
        if os.path.exists(ENV_FILE_PATH):
            with open(ENV_FILE_PATH, "r", encoding="utf-8") as f:
                for raw_line in f:
                    if re.match(r"^\s*E220_POWER\s*=", raw_line):
                        lines.append(f"E220_POWER={target_power_dbm}\n")
                        found = True
                    else:
                        lines.append(raw_line)
        if not found:
            lines.append(f"E220_POWER={target_power_dbm}\n")
        os.makedirs(os.path.dirname(ENV_FILE_PATH), exist_ok=True)
        with open(ENV_FILE_PATH, "w", encoding="utf-8", newline="\n") as f:
            f.writelines(lines)
        print(f"[+] Saved E220_POWER={target_power_dbm} to {ENV_FILE_PATH}.", flush=True)
        persisted = True
    except OSError as e:
        print(f"[WARN] Could not update {ENV_FILE_PATH} ({e}); run with sudo if permission denied.", flush=True)

    # 2. If installed unit still has a hardcoded numeric '--power 21/24/27/30', sync it too
    if os.path.exists(SERVICE_UNIT_PATH):
        try:
            with open(SERVICE_UNIT_PATH, "r", encoding="utf-8") as f:
                unit_txt = f.read()
            updated_txt, sub_count = re.subn(
                r"(--power(?:=|\s+))(?:21|24|27|30)\b",
                rf"\g<1>{target_power_dbm}",
                unit_txt,
            )
            if sub_count > 0 and updated_txt != unit_txt:
                with open(SERVICE_UNIT_PATH, "w", encoding="utf-8", newline="\n") as f:
                    f.write(updated_txt)
                subprocess.run(["systemctl", "daemon-reload"], check=False)
                print(
                    f"[+] Synced hardcoded --power flag in {SERVICE_UNIT_PATH} -> {target_power_dbm} dBm and reloaded systemd.",
                    flush=True,
                )
            persisted = True
        except OSError as e:
            print(f"[WARN] Could not inspect/update {SERVICE_UNIT_PATH} ({e}).", flush=True)

    return persisted


def _read_registers_mode3(ser: "serial.Serial") -> bytes | None:
    """Send 0xC1 0x00 0x08 in Mode 3 (9600 8N1) and return 8 register bytes on valid reply."""
    ser.reset_input_buffer()
    ser.write(bytes([0xC1, 0x00, 0x08]))
    ser.flush()
    time.sleep(0.085)
    res = ser.read(11)
    if len(res) == 11 and res[0] == 0xC1 and res[1] == 0x00 and res[2] == 0x08:
        return bytes(res[3:11])
    return None


def set_power(target_power_dbm: int, port: str = PORT) -> bool:
    """Safely switch Base E220 EEPROM TX power and sync systemd persistence."""
    if target_power_dbm not in POWER_LEVELS:
        print(f"[ERROR] Invalid power level {target_power_dbm} dBm. Valid choices: 21, 24, 27, 30", flush=True)
        return False

    if GPIO is None or serial is None:
        print("[ERROR] RPi.GPIO and pyserial are required.", flush=True)
        return False

    if hasattr(os, "geteuid") and os.geteuid() != 0:
        print("[WARN] Not running as root; systemd unit / GPIO / serial updates may require sudo.", flush=True)

    print("[*] Pausing e220-pacer service...", flush=True)
    subprocess.run(["systemctl", "stop", "e220-pacer"], check=False)
    time.sleep(0.20)

    success = False
    gpio_ready = False
    ser = None
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for p in (PIN_M0, PIN_M1, PIN_RST):
            GPIO.setup(p, GPIO.OUT)
        GPIO.setup(PIN_AUX, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.output(PIN_RST, GPIO.HIGH)
        gpio_ready = True

        # Enter Configuration Mode 3: assert M0=1, M1=1 BEFORE opening/switching 9600 8N1
        print("[*] Switching E220 to Configuration Mode 3 (M0=1, M1=1 @ 9600 8N1)...", flush=True)
        GPIO.output(PIN_M0, GPIO.HIGH)
        GPIO.output(PIN_M1, GPIO.HIGH)
        time.sleep(0.065)
        wait_aux(2.0)

        try:
            ser = serial.Serial(port, 9600, timeout=0.6, exclusive=True)
        except TypeError:
            ser = serial.Serial(port, 9600, timeout=0.6)

        cur_regs = _read_registers_mode3(ser)
        if cur_regs is None:
            time.sleep(0.035)
            cur_regs = _read_registers_mode3(ser)
        if cur_regs is None or len(cur_regs) != 8:
            print("[ERROR] Failed to read current E220 configuration in Mode 3.", flush=True)
            return False

        regs = bytearray(cur_regs)
        old_power_bits = regs[3] & 0b11
        old_power_dbm = INV_POWER.get(old_power_bits, -1)

        # Update power bits (REG1 bits [1:0]) and keep Mode 0 RSSI Register enabled (REG1 bit 5 = 0x20)
        new_power_bits = POWER_LEVELS[target_power_dbm]
        regs[3] = ((regs[3] & 0xFC) | 0x20) | new_power_bits
        # Enforce Transparent Mode (bit 6 = 0), LBT Disabled (bit 4 = 0), no appended RX RSSI byte (bit 7 = 0)
        regs[5] = regs[5] & ~(0x80 | 0x40 | 0x10)

        # Guard >= 20 ms between consecutive Mode 3 commands (C1 -> C0)
        time.sleep(0.020)
        old_lbl = f"{old_power_dbm}" if old_power_dbm > 0 else f"raw({old_power_bits})"
        print(f"[*] Updating EEPROM power: {old_lbl} dBm -> {target_power_dbm} dBm (Level {new_power_bits})...", flush=True)

        ack_ok = False
        for attempt in range(1, 4):
            ser.reset_input_buffer()
            ser.write(bytes([0xC0, 0x00, 0x08]) + bytes(regs))
            ser.flush()
            time.sleep(0.100)
            res_write = ser.read(11)
            if len(res_write) == 11 and res_write[0] in (0xC0, 0xC1) and res_write[3:11] == bytes(regs):
                ack_ok = True
                break
            if attempt < 3:
                time.sleep(0.035)

        if not ack_ok:
            print("[ERROR] Failed to verify Mode 3 EEPROM write acknowledgment.", flush=True)
            return False

        # Guard >= 20 ms before verification readback
        time.sleep(0.020)
        verify = _read_registers_mode3(ser)
        if verify != bytes(regs):
            print(
                f"[ERROR] EEPROM readback mismatch (actual={verify.hex().upper() if verify else 'none'} "
                f"expected={bytes(regs).hex().upper()}).",
                flush=True,
            )
            return False

        print(f"[+] Successfully programmed and verified E220 EEPROM at {target_power_dbm} dBm!", flush=True)
        _persist_systemd_power(target_power_dbm)
        success = True
    finally:
        if ser is not None and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
        if gpio_ready:
            # Restore Mode 0 (M0=0, M1=0) and settle >= 80 ms before restarting service
            print("[*] Returning E220 to Normal Mode 0 (M0=0, M1=0)...", flush=True)
            GPIO.output(PIN_M0, GPIO.LOW)
            GPIO.output(PIN_M1, GPIO.LOW)
            time.sleep(0.080)
            wait_aux(2.0)
            GPIO.cleanup()

        print("[*] Resuming e220-pacer service...", flush=True)
        subprocess.run(["systemctl", "start", "e220-pacer"], check=False)
        time.sleep(0.50)
        res_status = subprocess.run(["systemctl", "is-active", "e220-pacer"], capture_output=True, text=True)
        status = res_status.stdout.strip()
        print(f"[+] Service status: {status.upper()}", flush=True)

    if success:
        print(f"[+] All done! Base E220 EEPROM and systemd config are synced at {target_power_dbm} dBm.", flush=True)
    return success


def main() -> int:
    print(f"=== {__filename__} {__version__} ===", flush=True)
    parser = argparse.ArgumentParser(
        description=f"Switch Base E220 TX power cleanly and sync systemd persistence ({__version__}).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("power", type=int, choices=[21, 24, 27, 30], help="Target TX power in dBm (21=125mW, 24=250mW, 27=500mW, 30=1000mW)")
    parser.add_argument("--port", default=PORT, help="Base E220 UART device path")
    parser.add_argument("-v", "--version", action="version", version=f"{__filename__} {__version__} ({__date__})")
    args = parser.parse_args()

    success = set_power(args.power, port=args.port)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
