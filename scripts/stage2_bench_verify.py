#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: stage2_bench_verify.py
Description: Architecture 2B Split Desk Verification Suite for Base Pi (e220_pacer.py v3.7.0+)
             and Rover ESP32-S3 (RTK_Rover v0.8.12+). Supports three execution modes:
               1. --self-test: Pure offline deterministic simulation & real-thread harness testing
                  calibrated ToA, Ping-Pong Mailboxes, TTL expiration, <=200B coalescing,
                  starvation-fair round robin (avg/maxsat/extreme 400B @ 4.8k/9.6k/19.2k),
                  Second 0 pause gating, V3 telemetry uplink, and 4-ACK rate switch execution.
               2. --mode pi-feeder: Deterministic TCP RTCM3 stream generator (127.0.0.1:5017)
                  simulating RTKBase avg/maxsat/extreme constellations for live Pi Base testing.
               3. --mode rover-monitor: Live ESP32 USB-CDC serial monitor & scorecard for
                  [RTCM OK], Trunc, CRC, Orph, 5B RSSI-spill regression, Noise/Pkt RSSI, and switches.

Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import struct
import sys
import threading
import time
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import e220_pacer  # noqa: E402


__filename__ = os.path.basename(__file__)
__app_name__ = "stage2_bench_verify"
__version__ = "1.0.0"
__date__ = "2026-11-16"

# Deterministic RTCM3 synthetic epoch profiles: (msg_type, total_wire_bytes)
STRESS_PROFILES: dict[str, tuple[tuple[int, int], ...]] = {
    "avg": (
        (1006, 30),
        (1230, 42),
        (1074, 220),
        (1084, 142),
        (1094, 248),
        (1124, 210),
    ),
    "maxsat": (
        (1006, 30),
        (1230, 42),
        (1074, 340),
        (1084, 250),
        (1094, 360),
        (1124, 310),
    ),
    "extreme": (
        (1006, 30),
        (1230, 42),
        (1074, 400),
        (1084, 385),
        (1094, 400),
        (1124, 390),
    ),
}


def build_rtcm3_frame(msg_type: int, total_len: int, seq: int = 0) -> bytes:
    """Construct a valid CRC24Q-signed RTCM3 frame of exact wire length total_len."""
    if total_len < 12 or total_len > 1029:
        raise ValueError(f"total_len {total_len} out of valid RTCM3 range [12..1029]")
    payload_len = total_len - 6
    payload = bytearray(payload_len)
    payload[0] = (msg_type >> 4) & 0xFF
    payload[1] = ((msg_type & 0x0F) << 4) | (seq & 0x0F)
    for i in range(2, payload_len):
        payload[i] = (msg_type + seq + i * 17) & 0xFF
    body = bytes([0xD3, (payload_len >> 8) & 0x03, payload_len & 0xFF]) + bytes(payload)
    crc = e220_pacer.rtcm_crc24q(body)
    return body + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


def unpack_rtcm3_stream(blob: bytes) -> list[tuple[int, bytes]]:
    """Decode one or more concatenated RTCM3 frames and verify every frame CRC24Q == 0."""
    frames: list[tuple[int, bytes]] = []
    idx = 0
    while idx < len(blob):
        assert blob[idx] == 0xD3, f"Expected RTCM3 preamble 0xD3 at offset {idx}, got 0x{blob[idx]:02X}"
        assert (blob[idx + 1] & 0xFC) == 0x00, f"Invalid reserved bits at offset {idx}"
        plen = ((blob[idx + 1] & 0x03) << 8) | blob[idx + 2]
        flen = plen + 6
        assert (idx + flen) <= len(blob), f"Truncated RTCM3 frame at offset {idx}: need {flen}, have {len(blob) - idx}"
        frame = blob[idx : idx + flen]
        assert e220_pacer.rtcm_crc24q(frame) == 0, f"CRC24Q mismatch on unpacked frame len={flen}"
        mtype = (frame[3] << 4) | (frame[4] >> 4)
        frames.append((mtype, frame))
        idx += flen
    return frames


class VirtualClock:
    """Deterministic monotonic + wall-clock shim for fast offline Arch 2B simulation."""

    def __init__(self, start_wall_s: float = 1763316000.0):
        # 1763316000 is an exact multiple of 10 (sf_sec == 0 at t=0)
        self.wall_s = float(start_wall_s)
        self.mono_s = 1000.0

    def time_fn(self) -> float:
        return self.wall_s

    def monotonic_fn(self) -> float:
        return self.mono_s

    def advance(self, dt_s: float) -> None:
        self.wall_s += dt_s
        self.mono_s += dt_s


class MockE220Serial:
    """In-memory serial transport capturing Base TX payloads and injecting Rover uplinks."""

    def __init__(self, clock: Optional[VirtualClock] = None):
        self.is_open = True
        self.rx_buf = bytearray()
        self.tx_log: list[tuple[float, bytes]] = []
        self.clock = clock
        self._lock = threading.Lock()

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self.rx_buf)

    def feed_uplink(self, pkt: bytes) -> None:
        with self._lock:
            self.rx_buf.extend(pkt)

    def read(self, n: int = 1) -> bytes:
        with self._lock:
            out = bytes(self.rx_buf[:n])
            del self.rx_buf[:n]
            return out

    def write(self, data: bytes) -> int:
        now_s = self.clock.time_fn() if self.clock else time.time()
        with self._lock:
            self.tx_log.append((now_s, bytes(data)))
        return len(data)

    def flush(self) -> None:
        return None


def make_test_pacer(air_rate_kbps: float = 19.2, macro_cycle_s: int = 5, packet_size: int = 200) -> e220_pacer.E220Pacer:
    """Instantiate E220Pacer configured safely for offline simulation (no GPIO/CSV side effects)."""
    p = e220_pacer.E220Pacer(
        air_rate_kbps=air_rate_kbps,
        packet_size=packet_size,
        quiet=True,
        no_csv=True,
        macro_cycle_s=macro_cycle_s,
    )
    p.gpio_available = False
    return p


def build_v3_telemetry_packet(seq: int = 1, rssi_dbm: int = -72, noise_dbm: int = -108) -> bytes:
    """Build a valid CRC16-signed 48-byte Rover V3 uplink telemetry frame (0xAA 0x55 0x01)."""
    body = struct.pack(
        e220_pacer.FMT_TELEMETRY,
        e220_pacer.ROVER_SYNC,
        e220_pacer.MSG_ID_TELEMETRY,
        42,
        44.12345678,
        -73.98765432,
        125.5,
        14,
        4,
        4,
        1020,
        15,
        1,
        18,
        16,
        12345,
        88,
        7950,
        rssi_dbm,
        noise_dbm,
        seq & 0xFF,
        0,
        8,
        12,
        0,
    )
    crc = e220_pacer.crc16_ccitt(body[:-2])
    return body[:-2] + struct.pack("<H", crc)


def build_switch_req_packet(target_rate_kbps: float, chan: int = 75, req_id: int = 42) -> bytes:
    """Build a valid CRC16-signed 10-byte Rover air-rate switch request frame (0xAA 0x55 0x02)."""
    rate_to_code = {2.4: 0x02, 4.8: 0x03, 9.6: 0x04, 19.2: 0x05, 38.4: 0x06, 62.5: 0x07}
    code = rate_to_code[target_rate_kbps]
    body = struct.pack(
        e220_pacer.FMT_REQ_SWITCH,
        e220_pacer.ROVER_SYNC,
        e220_pacer.MSG_ID_REQ_SWITCH,
        4,
        code,
        chan,
        req_id,
        0,
    )
    crc = e220_pacer.crc16_ccitt(body[:-2])
    return body[:-2] + struct.pack("<H", crc)


def test_calibrated_toa_and_macro_windows() -> None:
    """Verify Stage-1-calibrated ToA formulas and consolidated macro-cycle budgets."""
    clk = VirtualClock(start_wall_s=1763316000.0)
    orig_time = e220_pacer.time.time
    try:
        e220_pacer.time.time = clk.time_fn
        p = make_test_pacer(air_rate_kbps=4.8, macro_cycle_s=5)
        assert abs(p.estimate_subpacket_air_s(200) * 1000.0 - 444.0) < 0.1
        assert abs(p.estimate_frame_air_s(400) * 1000.0 - 888.0) < 0.1
        assert p.estimate_frame_air_s(400) * 1000.0 >= 847.0
        assert p._get_sec0_downlink_cutoff_ms() == 450.0

        p.air_rate_kbps = 9.6
        assert abs(p.estimate_subpacket_air_s(200) * 1000.0 - 283.0) < 0.1
        assert p._get_sec0_downlink_cutoff_ms() == 620.0

        p.air_rate_kbps = 19.2
        assert abs(p.estimate_subpacket_air_s(200) * 1000.0 - 189.0) < 0.1
        assert p._get_sec0_downlink_cutoff_ms() == 680.0

        # At Second 0, t=0.100s (cutoff=680ms) -> 580ms left; at t=0.700s -> 0.0ms (paused)
        clk.wall_s = 1763316000.100
        assert abs(p._get_remaining_downlink_ms() - 580.0) < 0.5
        clk.wall_s = 1763316000.700
        assert p._get_remaining_downlink_ms() == 0.0

        # At Second 1..4: unbroken window all the way through next Second 0 cutoff
        clk.wall_s = 1763316001.200
        assert abs(p._get_remaining_downlink_ms() - 4480.0) < 0.5

        # 10s macro-cycle option
        p10 = make_test_pacer(air_rate_kbps=19.2, macro_cycle_s=10)
        clk.wall_s = 1763316001.200
        assert abs(p10._get_remaining_downlink_ms() - 9480.0) < 0.5
        clk.wall_s = 1763316005.200
        # Inside 10s cycle, utc_sec % 10 == 5 is downlink (NOT a pause!)
        assert abs(p10._get_remaining_downlink_ms() - 5480.0) < 0.5
    finally:
        e220_pacer.time.time = orig_time


def test_mailbox_supersede_ttl_and_filtering() -> None:
    """Verify ping-pong mailbox overwrite, CRC/empty MSM rejection, and TTL expiration."""
    clk = VirtualClock()
    orig_mono = e220_pacer.time.monotonic
    try:
        e220_pacer.time.monotonic = clk.monotonic_fn
        p = make_test_pacer(air_rate_kbps=19.2)
        buf = bytearray()
        last_rx = clk.monotonic_fn()

        # Deposit corrupt frame + empty 20B MSM + 3 consecutive epochs
        bad_frame = bytearray(build_rtcm3_frame(1074, 120, seq=9))
        bad_frame[-1] ^= 0xFF
        empty_1084 = build_rtcm3_frame(1084, 20, seq=0)
        last_rx = p._ingest_rtcm_bytes(buf, bytes(bad_frame) + empty_1084, last_rx)
        assert p.src_crc_errors == 1
        assert p.empty_msm_dropped == 1

        for ep in range(3):
            chunk = b"".join(build_rtcm3_frame(m, sz, seq=ep) for m, sz in STRESS_PROFILES["avg"])
            clk.advance(0.1)
            last_rx = p._ingest_rtcm_bytes(buf, chunk, last_rx)

        assert p.mailbox_superseded == 12, f"Expected 12 superseded slots, got {p.mailbox_superseded}"

        # Advance 2.7s -> dynamic MSMs (>2.5s) expire, while static 1006/1230 (<=15s) survive
        clk.advance(2.7)
        batch1 = p._pop_next_tx_batch(5000.0)
        batch_types = [m for m, _, _ in batch1]
        assert batch_types == [1006, 1230], f"Expected coalesced static [1006, 1230], got {batch_types}"
        assert p.mailbox_stale_dropped == 4, f"Expected 4 stale MSM drops, got {p.mailbox_stale_dropped}"
    finally:
        e220_pacer.time.monotonic = orig_mono


def test_smart_coalescing_limits() -> None:
    """Verify <= packet_size coalescing packs small frames without crossing into subpacket 2."""
    clk = VirtualClock()
    orig_mono = e220_pacer.time.monotonic
    try:
        e220_pacer.time.monotonic = clk.monotonic_fn
        p = make_test_pacer(air_rate_kbps=19.2, packet_size=200)
        buf = bytearray()
        last_rx = clk.monotonic_fn()

        # Case A: 1006(30B) + 1230(42B) + 1084(110B) = 182B <= 200B; 1074(240B) excluded cleanly
        blob_a = (
            build_rtcm3_frame(1006, 30, 1)
            + build_rtcm3_frame(1230, 42, 1)
            + build_rtcm3_frame(1084, 110, 1)
            + build_rtcm3_frame(1074, 240, 1)
        )
        p._ingest_rtcm_bytes(buf, blob_a, last_rx)
        b1 = p._pop_next_tx_batch(2000.0)
        assert [m for m, _, _ in b1] == [1006, 1230, 1084], f"Unexpected batch b1: {b1}"
        assert sum(len(f) for _, f, _ in b1) == 182
        b2 = p._pop_next_tx_batch(2000.0)
        assert [m for m, _, _ in b2] == [1074]
        assert p.coalesced_batches == 1 and p.coalesced_frames == 3

        # Case B: Custom 128B packet size obeys 128B cap (30 + 42 + 70 = 142 > 128 -> only 30+42=72)
        p128 = make_test_pacer(air_rate_kbps=19.2, packet_size=128)
        blob_b = (
            build_rtcm3_frame(1006, 30, 2)
            + build_rtcm3_frame(1230, 42, 2)
            + build_rtcm3_frame(1084, 70, 2)
        )
        p128._ingest_rtcm_bytes(bytearray(), blob_b, last_rx)
        b3 = p128._pop_next_tx_batch(2000.0)
        assert [m for m, _, _ in b3] == [1006, 1230]
        assert sum(len(f) for _, f, _ in b3) == 72
    finally:
        e220_pacer.time.monotonic = orig_mono


def _run_virtual_epoch_sim(air_rate: float, profile_name: str, epochs: int = 12) -> dict:
    """Simulate real multi-epoch Arch 2B TX scheduling under a virtual clock."""
    clk = VirtualClock(start_wall_s=1763316000.025)
    orig_time = e220_pacer.time.time
    orig_mono = e220_pacer.time.monotonic
    orig_sleep = e220_pacer.time.sleep
    mock_ser = MockE220Serial(clock=clk)

    def sim_sleep(dt_s: float) -> None:
        clk.advance(dt_s)

    try:
        e220_pacer.time.time = clk.time_fn
        e220_pacer.time.monotonic = clk.monotonic_fn
        e220_pacer.time.sleep = sim_sleep

        p = make_test_pacer(air_rate_kbps=air_rate, macro_cycle_s=5)
        p.ser = mock_ser  # type: ignore[assignment]
        ingest_buf = bytearray()
        rx_ts = clk.monotonic_fn()
        sec0_violations = 0
        delivered_counts: dict[int, int] = {}

        for ep in range(epochs):
            # Step cleanly to epoch start + 25 ms (matching 1 Hz RTKBase burst arrival)
            target_wall = 1763316000.025 + float(ep)
            if clk.wall_s < target_wall:
                clk.advance(target_wall - clk.wall_s)

            raw_epoch = b"".join(build_rtcm3_frame(m, sz, seq=ep) for m, sz in STRESS_PROFILES[profile_name])
            rx_ts = p._ingest_rtcm_bytes(ingest_buf, raw_epoch, rx_ts)

            # Drain scheduler ticks across this 1s epoch window until next epoch
            next_wall = 1763316000.0 + float(ep + 1)
            while clk.wall_s < next_wall - 0.002:
                rem_ms = p._get_remaining_downlink_ms()
                if rem_ms <= 0.0:
                    clk.advance(min(0.010, next_wall - clk.wall_s))
                    continue
                batch = p._pop_next_tx_batch(rem_ms)
                if not batch:
                    clk.advance(min(0.020, next_wall - clk.wall_s))
                    continue

                tx_start_sec = int(clk.wall_s) % 5
                tx_start_ms = (clk.wall_s - float(int(clk.wall_s))) * 1000.0
                p.send_rtcm_batch(batch)
                tx_end_sec = int(clk.wall_s) % 5
                tx_end_ms = (clk.wall_s - float(int(clk.wall_s))) * 1000.0

                cutoff = p._get_sec0_downlink_cutoff_ms()
                if (tx_start_sec == 0 and tx_start_ms >= cutoff) or (
                    tx_end_sec == 0 and tx_end_ms > (cutoff + 20.0)
                ):
                    sec0_violations += 1

                for m, _, _ in batch:
                    delivered_counts[m] = delivered_counts.get(m, 0) + 1

        # Verify every byte written on serial decodes to intact CRC24Q RTCM3 frames
        unpacked_total = 0
        for _, blob in mock_ser.tx_log:
            frames = unpack_rtcm3_stream(blob)
            unpacked_total += len(frames)

        assert sec0_violations == 0, f"{air_rate}k/{profile_name}: {sec0_violations} Second-0 cutoff violations"
        return {
            "delivered": delivered_counts,
            "unpacked_total": unpacked_total,
            "superseded": p.mailbox_superseded,
            "coalesced": p.coalesced_frames,
            "max_age_ms": p.mailbox_max_age_ms,
        }
    finally:
        e220_pacer.time.time = orig_time
        e220_pacer.time.monotonic = orig_mono
        e220_pacer.time.sleep = orig_sleep


def test_starvation_fairness_and_profiles() -> None:
    """Verify multi-rate Arch 2B fairness and zero Second-0 violations across 4.8k/9.6k/19.2k."""
    # 1. 4.8k + extreme 400B frames: static delivered, every MSM gets fair airtime (zero starvation)
    res_48 = _run_virtual_epoch_sim(4.8, "extreme", epochs=16)
    for m in (1006, 1230, 1074, 1084, 1094, 1124):
        cnt = res_48["delivered"].get(m, 0)
        assert cnt >= 2, f"4.8k/extreme starvation on Type {m}: {res_48['delivered']}"

    # 2. 9.6k + maxsat: high fairness across all 4 constellations + coalesced statics
    res_96 = _run_virtual_epoch_sim(9.6, "maxsat", epochs=12)
    for m in (1006, 1230, 1074, 1084, 1094, 1124):
        cnt = res_96["delivered"].get(m, 0)
        assert cnt >= 4, f"9.6k/maxsat starvation on Type {m}: {res_96['delivered']}"
    assert res_96["coalesced"] >= 2, f"Expected coalesced frames @ 9.6k, got {res_96}"

    # 3. 19.2k + avg: saturates physical Stage-1 channel cleanly (~936ms/s duty) with balanced round-robin
    res_192 = _run_virtual_epoch_sim(19.2, "avg", epochs=12)
    for m in (1006, 1230, 1074, 1084, 1094, 1124):
        cnt = res_192["delivered"].get(m, 0)
        assert cnt >= 8, f"19.2k/avg under-delivery on Type {m}: {res_192['delivered']}"
    sec_counts = [res_192["delivered"].get(m, 0) for m in (1084, 1094, 1124)]
    assert (max(sec_counts) - min(sec_counts)) <= 2, f"Unfair secondary balance @ 19.2k: {res_192['delivered']}"


def test_live_thread_uplink_and_switch_flow() -> None:
    """Exercise real process_stream() ingress thread, V3 uplink telemetry, and 4-ACK switch."""
    clk = VirtualClock(start_wall_s=1763316001.100)  # Second 1 (1100ms past Second 0)
    orig_time = e220_pacer.time.time
    orig_mono = e220_pacer.time.monotonic
    mock_ser = MockE220Serial(clock=clk)
    reconfig_calls: list[float] = []

    try:
        # Step wall clock deterministically by advancing on read/reconfigure without slowing wall-clock waits
        e220_pacer.time.time = clk.time_fn
        e220_pacer.time.monotonic = clk.monotonic_fn

        p = make_test_pacer(air_rate_kbps=19.2, macro_cycle_s=5)
        p.ser = mock_ser  # type: ignore[assignment]
        p.reconfigure_air_rate = lambda r: (reconfig_calls.append(r), setattr(p, "air_rate_kbps", r), True)[-1]  # type: ignore[method-assign]

        # Test real ingress thread + _pop_next_tx_batch handoff using process_stream + stop sentinel
        feed_chunks = [
            build_rtcm3_frame(1006, 30, 1) + build_rtcm3_frame(1230, 42, 1) + build_rtcm3_frame(1074, 115, 1),
            b"",
        ]
        feed_idx = 0

        def read_fn() -> bytes:
            nonlocal feed_idx
            if feed_idx < len(feed_chunks):
                chunk = feed_chunks[feed_idx]
                feed_idx += 1
                if feed_idx == len(feed_chunks):
                    time.sleep(0.35)
                return chunk
            raise EOFError("stream_end")

        try:
            p.process_stream(read_fn)
        except EOFError:
            pass

        # Real TX occurred via real background thread -> unpackable CRC24Q RTCM3 frames
        assert len(mock_ser.tx_log) >= 1, "Expected at least 1 TX from real ingress thread"
        all_tx_frames: list[int] = []
        for _, blob in mock_ser.tx_log:
            for mtype, _ in unpack_rtcm3_stream(blob):
                all_tx_frames.append(mtype)
        assert set(all_tx_frames) == {1006, 1230, 1074}, f"Unexpected frames from live thread: {all_tx_frames}"

        # Now verify V3 Telemetry Uplink + REQ_SWITCH 4-ACK train & Mode 3 switch at Second 2
        clk.wall_s = 1763316000.550  # Second 0 uplink window
        mock_ser.tx_log.clear()
        mock_ser.feed_uplink(build_v3_telemetry_packet(seq=7, rssi_dbm=-71, noise_dbm=-107))
        mock_ser.feed_uplink(build_switch_req_packet(target_rate_kbps=4.8, chan=75, req_id=99))
        p.poll_uplink()

        assert p.uplink_packets_received == 1
        assert p.last_rover_telemetry.get("rover_fw_version") == "v0.8.12"
        assert p.last_rover_telemetry.get("rover_rssi_dbm") == -71
        assert p.last_rover_telemetry.get("rover_noise_dbm") == -107
        assert p.pending_switch_exec_sec == 1763316002
        assert p.pending_switch_target_rate == 4.8

        # Pre-populate a pending mailbox frame to verify _clear_pending_mailboxes clears it on switch
        p._deposit_mailbox_frame(1094, build_rtcm3_frame(1094, 220, 5), clk.monotonic_fn())

        # Step manually through Second 1 ACK slots and Second 2 switch execution
        for ms_slot in (85, 265, 445, 625):
            clk.wall_s = 1763316001.0 + (ms_slot / 1000.0)
            p._transmit_pending_switch_ack_if_due(1763316001, float(ms_slot))
        assert p.pending_switch_acks_sent == 4
        assert len(mock_ser.tx_log) == 4

        clk.wall_s = 1763316002.005
        p._drive_pending_rate_switch()
        assert reconfig_calls == [4.8]
        assert p.active_air_rate == 4.8
        assert p.rate_switch_count == 1
        assert all(slot.frame is None for slot in p._mailboxes.values()), "Mailboxes not cleared after switch"
    finally:
        e220_pacer.time.time = orig_time
        e220_pacer.time.monotonic = orig_mono


def run_self_test() -> int:
    """Execute complete offline Arch 2B unit & integration suite."""
    tests = (
        ("Stage-1 calibrated ToA & macro-cycle windows", test_calibrated_toa_and_macro_windows),
        ("Ping-pong mailbox superseding, filtering & TTL", test_mailbox_supersede_ttl_and_filtering),
        ("Smart <=200B & custom packet-size coalescing", test_smart_coalescing_limits),
        ("Starvation fairness across 4.8k/9.6k/19.2k profiles", test_starvation_fairness_and_profiles),
        ("Live ingress thread, V3 uplink & 4-ACK rate switch", test_live_thread_uplink_and_switch_flow),
    )
    passed = 0
    for idx, (label, fn) in enumerate(tests, start=1):
        fn()
        passed += 1
        print(f"[SELF-TEST {idx}/{len(tests)}] PASS: {label}", flush=True)
    print(f"[PASS] {passed}/{len(tests)} Architecture 2B self-tests passed.", flush=True)
    return 0


def run_pi_feeder(
    host: str,
    port: int,
    profile: str,
    step_seconds: int,
    duration_seconds: int,
    status_json: str,
) -> int:
    """Serve deterministic 1 Hz RTCM3 stress epochs over TCP for live Pi Base verification."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(1.0)
    print(
        f"[FEEDER] Listening on {host}:{port} | Profile:{profile} | Step:{step_seconds}s | "
        f"Start e220_pacer.py with --tcp-connect {host}:{port}",
        flush=True,
    )

    cycle_order = ("avg", "maxsat", "extreme")
    t_start = time.monotonic()
    last_stat_print = 0.0
    total_epochs = 0
    total_bytes = 0
    client: Optional[socket.socket] = None

    try:
        while duration_seconds <= 0 or (time.monotonic() - t_start) < float(duration_seconds):
            if client is None:
                try:
                    client, addr = srv.accept()
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print(f"[FEEDER] e220_pacer connected from {addr}", flush=True)
                except socket.timeout:
                    continue

            # Align next 1 Hz burst cleanly to wall-clock second + 25 ms
            now_wall = time.time()
            next_sec = int(now_wall) + 1
            sleep_s = (float(next_sec) + 0.025) - now_wall
            if sleep_s > 0.0:
                time.sleep(sleep_s)

            elapsed = time.monotonic() - t_start
            active_profile = (
                cycle_order[int(elapsed // max(1, step_seconds)) % len(cycle_order)]
                if profile == "cycle"
                else profile
            )
            payload = b"".join(
                build_rtcm3_frame(m, sz, seq=total_epochs) for m, sz in STRESS_PROFILES[active_profile]
            )
            try:
                client.sendall(payload)
            except OSError as exc:
                print(f"[FEEDER] Client disconnected ({exc}); waiting for reconnect...", flush=True)
                client.close()
                client = None
                continue

            total_epochs += 1
            total_bytes += len(payload)

            if (time.monotonic() - last_stat_print) >= 5.0:
                last_stat_print = time.monotonic()
                rover_note = ""
                if status_json and os.path.isfile(status_json):
                    try:
                        with open(status_json, "r", encoding="utf-8") as f:
                            st = json.load(f)
                        rover_note = (
                            f" | Rover:{st.get('fix_name', '?')} RSSI:{st.get('rover_rssi_dbm', '?')}dBm "
                            f"Noise:{st.get('rover_noise_dbm', '?')}dBm FW:{st.get('rover_fw_version', '?')}"
                        )
                    except Exception:
                        pass
                print(
                    f"[FEEDER] Epochs:{total_epochs} | Profile:{active_profile} ({len(payload)}B/ep) | "
                    f"TotalSent:{total_bytes}B{rover_note}",
                    flush=True,
                )
    except KeyboardInterrupt:
        print("\n[FEEDER] Stopped by user.", flush=True)
    finally:
        if client is not None:
            client.close()
        srv.close()

    print(f"[FEEDER DONE] Sent {total_epochs} epochs ({total_bytes} B).", flush=True)
    return 0


def run_rover_monitor(serial_port: str, baud: int, duration_seconds: int, out_log: str) -> int:
    """Monitor live ESP32 USB-CDC serial output and score Arch 2B reception quality."""
    if serial is None:
        print("[ERROR] pyserial is required for --mode rover-monitor (install via: pip install pyserial)", flush=True)
        return 2

    log_fp = open(out_log, "w", encoding="utf-8") if out_log else None
    ser = serial.Serial(serial_port, baud, timeout=0.25)
    print(f"[ROVER MON] Connected to {serial_port} @ {baud} bps...", flush=True)

    re_rtcm = re.compile(r"\[RTCM OK\].*Type:(\d+).*Trunc:(\d+)\s+CRC:(\d+)\s+Orph:(\d+)")
    re_rssi = re.compile(r"\[E220 RSSI\].*Noise:(-?\d+)dBm.*LastPkt:(-?\d+|--)dBm")
    re_burst_5b = re.compile(r"\[E220 RX BURST\]\s+5 B")
    re_mode0_warn = re.compile(r"\[E220 MODE0 WARN\]")
    re_switch = re.compile(r"\[LORA SWITCH\]")

    t0 = time.monotonic()
    rtcm_ok = 0
    per_type: dict[int, int] = {}
    trunc_vals: list[int] = []
    crc_vals: list[int] = []
    orph_vals: list[int] = []
    spurious_5b_bursts = 0
    mode0_warns = 0
    switch_events = 0
    noise_samples: list[int] = []
    pkt_rssi_samples: list[int] = []

    try:
        while duration_seconds <= 0 or (time.monotonic() - t0) < float(duration_seconds):
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            print(line, flush=True)
            if log_fp is not None:
                log_fp.write(line + "\n")

            m = re_rtcm.search(line)
            if m:
                rtcm_ok += 1
                mtype = int(m.group(1))
                per_type[mtype] = per_type.get(mtype, 0) + 1
                trunc_vals.append(int(m.group(2)))
                crc_vals.append(int(m.group(3)))
                orph_vals.append(int(m.group(4)))
                continue

            m_r = re_rssi.search(line)
            if m_r:
                noise_samples.append(int(m_r.group(1)))
                if m_r.group(2) != "--":
                    pkt_rssi_samples.append(int(m_r.group(2)))
                continue

            if re_burst_5b.search(line):
                spurious_5b_bursts += 1
            elif re_mode0_warn.search(line):
                mode0_warns += 1
            elif re_switch.search(line):
                switch_events += 1
    except KeyboardInterrupt:
        print("\n[ROVER MON] Stopped by user.", flush=True)
    finally:
        ser.close()
        if log_fp is not None:
            log_fp.close()

    trunc_delta = (trunc_vals[-1] - trunc_vals[0]) if len(trunc_vals) >= 2 else (trunc_vals[0] if trunc_vals else 0)
    crc_delta = (crc_vals[-1] - crc_vals[0]) if len(crc_vals) >= 2 else (crc_vals[0] if crc_vals else 0)
    orph_delta = (orph_vals[-1] - orph_vals[0]) if len(orph_vals) >= 2 else (orph_vals[0] if orph_vals else 0)
    avg_noise = (sum(noise_samples) / len(noise_samples)) if noise_samples else 0.0
    avg_rssi = (sum(pkt_rssi_samples) / len(pkt_rssi_samples)) if pkt_rssi_samples else 0.0

    print("\n=== STAGE 2 ROVER SCORECARD ===", flush=True)
    print(f"Valid RTCM Frames : {rtcm_ok} | Types: {dict(sorted(per_type.items()))}", flush=True)
    print(f"Trunc / CRC / Orph: {trunc_delta} / {crc_delta} / {orph_delta}", flush=True)
    print(f"5B RSSI Spills    : {spurious_5b_bursts} (expected 0)", flush=True)
    print(f"Mode0 Warn / Sw   : {mode0_warns} / {switch_events}", flush=True)
    if noise_samples:
        print(f"Ambient Noise Avg : {avg_noise:.1f} dBm ({len(noise_samples)} samples)", flush=True)
    if pkt_rssi_samples:
        print(f"Last Pkt RSSI Avg : {avg_rssi:.1f} dBm ({len(pkt_rssi_samples)} samples)", flush=True)

    clean = (rtcm_ok > 0) and (trunc_delta == 0) and (crc_delta == 0) and (spurious_5b_bursts == 0)
    print(f"[RESULT] {'PASS' if clean else 'CHECK_METRICS'}", flush=True)
    return 0 if clean else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=f"{__app_name__} v{__version__} - Stage 2 Split Desk Verifier",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run complete offline Arch 2B simulation and unit test suite",
    )
    parser.add_argument(
        "--mode",
        choices=["pi-feeder", "rover-monitor"],
        help="Live split desk mode (Pi RTCM3 TCP feeder or Rover USB-CDC monitor)",
    )
    parser.add_argument("--tcp-host", default="127.0.0.1", help="TCP bind host for --mode pi-feeder")
    parser.add_argument("--tcp-port", type=int, default=5017, help="TCP bind port for --mode pi-feeder")
    parser.add_argument(
        "--profile",
        choices=["avg", "maxsat", "extreme", "cycle"],
        default="cycle",
        help="Synthetic RTCM3 profile for --mode pi-feeder",
    )
    parser.add_argument("--step-seconds", type=int, default=25, help="Seconds per profile step when --profile cycle")
    parser.add_argument("--duration", type=int, default=0, help="Duration in seconds (0 = run until Ctrl+C)")
    parser.add_argument(
        "--status-json",
        default="/run/rtk_rover_status.json",
        help="Optional status JSON path tailed by --mode pi-feeder",
    )
    parser.add_argument("--serial", help="ESP32 USB-CDC COM port or /dev/ttyACM0 for --mode rover-monitor")
    parser.add_argument("--baud", type=int, default=115200, help="ESP32 monitor baud rate")
    parser.add_argument("--out-log", default="", help="Optional output file path for --mode rover-monitor")
    return parser.parse_args()


def main() -> int:
    print(f"=== {__filename__} {__version__} ===", flush=True)
    args = parse_args()
    if args.self_test:
        return run_self_test()
    if args.mode == "pi-feeder":
        return run_pi_feeder(
            host=args.tcp_host,
            port=args.tcp_port,
            profile=args.profile,
            step_seconds=args.step_seconds,
            duration_seconds=args.duration,
            status_json=args.status_json,
        )
    if args.mode == "rover-monitor":
        if not args.serial:
            print("[ERROR] --serial is required when --mode rover-monitor is selected.", flush=True)
            return 2
        return run_rover_monitor(
            serial_port=args.serial,
            baud=args.baud,
            duration_seconds=args.duration,
            out_log=args.out_log,
        )
    print("[ERROR] Specify --self-test or --mode {pi-feeder,rover-monitor}.", flush=True)
    return 2


if __name__ == "__main__":
    sys.exit(main())
