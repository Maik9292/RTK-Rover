# Session Kickoff Prompt: Phase 9 — GPS-Disciplined TDMA & Data-Path Integrity Hardening

> **Usage:** Copy and paste the prompt below into a new Antigravity session to initiate the implementation of **Phase 9: GPS-Disciplined 5-Second Superframe TDMA & Data-Path Integrity Hardening**.

---

```markdown
# Session Goal: Phase 9 — GPS-Disciplined 5-Second Superframe TDMA & Data-Path Integrity Hardening

## 1. Context & Objective
In Field Test #035, while LoRa range exceeded 850m with 94% RTK Fix/Float, our Rover-to-Base uplink telemetry achieved only a **28.1% packet delivery ratio** (116/413 packets received). Log analysis proved this was caused by:
1. **Rover false quiet-window triggers:** In `src/e220_driver/e220_driver.cpp`, `in_quiet_window` fired 100ms after *any* intermediate RTCM frame (`last_burst_end_ms >= 100U`), causing 65.6% of rover transmissions to collide mid-burst with base transmissions.
2. **Base station empty MSM stub flooding:** In `scripts/e220_pacer.py`, the base transmitted 12,885 empty 28-byte MSM frames (Types 1074, 1084, 1094) containing 0 satellite observables, burning ~112ms each and deafening the base receiver for 65.9% of the test.
3. **Half-duplex SX1262 collision:** Uncoordinated transmission while the base receiver was deaf.

Additionally, our independent architecture review identified several critical data-path and robustness issues in the current prototype:
* **F-02 (Data Staleness):** If the GNSS receiver hangs or cable drops, frozen `RTK_FIX` data is logged, and un-disciplined clock drift would cause TDMA packet collisions.
* **F-04 (Lever-Arm Geometry):** When dual-antenna heading drops to COG (direction of travel) or HOLD, applying the $B/2$ lever arm offsets ground-tip coordinates by up to 55 cm in the wrong direction, which is transmitted in uplink telemetry to the base.
* **F-09 (Core 0 Mutex Deadlocks):** `s_com1_tx_mutex` takes with `portMAX_DELAY`, which can permanently freeze the radio task.
* **F-10 (Warm Boot Reconfiguration):** Every boot runs `UNLOGALL` and `SAVECONFIG`, dropping active carrier-phase lock on warm resets.
* **F-19 (`COM2` Naming Debt):** 47 residual references to `COM2` remain after Phase 8's Single-UART consolidation.
* **AXP2101 USB-to-Battery Transition Bug (`src/axp2101_bsp/axp2101_bsp.c`):** When unplugging USB, the display remains stuck on the USB icon showing `16.37V+` instead of switching to the battery percentage and voltage. This is caused by reading the open-circuit 14-bit `VBUS` ADC full-scale sentinel (`~16,370 mV`), latching `vbus_detected` via `raw_vbus_v >= 4.0f`, and misdecoding `STATUS2` bits `[6:5] == 0x02` (Discharging) as `is_charging = true`.

Our objective for this session is to implement **Phase 9: GPS-Disciplined 5-Second Superframe TDMA & Data-Path Integrity Hardening**, eliminating LoRa collisions, suppressing empty MSM stubs, disciplining transmission to GPS second boundaries without PPS wiring, enabling remote air-rate/channel switching, resolving findings F-02, F-04, F-09, F-10, and F-19, and fixing the AXP2101 USB-to-battery transition in `src/axp2101_bsp/axp2101_bsp.c`.

---

## 2. Technical Architecture & Timing Specifications

### 2.1 5-Second Superframe (`second % 5`)
Both Base Station (`e220_pacer.py` on Pi 4B) and Rover (`e220_driver.cpp` on ESP32-S3) synchronize to true UTC seconds:
* **Seconds 1, 2, 3, 4 (Downlink Only):**
  - **0 to 1,000 ms (100% Bandwidth):** Dedicated to Base Downlink RTCM3 frames.
  - Rover never transmits on Seconds 1, 2, 3, 4.
* **Second 0 (`second % 5 == 0` — Shared TDMA Slot):**
  - **0 to 750 ms (Base Downlink):** Base transmits high-priority RTCM (GPS MSM4 1074 + GLO 1084 or Base ARP 1006). If estimated ToA of the next frame exceeds 750 ms, the frame is held until Second 1.
  - **750 to 790 ms (Guard Window #1 — 40 ms):** Complete silence. Base TX finishes, clears FIFO, and SX1262 enters RX listening mode.
  - **790 to 950 ms (Rover Uplink — 160 ms):** Rover transmits 45-byte telemetry packet (ToA: 90.8 ms @ 9.6k, 53.4 ms @ 19.2k) or 10-byte command request (44.2 ms @ 9.6k).
  - **950 to 1,000 ms (Guard Window #2 — 50 ms):** Complete silence. Rover TX finishes and enters RX mode; Base prepares for Second 1 RTCM at 0 ms.

*(Note for 4.8 kbps: On 4.8k, adjust Second 0 to Base Downlink 0–700 ms, Guard #1 700–730 ms, Rover Uplink 730–940 ms, Guard #2 940–1000 ms).*

### 2.2 Empty 28-Byte MSM Stub Suppression
In `e220_pacer.py`:
- Inspect RTCM3 MSM frames (Types 1071–1127).
- If payload length is $\le 28$ bytes (or contains 0 satellite observables), silently discard the frame before it reaches the E220 UART FIFO.

### 2.3 Clock Disciplining & Staleness Gatekeeper (F-02)
- **Rover (ESP32-S3):** Parse UTC time field (`hhmmss.ss`) in UM982 `$GNRMC` / `$GNGGA` sentences in `um982_driver.cpp`. Record ESP32 `millis()` at top-of-second sentence arrival. Derive `current_sec % 5` and millisecond phase within the second.
- **Staleness Invalidation:** Track `last_gga_ms` and `last_uniheadinga_ms`. If no GGA for $>2000\text{ ms}$, set `fix_type = GNSS_FIX_NONE` and `gps_time_sync.time_synced = false`.
- **TDMA Uplink Suppression:** When `time_synced == false`, the rover **strictly halts all TDMA uplink transmissions** to prevent unsynchronized clock drift from colliding with base RTCM frames.
- **Base (Raspberry Pi 4B):** Read Linux system clock via `time.time()` (disciplined by Chrony / NTP to $<5\text{ ms}$ UTC offset). Calculate `int(time.time()) % 5` and `int(time.time() * 1000) % 1000`.

### 2.4 Lever-Arm Gating for Telemetry Coordinates (F-04)
In `nav_engine.cpp` / `RTK_Rover.ino`:
- Apply the $B/2$ lever arm **only** when `heading_source == HDG_SRC_DUAL_FIX` or `HDG_SRC_DUAL_FLT`.
- For `HDG_SRC_COG`, `HDG_SRC_HOLD`, `HDG_SRC_DISP`, or `HDG_SRC_NONE`: Leave tip coordinates equal to Master antenna coordinates (`tip_lat = master_lat`, `tip_lon = master_lon`), and flag as uncompensated in telemetry.

### 2.5 Core 0 Mutex Timeouts, Watchdog, Query-First Boot & PMIC Power Transition (F-09, F-10, F-19)
- **Non-blocking Mutex:** In `um982_driver.cpp`, replace `portMAX_DELAY` on `s_com1_tx_mutex` with `pdMS_TO_TICKS(50)`.
- **Task Watchdog:** Register `RoverCore0` with `esp_task_wdt_add(NULL)` and feed it on each loop iteration.
- **Query-First Init:** In `um982_configure_rover_defaults()`, query receiver state first; if `known_state_ok == true` and `mode == ROVER SURVEY`, skip `UNLOGALL` and `SAVECONFIG` on warm reboots.
- **Naming Debt:** Rename residual `com2_tx_bytes` $\to$ `rtcm_tx_bytes`, `last_com2_line` $\to$ `last_cmd_ack_line`, and `com2_test_passed` $\to$ `single_uart_audit_passed`.
- **AXP2101 USB-to-Battery Indicator Fix (`src/axp2101_bsp/axp2101_bsp.c` & `RTK_Rover.ino`):**
  - Gate `VBUS` ADC validity on `STATUS1` bit 5 (`raw_vbus_good`) and valid USB voltage bounds (`3800 <= vbus_mv <= 6500`), clamping 14-bit open-circuit readings (`>= 16000 mV` or `!raw_vbus_good`) to `0.0f` so `raw_vbus_v >= 4.0f` cannot latch `vbus_good = true` after unplugging USB.
  - Decode `STATUS2` (`0x01`) charging state strictly (`status->vbus_good` AND current direction `((st[1] >> 5) & 0x03) == 0x01` or active charger phase `(st[1] & 0x07)` in `1..3`), never treating `0x02` (Discharging) as charging.
  - Guard `VBAT` (`0x34`/`0x35`) against 14-bit open-circuit readings (`mv < 5000` and `STATUS1` bit 3 battery-present check).
  - Log explicit `[POWER]` transitions (`USB -> BATTERY` and `BATTERY -> USB`) in `RTK_Rover.ino` and include power state in `[TELEM]` and `[E220 LORA TX]` serial logs.

### 2.6 Synchronous Remote Command & Switch Handshake
When an air-rate or channel change is commanded:
1. **$T_0$ (Second 0 @ 790 ms):** Rover replaces 45-byte telemetry with 10-byte `RoverRateSwitchReq_t` (`0xAA 0x55 0x02`, target air code, target channel, ReqID).
2. **$T_0 + 1$ (Second 1 @ 0 ms):** Base receives request, queues 11-byte `RoverRateSwitchAck_t` (`0xAA 0x55 0x82`, ReqID, Status=0, Delay=1000ms), and transmits it immediately at $t = 0\text{ ms}$ before any RTCM frames. Rover verifies ACK during Second 1.
3. **$T_0 + 2$ (Second 2 @ 0 ms):** Both Base and Rover synchronously switch E220 configuration registers (M0/M1=HIGH $\to$ REG0 write $\to$ M0/M1=LOW) at the exact top of Second 2.

---

## 3. Files to Modify

1. **`scripts/e220_pacer.py`:**
   - Add empty MSM frame suppression (`is_empty_msm_frame()`).
   - Implement TDMA slot scheduling in `process_stream()` / `send_rtcm_frame()`: Second 0 $750\text{ ms}$ cutoff and active uplink listening window ($750\text{–}1000\text{ ms}$).
   - Implement top-of-Second-1 ACK transmission and top-of-Second-2 register reconfiguration.
2. **`src/um982_driver/um982_driver.h` & `src/um982_driver/um982_driver.cpp`:**
   - Extract integer UTC seconds and record `millis()` epoch from `$GNRMC` / `$GNGGA` Field 1.
   - Expose `gps_time_sync` state (`last_utc_sec`, `last_sync_millis`, `time_synced`).
   - Implement staleness timeouts (GGA $>2000\text{ ms} \implies \text{fix}=\text{NONE}$, `time_synced}=\text{false}$).
   - Replace `portMAX_DELAY` on `s_com1_tx_mutex` with `pdMS_TO_TICKS(50)`.
   - Add query-first check in `um982_configure_rover_defaults()`.
   - Rename `com2_*` naming debt symbols.
3. **`src/e220_driver/e220_driver.h` & `src/e220_driver/e220_driver.cpp`:**
   - Remove legacy quiet-window triggers (`last_burst_end_ms >= 100U`, `last_1094_rx_ms`).
   - Implement disciplined TDMA uplink trigger: fire only when `(utc_sec % 5 == 0)`, `ms_in_sec >= 790 && ms_in_sec <= 860`, and `time_synced == true`.
   - Update remote rate switch handler to execute the hardware switch at $T_0 + 2$.
4. **`src/nav_engine/nav_engine.cpp` & `RTK_Rover.ino`:**
   - Gate lever-arm application strictly to `HDG_SRC_DUAL_FIX` and `HDG_SRC_DUAL_FLT`.
   - Register `RoverCore0` with `esp_task_wdt`.
   - Log `[POWER]` `USB -> BATTERY` / `BATTERY -> USB` transitions and include power source/voltage in `[TELEM]` logs.
5. **`src/axp2101_bsp/axp2101_bsp.c`:**
   - Gate `VBUS` ADC on `STATUS1` bit 5 (`raw_vbus_good`) and valid USB bounds (`3800..6500 mV`), rejecting 14-bit open-circuit `16.37V` (`~16,370 mV`) readings.
   - Fix `STATUS2` (`0x01`) charging decode so `0x02` (Discharging) is never treated as `is_charging = true`.
   - Guard `VBAT` (`0x34`/`0x35`) on `STATUS1` bit 3 and `< 5000 mV`.

---

## 4. Verification & Testing Steps
1. **Syntax & Unit Checks:** Run Python syntax check on `e220_pacer.py`.
2. **Build Verification:** Build the Arduino sketch with `arduino-cli compile --fqbn esp32:esp32:esp32s3` to verify zero compile errors or memory regressions.
3. **Bench Verification:**
   - Deploy `e220_pacer.py` to the Pi 4B base station.
   - Power up the ESP32 rover connected to UM982 and monitor Serial output.
   - Verify that Rover transmits *strictly* at `Second 0, 790 ms` and that the Base logs `[UPLINK] Received Telemetry Frame (45B)` with zero collisions over a 15-minute test.
   - Verify that empty MSM frames (28 bytes) are suppressed in `pacer_test.log`.
   - Unplug UM982 serial line: confirm that rover immediately displays `NO GNSS DATA` and completely halts TDMA uplink bursts within 2 seconds.
   - Unplug USB power: confirm header transitions cleanly from `USB 4.78V+` to battery icon, percentage, and `VBAT` (e.g., `4.1xV`) without `16.37V` or lingering `+` within the debounce window, and verify `[POWER] USB -> BATTERY` in the serial log.
   - Test a remote air-rate switch request from Rover (e.g. 9.6k $\to$ 19.2k); confirm synchronized switch at $T_0 + 2$ and resume of RTK Fix.

---

## 5. Reference Documentation
- **Architecture Plan:** `docs/plan_phase9_gps_disciplined_tdma.md`
- **Field Test #035 Analysis:** `docs/test_results/field_test_035_analysis.md`
- **Sonnet Architecture Review:** `docs/project_review_sonnet.md`
- **E220 Datasheet & Reference:** `docs/e220-reference.md`
```
