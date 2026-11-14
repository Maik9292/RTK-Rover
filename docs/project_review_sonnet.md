# RTK Rover - Independent Project Review

| | |
| :--- | :--- |
| **Reviewer** | Claude Sonnet 5.5 (via Antigravity) |
| **Date** | November 8, 2026 |
| **Revision reviewed** | `master` @ `2849127` (firmware `v0.8.0`, `e220_pacer.py v3.4.1`) |
| **Method** | Static review of source, build configuration, docs, git history; one successful `arduino-cli` compile. **No hardware was run.** |
| **Status of this document** | Opinion + evidence. Every finding cites a file/line so you can confirm or reject it. |

---

## 1. Executive Summary

**Verdict:** the project has a *strong core* (RTCM3 gatekeeper, memory discipline, I2C recovery, empirical method) but the **survey-critical data path trusts its inputs too much**. Today a point can be logged with a better quality label than it earned, with a lever-arm correction applied in the wrong direction, or from GNSS data that stopped arriving. None of these needs new hardware to fix, and all of them matter more than any new feature in the roadmap.

Separately, the **Smart-Pole pin map in the plan does not match the Waveshare 3.49-V2 schematic** (Section 5). The 3.49-V2 wiring diagram and pin CSV I generated earlier in this session inherited that error and **must not be used to build hardware**.

### Scorecard (subjective, 1 = poor, 5 = excellent)

| Area | Score | One-line reason |
| :--- | :---: | :--- |
| Architecture & modularity | 4 | Clear driver/engine/UI split, sensible Core 0 / Core 1 partition. |
| Survey-path correctness | 2 | Quality labels, lever arm and staleness handling can silently mislead (F-02..F-04). |
| Fault tolerance | 2.5 | Great I2C/UPS recovery; no watchdog, no stale-data detection, no reset diagnostics. |
| Testing | 1.5 | No unit tests/CI; two hardware-in-loop scripts. 34 field captures are unused as regression data. |
| Security | 2 | Unauthenticated file manager, fixed SoftAP password (F-11). Acceptable only if Wi-Fi stays rarely on. |
| Documentation | 3 | Large and detailed, but drifting and hand-synchronised across 12 files (F-16). |
| Process / VCS | 4 | 165 descriptive commits, changelog discipline, pre-push secret scanner. |
| Hardware plan (Smart Pole) | 2.5 | Good power-gating idea; pin map wrong; power margin untested (F-01, F-15). |

### Top 8 actions (in order)

1. **F-01** Re-derive the 3.49-V2 pin allocation from the schematic *before* ordering anything.
2. **F-02** Add data-age tracking: no GGA for 2 s means no fix; no UNIHEADINGA for 3 s means no heading.
3. **F-03** Gate and re-label survey averaging (fixed-only, per-epoch, sigma, circular mean).
4. **F-04** Apply the lever arm only for `DUAL_FIX`/`DUAL_FLT` headings.
5. **F-05** Project gyro rate onto gravity (current code integrates the wrong axis when clipped to a pole).
6. **F-09** Add a task watchdog, reset-reason logging and stack watermarks.
7. **F-08** Add the magnetometer axis/handedness self-test *before* the sensor arrives.
8. **F-17** Replay the existing `test/_0xx` captures through the parsers as a regression suite.

---

## 2. Scope, Method and Confidence

**Read in full:** [`um982_driver.cpp`](../src/um982_driver/um982_driver.cpp), [`nav_engine.cpp`](../src/nav_engine/nav_engine.cpp), [`RTK_Rover.ino`](../RTK_Rover.ino), [`rover_api.h`](../src/rover_api.h), [`user_config.h`](../user_config.h), [`mmc5983ma_driver.cpp`](../src/mag_bsp/mmc5983ma_driver.cpp), survey-averaging code in [`waypoint_manager.cpp`](../src/waypoint_manager/waypoint_manager.cpp), plus the Smart-Pole and Phase 8 plans.

**Skimmed / targeted grep only:** [`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp) (940 lines), [`web_server.cpp`](../src/web_server/web_server.cpp), [`settings_manager.cpp`](../src/settings_manager/settings_manager.cpp), [`imu_bsp.c`](../src/imu_bsp/imu_bsp.c).

**Not reviewed:** [`lvgl_port.c`](../src/lvgl_port/lvgl_port.c) (3,783 lines; only grepped), `cogo_solver.py`, `e220_pacer.py` internals, vendor driver code (`esp_codec_dev`, `codec_board`), the Android plan beyond architecture.

**Confidence:** high for items that cite code directly (F-02..F-07, F-09..F-14). Medium for hardware/RF items (F-08, F-15) which are engineering estimates and need a bench check. The J3/J6 pin table in Appendix A was extracted from the PDF text layer by position; the pin/label offsets were consistent for all 22 pins, but **please eyeball it against the schematic image**.

---

## 3. What Is Working Well

These are real strengths; keep them when refactoring.

* **RTCM3 gatekeeper.** Store-and-verify with CRC24Q before any byte reaches the receiver, rate-adaptive truncation timeouts, orphan-byte accounting. This came from forensic analysis of 16+ captures, which is the right way to do radio work.
* **Memory discipline.** The `DRAM0` / PSRAM analysis (`ESP_ERR_NO_MEM` hidden under `DebugLevel=none`) is excellent engineering and is encoded as rules in `GEMINI.md`.
* **Bus and power resilience.** GPIO bus-clear on warm reset, `i2c_bsp_lock()`, optional-address probing (no recovery storm when the compass is unplugged).
* **Phase 8 design.** TX mutex + atomic CRC-verified RTCM frames, 3-way line demultiplexer, quiet-first command sequence, known-state audit. The plan document is clear and its trade-offs are explicit.
* **Data/code separation.** Private deed data lives in a sibling repo, `check_secrets.py` plus a `pre-push` hook enforces it (it caught two path leaks during this session).
* **Process.** Semver discipline, 890-line changelog, descriptive commits, schema-driven COGO tooling with a measured 0.78 mm parity result.

---

## 4. Findings

Severity: **Critical** = can silently produce wrong survey output; **High** = wrong behaviour in a headline feature or major robustness gap; **Medium** = should fix soon; **Low** = hygiene.
Effort: **S** < 1 day, **M** 1-3 days, **L** > 3 days.

### Summary table

| ID | Sev | Area | Finding | Effort |
| :--- | :---: | :--- | :--- | :---: |
| F-01 | Critical | Hardware plan | 3.49-V2 pin map in plan conflicts with the schematic | M |
| F-02 | Critical | Firmware | No staleness handling for fix/heading data | S |
| F-03 | Critical | Firmware | Survey records mislabeled / ungated | M |
| F-04 | Critical | Firmware | Lever arm applied with non-baseline headings | M |
| F-05 | High | Firmware | Gyro integrates sensor Z axis (wrong when pole-mounted) | S |
| F-06 | High | Firmware | Heading frame semantics mixed; HOLD/GYRO never expire | S |
| F-07 | High | Firmware | Walk heading not gated by fix quality | S |
| F-08 | High | Compass | Handedness, tilt frame, hard-iron and current-field risks | M |
| F-09 | High | Firmware | No watchdog, reset reason, or stack monitoring | S |
| F-10 | Medium | Firmware | UM982 reconfigured + `SAVECONFIG` on every ESP32 boot | S-M |
| F-11 | Medium | Security | Unauthenticated web portal, fixed SoftAP password | S-M |
| F-12 | Medium | Firmware | Settings not tmp+rename; shared struct unsynchronised | S |
| F-13 | Medium | Firmware | Unicore `#` CRC32 never verified | S |
| F-14 | Medium | Firmware | `um982_query_receiver_state()` unsafe outside boot | S |
| F-15 | Medium | Hardware plan | Boost converter margin, no under-voltage cutoff | M |
| F-16 | Medium | Docs | Drift, sprawl, git-ignored handoff file | M |
| F-17 | Medium | Quality | No automated tests or CI | M |
| F-18 | Low | Repo | Vendor/third-party PDFs in a public GPL repo; 82 MB `.git` | S |
| F-19 | Low | Code | `COM2` naming debt after Phase 8; stale `platformio.ini` | S |
| F-20 | Low | Code | Non-ASCII output vs the project's own ASCII rule | S |
| F-21 | Low | Firmware | Auto-shutdown saves settings under LVGL lock, no log close | S |
| F-22 | Low | Code | `lvgl_port.c` is a 3.8k-line monolith | L |

---

### F-01 (Critical) Smart-Pole pin map does not match the 3.49-V2 schematic

**Evidence.** [`plan_smart_pole_and_android_app.md`](plan_smart_pole_and_android_app.md) section 3.2 assigns UM982 `COM1` to GPIO 15/16, `COM2` to GPIO 6, the compass to GPIO 17/18 (`J3` pins 17/18), E220 `AUX` to GPIO 48, compass `INT` to GPIO 47, and describes a 22-pin `J3`. The Waveshare schematic (copy in the sibling `Magnetic_Field_Scanner` project) and that project's own `gpio_pin_mapping.csv` show something different. See Appendix A for the full extraction.

| Plan says | Schematic / your own mapping CSV says |
| :--- | :--- |
| UM982 `COM1` on GPIO 15 / 16 | `IO15` = `I2S_SCLK`, `IO16` = `SYS_OUT` (internal, **not on any header**) |
| UM982 `COM2` on GPIO 6 | `IO6` = `I2S_DSOUT` (internal) |
| MMC5983MA on GPIO 17 / 18 | `IO17/18` = **touch panel I2C** (internal). The sensor bus is `IO47` (SDA) / `IO48` (SCL), `J3` pins 7 / 8, shared with QMI8658A, PCF85063A and ES8311 |
| E220 `AUX` on GPIO 48 | `IO48` is the sensor-bus **SCL** |
| Compass `INT` on GPIO 47 | `IO47` is the sensor-bus **SDA** |
| `J3` is a 22-pin header; VBAT = pin 1, 3V3 = pin 2 | `J3` and `J6` are **two 11-pin headers**. `J3`: VBAT = pin 11, 3V3 = pin 10, GND = pin 9 |
| E220 UART on `J3` pins 7 / 8 | `IO43/44` are `J3` pins **5 / 6** (silkscreen TXD/RXD) |

**Impact.** [`wiring_diagram_3_49_V2_smart_pole.png/.svg`](schematics/wiring_diagram_3_49_V2_smart_pole.png) and [`Waveshare-ESP32-S3-Touch-LCD-3.49-V2-GPIO_Pin_Assignment.csv`](Waveshare-ESP32-S3-Touch-LCD-3.49-V2-GPIO_Pin_Assignment.csv) were generated from the plan's table and are therefore **wrong**. The firmware pin defines for the Smart Pole do not exist yet, so nothing has been built on the bad table; this is the cheapest moment to fix it.

**Recommendation.** Treat the GPIOs that actually reach the headers as the budget:

* Free and usable: `IO1`, `IO2`, `IO3`, `IO5` (`J6`), `IO43/IO44` (`J3`, UART0), `IO0` (boot strap, avoid).
* `IO38..IO41` reach `J3` but are the microSD pins; usable only if the SD slot is sacrificed.
* Needs: two UARTs (4 pins), E220 `AUX` (1), `M0`+`M1` (2) = **7 pins**, **minus** the I2C compass (already on the shared `IO47/48` bus).
* The firmware only ever uses E220 Mode 0 (`M0=M1=0`) and Mode 3 (`M0=M1=1`), so **`M0` and `M1` can be wired together to one GPIO**. That brings the need to 6 and it fits exactly.

Candidate allocation (verify against the schematic before committing):

| Function | Pin | Header |
| :--- | :--- | :--- |
| E220 UART (keep on `Serial0`) | `IO43` TX / `IO44` RX | `J3` 5 / 6 |
| UM982 `COM1` (`Serial1`) | `IO1` TX / `IO2` RX | `J6` 2 / 3 |
| E220 `AUX` | `IO3` (strap pin: confirm `AUX` idle level cannot change boot mode) or `IO5` | `J6` 4 / 6 |
| E220 `M0`+`M1` tied | `IO5` or `IO3` | `J6` 6 / 4 |
| MMC5983MA I2C | `IO47` SDA / `IO48` SCL | `J3` 7 / 8 |
| Power gate / 3V3 / VBAT / GND | | `J3` 10 / 11 / 9 |

Zero spare GPIO remains, so decide now whether a future BLE-independent feature needs one. Then regenerate the diagram, the CSV and the plan table from one source of truth (see F-16).

### F-02 (Critical) No staleness handling for fix or heading data

**Evidence.** [`um982_driver.cpp`](../src/um982_driver/um982_driver.cpp#L13) keeps `gnss_state` as persistent state updated by sentence handlers; nothing records *when* a field was last updated. A search for `stale`, `last_gga`, `last_nmea`, `nmea_age` finds only LoRa-RSSI and base-label staleness in [`lvgl_port.c`](../src/lvgl_port/lvgl_port.c#L2629-L2636). If the UM982 stops talking (cable, power dip, hang) the last fix quality, position, `diff_age_s` and heading remain frozen and valid.

**Impact.** The UI keeps showing `RTK FIX`; the survey logger ([`waypoint_manager.cpp`](../src/waypoint_manager/waypoint_manager.cpp#L914-L1002)) and route logger keep recording a frozen position labelled `RTK_FIX`. For a pin-finding instrument this is the worst failure mode because nothing looks wrong.

**Recommendation.** Record `millis()` per sentence class (GGA, RMC, GST, UNIHEADINGA, BASEINFOA). Derive `age_ms` in `um982_get_latest_state()`. Policy: no GGA for 2 s sets `fix_type = NONE`; no UNIHEADINGA for 3 s sets `heading_valid = false`; either condition blocks survey logging and shows a `NO GNSS DATA` banner. Also compute the differential age locally (time since the last verified RTCM frame was forwarded) and cross-check it against the GGA field, which is empty when the receiver has no corrections.

### F-03 (Critical) Survey point records can be mislabeled and are not quality-gated

**Evidence** in [`waypoint_manager.cpp`](../src/waypoint_manager/waypoint_manager.cpp#L914-L1002) and [`um982_driver.cpp`](../src/um982_driver/um982_driver.cpp#L113-L119):

1. `survey_best_fix` becomes `RTK_FIXED` if **any** epoch in the window was fixed (L974-976). A 10 s average with one fixed epoch and the rest FLOAT/SINGLE is recorded as `RTK_FIX`.
2. No gate on fix type, `diff_age_s`, `h_acc_m`, or pole tilt. Tilt is averaged and logged but never enforced.
3. The function runs at the 30 Hz UI push rate, but GGA arrives at 2 Hz (`GPGGA COM1 0.5`). Each GNSS epoch is counted about 15 times, so the logged `epochs` value is inflated and no independent-sample statistic is possible. No standard deviation is recorded.
4. Heading is averaged arithmetically (`survey_sum_hdg`, L955-961, L991). Headings of 359 and 1 degrees average to 180.
5. `hdg_fix_str` is `HDG_FLT` whenever any heading was valid, including COG/HOLD-sourced headings (see F-04).
6. `handle_gga_sentence` overwrites `h_acc_m` with nominal constants (`0.18`, `0.65`, `1.5` m) for every non-fixed solution, and with `0.014` when no GST has arrived yet. GST and GGA both arrive every 0.5 s, so the stored value flickers between measured and invented numbers, and the CSV `h_acc_m` column can contain an invented value.

**Recommendation.** Accumulate only when the GGA sequence number changes. Use Welford's algorithm for mean and sigma. Add a setting `survey_min_fix` (default `RTK_FIXED`) and pause the average with an on-screen reason (`PAUSED: FLOAT`, `PAUSED: TILT 2.1deg`, `PAUSED: AGE 4s`) instead of silently accepting the epoch. Write `valid_epochs`, `total_epochs`, `fixed_fraction`, `min_fix`, `max_tilt`, `max_diff_age`, `sigma_n`, `sigma_e`, and `h_acc_src` (`GST` or `NOMINAL`). Use a circular mean for heading.

**Related accuracy note (not a bug, but a limit).** The tip is computed assuming a *plumb* pole. At `H = 1.80 m`, every degree of tilt moves the tip by `1.8 m x tan(1 deg) = 3.1 cm`. A hand-held bubble is rarely better than 1-2 degrees, so a tilt gate of 1.0 degree (3 cm) is the minimum honest policy for the claimed 2 cm bullseye; true tilt compensation needs the IMU plus the dual-antenna heading to resolve the tilt azimuth (straightforward on the Smart Pole where the IMU is rigid with the beam).

### F-04 (Critical) Lever-arm correction is applied with non-baseline headings

**Evidence.** [`RTK_Rover.ino`](../RTK_Rover.ino#L193-L201) calls `nav_compute_pole_tip_position(..., state.heading_valid, state.true_heading_deg, ...)` on Core 0 with no check of `heading_source`. [`nav_engine.cpp`](../src/nav_engine/nav_engine.cpp#L39-L47) treats that value as the Master-to-Slave azimuth and moves the tip by `B/2` along it. But [`handle_rmc_sentence`](../src/um982_driver/um982_driver.cpp#L162-L175) can set `heading_valid = true` with `heading_source = HDG_SRC_COG` (direction of *travel*) or leave `HDG_SRC_HOLD` valid with a stale value.

**Impact.** The correction vector is `B/2 = 27.75 cm` (more on the longer beam). Applied along the wrong azimuth the tip is off by up to `B = 55 cm` (wrong sign) and about 39 cm for a 90-degree mismatch. The survey CSV records the point and labels it `HDG_FLT`. Conversely, in deep woods where dual-antenna lock really is lost, **no lever-arm correction is applied at all** and the fused heading computed later on Core 1 is never fed back, so the rotating-orbit error the lever arm exists to remove returns exactly when conditions are worst.

**Recommendation.**

* Apply the lever arm only when `heading_source` is `HDG_SRC_DUAL_FIX` or `HDG_SRC_DUAL_FLT`.
* Otherwise leave the tip at the Master position, flag it `UNCOMPENSATED (+/-B/2)`, and block (or loudly flag) survey logging.
* Once the compass is calibrated, allow `HDG_SRC_COMP` to drive the lever arm with the error budget made explicit: `sigma_pos = (B/2) x sigma_heading`, i.e. 1.5 cm for 3 degrees at `B = 55.5 cm`.
* Run fusion *before* the tip computation (move fusion to Core 0, or the tip computation to Core 1) so one heading feeds one position.

### F-05 (High) Gyro yaw integration uses the sensor Z axis

**Evidence.** [`RTK_Rover.ino`](../RTK_Rover.ino#L424-L426) passes `cached_gyr[2]` as yaw rate; [`imu_bsp.c`](../src/imu_bsp/imu_bsp.c#L161-L188) documents that sensor Z is normal to the screen. That is correct only when the display is held flat. Clipped vertically to a vertical pole (`DISPLAY_POSE_VERTICAL`, the field use case) gravity lies along the screen's vertical axis, so rotating the pole in place produces almost no Z rate, while pitching it forward/back produces a spurious one.

**Recommendation.** `yaw_rate = (omega . g_hat)` where `g_hat` is the low-passed accelerometer unit vector. This works in every pose and needs no new hardware. Add a stationary gyro-bias estimate (zero-velocity update when `|omega|` is small and `|a| ~ 1 g`), which allows the 1.5 deg/s dead-band to drop; at present any real turn slower than 1.5 deg/s (a quarter-turn in a minute) is ignored.

### F-06 (High) Heading frame semantics are mixed, and HOLD/GYRO never expire

**Evidence.**

* `DUAL_FIX/FLT` and `COMP` (tared against dual in [`nav_engine.cpp`](../src/nav_engine/nav_engine.cpp#L485-L499)) are the **baseline azimuth**. `WALK` and `COG` are the **direction of travel**. They share one `true_heading_deg`, so on a fallback the displayed heading jumps by the angle between the beam and the walking direction unless the operator happens to walk along the baseline.
* [`rover_api.h`](../src/rover_api.h#L123-L126), the README and the handoff doc describe a "60-second hold window". [`nav_engine.cpp`](../src/nav_engine/nav_engine.cpp#L594-L604) has no expiry: `s_fused_valid` stays true forever and gyro drift accumulates unbounded while the UI shows a valid heading.

**Recommendation.** Define the convention in one paragraph of `project_overview.md`. Learn a `travel_to_baseline_offset` automatically whenever DUAL and COG are both valid above 0.5 m/s, and apply it to WALK/COG. Add a configurable expiry (default 60 s) after which `heading_valid = false` and the UI shows `HDG LOST`.

### F-07 (High) Walk heading is not gated by fix quality

**Evidence.** [`nav_engine.cpp`](../src/nav_engine/nav_engine.cpp#L560-L573) triggers on `dist_m >= 0.70` with no knowledge of fix type, `h_acc_m` or speed (the function signature does not receive them). The rover's own nominal accuracies are 1.5 m (SINGLE), 0.65 m (DGPS), 0.18 m (FLOAT). Position wander while *standing still* under canopy can exceed 0.7 m and set a random azimuth labelled `WALK`.

**Recommendation.** Pass `fix_type`, `h_acc_m`, and `speed_m_s` in. Threshold `= max(0.70, 4 x sigma_h)`; require speed above 0.3 m/s; refuse below FLOAT.

### F-08 (High) Magnetometer bring-up risks (sensor arrives next week)

All of these are untestable until the hardware is in hand, which is why they are listed now. v0.8.0 was verified only by a successful compile.

1. **Axis handedness.** [`mmc5983ma_poll()`](../src/mag_bsp/mmc5983ma_driver.cpp#L231-L242) uses the AN4248 tilt-compensation equations and `atan2(-b_yh, b_xh)`, which assume X forward, Y right, Z down. Chips like the MMC5983MA are normally mounted with Z pointing *up* out of the package, which gives X forward, Y *left*, Z up: a mirror image. A **constant** tare offset cannot fix a mirrored sense, so the heading would be right only at the heading where the tare was learned. This is the most probable first-power-up failure. Confirm the real axes against the SparkFun silkscreen/hookup guide already in `docs/datasheets/MMC5983MA`.
   *Fix:* make the axis map a 3x3 setting, and add a self-check that correlates the change in compass heading with the change in dual-antenna heading over at least 30 degrees of rotation: slope near +1 is good, near -1 auto-flips and logs, anything else sets `mag_cal_valid = false`.
2. **Tilt frame.** [`RTK_Rover.ino`](../RTK_Rover.ino#L336) feeds the *display* IMU's tared bubble angles into the compass tilt compensation. That is only valid if the display IMU axes equal the magnetometer axes and the tare is zero. On the 3.5B prototype the display is clipped at an arbitrary angle while the sensor is on the beam.
   *Fix:* use the gravity vector in the sensor frame: `east = m x g`, `north = g x east`, `heading = atan2(east . f, north . f)`, plus one fixed IMU-to-magnetometer remap. This also removes Euler singularities. On the Smart Pole the beam IMU is rigid with the sensor, so this works cleanly.
3. **Hard/soft iron.** SET/RESET removes the *sensor's* bridge offset, not the platform's. Steel screws, the 18650 can, speaker magnets and wire currents add a heading-dependent error of about `B_hi / B_earth` radians (5 uT is 5.7 degrees). A constant tare cannot remove it.
   *Fix:* you have ground truth for free whenever the dual antenna is fixed. Fit hard-iron offsets online (2-D circle/ellipse fit) once heading coverage exceeds about 270 degrees. The `Magnetic_Field_Scanner` project already has calibration scripts you can port.
4. **Tare gain is too fast.** [`nav_engine.cpp`](../src/nav_engine/nav_engine.cpp#L498-L499) applies `0.05 x err` on every call (about 30 Hz), a time constant near 0.7 s. While dual is fixed the tare simply tracks `dual - compass`, including all heading-dependent error, so the offset frozen at the moment of signal loss is the error *at that instant's heading*. Use a time constant of 30-60 s, or per-heading bins.
5. **Current-induced field.** A wire carrying `I` at distance `r` produces `B = mu0 I / (2 pi r)`: 1 A at 15 cm is 1.3 uT, about 1.5 degrees of heading error, and the E220 transmit pulses (0.65 A) make it ripple. The 3.5 uT anomaly threshold would not notice it. Use twisted supply pairs, keep the power wiring at least 15 cm from the sensor, and log `|B|` against load during bring-up.
6. **Stronger anomaly test.** `|B|` alone misses distortions that preserve magnitude. Also compare the measured inclination (dip angle) with the expected value for the site.

Bring-up checklist is in Section 6.

### F-09 (High) No watchdog, reset diagnostics or stack monitoring

**Evidence.** Search for `esp_task_wdt`, `esp_reset_reason`, `HighWaterMark` returns nothing. [`RTK_Rover.ino`](../RTK_Rover.ino#L164-L290) runs RTCM forwarding, GNSS parsing, navigation, a 25-argument `Serial.printf`, and queue pushes in one 8 KB task. `portMAX_DELAY` mutex takes at [`um982_driver.cpp`](../src/um982_driver/um982_driver.cpp#L616) and [`#L825`](../src/um982_driver/um982_driver.cpp#L825) can block forever.

**Impact.** A stall in `RoverCore0` stops RTCM forwarding silently; a brown-out or panic reboot is indistinguishable from a normal power-up.

**Recommendation.** Register `RoverCore0` with `esp_task_wdt` (2-5 s) and feed it each iteration; log `esp_reset_reason()` plus a persisted boot counter at start-up; add `uxTaskGetStackHighWaterMark()` for all three tasks to the 5 s `[TELEM]` line; replace `portMAX_DELAY` with a 20-50 ms timeout plus an error counter.

### F-10 (Medium) UM982 is reconfigured and `SAVECONFIG`'d on every ESP32 boot

**Evidence.** [`um982_configure_rover_defaults()`](../src/um982_driver/um982_driver.cpp#L685-L751) always sends `UNLOGALL`, `MODE ROVER SURVEY`, `CONFIG HEADING ...`, the seven log subscriptions and `SAVECONFIG`.

**Impact.** A warm ESP32 reset (watchdog, brown-out, UPS swap) re-initialises a receiver that may already hold a converged RTK and heading solution. Whether this drops the solution should be confirmed on the bench (I could not test it), but it is a plausible cause of long re-acquisition after any reset, and it writes the receiver's non-volatile memory each boot.

**Recommendation.** Use the Phase 8 audit that already exists: query first, compare to the expected state (`known_state_ok` plus heading length), reconfigure and `SAVECONFIG` only on mismatch. Keep a separate "restore known state" action.

### F-11 (Medium) Web portal has no authentication and a fixed SoftAP password

**Evidence.** [`web_server.cpp`](../src/web_server/web_server.cpp#L470-L475) registers `/download`, `/view`, `/delete`, `/upload` with no authentication; `/delete` is not restricted to POST. The fallback hotspot is `RTK-Rover` / `12345678`, hard-coded at L655 and printed at L690 (and public in the repo).

**Impact.** Anyone within Wi-Fi range while the portal is on can download survey data, delete files, or upload a replacement `settings.json` (including Wi-Fi credentials).

**Recommendation.** Generate a per-device AP password from the MAC plus a boot nonce and show it on screen; require a session token and POST for state-changing routes; auto-stop the portal after N minutes idle.

### F-12 (Medium) Settings persistence is not tmp+rename; shared struct is unsynchronised

**Evidence.** [`save_settings()`](../src/settings_manager/settings_manager.cpp#L426) opens `/settings.json` with `"w"` (truncate) and writes in place, with a `.bak` fallback. The Smart-Pole plan describes an atomic `.tmp` then `rename()` scheme that the code does not implement. `current_settings` is written by the UI task, Core 1 fusion (`mag_*` tare) and Wi-Fi code, and serialised by `loop()` with only `fs_mux` (which guards the filesystem, not the struct).

**Recommendation.** Snapshot the struct under a short mutex before serialising; write `settings.json.tmp`, flush, `rename`; add a version and CRC field so a torn file is detected.

### F-13 (Medium) Unicore `#` sentences are not integrity-checked

**Evidence.** [`verify_nmea_xor_checksum()`](../src/um982_driver/um982_driver.cpp#L388-L399) only covers `$` lines and accepts a line with no checksum. `#UNIHEADINGA` and `#BASEINFOA` carry a CRC32 that is never verified. The baseline-length sanity gate mitigates but does not replace a CRC.

**Recommendation.** Implement the Unicore CRC32 from the receiver manual for `#` lines and reject GGA/RMC/GST lacking a checksum (keep command responses lenient).

### F-14 (Medium) `um982_query_receiver_state()` is only safe at boot

**Evidence.** The function spins `um982_poll_com1()` itself ([L630-L683](../src/um982_driver/um982_driver.cpp#L630-L683)); `RoverCore0` does the same, and both share `com1_line_buf` and `port_stats`. Called from any other task at runtime it would corrupt lines.

**Recommendation.** Before exposing it to the UI, web or BLE, turn it into a request flag consumed by Core 0 (small state machine: send, wait for ACK, next).

### F-15 (Medium) Smart-Pole power stage margin

The high-side switch idea (`NDP6020P` + NPN driven by the board's 3V3 rail) is sound, and the schematic you showed is correct (source on `VBAT`). Items to bench-check:

* **Boost headroom.** At the plan's 1.85 A peak battery current the cell sags, and a typical `MT3608` module is only comfortable near 1 A average. At 3.3 V input and a 650 mA 1 W transmit burst, input current approaches 2 A and the module's switch limit and heat become the constraint. Test with the real load at 3.3 V input; consider a synchronous boost of at least 3 A with a true UVLO.
* **Under-voltage cutoff.** Nothing in the design cuts power before an unprotected 18650 is over-discharged; the plan only flushes files at 3.40 V. Use protected cells and have the firmware drop the board's power latch at about 3.30 V; verify the Waveshare board's own cutoff.
* **Noise.** Add a ferrite bead and 10 uF at the GNSS 5 V entry; the module has onboard regulation but boost ripple is the usual cause of unexplained C/N0 loss.
* **I2C pull-ups.** The Qwiic board brings its own pull-ups; with the onboard sensors the parallel value must stay above roughly 1 kOhm at 3.3 V / 400 kHz.
* `2N2222` vs `2N3904`: electrically equivalent here (0.4 mA load); check the pinout of whichever TO-92 part you use.

### F-16 (Medium) Documentation drift and sprawl

* [`project_overview.md`](project_overview.md#L33-L39) states 38.4 kbps / `REG0 = 0xE6` as the default; code and `settings.json` default to 19.2 kbps / `0xE5` ([`user_config.h`](../user_config.h#L107)). [`hyper-context-anchor.md`](hyper-context-anchor.md) states both within six lines.
* `docs/session_handoff.md` is **git-ignored** (`.gitignore` L19) yet is cited as required reading; edits to it are never committed (the v0.8.0 update made earlier this session was not tracked). `GEMINI.md` is ignored too.
* `GEMINI.md` requires hand-synchronising 12 documents. Numeric facts (baud, REG values, byte counts, pin numbers) are repeated across them, so drift is guaranteed.
* Two claims written in this session were wrong and have been corrected: `hyper-context-anchor.md` said v0.8.0 was "verified and operational" (it is compile-verified only) and `CHANGELOG.md` listed a `[TARE MAG]` button that does not exist.

**Recommendation.** One source of truth: generate a spec table (pins, radio profile, versions) from `user_config.h` and `settings.json` with a small script, and have the other docs link to it. Cut the mandatory sync list to README, CHANGELOG and project_overview; mark the rest as dated snapshots. Un-ignore the handoff file (or move it outside the repo deliberately).

### F-17 (Medium) No automated tests or CI

**Evidence.** No workflow files; the only tests are hardware-in-loop scripts (`test_e220_link.py`, `test_rover_telemetry.py`). `test/` holds 34 real captures (serial logs, NMEA, RTCM) but is git-ignored and unused as regression data.

**Recommendation (smallest useful set).**

1. CI job: `arduino-cli compile` for the target FQBN plus `check_secrets.py`.
2. Host unit tests for pure logic: geodesy and stakeout in `nav_engine`, NMEA/Unicore parsers, RTCM CRC24Q and framer, CSV waypoint parsing.
3. Replay the captured NMEA/RTCM logs through the parsers and assert invariants (no heading flip without cause, no fix upgrade without `GGA` quality change). This would have caught F-03 and F-04.

### F-18 (Low) Repository hygiene and licensing

`.git` is 82 MB. Tracked files include vendor manuals and schematics (Espressif TRM 14.9 MB, a 16.9 MB STEP model, Unicore manuals) and a printed copy of a third-party blog article (`docs/articles/...Wolles Elektronikkiste.pdf`) in a public GPL-3.0 repository. Redistribution rights for these are unclear. Keep links plus a local-only `datasheets/` folder, or use Git LFS / a private datasheet repo. This matters more for the planned `Field_Instrument_Core` library, which would multiply the copies.

### F-19 (Low) `COM2` naming debt and stale PlatformIO config

47 `COM2` occurrences remain. `t4_com2_lbl` now holds the receiver-state card, `com2_tx_bytes` carries RTCM bytes, [`rover_api.h`](../src/rover_api.h#L57-L61) comments still say "forwarded to UM982 COM2", and `send_um982_com2_test_pulse()` runs the whole audit. These were deliberate compatibility shortcuts in Phase 8 but will mislead the next debugging session; do one rename pass (`rtcm_tx_bytes`, `cmd_ack_line`, `t4_um982_state_lbl`). [`platformio.ini`](../platformio.ini#L3-L6) pins arduino-esp32 `3.0.7` while the docs and Wi-Fi fixes rely on `3.3.x`; the project actually builds with `arduino-cli`. Either test and maintain the PlatformIO env or delete it and document the exact FQBN.

### F-20 (Low) Non-ASCII output vs the project's own rule

`GEMINI.md` and `hyper-context-anchor.md` say UI strings must be ASCII. [`nav_format_quadrant_bearing`](../src/nav_engine/nav_engine.cpp#L216) emits a degree sign and several serial strings contain em-dashes (they compile fine; any odd rendering seen in a terminal is a code-page artifact). Either relax the rule for the degree sign or change the strings.

### F-21 (Low) Auto-shutdown path

[`RTK_Rover.ino`](../RTK_Rover.ino#L379-L385) calls `save_settings()` and a 350 ms delay while holding the LVGL lock on the `RoverUI` task, contrary to the project's own "no flash/SD writes on UI tasks" rule, and powers off without flushing or closing the NMEA/RTCM/serial loggers. Set a `shutdown_requested` flag handled by `loop()`, which flushes everything and then calls the PMIC.

### F-22 (Low) `lvgl_port.c` is a 3,783-line single file

Split by tile (`ui_tile_nav.c`, `ui_tile_survey.c`, `ui_tile_diag.c`, ...). This is the largest maintainability cost in the repo and the main obstacle to sharing the UI with the Smart Pole.

---

## 5. Smart Pole / Android Plan Review

* **Sequencing is the main risk.** Three new work streams (3.49-V2 hardware, the `Field_Instrument_Core` repo split, the Android app) are planned while the handheld core still has F-02..F-09 open and v0.8.0 has not touched real hardware. Recommended order: fix the trust issues, field-validate v0.8.0, **then** split the repo (when a second consumer actually needs it), **then** the Smart Pole firmware.
* **Pin map:** see F-01 (blocking). Add a "schematic verified" checkbox with date to the plan before any hardware order.
* **Shared library:** a library with two consumers is right, but create it by *extracting from a working second sketch*, not up front. Use a versioned Arduino library (or git submodule), and keep datasheets out of it (F-18).
* **BLE + Wi-Fi + LVGL memory:** the `DRAM0` guardrails are well understood; keep the `[MEM] Post-Setup Heap` line as a CI-visible boot assertion (fail if largest internal block drops below your threshold).
* **Phone as master record:** a good decision. Make the BLE protocol carry the same quality metadata as the CSV (fix, sigma, tilt, age, heading source) or the app will repeat F-03.
* **UM982 `COM1` for RTCM input:** the Unicore receiver is documented to accept ASCII commands and RTCM on any port; confirm on the bench that RTCM injected on `COM1` yields `RTK_FIX` and a sane differential age while NMEA streams out.

---

## 6. Phase 8 (v0.8.0) Specific Review and Bench Checklist

**Honest status:** compiled cleanly (53% flash, 20% RAM) and passed `check_secrets.py`. Nothing in Phase 8 has run on hardware.

**Phase 8 issues already covered above:** F-08 (compass assumptions written without the sensor), F-10 (boot reconfiguration retained from earlier code), F-14 (audit not re-entrant), F-19 (naming debt).

### Single-UART `COM1` bench test (no new hardware)

1. Boot with the receiver already running; confirm `[UM982 AUDIT SUMMARY] State:KNOWN STATE OK` and `Ver:` populated.
2. With the base transmitting, confirm `[RTCM OK -> COM1]` lines, then `Fix:4`, differential age below 2 s. Compare RTK-fix time against v0.7.7 on the same bench.
3. Confirm `.nmea` log files contain no `$command` / `#CONFIG` / `#LOGLIST` lines.
4. Soak 30 min: `cmd_responses_ok` constant after boot, `com1_rx_bytes` rising, zero `RTCM DROP` increments from the TX path.
5. Warm-reset the ESP32 only; note how long the fix takes to return (this measures F-10).
6. Unplug UM982 TX: confirm the UI goes stale (this will fail until F-02 is fixed, which is the point of the test).

### MMC5983MA bring-up checklist (next week)

1. Boot: expect `[MAG] ... ONLINE`, product ID `0x30`, offsets near 131072 LSB, temperature plausible.
2. Static `|B|` at the bench and outdoors: 40-60 uT outdoors in Maine; record the value as the reference.
3. Rotate slowly through 360 degrees flat: plot `mag_uT[0..2]`; the XY locus should be a centred circle. Any offset from centre is your hard-iron term.
4. **Handedness test (F-08.1):** with `HDG FIX`, rotate 90 degrees clockwise and compare the sign of the change in compass vs dual heading before trusting `mag_cal_valid`.
5. Tilt test: tilt the beam 10 and 20 degrees in both axes; heading must change by less than 2 degrees.
6. Load test: key the E220 at 1 W and run the UM982; log `|B|` and heading ripple versus transmit state (F-08.5).
7. Only after 1-6, let the auto-tare run and compare `COMP` against `DUAL_FIX` over a 5-minute walk.

---

## 7. Recommended Roadmap

| Release | Theme | Contents |
| :--- | :--- | :--- |
| **v0.8.1** | *Trust & integrity* | F-02 data age, F-03 survey gating and re-labelling, F-04 lever-arm gating, F-05 gyro-gravity projection, F-06 expiry, F-07 walk gating, F-09 watchdog and reset reason. Add host replay tests (F-17) for the above. |
| **v0.8.2** | *Compass bring-up* | Bench checklist results, F-08 fixes (axis map, handedness self-test, gravity-based tilt, slower tare, hard-iron fit), F-10 query-first configuration, F-13 CRC32. |
| **v0.9.0** | *Hardening* | F-11 portal auth, F-12 atomic settings, F-14 audit state machine, F-19/F-20 clean-up, CI. |
| **Smart Pole HW** | *After F-01* | Re-derive pins, regenerate diagram/CSV from one source, bench-test power stage (F-15), then firmware port. |
| **Library / Android** | *After a second consumer compiles* | `Field_Instrument_Core` extraction, BLE protocol with quality metadata. |

---

## 8. Open Questions for the Owner

1. Is the beam intended to be perpendicular to, or aligned with, the walking direction? This decides the heading convention in F-06.
2. What is the minimum acceptable fix and maximum tilt for a logged point? (Suggested defaults: `RTK_FIXED`, 1.0 degree.)
3. Is Wi-Fi expected to be on during field work, or only when copying files? This sets the urgency of F-11.
4. Can the Smart Pole give up the microSD slot (frees `IO38..41`), or must it keep logging?
5. Are the vendor PDFs in `docs/` intended to be public (F-18)?

---

## Appendix A - 3.49-V2 header pins extracted from the schematic

Source: Waveshare `ESP32-S3-Touch-LCD-3.49 V2` schematic (page 1, `J3` and `J6`), cross-checked with the `gpio_pin_mapping.csv` in the sibling `Magnetic_Field_Scanner` project. Extracted from PDF text positions; verify visually.

| `J3` pin | Net | Notes | `J6` pin | Net | Notes |
| :---: | :--- | :--- | :---: | :--- | :--- |
| 1 | `IO38` | SD_CS | 1 | `IO0` | BOOT strap |
| 2 | `IO39` | SD_MOSI | 2 | `IO1` | free |
| 3 | `IO40` | SD_MISO | 3 | `IO2` | free |
| 4 | `IO41` | SD_SCLK | 4 | `IO3` | free (strap pin) |
| 5 | `IO43` | silkscreen **TXD** (UART0) | 5 | `IO4` | `BAT_ADC` |
| 6 | `IO44` | silkscreen **RXD** (UART0) | 6 | `IO5` | free |
| 7 | `IO47` | silkscreen **SDA** (sensor bus) | 7 | `IO19` | `U_N` (USB) |
| 8 | `IO48` | silkscreen **SCL** (sensor bus) | 8 | `IO20` | `U_P` (USB) |
| 9 | `GND` | | 9 | `GND` | |
| 10 | `3V3` | | 10 | `3V3` | |
| 11 | `VBAT` | | 11 | `VBUS` | |

Internal-only (not on any header): `IO6` `I2S_DSOUT`, `IO7` `I2S_MCLK`, `IO8` `EXIO_INT`, `IO9..14` LCD, `IO15` `I2S_SCLK`, `IO16` `SYS_OUT`, `IO17/18` touch I2C, `IO21` `LCD_TE`, `IO42` `LCD_BL`, `IO45` `I2S_DSIN`, `IO46` `I2S_LRCK`.

## Appendix B - Codebase metrics

| Item | Value |
| :--- | :--- |
| Commits | 165 |
| `.git` size | 82 MB |
| Largest source files | `lvgl_port.c` 3,783 / `e220_pacer.py` 1,352 / `cogo_solver.py` 1,165 / `e220_driver.cpp` 940 / `waypoint_manager.cpp` 913 / `um982_driver.cpp` 839 lines |
| v0.8.0 build | 1,680,202 B (53% of 3 MB app partition); 66,976 B static RAM (20%) |
| Automated tests | 0 unit tests; 2 hardware-in-loop scripts |
| Docs required to be hand-synced by `GEMINI.md` | 12 |
