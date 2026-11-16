# Composite Plan: Token-Efficient Modularization & Beam-Mounted DFRobot Gravity 10DOF (`SEN0696` / `Product 3126`) 9-DOF Heading Fusion

**Document Version:** `1.0.1`  
**Date:** November 16, 2026  
**Product Reference:** DFRobot Gravity: BMI323 + BMM350 + BMP581 10DOF IMU (`SKU: SEN0696` / [`Product 3126`](https://www.dfrobot.com/product-3126.html))  
**Firmware Baseline:** `v0.8.14` (`e220_pacer.py v3.7.1`, `stage2_bench_verify.py v1.0.3`)  
**Target Firmware Milestones:** `v0.8.15` (Hygiene & Modularization) $\rightarrow$ `v0.9.0` (`SEN0696` Beam 10DOF Driver, 3D AHRS & Diagnostics)  
**Supersedes:** Part 2 (outboard compass/IMU roadmap) of [`plan_phase8_single_uart_um982_and_mmc5983ma_compass.md`](plan_phase8_single_uart_um982_and_mmc5983ma_compass.md) *(Note: Part 1 Single-UART `UM982 COM1` shipped in `v0.8.0` and remains active)*  
**Consolidates & Updates:** [`plan_project_structure_and_token_optimization.md`](plan_project_structure_and_token_optimization.md) (Phases 1–6 updated to `v0.8.14` line counts)  
**Forward Compatibility:** [`plan_smart_pole_and_android_app.md`](plan_smart_pole_and_android_app.md) (`ESP32-S3-Touch-LCD-3.49-V2` beam box)

---

## 1. Executive Summary & Architectural Shift

Now that `v0.8.14` has proven clean `4.8 kbps` LoRa downlink and continuous `RTK_FIX` reception (`test/_050`: `99.94%` post-boot frame delivery, `0` truncations, `0` CRC drops, `3/3` dynamic rate switches), field reliability under heavy forest canopy shifts to **fast, drift-free azimuth & beam tilt whenever Antenna 2 (`SLV`) loses `HDG FIX` / `HDG FLT`**.

Previously, the handheld **Waveshare `QMI8658` (`0x6B`)** measured the orientation of the *display enclosure*, not the dual-antenna beam, forcing a fragile mental/UI decoupling and providing only 6-DOF gyro/hold fallback unless a standalone `MMC5983MA` was mounted on the crossbar.

Mounting the newly arrived **DFRobot Gravity: 10DOF IMU Sensor (`SKU: SEN0696`)** **directly onto the rigid antenna beam** (between the center pole and Slave antenna) transforms the sensor architecture:
1. **Single Rigid Body Frame Shared by Accel + Gyro + Mag + Baro:** Because the `BMI323` (6-axis accel/gyro), `BMM350` (3-axis TMR magnetometer), and `BMP581` (barometric pressure & temperature) live together on one PCB bolted directly to the beam, **beam tilt compensation, 3D gravity-projected yaw rate, Dual-GNSS True-North tare learning, and pole bubble level all operate natively in the beam frame** — no screen-to-pole coordinate translation is ever needed.
2. **Modular Codebase First (`A4` Decision):** Before adding a new multi-sensor driver, 3D AHRS calibration, and UI diagnostic cards, **Phase 1** executes the deferred hygiene and source-file modularization work from [`plan_project_structure_and_token_optimization.md`](plan_project_structure_and_token_optimization.md) so every `.c`/`.cpp`/`.py` module stays under Antigravity's single-call `read_file` window (`< 800 lines` / `< 46 KB`) and eliminates duplicate search noise.

> **Important Sensor Capability Note (Temperature + Pressure vs. Humidity):**  
> The Bosch **`BMI323`**, **`BMM350`**, and **`BMP581`/`BMP580`** datasheets in [`docs/datasheets/`](../datasheets/) provide **3-axis acceleration, 3-axis angular rate, 3-axis magnetic flux, 24-bit barometric pressure (`300–1250 hPa`, `±6 Pa` / `±50 cm` relative altitude), and ambient/die temperature** (**10 DOF + Temperature**). **None of these three Bosch chips includes a relative humidity (`%RH`) transducer.** If relative humidity logging is wanted later in the field, an optional I2C humidity sensor (e.g., `SHT40` at `0x44`) can share the same `I2C0` bus with zero conflict.

---

## 2. Locked Hardware Decisions & Multi-Sensor Architecture

Per architecture selections confirmed on November 16, 2026:

| Domain | Locked Decision | Engineering Rationale & Constraints |
| :--- | :--- | :--- |
| **Beam 10DOF Module** | **DFRobot Gravity `SEN0696`** (`BMI323` + `BMM350` + `BMP581` behind onboard Gravity MCU) | Operates from `3.3V–5V`; Gravity `PH2.0-4P` connector with onboard protocol MCU (`DFRobot_Multi_DOF_IMU` register/RTU protocol) supporting I2C (`0x4A..0x4D`) or UART. |
| **Wiring Bus (`3.5B`)** | **Shared `I2C0` (`GPIO 7 SCL`, `GPIO 8 SDA`, `400 kHz`)** using **`0x4A`** (DIP `A1=0, A0=0`, I2C mode) | Zero I2C address conflict with `0x18` (`ES8311`), `0x20` (`TCA9554`), `0x30` (`MMC5983MA`), `0x34` (`AXP2101`), `0x3B` (`Touch`), `0x51` (`RTC`), `0x6B` (`QMI8658`). Directly mirrors `3.49-V2` Smart-Pole `J3 Pin 17/18`. **Contingency:** freed `GPIO 47/48` remain available for isolated `I2C1` if long beam cable capacitance elevates internal `I2C0` `err=0x103` retries. |
| **I2C Driver Stack** | **Custom `i2c_bsp`-native driver (`src/beam_imu_bsp/`)** — NOT Arduino `Wire` | **`TwoWire` (`&Wire`) cannot share `GPIO 7/8` with ESP-IDF `i2c_new_master_bus()`.** All `0x4A` reads/writes MUST run through `i2c_bsp_lock()` + `user_i2c_port0_handle` (`i2c_master_transmit_receive`) alongside `Touch/PMIC/IMU/RTC`. |
| **Sensor Priority Hierarchy** | **Tier A:** `SEN0696` (`0x4A`) on Beam $\rightarrow$ **Tier B Fallback:** `MMC5983MA` (`0x30`) + `QMI8658` (`0x6B`) $\rightarrow$ **Tier C Fallback:** `QMI8658` (`0x6B`) 6-DOF | Plug-and-play across bench & field configurations; retains `MMC5983MA` (`0x30`) auto-detection while shifting primary development to `SEN0696`. `QMI8658` stays active solely for handheld LCD `DisplayPose_t` (`FLAT / ANGLED / VERTICAL`) when `SEN0696` handles pole/beam telemetry. |
| **Execution Sequence** | **Phase 1: Repo Hygiene & File Modularization FIRST** $\rightarrow$ **Phases 2–5: `SEN0696` Driver, 3D AHRS, Heading Fusion & UI** | Splits `lvgl_port.c` (`4,206 L`), `e220_driver.cpp` (`1,596 L`), `um982_driver.cpp` (`1,146 L`), and `waypoint_manager.cpp` (`1,017 L`) before adding new beam 10DOF driver, UI cards, and calibration logic. |

### Gravity 4-Pin Wiring Color Code Hazard (vs. SparkFun Qwiic)

Reference: [`docs/images/DFRobot 10DOF (SEN0696) 4-pin Cable Color Code.png`](../images/DFRobot%2010DOF%20(SEN0696)%204-pin%20Cable%20Color%20Code.png)

| Gravity 4-Pin Wire | `SEN0696` Silkscreen | I2C Function (DIP = I2C Mode) | `3.5B` Target Pin | **Watch-Out vs. SparkFun Qwiic** |
| :--- | :---: | :--- | :--- | :--- |
| **Green** | `D/T` | **`SDA` (I2C Data)** | **`GPIO 8`** (`I2C0_SDA`) | *Different from Qwiic (Qwiic Blue = SDA)!* |
| **Blue** | `C/R` | **`SCL` (I2C Clock)** | **`GPIO 7`** (`I2C0_SCL`) | *Different from Qwiic (Qwiic Yellow = SCL)!* |
| **Red** | `+` | **`VCC` (`3.3V–5V`)** | **`3V3`** (`+3.3V` Rail) | Power from `3V3` so `SEN0696` pull-ups stay strictly `3.3V` safe for ESP32-S3 GPIO7/8 |
| **Black** | `-` | **`GND`** | **`GND`** | Common Star Ground |

---

## 3. Phase 1: Repository Hygiene & Token-Efficient Modularization (`v0.8.15`)

Updated from the `v0.8.7` baseline in [`plan_project_structure_and_token_optimization.md`](plan_project_structure_and_token_optimization.md) against actual **`v0.8.14`** file sizes:

| Target File / Directory | Current Size (`v0.8.14`) | `read_file` Window Fit (`800 L / 46 KB`) | Planned Target Units |
| :--- | :---: | :---: | :--- |
| `src/lvgl_port/lvgl_port.c` | `4,206 lines` (`192,855 B`) | `~6x` chunks | `lvgl_ui_internal.h` + **7 modules** (`~400–750 L` each) |
| `src/e220_driver/e220_driver.cpp` | `1,596 lines` (`68,372 B`) | `~2x` chunks | `e220_driver_internal.h` + **3 modules** (`< 650 L` each) |
| `src/um982_driver/um982_driver.cpp` | `1,146 lines` (`47,460 B`) | `> 1x` (over both caps!) | `um982_driver_internal.h` + **2 modules** (`< 650 L` each) |
| `src/waypoint_manager/waypoint_manager.cpp` | `1,017 lines` (`39,796 B`) | `> 1x` (over `800 L` cap) | **2 modules** (`< 550 L` each) |
| `CHANGELOG.md` | `1,295 lines` (`> 150 KB`) | `~3–4x` chunks | Keep `v0.7.0+` (`~387 L`); archive `v0.1.0..v0.6.2` |
| `RTK_Rover/build/` | Unstaged cache | Doubles grep hits | Purge in-tree `build/`; compile via `%LOCALAPPDATA%\Temp\rtk_rover_build` |
| `docs/` root + `scripts/` | 16 loose `docs/` files + 23 scripts | High discovery cost | Sort into `docs/hardware/`, `docs/images/`; add `scripts/README.md` |

### Step 1.1 — Zero-Risk Repo, Cache & Docs Cleanup (No Firmware Code Touched)
1. **Purge In-Tree `RTK_Rover/build/` Cache:** Remove `RTK_Rover/build/` and standardize passing `--build-path "$env:LOCALAPPDATA\Temp\rtk_rover_build"` (optional alongside existing `.gitignore` `build/`) so workspace `Select-String` / searches never scan duplicate build artifacts.
2. **Archive Legacy Changelog:** Move lines `v0.1.0` through `v0.6.2` (`L388..L1295`) from [`CHANGELOG.md`](../../CHANGELOG.md) to `docs/archive/CHANGELOG_v0.1_to_v0.6.md` with a top link, shrinking `CHANGELOG.md` to `< 400 lines`.
3. **`docs/` & `scripts/` Hygiene:**
   * Move loose pinout CSV/XLSX & link-budget spreadsheets (`Waveshare*.csv`, `Waveshare*.xlsx`, `Expected LoRa Signal Strength.xlsx`) to `docs/hardware/` (updating any links in `docs/schematics/`, `README.md`, `GEMINI.md`, and `docs/plans/`).
   * Move loose setup/pinout images (`ESP32-S3-Touch-LCD-3.5-Arduino_Setup.png/.webp`, `Waveshare-ESP32-S3-Touch-LCD-3.5B-GPIO_Pin_Assignment-rev3.png`) into `docs/images/`.
   * Add a concise `scripts/README.md` (`~60–80 lines`) categorizing the 23 utilities (Pi Base Pacer, Stage 1/2 Bench Verifiers, Field Log Analyzers, COGO Solver, Security Gate) and remove/merge obsolete `302 B` wrapper stubs (`setup_um980.py`, `setup_unicore.py`) after checking references.

### Step 1.2 — Progressive On-Demand Workspace Skills (`.agents/skills/`)
Create `.gitignore`'d workspace runbooks matching `.agents/` boundaries:
1. **`.agents/skills/firmware-build-and-release/SKILL.md`:** Windows `arduino-cli` exact FQBN command, `--warnings default` zero-warning rule, `DRAM0 <= 25%` (`< 80 KB`) + `xtensa-esp32s3-elf-size -A` trigger (`>= 23%`), `stage2_bench_verify.py --self-test`, `test_rover_telemetry.py`, `check_secrets.py`, tiered doc sync, and atomic commit/push workflow.
2. **`.agents/skills/field-test-log-analysis/SKILL.md`:** Standard metrics extraction for `test/_0XX/pacer_test.log` + `serial-log_*.txt` (`stage2_bench_verify.py --from-log`, subpacket delivery by size/rate, `AUX_low` anomaly correlation, `Fix`/`HDG` transitions, `I2C WARN` counts).

### Step 1.3 — Surgical C/C++ Module Splits (One PR/Build Gate Per Subsystem)
1. **Split `src/e220_driver/e220_driver.cpp` (`1,596 L` $\rightarrow$ `e220_driver_internal.h` + 3 Files):**
   * `e220_config_mode3.cpp` (`~520 L`): Pin init, `REG0/1/3` builders, Mode 0$\leftrightarrow$3 transitions, `e220_exit_mode3_to_mode0()`, `e220_probe_mode0_liveness()`, readback/write retries, selftest.
   * `e220_rtcm_gatekeeper.cpp` (`~400 L`): `rx_stage[]`, `rtcm_crc24q_compute()`, `e220_process_rx_stage()`, late RSSI intercept (`0b`), `e220_ingest_rx_bytes()`, burst logging.
   * `e220_tdma_scheduler.cpp` (`~550 L`): `e220_aux_isr()`, superframe clock, `e220_is_uplink_slot_active()`, `e220_poll_and_forward_rtcm()`, RSSI query state machine, uplink telemetry & 4-ACK switch execution, 60s watchdog.
2. **Split `src/um982_driver/um982_driver.cpp` (`1,146 L` $\rightarrow$ `um982_driver_internal.h` + 2 Files):**
   * `um982_config_audit.cpp`: `Serial1` init, `s_com1_tx_mutex`, quiet-first setup, `um982_query_receiver_state()`, powerup selftest.
   * `um982_nmea_parser.cpp`: Strict 3-way `process_com1_line()` demux (`NMEA`/`UNIHEADINGA`/`BASEINFOA`/CMD ACK), `nmea_logger_feed()`, `um982_forward_rtcm_to_com2()` (`COM1` TX).
3. **Split `src/waypoint_manager/waypoint_manager.cpp` (`1,017 L` $\rightarrow$ 2 Files):**
   * `waypoint_csv_store.cpp`: Chunked `512 B` loader, stem cache, file switcher.
   * `waypoint_survey_log.cpp`: Multi-epoch averaging, centroid calculation, CSV pin writer.
4. **Split `src/lvgl_port/lvgl_port.c` (`4,206 L` $\rightarrow$ `lvgl_ui_internal.h` + 7 Files):**
   * `lvgl_port_core.c` (`~460 L`): QSPI `AXS15231B` panel/touch init, `heap_caps_malloc_extmem_enable(64)` wrapper (`AGENTS.md` PSRAM discipline preserved!), `rover_lvgl_lock()/unlock()`, `rover_lvgl_port_task`.
   * `ui_header_nav.c` (`~550 L`): Persistent 4-row status header bar & bottom nav bar.
   * `ui_tile0_guidance.c` (`~750 L`): Guidance, Line Stakeout, Mode 2 Map View + pan/zoom HUD.
   * `ui_tile1_logger_settings.c` (`~700 L`): Survey Logger + 3x4 Quick Settings Deck (`LORA`, `WIFI`, etc.).
   * `ui_tile2_csv_browser.c` (`~450 L`): Multi-file CSV selector & waypoint table.
   * `ui_tile3_compass_level.c` (`~430 L`): Compass dial, 2D Pole Bubble Level, `TARE` button *(Phase 4 target for Beam 10DOF `CAL MAG` + Temp/Pressure readout)*.
   * `ui_tile4_diagnostics.c` (`~520 L`): Dual-power UPS, LoRa RF stats, UM982 State/LOGLIST, Beam 10DOF & fallback sensor diagnostics *(Phase 4 target)*.
5. **Phase 1 Gate:** Compile via exact `AGENTS.md` `arduino-cli` command after **every** split (`0` compiler warnings, static `DRAM0 <= 25%` / `< 80 KB`), preserve `LV_USE_FLOAT = 0` (`snprintf` + `lv_label_set_text()`), verify `stage2_bench_verify.py --self-test`, `test_rover_telemetry.py`, and `check_secrets.py`.

---

## 4. Phase 2: `SEN0696` Hardware Bring-Up & `i2c_bsp`-Native Driver (`v0.9.0-alpha`)

### Step 2.1 — Extract Exact `SEN0696` Register/RTU Protocol & Build `src/beam_imu_bsp/`
Because `SEN0696` places an onboard aggregator MCU in front of the `BMI323`, `BMM350`, and `BMP581` at I2C address **`0x4A`**, DFRobot's Arduino library (`DFRobot_Multi_DOF_IMU` + `DFRobot_RTU`) talks to that MCU over `TwoWire` (`Wire`). **We must NOT instantiate `TwoWire` alongside `i2c_bsp`'s ESP-IDF `i2c_master_bus_handle_t` (`user_i2c_port0_handle`) on `IO7`/`IO8`.**

1. **Verify Hardware I2C Address Scan & Exact `DFRobot_Multi_DOF_IMU` Wire Framing:** Run an initial `I2C0` bus scan on the wired `Product 3126` (`SEN0696`) board in I2C mode (confirming `0x4A..0x4D` Gravity MCU response vs. raw `0x69`/`0x15`/`0x47`), and inspect the official `DFRobot/DFRobot_Multi_DOF_IMU` + `DFRobot_RTU` I2C packet format (`reg` layout, byte order, scaling for `accel`, `gyro`, `mag`, `pressure`, `temperature`, calibration/mode registers, and whether onboard orientation/Euler/quaternion registers exist or if raw scaled 3-axis vectors are read) before writing the C++ driver.
2. **Add `ROVER_I2C_ADDR_BEAM_IMU = 0x4A` (`user_config.h`) & Device Handle (`i2c_bsp.c/.h`):**
   * Register `beam_imu_dev_handle` on `user_i2c_port0_handle` (`400 kHz`, `.scl_wait_us = 20000`, `i2c_bsp_lock()` protected, covered by `i2c_bsp_reinit_bus()` teardown/re-add so warm resets & bus recovery never leak device handles!).
   * Keep `i2c_bsp_probe_optional_addr(0x4A)` quiet so units without `SEN0696` attached print zero error warnings and seamlessly fall back to `MMC5983MA` (`0x30`) / `QMI8658` (`0x6B`).
3. **Create `src/beam_imu_bsp/sen0696_driver.h` & `.cpp` (`< 450 lines`):**
   * Struct `BeamImu10DofState_t`:
     * `present`, `valid`, `sample_ms`
     * `acc_g[3]`, `gyr_dps[3]`, `mag_uT[3]`, `mag_total_uT`
     * `pressure_pa` / `pressure_hpa`, `baro_alt_m`, `temp_c`
     * `raw_mag_heading_deg`, `beam_pitch_deg`, `beam_roll_deg`
     * `i2c_error_count`, `consecutive_failures`
   * Poll cadence upgrade in [`RTK_Rover.ino`](../../RTK_Rover.ino#L348): increase Core 1 (`RoverUI` Priority 2) sensor polling from `10 Hz` (`100 ms`) to **`25–40 Hz` (`25–40 ms`)** when `sen0696_is_present()` is true (using a single compact multi-register burst read per sample to minimize `I2C0` mutex hold time while `AXS15231B` touch runs!).
4. **Bench Electrical Stress Gate on `I2C0` (`GPIO 7/8`):**
   * Connect `SEN0696` via actual pole cable length to `GPIO 7/8` + `3V3`/`GND`.
   * Run a 15-minute desk soak with active touch UI dragging, LCD flushing, SD logging, and LoRa RTCM traffic:
     * **Pass:** `SEN0696` sample stream clean, zero `I2C0` bus lockups, `err=0x103` warning rate stays low (`<=` baseline).
     * **Contingency (`Re-plan / Hardware Pivot Trigger`):** If external pole-cable capacitance or EMI causes `I2C0` timeouts affecting `Touch`/`AXP2101`/`QMI8658`, switch `SEN0696` immediately to an isolated **`I2C1` (`I2C_NUM_1`) on freed `GPIO 47/48`** (configurable in `user_config.h` with internal `i2c_bsp` port1 handle) with **zero** changes to `nav_engine` or UI!

---

## 5. Phase 3: Beam Coordinate Frame, 3D AHRS & Enhanced Woods Heading Engine (`nav_engine`)

This phase directly targets **maximum heading accuracy and zero drift when Antenna 2 loses `SLV FIX` / `SLV FLT`**.

```mermaid
flowchart TD
    S["SEN0696 on Rigid Beam (0x4A)\nAccel + Gyro + Mag + Baro/Temp"] -->|Valid| BEAM["Beam Body Frame Transform\n& Stationary Gyro Bias Est (bg)"]
    M["Legacy MMC5983MA (0x30) + QMI8658"] -.->|Fallback if SEN0696 absent| F69["Legacy Tilt + Mag"]
    Q["Onboard QMI8658 (0x6B)"] -->|Display Enclosure Only| POSE["LCD DisplayPose_t\n(FLAT / ANGLED / VERTICAL)"]

    BEAM --> TILT["Gated 3D Beam Attitude (q_beam)\nBeam Pitch (theta), Roll (phi), World Yaw Rate (omega_z)"]
    UM["UM982 UNIHEADINGA\n(HDG FIX / HDG FLT)"] --> ARB{"Heading Priority\n& Auto-Teacher"}
    TILT --> ARB

    UM -->|HDG FIX active| CAL["Auto-Calibrate While Clear-Sky:\n1. Mag True-North Tare (EMA)\n2. Hard-Iron Bias (bx0, by0) & B_ref/Bz_ref\n3. Beam Pitch Bias (theta - baseline_pitch)"]
    CAL --> COMP["Tier 3: Beam 9-DOF AHRS (HDG COMP)\nWorld Gyro Predict + Innovation-Gated Mag"]
    ARB -->|No SLV FIX/FLT| WALK{"Moving >= 0.70m?"}
    WALK -->|Yes| DISP["Tier 2: HDG DISP"]
    WALK -->|No| ANOM{"Iron Anomaly Gate?\n||B||, Bz, |dPsi_mag - dPsi_gyr|"}
    ANOM -->|Clean Mag| COMP
    ANOM -->|Iron Spike| GYRO["Tier 4: Bias-Compensated\nBMI323 Gyro Hold (HDG GYRO/HOLD)"]
```

### Step 3.1 — Rigid Beam Coordinate Alignment & Mounting Offsets
1. **Axis Orientation Matrix (`beam_imu_orient_mode` in `settings.json`):** Because the `SEN0696` PCB on the beam might be mounted face-up, face-down, or rotated $90^\circ / 180^\circ$ to clear hardware/cables, define a compile-time default + JSON-configurable $3 \times 3$ signed axis permutation matrix (`R_mount`) mapping `SEN0696` `(X, Y, Z)` into the canonical **Beam Frame**:
   * **$+X_{\text{beam}}$**: Along the dual-antenna crossbar from Pole Center toward `ANT2` (Slave), or configurable axis sign.
   * **$+Y_{\text{beam}}$**: Orthogonal horizontal left/right relative to facing direction.
   * **$+Z_{\text{beam}}$**: Upward along survey pole vertical centerline (`+1.0g` rest reaction when pole is plumb).
2. **Effect of Offset Distance Along the Beam (`center pole <-> Slave`):**
   * Rotational orientation (`pitch`, `roll`, `heading`) and angular rate (`omega`) of a rigid aluminum beam are **invariant** to linear position along the beam.
   * The only position-dependent artifact at offset $r_{\text{beam}}$ (`TBD`, e.g., `0.10–0.22 m` from pole centerline toward Slave) is centripetal + Euler tangential acceleration during brisk twists: $|\vec{a}_{\text{cent}}| \approx \omega^2 r_{\text{beam}}$. By gating accelerometer gravity updates strictly to $|\|\vec{a}\| - 1g| < 0.12g$ **and** $|\vec{\omega}| < 12^\circ/\text{s}$, linear offset along the beam introduces zero false bubble/tilt error even during pole spins!
3. **Retire Display-to-Pole Translation Ambiguity:**
   * When `SEN0696` is active:
     * `pole_bubble_x_deg` and `pole_bubble_y_deg` come **directly from beam roll/pitch** (never affected by tilting the handheld 3.5B screen!).
     * `QMI8658` (`0x6B`) drives **only** `ui_state.display_pose` (or serves as backup if `SEN0696` is unplugged).

### Step 3.2 — Upgraded 3D Beam AHRS & Dual-GNSS Auto-Calibration (`nav_engine`)
Split `nav_heading_9dof_update()` internals cleanly inside `nav_engine.cpp` (preserving pure C compatibility) with 5 focused improvements:

1. **Dynamic-Motion-Gated 3D Tilt Estimator (`pitch`, `roll`, Gravity Unit Vector $\hat{g}$):**
   * Replace simple EMA accelerometer-only `atan2` on [`imu_bsp.c`](../../src/imu_bsp/imu_bsp.c#L172-L182) with a gyro-propagated + accel-corrected **3D complementary / Mahony attitude filter** running on every `25–40 Hz` `SEN0696` sample.
   * Reject accel tilt corrections when $|\|\vec{a}\| - 1g| > 0.12g$ or $|\vec{\omega}| > 12^\circ/\text{s}$, eliminating walk-bounce jitter on the Bubble Level AND giving crisp tilt-compensation of `BMM350` `(Bx, By, Bz)` during turns!
2. **World-Vertical Projected Gyro Rate ($\omega_{\text{world},z}$) + Continuous Stationary Bias Tracking ($\vec{b}_g$):**
   * Project calibrated 3D gyro onto the estimated gravity vertical vector $\hat{g}$: $\omega_{\text{world},z} = \hat{g} \cdot (\vec{\omega}_{\text{beam}} - \vec{b}_g)$, rather than raw unprojected `gyr_z_dps`.
   * Detect beam stationarity (sliding window of gyro & accel norm variance `< threshold` for `>= 1.5 s`) to continuously update $\vec{b}_g$ (slow EMA), reducing stationary gyro drift during iron-anomaly coasts by an order of magnitude vs. uncalibrated `±1 deg/s` zero-rate offset.
3. **3-Axis Geomagnetic Field Calibration (`BMM350`) Trained by `UM982 HDG FIX` + 360° Spin Routine:**
   * **Offline/Assisted Hard-Iron Fit (Optional Tile 3 `CAL MAG` Button):** User slowly rotates the pole `360°` horizontally (keeping bubble within `±5°`); collects `(Bx, By)` samples across all quadrants to fit hard-iron center `(bx_off, by_off)` and field circle radius `B_horiz_ref`, persisted to `settings.json`. *(Important: beam-bolted aluminum/coax/connectors stay rigid relative to `SEN0696`, so hard-iron offsets remain stable across trips once learned!)*
   * **Continuous Online Teacher (`HDG FIX`):** Whenever `UM982 UNIHEADINGA` has `HDG_SRC_DUAL_FIX` and tilt $< 15^\circ$:
     * Updates circular EMA of `mag_true_north_offset_deg` (`alpha = 0.05`).
     * Updates **both** total field norm $B_{\text{ref}}$ and vertical magnetic component $B_{z,\text{ref}}$.
     * Updates **`beam_pitch_tare_deg`** against `UM982` `baseline_pitch_deg` (confirming physical plumb alignment between the beam GNSS antennas and `SEN0696`).
4. **Triple-Condition Iron Anomaly Gatekeeper:**
   Instead of checking only `| ||B|| - B_ref | > 3.5 uT`, flag `mag_anomaly = true` whenever **any** holds:
   * Magnitude spike: $\big|\|\mathbf{B}\| - B_{\text{ref}}\big| > 3.5\,\mu\text{T}$ (or `< 25` / `> 75 uT`),
   * Vertical dip distortion: $|B_z^{\text{world}} - B_{z,\text{ref}}| > 3.0\,\mu\text{T}$,
   * **Gyro-vs-Mag Innovation Mismatch:** $|\text{wrap180}(\Delta\psi_{\text{mag}} - \omega_{\text{world},z}\,\Delta t)| > 4.0^\circ$ sustained over `> 200 ms` (catches subtle ferrous distortion from nearby monuments/pipes that twists direction before changing total magnitude!).
   * While `mag_anomaly == true`: immediately coast using bias-corrected $\omega_{\text{world},z}$ (`HDG_SRC_GYRO` / `HDG_SRC_HOLD`) and smoothly blend back once clean geomagnetic consistency returns for `>= 0.5 s`.
5. **Calibrated Beam Lever-Arm Guard (`nav_compute_pole_tip_position()`):**
   * In `v0.8.x`, `nav_compute_pole_tip_position()` disabled Master$\rightarrow$Pole-Center (`<= 27.75 cm`) lever-arm compensation whenever `HDG_SRC_COMP` was active because the handheld compass didn't measure beam orientation.
   * With **`SEN0696` mechanically rigid on the beam** + `mag_cal_valid && !mag_anomaly`, we now *do* know true beam azimuth & beam pitch! Add an opt-in setting `apply_lever_arm_on_calibrated_beam` (or enable strictly when `HDG_SRC_COMP` has validated `beam_cal_quality`, defaulting to conservative safe behavior until outdoor error trials verify `< 2 deg` accuracy under canopy).

---

## 6. Phase 4: Environmental (`BMP581`), UI Cards, Route Logging & Telemetry Integration

1. **Barometric Pressure, Relative Elevation Aid & Ambient Temperature (`BMP581`):**
   * Read ambient `temp_c` and `pressure_hpa` at `2–5 Hz` from `SEN0696`.
   * Whenever `fix_type == GNSS_FIX_RTK_FIXED`, slowly discipline a local QNH/baro elevation bias (`h_diff = tip_height_m - baro_alt_m`), exposing a clean **`Baro Rel Elev`** (`m`/`ft`) sanity indicator if GNSS temporarily drops to Single/DGPS under extreme canopy.
   * Surface `beam_temp_c` and `beam_pressure_hpa` inside `RoverUIData_t` (pure 7-bit ASCII formatting via `snprintf` + `lv_label_set_text()`, avoiding `LV_USE_FLOAT = 0` traps!).
2. **Modular UI Additions (Inside Phase 1 Split Files):**
   * **`ui_tile3_compass_level.c`:**
     * Show active heading provenance (`DUAL FIX`, `BEAM 9DOF`, `WALK`, `GYRO COAST`, `HOLD`) + `IRON ANOMALY` alert pill.
     * Drive 2D Pole Bubble Level directly from `SEN0696` beam attitude when online (labeling badge `[BEAM 10DOF]` vs `[LCD FALLBACK]`), add `CAL MAG` (360° spin button) beside `TARE`, and show live `Temp` & `Baro` readout.
   * **`ui_tile4_diagnostics.c`:**
     * Update sensor card to display multi-sensor probe state: e.g., `Beam 10DOF (SEN0696 @ 0x4A): ONLINE [BMI323+BMM350+BMP581]`, `Temp / Press`, `BMM350 |B| / Tare`, stationary gyro bias status, and `QMI8658` Display-Pose status.
   * **`ui_header_nav.c`:** Retain `[CMP]` badge (Teal `#14B8A6`) when `HDG_SRC_COMP` drives guidance under canopy.
3. **SD Route CSV & Base Station Telemetry:**
   * Check CSV schema compatibility before optionally appending `beam_pitch_deg, beam_roll_deg, baro_hpa, beam_temp_c, hdg_src` to `route_logger.cpp` (or keeping `route_*.csv` stable and adding an optional sensor/diagnostics log field).
   * Keep the `48-byte` LoRa uplink telemetry frame (`RoverUplinkTelemetry_t` in [`rover_telemetry.h`](../../src/e220_driver/rover_telemetry.h#L44-L69)) **binary-compatible at 48 bytes** (`heading_source = 7` already encodes `HDG_SRC_COMP`; ensure `e220_pacer.py` displays `[BEAM-9DOF]`/`[COMP]` cleanly).

---

## 7. Phase 5: Verification Plan & Field Acceptance Gates

| Stage | Verification Gate & Pass Criteria |
| :--- | :--- |
| **Gate 1 (End of Phase 1 Splits)** | Exact `AGENTS.md` `arduino-cli` build (`--warnings default`): **`0` warnings**, static `DRAM0 <= 25%` (`< 80 KB`), `[MEM] Post-Setup Heap` internal free unchanged; every core `src/` file `< 800 lines`; `stage2_bench_verify.py --self-test` & `test_rover_telemetry.py` & `check_secrets.py` `100% PASS`. |
| **Gate 2 (Bench `SEN0696` I2C0 Soak)** | `SEN0696` @ `0x4A` polls at `25–40 Hz` alongside touch drag + QSPI LCD + SD logging for `15 min`: `0` I2C hangs, `[I2C WARN] 0x103` count does not degrade touch/PMIC; verify clean unplug fallback (`MMC5983MA` / `QMI8658`). *(Trigger `I2C1 GPIO 47/48` contingency immediately if cable loading destabilizes `I2C0`.)* |
| **Gate 3 (Mechanical Axis & Pitch Verification)** | Tilt beam +/-15° Pitch & Roll: Bubble Level (`Tile 3`) tracks physical direction properly; `beam_pitch_deg` agrees with `UM982 UNIHEADINGA` `baseline_pitch_deg` within **`<= 1.0 deg`** while `Display Pose` (`QMI8658`) responds independently to screen tilt. |
| **Gate 4 (Open-Sky Auto-Cal & Canopy `SLV` Dropout Test)** | Rotate pole 360° under open sky with `HDG FIX`: `mag_true_north_offset_deg` + hard-iron offset converge (`< 1.5 deg` residual across N/E/S/W). Disconnect `ANT2` (`SLV: NONE`): instant **`0.0 s`** switch to `[CMP]` (`< 2.0 deg` step change), zero stationary drift over `5+ min`, rebar proximity triggers `IRON ANOMALY` + gyro coast (`< 2 deg` drift over `15 s`), and **`4.8 kbps` LoRa `stage2_bench_verify.py --from-log`** stays at **`0` Trunc / `0` CRC**. |
| **Gate 5 (Release Sync)** | Bump firmware SemVer (`v0.8.15` for Phase 1 splits; `v0.9.0` for `SEN0696`), sync `README.md`, `CHANGELOG.md`, `docs/schematics/system_wiring_schematics.md`, `docs/user_manual.md`, `docs/project_overview.md`, pass `check_secrets.py`, commit & push atomically. |
