# Ebyte E220-900T30S Practical Engineering Reference

**Document Version:** 2.9  
**Last Updated:** November 15, 2026  
**Firmware Target:** `v0.8.11`  
**Companion Software:** `e220_pacer.py v3.6.5`  
**Target Hardware:** Ebyte `E220-900T30S` (`850.125 - 930.125 MHz`, `1 W` / `+30 dBm` SMD UART LoRa Module)  
**Applicability:** Directly applies to `E220-900T30S` / `E220-900T30D` (`1 W`) and `E220-900T22S` / `E220-900T22D` (`160 mW`, noting TX power bit differences in Section 4.4).  

**Reference Implementations in This Repository:**
* **ESP32-S3 Rover Driver & RTCM3 Gatekeeper:** [`src/e220_driver/e220_driver.cpp`](../src/e220_driver/e220_driver.cpp), [`src/e220_driver/rover_telemetry.h`](../src/e220_driver/rover_telemetry.h), [`user_config.h`](../user_config.h)
* **Raspberry Pi Hardware-Paced Transmitter Daemon:** [`scripts/e220_pacer.py`](../scripts/e220_pacer.py), [`scripts/e220-pacer.service`](../scripts/e220-pacer.service), [`scripts/update_e220_pacer.sh`](../scripts/update_e220_pacer.sh)
* **Spectrum Scanner & Configuration Utilities:** [`scripts/scan_e220_channels.py`](../scripts/scan_e220_channels.py), [`scripts/setup_e220.py`](../scripts/setup_e220.py), [`scripts/set_e220_power.py`](../scripts/set_e220_power.py)

---

## Table of Contents
1. [Why This Document Exists](#1-why-this-document-exists)
2. [Internal Hardware Architecture](#2-internal-hardware-architecture)
3. [Pinout, Power Supply & Electrical Traps](#3-pinout-power-supply--electrical-traps)
4. [Complete Register Map & Mode 3 Programming Protocol](#4-complete-register-map--mode-3-programming-protocol)
5. [The AUX Pin Truth & Two-Regime Pacing](#5-the-aux-pin-truth--two-regime-pacing)
6. [Air Rates, Link Capacity & Multi-Constellation Interleaving](#6-air-rates-link-capacity--multi-constellation-interleaving)
7. [Live Mode 0 RSSI & Ambient Noise Floor Queries](#7-live-mode-0-rssi--ambient-noise-floor-queries)
8. [RF Spectrum Selection & Near-Field Desense](#8-rf-spectrum-selection--near-field-desense)
9. [Bi-Directional Half-Duplex LoRa Protocol & Telemetry Uplink](#9-bi-directional-half-duplex-lora-protocol--telemetry-uplink)
10. [Touchscreen Remote Air-Rate Switching Protocol](#10-touchscreen-remote-air-rate-switching-protocol)
11. [Base Station Driver Architecture (e220_pacer.py)](#11-base-station-driver-architecture-e220_pacerpy)
12. [Store-and-Verify Sliding-Window Gatekeeper Pattern](#12-store-and-verify-sliding-window-gatekeeper-pattern)
13. [Rapid Field Troubleshooting Matrix](#13-rapid-field-troubleshooting-matrix)

---

## 1. Why This Document Exists

The official Ebyte manuals (`E220-900T30S_UserManual_EN_v1.0` and `E220-xxxTxxx_UserManual_EN_v1.7`) suffer from severe translation ambiguities, missing timing diagrams, and several outright factual errors regarding `AUX` pin behavior, EEPROM write acknowledgments, register reply framing, and half-duplex arbitration.

This guide documents the **actual silicon behavior** verified empirically across dozens of bench and field test campaigns (oscilloscope/ISR pulse timing, raw byte-level UART wire-tapping, RF spectrum scans, and cadastral survey field trials) while operating a `100.00%` lossless `1 Hz` multi-constellation GNSS `RTCM3` radio link with bi-directional remote telemetry.

---

## 2. Internal Hardware Architecture

Understanding the `E220-900T30S` requires recognizing that **your host MCU does not talk to a raw LoRa chip**. It communicates with an internal proprietary Ebyte protocol microcontroller that manages a Semtech `LLCC68` LoRa transceiver:

```text
+--------------------+        UART         +-------------------------------------------------------+
|                    |  TX  --------> RXD  |  Internal Ebyte MCU      SPI      Semtech LLCC68      |
|  Host Processor    |  RX  <-------- TXD  |  - 400-Byte RAM FIFO  <=======>  - 200B Max Payload   |
|  (ESP32 / RPi /    |  GPIO -------> M0   |  - Mode 0..3 State                - 32 MHz Passive    |
|   STM32 / RP2040)  |  GPIO -------> M1   |    Machine & AT/Hex                 XTAL (NO TCXO!)   |
|                    |  ISR <-------- AUX  |    Command Parser                         |           |
|                    |  GPIO -------> RST  |  - ~500ms RC Sleep Timer          +-------+-------+   |
+--------------------+                     |    (~320-340 ppm drift)           | 1W PA / +14dB |   |
                                           |                                   | RX LNA Switch |   |
                                           +-----------------------------------+-------+-------+---+
                                                                                       |
                                                                                   50-Ohm RF
```

### Critical Silicon Realities

1. **400-Byte MCU RAM Buffer vs. 200-Byte LoRa Packet Limit:**
   * The internal MCU buffers up to **400 bytes** from `RXD`, while each over-the-air `LLCC68` LoRa packet carries at most **200 bytes** (`REG1[7:6] = 00`).
   * This 400-byte buffer enables **zero-dead-air hardware packet chaining**: when you write a `201..400 B` message at `115200` baud with no inter-byte gaps, the MCU loads the first `200 B` into the `LLCC68` immediately and holds bytes `201..N` in RAM, chaining Subpacket 2 onto the air the instant Subpacket 1 completes.
2. **Passive `32 MHz` Crystal (`XTAL`), NOT a `TCXO`:**
   * Unlike industrial modules fitted with a temperature-compensated oscillator (`TCXO`), the `E220-900T30S` uses a **passive `32 MHz` quartz crystal** adjacent to a `1 W` power amplifier.
   * Under high duty cycles (`> 60%`), PA self-heating induces thermal carrier drift. Wide bandwidth rates (`38.4 kbps`, `62.5 kbps`) tolerate this drift effortlessly. Lower rates (`19.2 kbps`, `9.6 kbps`) provide exceptional link margin (`+3.5 dB` to `+6 dB`) but require managed duty cycles and conservative guard bands.
3. **Uncalibrated Internal RC Sleep Timer (`~330 ppm`):**
   * In Wake-on-Radio (`WOR`) Modes 1 and 2, the sleep/wake period (`500..4000 ms`) runs off an internal low-power RC oscillator measured at **`+320..+340 ppm`** (`~115 ms` drift per 6 minutes relative to GPS UTC). Transparent Mode 0 (`M0=0, M1=0`) bypasses this timer entirely.

---

## 3. Pinout, Power Supply & Electrical Traps

### 3.1 Pin Definitions

| Pin Name | Direction (E220 View) | Internal Pull | Electrical / Functional Notes |
| :--- | :---: | :---: | :--- |
| **`M0`** | Input | **Weak Pull-Up (`3.3 V`)** | Mode select bit 0 (`3.3 V` logic; **never leave floating**). |
| **`M1`** | Input | **Weak Pull-Up (`3.3 V`)** | Mode select bit 1 (`3.3 V` logic; **never leave floating**). |
| **`RXD`** | Input | Weak Pull-Up | UART data into E220 (`3.3 V` CMOS; use series `1k` resistor if driven from `5 V` TTL). |
| **`TXD`** | Output | Push-Pull (`3.3 V`) | UART data out of E220 (idles `HIGH = 3.3 V` once powered and initialized). |
| **`AUX`** | Output | Push-Pull / Open-Drain | `HIGH` = Idle/Buffer ready; `LOW` = Busy / Mode switching / RX UART shifting. Enable `INPUT_PULLUP` on host MCU. |
| **`VCC`** | Power | -- | `3.3 V - 5.5 V` DC (`>= 5.0 V` required to reach full `+30.0 dBm`; `3.3 V` rail caps RF output around `+26..27 dBm`, while `+21 dBm` works cleanly at `3.3 V` or `5.0 V`). |
| **`GND`** | Ground | -- | Low-impedance RF + power ground plane. |
| **`NRST` / `RST`** | Input | Internal Pull-Up | Active-`LOW` hardware reset (`drive LOW >= 50 ms`, release `HIGH`, then wait for `AUX == HIGH`). |

### 3.2 Power Supply & Bulk Decoupling Requirements
* **Peak TX Current (`E220-900T30S`):**
  * `@ 30 dBm (1000 mW)`: **`650 - 850 mA`** instantaneous current spikes on every packet.
  * `@ 21 dBm (125 mW)`: **`200 - 300 mA`** instantaneous current spikes.
  * `@ RX Continuous (Mode 0)`: **`~14 - 18 mA`**.
* Place **`>= 100 uF` to `470 uF` low-ESR electrolytic/tantalum + `100 nF` ceramic** directly across `VCC` and `GND` at the module pads. Brownouts mid-burst corrupt the internal MCU state machine and lock `AUX` `LOW`.

### 3.3 Operating Modes Truth Table

| Mode | `M1` | `M0` | Name | UART Baud | RF TX Behavior | RF RX Behavior | `C0 C1 C2 C3` RSSI Cmd? | `9600` EEPROM Cfg? |
| :---: | :---: | :---: | :--- | :---: | :--- | :--- | :---: | :---: |
| **0** | **`0`** | **`0`** | **Transparent / Normal** | `REG0` (`115200`) | Immediate TX (no WOR preamble) | **100% Continuous RX** | **YES (`< 30 ms`)** | No |
| **1** | **`0`** | **`1`** | **WOR Transmitting** | `REG0` (`115200`) | Prepends `WOR_Period` (`500 ms+`) wake preamble | Periodic `WOR` Sleep/Wake RX | YES (`~500 ms` delay) | No |
| **2** | **`1`** | **`0`** | **WOR Receiving** | `REG0` (`115200`) | TX Disabled | Periodic `WOR` Sleep/Wake RX | NO | No |
| **3** | **`1`** | **`1`** | **Deep Sleep / Config** | **Fixed `9600 8N1`** | TX Disabled | RX Disabled | No | **YES (`C0/C1/C2`)** |

### 3.4 The Open-`M0` Connector Failure ("Fake Mode 0")
Because **both `M0` and `M1` have internal weak pull-ups to `3.3 V` (`1`)**, a broken wire or cold solder joint on `M0` creates an insidious failure:
1. **Boot Self-Test (`Mode 3: M1=1, M0=1`) PASSES 100%:** Even with `M0` disconnected, the internal pull-up holds `M0 = 1`. The module enters Mode 3 and responds to EEPROM commands.
2. **Runtime Transition (`Mode 0: M1=0, M0=0`) Enters WOR Mode 1 (`M1=0, M0=1`):** MCU drives `M0 = 0`, but the open pin floats `HIGH`. The module enters Mode 1 (500 ms duty-cycled sleep).
3. **Diagnostic Signature:** Packets arrive with strong SNR, but **never closer than 500 ms apart** (`<= 1` subpacket/s ceiling). Multi-subpacket frames (`> 200 B`) always truncate. RSSI commands arrive phase-locked to a 500 ms grid.

#### Software Continuity Self-Test at Boot
When switching between any two modes, the E220 pulls `AUX` **`LOW` for `9..11 ms`**:
1. Drive `(M1=1, M0=1)` and wait for `AUX == HIGH`.
2. Toggle only `M0` `LOW` `(1,1) -> (1,0)` while monitoring `AUX` for 25 ms. An `AUX` `LOW` pulse verifies `M0` continuity.
3. Restore `(1,1)`, wait `AUX == HIGH`, then toggle only `M1` `LOW` `(1,1) -> (0,1)`. An `AUX` `LOW` pulse verifies `M1` continuity.
4. Drive `(M1=0, M0=0)` and wait for `AUX == HIGH` + 60 ms guard before opening operational traffic.

---

## 4. Complete Register Map & Mode 3 Programming Protocol

### 4.1 Mode 3 Serial Rules
* **Mode 3 (`M1=1, M0=1`) ALWAYS uses `9600 bps, 8N1`**, regardless of the operational baud rate in `REG0`.
* Flush Mode 0 UART TX, assert **`M0=1, M1=1` BEFORE switching host UART to `9600 bps`**, and wait `>= 60 ms` settle time. Switching the ESP32 UART to `9600` while the E220 is still listening at `115200` in Mode 0 injects a baud-mismatch framing glitch into the transceiver RX shift register (`test/_040`).
* Issue a pre-write Mode 3 read probe (`C1 00 06`) to verify UART alignment before sending `C0 00 06`, and keep `M0=1, M1=1` steady between initial retries instead of toggling `Mode 0 -> Mode 3` at `9600` baud.
* Wait `AUX == HIGH` + `>= 60 ms` (or flush Mode 3 reply) after returning to `M0=0, M1=0` when restoring operational baud (`115200`).

### 4.2 Binary Command Frame Formats

| Operation | Host Command Bytes (Hex) | Expected E220 Reply (Hex) | Persistence |
| :--- | :--- | :--- | :--- |
| **Write EEPROM** | `C0 <START> <LEN> <DATA[0..LEN-1]>` | `C1` *(or `C0`!)* `<START> <LEN> <DATA...>` | Saved to non-volatile EEPROM |
| **Read Registers** | `C1 <START> <LEN>` | `C1 <START> <LEN> <DATA[0..LEN-1]>` | Read-only |
| **Write Temp RAM** | `C2 <START> <LEN> <DATA[0..LEN-1]>` | `C1` *(or `C2`)* `<START> <LEN> <DATA...>` | Lost on power-cycle or `RST` |

> **`0xC0` vs `0xC1` Acknowledgment & Mode 3 Retry Protocol (`v0.8.7` / `v0.8.9`):** Certain E220 firmware builds echo `0xC0` instead of `0xC1` upon successful EEPROM write. Always accept `reply[0] in (0xC0, 0xC1)`. Field testing (`test/_038`, `test/_040`) showed that entering Mode 3 while a Mode 0 RSSI command or RF transmission is still active — or switching ESP32 UART to `9600` baud *before* raising `M0/M1` / bouncing `M0/M1` while locked at `9600` baud — injects a framing glitch and provokes repeated `FF FF FF` format-error replies (`got 3 bytes` / `got 4 bytes`) even while a subsequent `e220_read_config_registers()` (`set_mode(1,1)` *then* `9600`) succeeds immediately. Firmware `v0.8.9` cancels active Mode 0 RSSI queries, waits for `s_tx_air_busy_until_ms` + `AUX == HIGH` + `Serial0.flush()`, asserts **`e220_set_mode(1, 1)` BEFORE `Serial0.updateBaudRate(9600)`**, runs `e220_mode3_sync_read()` (`0xC1 0x00 0x06`), retries Attempts `1..3` in Mode 3 without bouncing `M0/M1` (`280 ms` timeout with `35 ms` frame-end exit for `>= 3 B` error frames), reserves a clean baud-matched `Mode 0 (115200) -> Mode 3 (9600)` bounce + sync read for Attempt `4`, and falls back to `e220_read_config_registers()` to resync `s_active_air_rate` if all attempts fail.

### 4.3 Production Register Profiles (Profile Rev 2)

| Parameter | Standard Link (`19.2 kbps`) | Deep-Canopy / Foliage (`9.6 kbps`) | High-Speed Bench (`38.4 kbps`) |
| :--- | :---: | :---: | :---: |
| **`ADDH` / `ADDL`** | `0x00` / `0x00` | `0x00` / `0x00` | `0x00` / `0x00` |
| **`REG0` (UART + Air Rate)** | **`0xE5`** (`115200 8N1`, `19.2k`) | **`0xE4`** (`115200 8N1`, `9.6k`) | **`0xE6`** (`115200 8N1`, `38.4k`) |
| **`REG1` (Pkt + RSSI + Power)**| **`0x23`** (`200B`, `RSSI=1`, `21dBm`) | **`0x23`** (`200B`, `RSSI=1`, `21dBm`) | **`0x23`** (`200B`, `RSSI=1`, `21dBm`) |
| **`CHAN` (Frequency)** | **`0x4B`** (`CH 75` = `925.125 MHz`) | **`0x4B`** (`CH 75` = `925.125 MHz`) | **`0x4B`** (`CH 75` = `925.125 MHz`) |
| **`REG3` (Transparent Mode)** | **`0x00`** (Transparent, LBT Off) | **`0x00`** (Transparent, LBT Off) | **`0x00`** (Transparent, LBT Off) |
| **`CRYPT_H` / `CRYPT_L`** | `0x00` / `0x00` | `0x00` / `0x00` | `0x00` / `0x00` |

---

## 5. The AUX Pin Truth & Two-Regime Pacing

### 5.1 `AUX == HIGH` Does NOT Mean RF Transmission Finished
`AUX` indicates whether the **internal 400-byte UART RAM buffer can accept bytes without overflow** — NOT whether the `LLCC68` RF power amplifier has finished airing.

At `115200` baud, end-of-frame is detected after a 3-character idle gap (`~0.26 ms`). What `AUX` does depends strictly on frame size:

### 5.2 Regime 1: Single-Subpacket Writes (`1 .. 200 Bytes`)
Because all bytes transfer into the idle `LLCC68` FIFO in `< 0.5 ms`, the 400-byte RAM buffer is immediately empty — **`AUX` stays `HIGH` while the packet transmits on the air!**

```text
Host UART TX:  |=== 144B (12.5ms) ===|---(idle)-------------------------------|
E220 RAM Buf:  [ Filling 0..144B     ][Xfer->LLCC68 (<0.5ms): RAM NOW EMPTY! ]
E220 AUX Pin:  HIGH ---------------------------------------------------------- (NEVER STAYS LOW!)
LLCC68 RF PA:                         |======= On-Air TX: 114.0 ms (@19.2k) =======|
Rover UART RX:                                                                    |== 144B (12.5ms) ==|
                                      |<----------- Must Wait: ToA + 35ms Guard ---------->|
```

* **Pacing Rule (`1..200 B`, `e220_pacer.py v3.6.5`):**
  1. Verify `AUX == HIGH` before writing.
  2. Write complete frame atomically and `flush()`.
  3. Sleep deterministically for **`ToA(N) + Guard_Delay`** (`Guard_Delay = max(guard_delay_s, 52 ms)` @ `<= 9.6 kbps`, `max(guard_delay_s, 40 ms)` @ `>= 19.2 kbps` to absorb the E220 UART idle frame timeout and `LLCC68` TX->RX turnaround before a following `1074`/`1094` preamble). Never wait for an `AUX` falling edge.
* **Mode 0 `C0 C1 C2 C3` Command Interaction During Active `<= 200 B` TX (`test/_038` Finding):**
  * Because `AUX` remains `HIGH` while a single-subpacket (`<= 200 B`, e.g., `48 B` Rover telemetry @ `4.8 kbps`, `~166 ms` on air) is actively transmitting on the `LLCC68`, writing `C0 C1 C2 C3 00 02` to `RXD` immediately after `Serial.flush()` causes the E220 MCU to hold the command in its UART RAM buffer and **pull `AUX` `LOW` for the remaining RF TX duration (`100-130 ms` at `4.8 kbps`)**, only outputting the 5-byte `0xC1 0x00 0x02 <noise> <pkt>` reply (`AUX` low `~7.0 ms`) once RF TX completes (`~166 ms` total latency).
  * **Guard Rule (`v0.8.11`):** After transmitting any Mode 0 uplink packet (`<= 200 B`), clear `isr_aux_fall_us = 0` when restoring `e220_suppress_aux_isr`, hold Mode 0 RSSI queries and Mode 3 transitions until `s_tx_air_busy_until_ms` expires, and allow `E220_RSSI_REPLY_TIMEOUT_MS = 95 ms` (bounded by `sf_ms >= 990 ms`).

### 5.3 Regime 2: Two-Subpacket Writes (`201 .. 400 Bytes`)
```text
Host UART TX:  |==== 343B (29.8ms) ====|
E220 RAM Buf:  [0..200B -> LLCC68][ 143B waiting in RAM... ][143B -> LLCC68: RAM EMPTY!]
E220 AUX Pin:  HIGH ---|_______________LOW (capped ~398ms @4.8k!)___|------ HIGH ----------------
LLCC68 RF PA:          |=== Subpkt 1: 200B (up to 545ms @4.8k) ===|=== Subpkt 2: tail ===|
                                                                   ^
                                       DANGER! At <=9.6 kbps, AUX can rise 100-147ms BEFORE
                                       Subpacket 1 finishes airing on LLCC68!
```

* **Critical `test/_038` Silicon Discovery (Early `AUX` Rise at `4.8 / 9.6 kbps`):**
  * At `19.2 kbps` (`Subpacket 1 ~ 148 ms`), `AUX_low` tracks Subpacket 1 closely (`~144.5 ms`).
  * At lower air rates (`4.8 kbps` where `200 B` takes `505-545 ms`, and `9.6 kbps` where `200 B` takes `265-281 ms`), the E220 MCU empties its 400-byte UART RAM buffer into internal SPI staging **before Subpacket 1 finishes transmitting on RF** (`AUX_low` measured at `398.5 ms` @ `4.8 kbps` and `~200 ms` @ `9.6 kbps`).
  * Waiting only `ToA(N - 200) + Guard_Delay` after `AUX` rises `HIGH` returns **`100-147 ms` too early** while Subpacket 2 is still airing, causing the next RTCM frame (`1094` / `1084`) to abort Subpacket 2 (`[RTCM DROP TRUNC]` on `1074` with `200 B` received) and lose Subpacket 1 of the following frame!
* **Pacing Rule Rev 2 (`201..400 B`, `e220_pacer.py v3.6.1`):**
  1. Verify `AUX == HIGH` before writing.
  2. Write all `N` bytes in one contiguous write (never pre-split in host software).
  3. Wait for `AUX` falling edge (`< 50 ms` timeout).
  4. Wait for `AUX` rising edge (`1.5 s` watchdog timeout).
  5. **Mandatory Tail + Full-ToA Floor Wait:** Sleep for **`max(ToA(N - 200), ToA(200) + ToA(N - 200) - elapsed_since_flush) + Guard_Delay`** so early `AUX` release at `<= 9.6 kbps` never allows the next frame to collide with Subpacket 2.

### 5.4 Hardware-Calibrated Time-on-Air (`ToA`) Formula

* **`>= 9.6 kbps` (`Guard_Delay = 35 ms`):**
$$\text{ToA}(N) = 0.016 + \frac{(N + 16) \times 8}{\text{AirRate}_{\text{bps}} \times 0.68}\ \text{seconds} \quad (1 \le N \le 200)$$

* **`<= 4.8 kbps` (`Guard_Delay = 50 ms`, calibrated `test/_039`):**
$$\text{ToA}(N) = 0.022 + \frac{(N + 16) \times 8}{\text{AirRate}_{\text{bps}} \times 0.60}\ \text{seconds} \quad (200\text{ B} \approx 622.0\text{ ms},\ 41\text{ B} \approx 180.3\text{ ms})$$

---

## 6. Air Rates, Link Capacity & Multi-Constellation Interleaving

### 6.1 Link Budget & Air Rate Comparison

| Air Rate | `REG0[2:0]` | Sensitivity | 200B Subpacket ToA | Max Net Capacity | RTCM Stream Viability (`~650 B/s`) |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **`2.4 kbps`** | `000` | `-129 dBm` | `1,180 ms` | `110 B/s` | Unusable for multi-constellation RTCM |
| **`4.8 kbps`** | `011` | `-126 dBm` | `622 ms` | `215 B/s` | Alternating single-constellation interleaving (`1074` held across Sec 0; `1005/1006/1230` gated to odd epochs; `+6 dB` margin) |
| **`9.6 kbps`** | `100` | **`-124 dBm`** | `295 ms` | `450 B/s` | **Requires Adaptive Interleaving** (`+3.5 dB` foliage gain) |
| **`19.2 kbps`**| `101` | **`-121 dBm`** | `148 ms` | `900 B/s` | **Standard Production Baseline** (`~60%` duty cycle, full 4-constellation) |
| **`38.4 kbps`**| `110` | `-118 dBm` | `82 ms` | `1,800 B/s` | Bench / High-rate telemetry (`500 kHz` BW) |
| **`62.5 kbps`**| `111` | `-115 dBm` | `57 ms` | `2,800 B/s` | Short range / Line of sight only |

### 6.2 The 9.6 kbps Channel Saturation Problem
A standard 4-constellation RTCM3 stream contains:
* `1006` Base Antenna Position: Every 5s (`27 B`)
* `1074` GPS MSM4: 1.0 Hz (`~197 - 245 B`, 2 subpackets)
* `1084` GLONASS MSM4: 1.0 Hz (`~138 - 197 B`, 1 subpacket)
* `1094` Galileo MSM4: 1.0 Hz (`~239 - 325 B`, 2 subpackets)
* `1230` GLONASS Code-Phase Biases: Every 10s (`~18 B`)
* **Total Throughput:** **`~625 - 650 Bytes/second` (5 to 6 physical subpackets per second)**.

At `19.2 kbps`, 5 subpackets require $\approx 550\text{--}620\text{ ms}$ airtime, leaving an ample $\approx 380\text{--}450\text{ ms}$ half-duplex quiet window each 1-second epoch.

At `9.6 kbps`, each 200B subpacket requires $295\text{ ms}$. Transmitting 5 subpackets requires:
$$5 \times 295\text{ ms} + 5 \times 35\text{ ms (guard)} = 1,650\text{ ms}$$
Because $1,650\text{ ms} > 1,000\text{ ms}$, **the 9.6 kbps channel saturates 100%**. Transmit queues explode, latency grows by seconds each epoch, the quiet window vanishes, and all rover telemetry is lost.

### 6.3 Closed-Loop Airtime Budgeting & Constellation Reservation ([`e220_pacer.py`](../scripts/e220_pacer.py))
To unlock the $+3.5\text{ dB}$ link margin of $9.6\text{ kbps}$ and $+7\text{ dB}$ margin of $4.8\text{ kbps}$ under varying satellite counts without dropping RTK fix or overflowing queues:
1. **Calibrated Wall-Time Estimator (`_estimate_frame_toa_ms()`):** Incorporates per-subpacket ToA, UART serialization time, multi-subpacket inter-frame guard (`>= 48 ms`), and `+8 ms` handover margin per extra subpacket (supporting `2` or `3+` subpackets if satellite count exceeds `400 B`).
2. **Starvation-Age Priority & Airtime Reservation (`_select_epoch_target_msm()`, `msm_epochs_since_tx`):** Tracks `last_len` and skipped-epoch age per constellation (`1074`, `1084`, `1094`, `1124`). If transmitting an earlier constellation (`1074`/`1084`) would push the epoch's priority starved constellation past the superframe cutoff, the earlier constellation yields cleanly in `0 ms` so no constellation is starved even as satellite counts grow.
3. **Fit-or-Skip Admission Control:** At `>= 9.6 kbps`, any MSM frame exceeding `cutoff_ms` (`700/740 ms` in Second 0, `850 ms` in Seconds `1..4`) is discarded immediately (`[SEC0 SKIP]` / `[BUDGET SKIP]`) rather than spilling across second boundaries. At `<= 4.8 kbps`, where single MSM frames exceed `850 ms`, Seconds `1..3` permit `1` lead MSM up to `1,750 ms` (after `1006 + 1230` or prior spillover) and `Second 4` caps spillover strictly at `1,680 ms` so TX never enters Second 0's `720..800 ms` uplink window, follow-on and non-MSM frames stay capped at `850 ms` (`sec_msm_count`), and a smaller secondary MSM (`1084`) is opportunistically admitted whenever `epoch_target_msm` cannot fit `cutoff_ms`. Only non-MSM station/bias frames (`1005/1006/1033/1230`) hold cleanly for the top of the next second.

### 6.4 Rate-Adaptive Truncation Timeouts ([`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp))
On the receiving rover, multi-subpacket frames (`201..400 B`) exhibit an inter-subpacket gap while the base E220 finishes airing Subpacket 1 before starting Subpacket 2. The sliding-window gatekeeper must scale its truncation timeout accordingly:

| Air Rate | Single-Subpacket ToA | Inter-Subpacket Gap | Truncation Timeout (`TRUNC_TIMEOUT_MS`) |
| :---: | :---: | :---: | :---: |
| **`19.2 kbps`** | `148 ms` | `~150 ms` | **`280 ms`** |
| **`9.6 kbps`** | `295 ms` | `~300 ms` | **`650 ms`** |
| **`4.8 kbps`** | `590 ms` | `~600 ms` | **`1,200 ms`** |

---

## 7. Live Mode 0 RSSI & Ambient Noise Floor Queries

### 7.1 Enabling Without Data Corruption
* Set **`REG1` Bit 5 = `1` (`0x20`)** and keep **`REG3` Bit 7 = `0`**.
* This enables on-demand Mode 0 register reads (`0x00` = ambient RF noise floor on `CHAN`; `0x01` = RSSI of last received packet) without appending extra bytes to payload data.

### 7.2 Undocumented 4-Byte vs. 5-Byte Firmware Reply
The datasheet claims `C0 C1 C2 C3 00 02` returns a 5-byte frame (`C1 00 02 <noise> <pkt>`). **Actual E220-900T30S firmware omits the length byte:**

| Query Sent in Mode 0 | Datasheet Reply | **Actual Silicon Reply** | Conversion |
| :--- | :--- | :--- | :--- |
| **`C0 C1 C2 C3 00 02`** | `C1 00 02 <noise> <pkt>` | **`C1 00 <noise> <pkt>` (4 B!)** | `dBm = int(u8) - 256` |
| **`C0 C1 C2 C3 00 01`** | `C1 00 01 <noise>` | **`C1 00 <noise>` (3 B!)** | `dBm = int(u8) - 256` |

Raw bytes lie in `0x80 .. 0xF5` (`>= 128`, representing `-128 dBm .. -11 dBm`).
* If `len == 4` and `buf[0..1] == C1 00` and `buf[2] >= 0x80`: 4-byte reply (`noise = buf[2] - 256`, `pkt = buf[3] - 256`).
* If `len == 5` and `buf[0..2] == C1 00 02`: 5-byte reply (`noise = buf[3] - 256`, `pkt = buf[4] - 256`).

### 7.3 Quiet-Window Synchronization (`v0.8.11` Superframe TDMA)
Never issue `C0 C1 C2 C3 00 02` blindly during active reception, during single-subpacket RF TX (`< s_tx_air_busy_until_ms`), or when a Mode 3 rate-switch / rollback is due:
1. **Second 0 Post-Uplink Gap (`sf_sec == 0`):** Poll strictly *after* the Rover's uplink TX finishes RF airing (`now >= s_tx_air_busy_until_ms` and `sf_ms` in `810..895 ms`), accommodating `48 B` uplink air time at all rates (`~60 ms` @ `19.2k`, `~105 ms` @ `9.6k`, `~168 ms` @ `4.8k`) and leaving a full `95 ms` reply margin before `.000 s`.
2. **Seconds 1..4 End-of-Second Micro-Slot (`sf_sec != 0`):** The Base (`e220_pacer.py v3.6.5`) enforces an `850.0 ms` downlink ceiling at `>= 9.6 kbps` (and `1` MSM up to `1,750 ms` in Sec 1..3 / `1,680 ms` in Sec 4 at `4.8 kbps`, with `850.0 ms` caps on follow-on/non-MSM frames). The Rover polls inside **`sf_ms = 855..895 ms`** (`E220_RSSI_POLL_INTERVAL_MS = 750 ms`, `last_rx_ms` idle `>= 25 ms`), requiring either `s_last_valid_msm_utc_sec == sf_utc_sec` (essential at `<= 5.0 kbps` where a single lead `1094` may span past `855 ms`) or `last_rx_ms` idle `>= 200 ms` at `> 5.0 kbps`. Also caps `idle_link_probe` at `sf_ms <= 880 ms`.
3. **Bounded Intercept, `95 ms` / `.990 s` Hard Expiry & Carrier-Contamination Filter:** Set `E220_RSSI_REPLY_TIMEOUT_MS = 95 ms` (accommodating the E220-900T30S's measured `70-85 ms` response latency observed in `test/_042`) and expire any active query at `sf_ms >= 990 ms` so queries never spill past `.000 s`. Gate Stage `0b` `0xC1 0x00` interception within `250 ms` of `last_rssi_poll_ms`, reject both `0x80` (`-128 dBm`) sentinel glitches and carrier-contaminated noise samples (`cand_noise_dbm > -65` within `8 dB` of `cand_pkt_dbm`), and bound `noise_raw` (`0x81..0xE4`) and `pkt_raw` (`0x80..0xF6`). Clear `isr_aux_fall_us = 0` when unmasking `e220_suppress_aux_isr`. Never mark `radio_online = false` or `rssi_valid = false` on poll timeout streak if RTCM bytes arrived within `10 s`.

---

## 8. RF Spectrum Selection & Near-Field Desense

1. **Antenna Relocation (15-20 dB Noise Reduction):**
   * The `E220-900T30S` has a sensitive `+14 dB` RX LNA. Mounting an antenna directly on a Raspberry Pi 4B exposes it to USB 3.0, HDMI, and switching regulator EMI (`-80 dBm` noise floor).
   * Relocating the antenna via a **4-foot (`1.2 m`) low-loss coaxial cable** away from digital electronics drops ambient noise to **`-96..-98 dBm`** at the Base and **`-114 dBm`** in field environments.
2. **Channel Selection (US 902-928 MHz):**
   * Consumer weather stations (e.g. Ecowitt `915.0 MHz`), power meters, and LoRaWAN crowd `902 - 916 MHz`.
   * **`CH 75` (`925.125 MHz`)** is ranked `#1` cleanest US ISM channel in empirical scans (`-96.2 dBm` average, `+10.125 MHz` clear of consumer devices).

---

## 9. Bi-Directional Half-Duplex LoRa Protocol & Telemetry Uplink

### 9.1 GPS-Disciplined 5-Second Superframe TDMA Architecture (`v0.8.6`)
Because both Base and Rover operate on the same frequency (`925.125 MHz`), downlink and uplink transmissions are deterministically scheduled against GPS UTC seconds (`sf_sec = utc_sec % 5`):

```text
5-Second Superframe (UTC_Second % 5):
  Seconds 1..4: [============ 100% Base RTCM3 Downlink (Zero Rover TX) ============]
  Second 0:     [=== Base Downlink (cutoff 700/740ms) ===|=== Rover Uplink Slot ===]
                                                          720..845ms (rate-adaptive)
```

* **Base Station (`e220_pacer.py v3.6.5`):** Enforces a closed-loop `850.0 ms` Seconds `1..4` ceiling at `>= 9.6 kbps` (and `1,750 ms` Sec 1..3 / `1,680 ms` Sec 4 caps for `1` MSM at `<= 4.8 kbps` with opportunistic smaller-MSM admission and `850 ms` follow-on caps), starvation-age constellation reservation + Fit-or-Skip MSM admission (`>= 52 ms` `<= 9.6 kbps` / `>= 40 ms` `>= 19.2 kbps` single-subpacket guards and `>= 48 ms` multi-subpacket guard), drops empty MSM frames (`<= 28 B`), and enforces Second 0 downlink cutoffs (`700 ms` @ `<= 9.6k`, `740 ms` @ `>= 19.2k`) before listening strictly for Rover uplink frames (`48 B` V3 telemetry with firmware version, or legacy `45 B` / `39 B`).
* **Rover (`e220_driver.cpp`):** Transmits its 48-byte binary telemetry frame (or 10-byte rate switch request) exclusively inside the GPS-disciplined Second 0 uplink window (`775..845 ms` @ `>= 19.2k`, `745..825 ms` @ `9.6k`, `720..800 ms` @ `4.8k`), waiting for E220 TX completion via `e220_wait_tx_complete(25, 300)`.

### 9.2 48-Byte Uplink Frame Specification (`RoverUplinkTelemetry_t`)

```text
Offset  Field                Type      Units / Encoding
-----------------------------------------------------------------------------
0..1    sync                 uint8[2]  0xAA, 0x55 (Preamble sync)
2       msg_id               uint8     0x01 (ROVER_MSG_ID_TELEMETRY)
3       payload_len          uint8     42 (Bytes following length field)
4..11   tip_lat              double    Ground-tip Latitude (WGS84 degrees)
12..19  tip_lon              double    Ground-tip Longitude (WGS84 degrees)
20..23  tip_height_m         float     Ground-tip Ellipsoidal Height (meters)
24..25  h_acc_mm             uint16    Horizontal Accuracy (millimeters)
26      fix_type             uint8     ANT1 Master Fix (0=None, 1=Single, 2=DGPS, 4=Fix, 5=Float)
27      ant2_fix_type        uint8     ANT2 Slave Moving-Baseline Fix (0=None, 4=Fix, 5=Float)
28..29  baseline_mm          uint16    Measured 3D Baseline (millimeters, 555mm nominal)
30..31  baseline_pitch_cdeg  int16     Crossbar Pitch (centidegrees, e.g. +20 = +0.20 deg)
32      heading_source       uint8     0=NONE, 1=DUAL_FIX, 2=DUAL_FLT, 3=WALK, 4=GYRO, 5=COG, 6=HOLD, 7=COMP
33      sats_master          uint8     Master Antenna satellites tracked
34      sats_slave           uint8     Slave satellites (bits [5:0]), b6=valid, b7=fixed
35..36  heading_cdeg         uint16    True Heading (centidegrees, 0..36000 -> 0.00..360.00 deg)
37      battery_pct          uint8     Battery State of Charge (0..100%, 255=USB Power)
38..39  battery_mv           uint16    Battery / VBUS Terminal Voltage (millivolts)
40      rover_rssi_dbm       int8      Last Base packet RSSI received at Rover (dBm)
41      rover_noise_dbm      int8      Ambient RF noise floor at Rover (dBm)
42      seq                  uint8     Rolling sequence counter (0..255)
43      fw_major             uint8     Rover Firmware Major Version (e.g. 0 for v0.8.6)
44      fw_minor             uint8     Rover Firmware Minor Version (e.g. 8 for v0.8.6)
45      fw_patch             uint8     Rover Firmware Patch Version (e.g. 6 for v0.8.6)
46..47  crc16                uint16    CRC16-CCITT across bytes 0..45
```

### 9.3 ANT2 Moving-Baseline Semantics
To prevent operator confusion between ANT1's base fix and ANT2's moving-baseline heading lock:
* **`SLV_FIX` (`4`):** Secondary antenna has integer carrier phase lock relative to ANT1 (`heading_source == HDG_SRC_DUAL_FIX`).
* **`SLV_FLT` (`5`):** Secondary antenna has float carrier phase lock relative to ANT1 (`heading_source == HDG_SRC_DUAL_FLT`).
* **`NONE` (`0`):** Secondary antenna baseline is rejected (e.g. sanity check failure `BL: 231.5cm`) or heading is operating in inertial/displacement fallback (`[HOLD]`, `[WALK]`, `[GYRO]`).
* Standalone `$GPGGAH` base fix quality is decoupled and does not override moving-baseline status.

### 9.4 FreeRTOS Dual-Core State Synchronization
* **Core 0 (`gnss_nav_task`):** Handles raw UART streaming, RTCM forwarding, and E220 half-duplex transmission.
* **Core 1 (`loopTask`):** Samples PMIC battery voltage, polls 6-DOF IMU, and executes `nav_heading_displacement_update()`.
* **Synchronization Rule:** `e220_feed_telemetry_state(&ui_state)` must be invoked on **Core 1** after sensor fusion, writing to a `portMUX_TYPE` spinlock-protected buffer snapshot. Feeding from Core 0 results in zero battery data and lost fallback heading tags.

---

## 10. Touchscreen Remote Air-Rate Switching Protocol

The system supports zero-credential, GPS-synchronized 3-rate switching (`19.2k -> 9.6k -> 4.8k -> 19.2k`) between Rover and Base:

```mermaid
sequenceDiagram
    autonumber
    participant Rover as Rover UI (ESP32-S3)
    participant Base as Base Pacer (Raspberry Pi)

    Note over Rover: User taps [LORA] to cycle 19.2k / 9.6k / 4.8k<br/>Pre-TX queue allows rapid cycling or cancel
    Rover->>Base: CMD_REQ_RADIO_SWITCH (10B, Second 0 Uplink, req_id)
    Note over Base: Schedules 4-ACK train@T0+1, Switch@T0+2<br/>Holds RTCM downlink during Second 1
    Base->>Rover: ACK_RADIO_SWITCH #1..#4 @ T0+1 80/260/440/620ms
    Note over Rover: Superframe-aligned switch at Second 2 (T0+2)
    Note over Base: At T0+2 0ms: Reconfigures E220 Mode 3 registers
    Note over Rover: At T0+2 0ms: Reconfigures E220 Mode 3 registers
    Note over Rover,Base: Both arm 60s rollback watchdog until first valid post-switch traffic
```

### 10.1 Binary Frame Formats
* **Request (`CMD_REQ_RADIO_SWITCH`, 10 Bytes):**
  `0xAA 0x55 | 0x02 | len=0x04 | target_rate_code(1) | target_channel(1) | req_id(2) | crc16(2)`
  * Rate codes: `0x02`=2.4k, `0x03`=4.8k, `0x04`=9.6k, `0x05`=19.2k, `0x06`=38.4k, `0x07`=62.5k.
* **Acknowledge (`ACK_RADIO_SWITCH`, 11 Bytes):**
  `0xAA 0x55 | 0x82 | len=0x05 | req_id(2) | status=0x00 | exec_delay_ms(2, 1000ms) | crc16(2)`

### 10.2 Dynamic Pre-Switch Fallback & Automatic Rollback Watchdog
Both Base and Rover arm a **60-second rollback watchdog** upon switching registers at Second 2 (`s_rate_switch_done_ms` on Rover). Because the UI supports 3 air rates (`19.2k / 9.6k / 4.8k`), both nodes record the **pre-switch verified working air rate** (`s_fallback_air_rate` on Rover, `self.fallback_air_rate` on Base) immediately before executing a switch (and refresh it once post-switch traffic is verified). The Rover watchdog disarms upon receiving a valid `CRC24Q` RTCM frame strictly *after* the switch timestamp; the Base disarms upon receiving valid rover telemetry after `watchdog_rate_switch_time`. Otherwise, both nodes revert cleanly to their recorded pre-switch fallback rate (e.g., reverting `4.8k` if `4.8k -> 19.2k` fails, instead of hardcoding `19.2k`).

---

## 11. Base Station Driver Architecture (`e220_pacer.py`)

### 11.1 Key Daemon Responsibilities
1. **RTCM3 Pacing Engine:** Ingests TCP stream (`127.0.0.1:5016`), pacing single-subpacket frames by calibrated ToA and two-subpacket frames by AUX edge detection.
2. **Adaptive Constellation Interleaver:** Automatically engages at $\le 9.6\text{ kbps}$ to alternate secondary constellations and protect the quiet window.
3. **Telemetry Processor & Storage:**
   * **Terminal Heartbeat:** Outputs clean 7-bit ASCII lines (using `deg` units):
     ```text
     [ROVER TEL #042] FW:v0.8.6 | ANT1:RTK_FIX (28M) | ANT2:SLV_FIX (26S, BL:55.4cm, P:+0.2deg) | Hdg:142.6deg [DUAL_FIX] | Bat:85% (4.02V) | RSSI:-84dBm | Pos:43.7319105, -70.6237371, 12.45m
     ```
   * **RAM State (`/run/rtk_rover_status.json`):** Atomic tmpfs writes in RAM (zero MicroSD card wear) for local web dashboard integration.
   * **Rolling Daily CSV (`~/rover_logs/rover_YYYY-MM-DD.csv`):** Auto-detects interactive user home directories in `/home/` when running under systemd `User=root`, creates directories with `0777` permissions, compresses yesterday's logs to `.csv.gz`, and enforces 30-day retention with a 50 MB hard FIFO cap.

---

## 12. Store-and-Verify Sliding-Window Gatekeeper Pattern

Never pass raw E220 UART output directly to a downstream GNSS receiver or parser:
1. In fringe conditions, Subpacket 1 (`200 B`) may pass LoRa CRC while Subpacket 2 (`140 B`) drops. Raw passthrough causes downstream parsers to swallow valid bytes from subsequent frames, destroying intact data.
2. **Implementation ([`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp)):**
   * Stage incoming bytes in a 1,536-byte ring buffer.
   * Validate preambles (`0xD3` for RTCM3, `0xAA 0x55` for telemetry).
   * Enforce rate-adaptive truncation timeouts (`280 ms` @ 19.2k, `650 ms` @ 9.6k).
   * Verify CRC (`CRC24Q` for RTCM3, `CRC16` for telemetry).
   * Forward only 100% complete, CRC-passing frames atomically to the GNSS receiver. On failure, shift by 1 byte to rescue following frames.

---

## 13. Rapid Field Troubleshooting Matrix

| Observed Symptom | Root Cause | Verification & Fix |
| :--- | :--- | :--- |
| **`E220 READ OK` at boot, strong RSSI, but `0` frames `> 200 B` succeed; packets arrive `>= 950 ms` apart.** | **Open `M0` connection.** Internal pull-up holds `M0 = 1`. Radio enters Mode 1 (500 ms WOR sleep) instead of Mode 0. | Measure `M0` pin voltage during run: if `3.3 V` instead of `0.0 V`, repair connection. |
| **Pacer stalls 500 ms on every small (`<= 200 B`) frame; backlog explodes.** | Waiting for `AUX` to fall `LOW` on `<= 200 B` writes. Single packets load into `LLCC68` in `< 0.5 ms` without holding `AUX` `LOW`. | Pace `<= 200 B` frames by `ToA(N) + 35 ms` without blocking on `AUX` falling edge (Section 5.2). |
| **`201..400 B` frames lose Subpacket 2 when followed by another frame.** | Sending next frame immediately when `AUX` rises `HIGH` (`AUX` rises when Subpacket 2 *starts* airing, not when it ends). | After `AUX` rises `HIGH`, sleep `ToA(N - 200) + 35 ms` before writing the next frame (Section 5.3). |
| **RTCM `1074` (`201..253 B`) loses Subpacket 2 (`[RTCM DROP TRUNC] 200B`) at `4.8k` or `9.6k` followed by missing `1094` head.** | At `<= 9.6 kbps`, E220 raises `AUX` `HIGH` `100-147 ms` *before* Subpacket 1 (`200 B`) finishes airing, causing `send_packet()` to transmit the next frame while Subpacket 2 is still on air. | Upgrade to `e220_pacer.py v3.6.1` enforcing `max(ToA(tail), ToA(200)+ToA(tail)-elapsed) + Guard` (Section 5.3). |
| **Mode 0 RSSI query at `4.8k` pulls `AUX` `LOW` for `100-130 ms` and replies after `166 ms` (`[E220 RX BURST] 5 B`).** | `C0 C1 C2 C3 00 02` was written right after `48 B` uplink `flush()` while the single-subpacket TX was still airing on `LLCC68`. | Track `s_tx_air_busy_until_ms` after uplink TX and use `180 ms` RSSI timeout in `v0.8.7` (Section 5.2 & 7.3). |
| **`[E220 WRITE FAIL] got 3 bytes` (`FF FF FF`) or `got 4 bytes` on rate switch, followed immediately by `E220 READ OK`.** | UART switched to `9600` baud *before* raising `M0/M1` (or `M0/M1` bounced at `9600`), injecting a framing glitch into Mode 3 parser. | Upgrade to `v0.8.9+` + `e220_pacer.py v3.6.2+` (asserts `M0/M1=1` before `9600` baud switch, runs `0xC1` sync read, avoids bounce on retries `1..3`). |
| **`[WARN] Shed X stale queued MSM frames` at `9.6k`/`19.2k`, or `Noise:-33 dBm` / `Subpacket 2` orphan tails at `4.8k`.** | Open-loop `1074+1094` epoch exceeds `850 ms` cutoff or `4.8k` `end_of_sec_gap` polls mid-packet without checking `s_last_valid_msm_utc_sec`. | Upgrade to `v0.8.10` + `e220_pacer.py v3.6.4` (closed-loop Fit-or-Skip + constellation reservation, `4.8k` `1,750 ms`/`1,680 ms` Sec 1..4 caps & opportunistic Sec 0 fallback, `>= 48 ms` guard, `s_last_valid_msm_utc_sec` & `885..935 ms` / `65 ms` / `.985 s` RSSI gate). |
| **`9.6k -> 4.8k` switch fails (`ACK #1 @ 0 ms` missed, single backup drops) or `[E220 RX BURST] 5 B` spills across `.000 s`.** | Base `0 ms` ACK hits NMEA-lagged Rover `Second 0` turnaround; `65 ms` RSSI timeout is shorter than `70-85 ms` E220 reply; `35 ms` `Regime A` guard under-paces at `<= 9.6k`. | Upgrade to `v0.8.11` + `e220_pacer.py v3.6.5` (`80/260/440/620 ms` 4-ACK `Second 1` train, `52 ms` `Regime A` guard @ `<= 9.6k`, `855..895 ms` / `95 ms` / `.990 s` RSSI window). |
| **At 9.6 kbps, Rover loses RTK Fix and Telemetry Heartbeats stop.** | Channel saturated by 4-constellation 650 B/s stream without interleaving. | Enable Adaptive Constellation Interleaving in `e220_pacer.py v3.3.1+` (Section 6.3). |
| **Subpacket 2 of 340B RTCM frames dropped at 9.6 kbps.** | Receiver truncation timeout too short (`260 ms`). | Scale gatekeeper timeout to `650 ms` at 9.6 kbps (Section 6.4). |
| **Telemetry shows `Bat:0% (0.00V)` and `Hdg: 0.0deg [NONE]` while Rover screen is valid.** | Telemetry state fed from Core 0 before Core 1 attaches PMIC battery and woods fallback heading. | Move `e220_feed_telemetry_state(&ui_state)` to Core 1 post-sensor-fusion (Section 9.4). |
| **Telemetry shows `ANT2:SLV_FIX` while Rover header says `HDG HLD`.** | `ant2_fix_type` populated from standalone `$GPGGAH` instead of moving baseline. | Align `ant2_fix_type` strictly with `heading_source` (Section 9.3). |
| **`Â°` characters appear in console logs.** | Non-UTF-8 terminal interpreting 2-byte UTF-8 degree symbol (`0xC2 0xB0`). | Use 7-bit ASCII `deg` in scripts and firmware logging (Section 11.1). |
| **`ls ~/rover_logs` returns `No such file or directory` on Base Pi.** | Service runs as root; logs created in `/root/rover_logs`. | Update to `e220_pacer.py v3.3.1` (auto-detects user `/home` and links directory). |
