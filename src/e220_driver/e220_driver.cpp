#include "e220_driver.h"
#include "rover_telemetry.h"
#include "../../user_config.h"
#include "../um982_driver/um982_driver.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/rtcm_logger.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <string.h>

static E220LinkStats_t e220_stats = {};
static volatile bool pending_cfg_read  = false;
static volatile bool pending_cfg_write = false;

// =========================================================================
// Bi-Directional Uplink Telemetry & Remote Air-Rate Switching State
// =========================================================================
static RoverUIData_t     s_latest_telemetry_state = {};
static bool              s_has_telemetry_state    = false;
static portMUX_TYPE      s_telem_mux              = portMUX_INITIALIZER_UNLOCKED;
static uint32_t          s_last_uplink_tx_ms      = 0;
static uint32_t          s_last_uplink_utc_sec    = 0xFFFFFFFFU;
static uint8_t           s_uplink_seq             = 0;

static volatile bool     s_rate_switch_requested  = false;
static float             s_requested_rate_kbps    = 0.0f;
static uint16_t          s_rate_switch_req_id     = 0;
static uint32_t          s_rate_switch_req_tx_ms  = 0;
static uint32_t          s_rate_switch_req_start_ms = 0;
static bool              s_rate_switch_ack_received = false;
static uint32_t          s_rate_switch_exec_ms    = 0;
static float             s_active_air_rate        = 19.2f;
static float             s_fallback_air_rate      = 19.2f;
static uint32_t          s_rate_switch_done_ms    = 0;
static uint32_t          s_last_valid_rtcm_ms     = 0;
static uint32_t          s_last_valid_msm_utc_sec = 0xFFFFFFFFU;
static bool              s_switched_requires_watchdog = false;
static uint32_t          s_tx_air_busy_until_ms   = 0;

static float e220_decode_air_rate_from_reg0(uint8_t reg0)
{
    static const float kAirRates[8] = {2.4f, 2.4f, 2.4f, 4.8f, 9.6f, 19.2f, 38.4f, 62.5f};
    return kAirRates[reg0 & 0x07U];
}

static uint32_t e220_estimate_tx_air_ms(size_t n_bytes, float air_rate_kbps)
{
    // Stage 1 silicon-calibrated TX air time (test/_045.._047) + 25ms TX->RX turnaround margin
    float air_ms;
    if (air_rate_kbps <= 3.0f) {
        float eff_bps = air_rate_kbps * 1000.0f * 0.60f;
        if (eff_bps < 1000.0f) eff_bps = 1000.0f;
        air_ms = 22.0f + ((float)(n_bytes + 16U) * 8000.0f) / eff_bps;
    } else if (air_rate_kbps <= 5.0f) {
        air_ms = 70.0f + 1.87f * (float)n_bytes;
    } else if (air_rate_kbps <= 10.0f) {
        air_ms = 45.0f + 1.19f * (float)n_bytes;
    } else if (air_rate_kbps <= 20.0f) {
        air_ms = 29.0f + 0.80f * (float)n_bytes;
    } else {
        float eff_bps = air_rate_kbps * 1000.0f * 0.68f;
        air_ms = 16.0f + ((float)(n_bytes + 16U) * 8000.0f) / eff_bps;
    }
    return (uint32_t)(air_ms + 25.0f);
}

// =========================================================================
// Mode 0 AUX (IO21) Hardware Edge Counter ISR
// Counts E220 Mode 0 wakeup/UART-output falling edges and measures LOW width.
// Suppressed during Mode 3 register config and local C0 C1 C2 C3 RSSI polls.
// =========================================================================
static volatile bool     e220_suppress_aux_isr = true;
static volatile uint32_t isr_aux_fall_count    = 0;
static volatile int64_t  isr_aux_fall_us       = 0;
static volatile uint32_t isr_aux_last_low_us   = 0;

static void IRAM_ATTR e220_aux_isr(void)
{
    if (e220_suppress_aux_isr) {
        return;
    }
    int level = gpio_get_level((gpio_num_t)ROVER_PIN_E220_AUX);
    int64_t now_us = esp_timer_get_time();
    if (level == 0) {
        isr_aux_fall_count = isr_aux_fall_count + 1;
        isr_aux_fall_us = now_us;
    } else if (isr_aux_fall_us > 0) {
        int64_t dt = now_us - isr_aux_fall_us;
        if (dt >= 0 && dt < 10000000LL) {
            isr_aux_last_low_us = (uint32_t)dt;
        }
        isr_aux_fall_us = 0;
    }
}

// =========================================================================
// Sliding-Window Store-and-Verify RTCM3 Gatekeeper State
// Stages incoming Mode 0 bytes, validates 0xD3 header + 24-bit CRC24Q == 0,
// and forwards ONLY verified complete RTCM3 frames atomically to UM982 COM2.
// =========================================================================
#define RTCM_STAGE_CAPACITY      1536U

static uint8_t  rx_stage[RTCM_STAGE_CAPACITY];
static size_t   rx_stage_len            = 0;
static uint32_t last_stage_rx_ms        = 0;
static uint32_t last_1074_rx_ms         = 0;
static uint32_t last_terminal_msm_rx_ms = 0;
static bool     rtcm_in_resync          = false;

static inline uint32_t e220_get_trunc_timeout_ms(void) {
    // Subpacket-2 inter-byte gap ceiling up to 400B frames:
    //   4.8k: 847.0 - 442.8 = 404.2ms -> 480ms cleanly quarantines broken heads within 0.5s
    //   9.6k: 527.2 - 282.8 = 244.4ms -> 380ms
    //  19.2k: 336.7 - 186.7 = 150.0ms -> 280ms
    if (s_active_air_rate <= 5.0f)  return 480U;
    if (s_active_air_rate <= 10.0f) return 380U;
    return 280U;
}

static bool e220_get_superframe_clock(uint32_t now_ms,
                                      uint32_t *utc_sec_out,
                                      uint32_t *superframe_sec_out,
                                      uint32_t *ms_in_sec_out)
{
    if (!gps_time_sync.time_synced) {
        return false;
    }
    uint32_t elapsed        = now_ms - gps_time_sync.last_sync_millis;
    uint32_t current_sec    = (gps_time_sync.last_utc_sec + (elapsed / 1000U)) % 86400U;
    uint32_t superframe_sec = current_sec % 5U;
    uint32_t ms_in_sec      = elapsed % 1000U;
    if (utc_sec_out)        *utc_sec_out        = current_sec;
    if (superframe_sec_out) *superframe_sec_out = superframe_sec;
    if (ms_in_sec_out)      *ms_in_sec_out      = ms_in_sec;
    return true;
}

extern "C" bool e220_is_uplink_slot_active(uint32_t now_ms)
{
    uint32_t superframe_sec = 0;
    uint32_t ms_in_sec      = 0;
    if (!e220_get_superframe_clock(now_ms, NULL, &superframe_sec, &ms_in_sec)) {
        return false;
    }
    if (superframe_sec != 0U) {
        return false;
    }
    // Arch 2B consolidated Second 0 uplink quiet slot (aligned to Base DL cutoffs 450/620/680ms):
    // Leaves >= 160ms+ after uplink air TX inside Second 0 for 1x collision-free Mode 0 RSSI query.
    if (s_active_air_rate <= 5.0f) {
        return (ms_in_sec >= 500U && ms_in_sec <= 660U);
    }
    if (s_active_air_rate <= 10.0f) {
        return (ms_in_sec >= 670U && ms_in_sec <= 760U);
    }
    return (ms_in_sec >= 730U && ms_in_sec <= 810U);
}

// =========================================================================
// Non-Blocking Mode 0 Live RSSI / Ambient Noise Register Poll State
// Queries C0 C1 C2 C3 00 02 once per 5-second macro-cycle strictly inside
// the Second 0 post-uplink quiet window (no Seconds 1..4 downlink polling).
// Parses both 4-byte (C1 00 <noise> <pkt>) and 5-byte (C1 00 02 <noise> <pkt>)
// E220-900T30S replies (requires REG1 Bit 5 = 0x20).
// =========================================================================
#define E220_RSSI_POLL_INTERVAL_MS  750U
#define E220_RSSI_IDLE_POLL_MS      2000U
#define E220_RSSI_REPLY_TIMEOUT_MS  180U

static bool     rssi_query_active       = false;
static uint32_t last_rssi_poll_ms       = 0;
static uint32_t s_last_rssi_poll_utc_sec = 0xFFFFFFFFU;
static uint32_t rssi_query_start_ms     = 0;
static uint8_t  rssi_reply_buf[5]    = {0};
static uint8_t  rssi_reply_len       = 0;
static uint8_t  s_rssi_timeout_streak = 0;
static uint32_t s_aux_low_since_ms    = 0;
static bool     s_was_radio_online    = false;

// Burst telemetry state
static uint32_t burst_bytes          = 0;
static uint32_t burst_last_byte_ms   = 0;
static uint32_t last_burst_end_ms    = 0;
static uint32_t last_burst_aux_edges = 0;

static uint32_t rtcm_crc24q_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 16);
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

static inline bool rtcm_is_plausible_msg_type(uint16_t msg_type)
{
    return (msg_type >= 1001U && msg_type <= 1300U) ||
           (msg_type >= 4001U && msg_type <= 4095U);
}

static bool e220_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (digitalRead(ROVER_PIN_E220_AUX) == LOW) {
        if (millis() - start >= timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

static void e220_wait_tx_complete(uint32_t fall_wait_ms, uint32_t rise_timeout_ms)
{
    uint32_t t0 = millis();
    while (digitalRead(ROVER_PIN_E220_AUX) == HIGH) {
        if (millis() - t0 >= fall_wait_ms) {
            break;
        }
        delayMicroseconds(500);
    }
    e220_wait_aux_high(rise_timeout_ms);
    isr_aux_fall_us = 0;
}

extern "C" void e220_set_mode(uint8_t m0, uint8_t m1)
{
    e220_suppress_aux_isr = true;
    rssi_query_active     = false;
    rssi_reply_len        = 0;

    e220_wait_aux_high(500);
    digitalWrite(ROVER_PIN_E220_M0, m0 ? HIGH : LOW);
    digitalWrite(ROVER_PIN_E220_M1, m1 ? HIGH : LOW);
    e220_stats.mode_m0 = m0 ? 1 : 0;
    e220_stats.mode_m1 = m1 ? 1 : 0;
    vTaskDelay(pdMS_TO_TICKS(100));
    e220_wait_aux_high(500);
    vTaskDelay(pdMS_TO_TICKS(60));

    if (m0 == 0 && m1 == 0) {
        isr_aux_fall_us       = 0;
        e220_suppress_aux_isr = false;
    }
}

extern "C" void e220_driver_init(void)
{
    pinMode(ROVER_PIN_E220_AUX, INPUT_PULLUP);
    pinMode(ROVER_PIN_E220_RST, OUTPUT);
    pinMode(ROVER_PIN_E220_M0,  OUTPUT);
    pinMode(ROVER_PIN_E220_M1,  OUTPUT);

    digitalWrite(ROVER_PIN_E220_RST, HIGH);

    // Probe IO41/IO42 with weak pull-down before Serial0.begin() to verify physical E220 TX line drive
    pinMode(ROVER_PIN_E220_TXD, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_E220_RXD, INPUT_PULLDOWN);
    delay(2);
    Serial.printf("[E220 LINE PROBE] IO41(ESP_RX/E220_TXD)=%d | IO42(ESP_TX/E220_RXD)=%d\n",
                  digitalRead(ROVER_PIN_E220_TXD),
                  digitalRead(ROVER_PIN_E220_RXD));

    // Boot directly into Mode 0 (Transparent Mode: M0=0, M1=0) at configured UART baud (default 115200 8N1)
    // Note: E220_TXD (IO41) is ESP32 RX; E220_RXD (IO42) is ESP32 TX
    e220_set_mode(0, 0);
    Serial0.setRxBufferSize(4096);
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    Serial0.begin(baud, SERIAL_8N1, ROVER_PIN_E220_TXD, ROVER_PIN_E220_RXD);

    attachInterrupt(digitalPinToInterrupt(ROVER_PIN_E220_AUX), e220_aux_isr, CHANGE);
    e220_stats.aux_pin_high = (digitalRead(ROVER_PIN_E220_AUX) == HIGH);

    s_active_air_rate = (current_settings.e220_air_rate_kbps > 0.0f) ? current_settings.e220_air_rate_kbps : 19.2f;
    s_fallback_air_rate = s_active_air_rate;
    e220_stats.active_air_rate_kbps = s_active_air_rate;
    e220_stats.rate_switch_pending = false;
}

extern "C" void request_e220_config_read(void)
{
    pending_cfg_read = true;
}

extern "C" void request_e220_config_write(void)
{
    pending_cfg_write = true;
}

static uint8_t e220_build_reg0_from_settings(void)
{
    // Bits [7:5] = UART baud (000=1200, 001=2400, 010=4800, 011=9600, 100=19200, 101=38400, 110=57600, 111=115200)
    // Bits [4:3] = 00 (8N1)
    uint8_t uart_bits = 0xE0; // Default 115200 bps (0b111 << 5)
    uint32_t baud = current_settings.e220_uart_baud;
    if (baud >= 115200)     uart_bits = 0xE0; // 115200
    else if (baud >= 57600) uart_bits = 0xC0; // 57600
    else if (baud >= 38400) uart_bits = 0xA0; // 38400
    else if (baud >= 19200) uart_bits = 0x80; // 19200
    else if (baud >= 9600)  uart_bits = 0x60; // 9600
    else if (baud >= 4800)  uart_bits = 0x40; // 4800
    else if (baud >= 2400)  uart_bits = 0x20; // 2400
    else                    uart_bits = 0x00; // 1200

    uint8_t air_bits = 0x06; // Default 19.2 kbps (0b101)
    float rate = current_settings.e220_air_rate_kbps;
    if (rate < 3.0f)       air_bits = 0x02; // 2.4 kbps
    else if (rate < 7.0f)  air_bits = 0x03; // 4.8 kbps
    else if (rate < 14.0f) air_bits = 0x04; // 9.6 kbps
    else if (rate < 28.0f) air_bits = 0x05; // 19.2 kbps
    else if (rate < 50.0f) air_bits = 0x06; // 38.4 kbps
    else                   air_bits = 0x07; // 62.5 kbps
    return (uint8_t)(uart_bits | air_bits);
}

static uint8_t e220_build_reg1_from_settings(void)
{
    // Bits [7:6] = Sub-packet size (00=200B, 01=128B, 10=64B, 11=32B)
    uint8_t pkt_bits = 0x00; // Default 200 bytes
    uint16_t pkt = current_settings.e220_subpacket_bytes;
    if (pkt <= 32)       pkt_bits = 0xC0;
    else if (pkt <= 64)  pkt_bits = 0x80;
    else if (pkt <= 128) pkt_bits = 0x40;
    else                 pkt_bits = 0x00; // 200 bytes

    // Bit 5 (0x20) = Enable Mode 0 Ambient Noise & Last Packet RSSI Register (C0 C1 C2 C3)
    // Note: Does NOT append extra bytes to RX UART frames (unlike REG3 Bit 7).
    const uint8_t rssi_reg_bit = 0x20;

    // Bits [1:0] = TX Power on E220-900T30S (00=30dBm, 01=27dBm, 10=24dBm, 11=21dBm)
    uint8_t pwr_bits = 0x03; // Default 21 dBm (Level 3)
    uint8_t pwr = current_settings.e220_tx_power_dbm;
    if (pwr >= 29)      pwr_bits = 0x00; // 30 dBm (Level 0)
    else if (pwr >= 26) pwr_bits = 0x01; // 27 dBm (Level 1)
    else if (pwr >= 23) pwr_bits = 0x02; // 24 dBm (Level 2)
    else                pwr_bits = 0x03; // 21 dBm (Level 3)

    return (uint8_t)(pkt_bits | rssi_reg_bit | pwr_bits);
}

static uint8_t e220_build_reg3_from_settings(void)
{
    // Bit 7 = 0 (No RSSI byte appended to RX data), Bit 6 = 0 (Transparent Mode), Bit 4 = LBT Enable
    return current_settings.e220_lbt_enable ? 0x10 : 0x00;
}

static void e220_log_decoded_registers(const char *tag)
{
    static const uint32_t kUartBauds[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    static const float    kAirRates[8]  = {2.4f, 2.4f, 2.4f, 4.8f, 9.6f, 19.2f, 38.4f, 62.5f};
    static const uint16_t kPktSizes[4]  = {200, 128, 64, 32};
    static const uint8_t  kTxPowers[4]  = {30, 27, 24, 21};

    uint8_t r0 = e220_stats.cfg_reg0;
    uint8_t r1 = e220_stats.cfg_reg1;
    uint8_t ch = e220_stats.cfg_channel;
    uint8_t r3 = e220_stats.cfg_reg3;

    uint32_t baud = kUartBauds[(r0 >> 5) & 0x07];
    float    air  = e220_decode_air_rate_from_reg0(r0);
    (void)kAirRates;
    uint16_t pkt  = kPktSizes[(r1 >> 6) & 0x03];
    uint8_t  pwr  = kTxPowers[r1 & 0x03];
    float    freq = 850.125f + (float)ch;

    uint8_t exp_r0 = e220_build_reg0_from_settings();
    uint8_t exp_r1 = e220_build_reg1_from_settings();
    uint8_t exp_ch = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
    uint8_t exp_r3 = e220_build_reg3_from_settings();

    e220_stats.config_matched = (r0 == exp_r0 && r1 == exp_r1 && ch == exp_ch && r3 == exp_r3);

    Serial.printf("[%s] Regs: ADDH=0x%02X ADDL=0x%02X REG0=0x%02X REG1=0x%02X CHAN=0x%02X(%u) REG3=0x%02X\n",
                  tag, e220_stats.cfg_addr_high, e220_stats.cfg_addr_low, r0, r1, ch, ch, r3);
    Serial.printf("[%s] Decoded: UART=%lu 8N1 | Air=%.1fk | Pkt=%uB | Pwr=%udBm | RSSI_Reg=%s | Freq=%.3fMHz (CH%u) | Mode=%s | Match=%s\n",
                  tag, (unsigned long)baud, air, pkt, pwr,
                  (r1 & 0x20) ? "ON" : "OFF",
                  freq, ch,
                  (r3 & 0x40) ? "Fixed" : "Transparent",
                  e220_stats.config_matched ? "YES" : "NO");
}

/**
 * Probe Mode 3 register read (0xC1 0x00 0x06) to drain and align the E220 UART
 * after entering configuration mode before issuing an EEPROM write command.
 */
static bool e220_mode3_sync_read(void)
{
    const uint8_t read_cmd[3] = { 0xC1U, 0x00U, 0x06U };
    while (Serial0.available()) Serial0.read();
    Serial0.write(read_cmd, sizeof(read_cmd));
    Serial0.flush();

    uint8_t raw[24] = {0};
    size_t idx = 0;
    uint32_t start = millis();
    uint32_t last_rx = start;

    while (millis() - start < 180U && idx < sizeof(raw)) {
        if (Serial0.available()) {
            raw[idx++] = (uint8_t)Serial0.read();
            last_rx = millis();
            for (size_t k = 0; k + 9U <= idx; ++k) {
                if (raw[k] == 0xC1U && raw[k + 1] == 0x00U && raw[k + 2] == 0x06U) {
                    return true;
                }
            }
        } else {
            if ((idx >= 9U) || (idx >= 3U && (millis() - last_rx >= 35U))) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    return false;
}

static void e220_exit_mode3_to_mode0(uint32_t baud)
{
    // Exit Mode 3 -> Mode 0 BEFORE switching UART baud back to operational 115200 bps,
    // waiting 25 ms before baud change, 55 ms after, and draining stale Mode 3 RX bytes.
    e220_set_mode(0, 0);
    vTaskDelay(pdMS_TO_TICKS(25));
    Serial0.updateBaudRate(baud);
    vTaskDelay(pdMS_TO_TICKS(55));
    e220_wait_aux_high(500);
    while (Serial0.available()) Serial0.read();
}

static bool e220_probe_mode0_liveness(uint32_t baud)
{
    // Verify the E220 responds to Mode 0 commands at operational baud; if wedged in Mode 3, bounce once.
    if ((e220_stats.cfg_reg1 & 0x20U) == 0U) {
        return true;
    }
    static const uint8_t kRssiCmd[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
    for (uint8_t probe_try = 0; probe_try < 2U; ++probe_try) {
        vTaskDelay(pdMS_TO_TICKS(5));
        while (Serial0.available()) Serial0.read();
        e220_suppress_aux_isr = true;
        Serial0.write(kRssiCmd, sizeof(kRssiCmd));
        Serial0.flush();

        uint8_t raw[16] = {0};
        size_t idx = 0;
        uint32_t t0 = millis();
        bool alive = false;
        while (millis() - t0 < 80U && idx < sizeof(raw)) {
            if (Serial0.available()) {
                raw[idx++] = (uint8_t)Serial0.read();
                for (size_t k = 0; k + 5U <= idx; ++k) {
                    if (raw[k] == 0xC1U && raw[k + 1] == 0x00U && raw[k + 2] == 0x02U) {
                        alive = true;
                        break;
                    }
                }
                if (alive || raw[0] == 0xD3U) {
                    alive = true;
                    break;
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        e220_suppress_aux_isr = false;
        isr_aux_fall_us = 0;
        if (alive) {
            return true;
        }
        if (probe_try == 0U) {
            Serial.printf("[E220 MODE0 WARN] Mode 0 liveness probe timed out (%u B); bouncing Mode 3 -> Mode 0...\n",
                          (unsigned)idx);
            e220_set_mode(1, 1);
            Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
            vTaskDelay(pdMS_TO_TICKS(40));
            e220_exit_mode3_to_mode0(baud);
        }
    }
    Serial.println("[E220 MODE0 WARN] Mode 0 liveness probe did not answer after bounce");
    return false;
}

extern "C" bool e220_write_default_config_registers(void)
{
    // Cancel any active Mode 0 RSSI intercept and wait for any in-flight RF TX before Mode 3
    rssi_query_active     = false;
    rssi_reply_len        = 0;
    e220_suppress_aux_isr = true;
    isr_aux_fall_us       = 0;

    uint32_t air_wait_start = millis();
    while ((int32_t)(millis() - s_tx_air_busy_until_ms) < 0) {
        if (millis() - air_wait_start >= 250U) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    e220_wait_aux_high(300);
    Serial0.flush();

    uint8_t reg0 = e220_build_reg0_from_settings();
    uint8_t reg1 = e220_build_reg1_from_settings();
    uint8_t chan = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
    uint8_t reg3 = e220_build_reg3_from_settings();

    Serial.printf("[E220 CFG] Writing target registers: REG0=0x%02X REG1=0x%02X CHAN=0x%02X(%u) REG3=0x%02X...\n",
                  reg0, reg1, chan, chan, reg3);

    uint8_t cmd[9] = {
        0xC0, 0x00, 0x06,
        ROVER_E220_TARGET_ADDH,
        ROVER_E220_TARGET_ADDL,
        reg0,
        reg1,
        chan,
        reg3
    };

    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;

    // Enter Mode 3 BEFORE switching UART baud so the E220 is off 115200-baud Mode 0 before ESP32 changes to 9600
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(60));
    e220_mode3_sync_read();

    bool ok = false;
    for (uint8_t attempt = 1; attempt <= 4U && !ok; ++attempt) {
        if (attempt == 4U) {
            // Last-resort clean Mode 0 -> Mode 3 bounce with matching baud rates to recover a wedged parser
            e220_set_mode(0, 0);
            Serial0.updateBaudRate(baud);
            vTaskDelay(pdMS_TO_TICKS(30));
            while (Serial0.available()) Serial0.read();
            e220_set_mode(1, 1);
            Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
            vTaskDelay(pdMS_TO_TICKS(60));
            e220_mode3_sync_read();
        } else {
            // Stay in Mode 3 between initial attempts to avoid M0/M1 bounce framing glitches
            vTaskDelay(pdMS_TO_TICKS(35));
        }
        while (Serial0.available()) Serial0.read();

        Serial0.write(cmd, sizeof(cmd));
        Serial0.flush();

        uint8_t raw[24] = {0};
        size_t idx = 0;
        uint32_t start = millis();
        uint32_t last_rx = start;
        bool matched = false;
        size_t match_off = 0;

        while (millis() - start < 280U && idx < sizeof(raw)) {
            if (Serial0.available()) {
                raw[idx++] = (uint8_t)Serial0.read();
                last_rx = millis();
                if (idx >= 9U) {
                    for (size_t k = 0; k + 9U <= idx; ++k) {
                        if ((raw[k] == 0xC1U || raw[k] == 0xC0U) &&
                            raw[k + 1] == 0x00U &&
                            raw[k + 2] == 0x06U &&
                            raw[k + 5] == reg0 &&
                            raw[k + 6] == reg1 &&
                            raw[k + 7] == chan &&
                            raw[k + 8] == reg3) {
                            matched = true;
                            match_off = k;
                            break;
                        }
                    }
                    if (matched) {
                        break;
                    }
                }
            } else {
                if (idx >= 3U && (millis() - last_rx >= 35U)) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        if (matched) {
            e220_stats.cfg_addr_high     = raw[match_off + 3];
            e220_stats.cfg_addr_low      = raw[match_off + 4];
            e220_stats.cfg_reg0          = raw[match_off + 5];
            e220_stats.cfg_reg1          = raw[match_off + 6];
            e220_stats.cfg_channel       = raw[match_off + 7];
            e220_stats.cfg_reg3          = raw[match_off + 8];
            e220_stats.config_valid      = true;
            e220_stats.radio_online      = true;
            e220_stats.last_radio_rsp_ms = millis();
            s_rssi_timeout_streak        = 0;
            s_was_radio_online           = true;
            ok = true;
            e220_log_decoded_registers("E220 WRITE OK");
        } else {
            char hex_buf[80];
            size_t hex_pos = 0;
            if (idx == 0U) {
                snprintf(hex_buf, sizeof(hex_buf), "none");
            } else {
                for (size_t i = 0; i < idx && hex_pos + 4U < sizeof(hex_buf); ++i) {
                    int n = snprintf(hex_buf + hex_pos, sizeof(hex_buf) - hex_pos,
                                     (i == 0U) ? "%02X" : " %02X", raw[i]);
                    if (n > 0) hex_pos += (size_t)n;
                }
            }
            Serial.printf("[E220 WRITE WARN] Attempt %u/4 failed (got %u bytes, AUX=%d, hex=%s)\n",
                          attempt, (unsigned)idx, digitalRead(ROVER_PIN_E220_AUX), hex_buf);
        }
    }

    // Return to Mode 0 (Transparent: M0=0, M1=0) BEFORE restoring operational UART baud + liveness check
    e220_exit_mode3_to_mode0(baud);

    if (ok) {
        e220_probe_mode0_liveness(baud);
    } else {
        Serial.println("[E220 WRITE FAIL] All 4 Mode 3 write attempts failed! Reading actual hardware REG0...");
        if (e220_read_config_registers()) {
            float actual_rate = e220_decode_air_rate_from_reg0(e220_stats.cfg_reg0);
            s_active_air_rate                   = actual_rate;
            e220_stats.active_air_rate_kbps     = actual_rate;
            current_settings.e220_air_rate_kbps = actual_rate;
            Serial.printf("[E220 CFG] Synced firmware state to hardware REG0 (%.1f kbps)\n", actual_rate);
        }
    }
    return ok;
}

extern "C" bool e220_read_config_registers(void)
{
    rssi_query_active     = false;
    rssi_reply_len        = 0;
    e220_suppress_aux_isr = true;
    isr_aux_fall_us       = 0;
    e220_wait_aux_high(300);
    Serial0.flush();

    // 1. Switch E220 to Mode 3 (Config/Sleep: M0=1, M1=1) BEFORE switching UART to 9600 8N1
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(60));
    e220_mode3_sync_read();

    uint8_t resp[24] = {0};
    int idx = 0;
    bool ok = false;

    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        while (Serial0.available()) Serial0.read();

        // 2. Send Read Register Command: 0xC1 0x00 0x06 (Read 6 registers starting at 0x00)
        uint8_t cmd[3] = {0xC1, 0x00, 0x06};
        Serial0.write(cmd, 3);
        Serial0.flush();

        idx = 0;
        int match_off = -1;
        uint32_t start = millis();
        while (millis() - start < 500 && idx < (int)sizeof(resp)) {
            if (Serial0.available()) {
                resp[idx++] = (uint8_t)Serial0.read();
                for (int k = 0; k + 9 <= idx; k++) {
                    if (resp[k] == 0xC1 && resp[k + 1] == 0x00 && resp[k + 2] == 0x06) {
                        match_off = k;
                        break;
                    }
                }
                if (match_off >= 0) {
                    break;
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        if (match_off >= 0) {
            e220_stats.cfg_addr_high = resp[match_off + 3];
            e220_stats.cfg_addr_low  = resp[match_off + 4];
            e220_stats.cfg_reg0      = resp[match_off + 5];
            e220_stats.cfg_reg1      = resp[match_off + 6];
            e220_stats.cfg_channel   = resp[match_off + 7];
            e220_stats.cfg_reg3          = resp[match_off + 8];
            e220_stats.config_valid      = true;
            e220_stats.radio_online      = true;
            e220_stats.last_radio_rsp_ms = millis();
            s_rssi_timeout_streak        = 0;
            s_was_radio_online           = true;
            ok = true;
            e220_log_decoded_registers("E220 READ OK");
        } else if (attempt == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (!ok) {
        Serial.printf("[E220 READ FAIL] Expected 9-byte 0xC1 response on Serial0 (IO41/42 @ 9600), got %d bytes (AUX=%d)\n",
                      idx, digitalRead(ROVER_PIN_E220_AUX));
    }

    // 3. Exit Mode 3 -> Mode 0 BEFORE restoring operational UART baud + verify Mode 0 liveness
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    e220_exit_mode3_to_mode0(baud);
    if (ok) {
        e220_probe_mode0_liveness(baud);
    }
    return ok;
}

static bool e220_verify_pin_continuity(uint8_t pin, const char *pin_name)
{
    // Start with E220 in Mode 3 (M0=1, M1=1) and ensure AUX is steady HIGH
    digitalWrite(ROVER_PIN_E220_M0, HIGH);
    digitalWrite(ROVER_PIN_E220_M1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(40));
    if (!e220_wait_aux_high(300)) {
        Serial.printf("[E220 TEST] %s continuity check skipped (AUX stuck LOW)\n", pin_name);
        return false;
    }

    // Pull the tested pin LOW (transition from Mode 3 to Mode 1 or Mode 2)
    digitalWrite(pin, LOW);

    // Per E220 manual Section 5.2.4, switching modes causes AUX to pulse LOW for 2..15ms.
    // If the wire is intact, E220 detects the transition and pulls AUX LOW.
    // If the wire is open/floating, E220's weak internal pullup keeps the pin HIGH (no mode change).
    bool saw_low = false;
    uint32_t start = millis();
    while (millis() - start < 40) {
        if (digitalRead(ROVER_PIN_E220_AUX) == LOW) {
            saw_low = true;
            break;
        }
        delayMicroseconds(250);
    }

    // Restore pin to HIGH (return to Mode 3)
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(30));
    e220_wait_aux_high(300);

    Serial.printf("[E220 TEST] %s Continuity: %s (AUX pulse=%s)\n",
                  pin_name, saw_low ? "PASS (OK)" : "FAIL (OPEN/FLOAT)", saw_low ? "DETECTED" : "MISSING");
    return saw_low;
}

extern "C" void e220_run_powerup_selftest(void)
{
    Serial.println("\n--- [E220-900T30S Power-Up Self-Test (UART0: RX=IO41, TX=IO42)] ---");
    Serial.printf("[E220 PINS] AUX(IO21)=%d | M0(IO39)=%d | M1(IO40)=%d | RST(IO38)=%d\n",
                  digitalRead(ROVER_PIN_E220_AUX),
                  digitalRead(ROVER_PIN_E220_M0),
                  digitalRead(ROVER_PIN_E220_M1),
                  digitalRead(ROVER_PIN_E220_RST));

    // 1. Hardware Wire Continuity Self-Test via Mode-Switch AUX Pulses
    e220_stats.m0_wire_ok = e220_verify_pin_continuity(ROVER_PIN_E220_M0, "M0 (IO39)");
    e220_stats.m1_wire_ok = e220_verify_pin_continuity(ROVER_PIN_E220_M1, "M1 (IO40)");

    // 2. Read Configuration Registers in Mode 3
    if (e220_read_config_registers()) {
        if (!e220_stats.config_matched) {
            Serial.println("[E220 TEST] Register mismatch vs settings.json -> Auto-programming target E220 config...");
            e220_write_default_config_registers();
        }
        Serial.printf("[E220 TEST] PASS -- E220-900T30S verified & running in Transparent Mode 0 @ %lu baud\n",
                      (unsigned long)current_settings.e220_uart_baud);
    } else {
        Serial.println("[E220 TEST] FAIL -- No register response from E220 on IO41/42 (check power/wiring)");
    }
}

static void e220_record_rssi_reply(uint8_t noise_raw, uint8_t pkt_raw, uint32_t now);

static void e220_shift_stage_one_byte(void)
{
    if (rx_stage_len > 1) {
        memmove(rx_stage, rx_stage + 1, rx_stage_len - 1);
        rx_stage_len--;
    } else {
        rx_stage_len = 0;
    }
}

static void e220_process_rx_stage(void)
{
    while (rx_stage_len > 0) {
        // 0. Intercept Downlink Telemetry / Control Frames (0xAA 0x55)
        if (rx_stage_len >= 2 && rx_stage[0] == ROVER_TELEM_SYNC0 && rx_stage[1] == ROVER_TELEM_SYNC1) {
            if (rx_stage_len < 4) {
                break; // Wait for message ID and payload length
            }
            uint8_t msg_id = rx_stage[2];
            uint8_t plen   = rx_stage[3];
            size_t tot_len = (size_t)plen + 6U;
            if (rx_stage_len < tot_len) {
                break; // Wait for complete frame
            }

            if (msg_id == ROVER_MSG_ID_ACK_RATE_SWITCH && tot_len == ROVER_ACK_SWITCH_FRAME_LEN) {
                RoverRateSwitchAck_t ack;
                if (rover_parse_rate_switch_ack(rx_stage, tot_len, &ack)) {
                    if (ack.status == 0 && ack.req_id == s_rate_switch_req_id) {
                        if (!s_rate_switch_ack_received) {
                            s_rate_switch_ack_received = true;
                            uint32_t rx_now_ms  = millis();
                            uint32_t ack_sf_sec = 0;
                            uint32_t ack_sf_ms  = 0;
                            if (e220_get_superframe_clock(rx_now_ms, NULL, &ack_sf_sec, &ack_sf_ms)) {
                                // Align hardware register reconfiguration precisely to top of Second 2 (T0 + 2 @ 0 ms)
                                uint32_t rem_ms = 1000U - ack_sf_ms;
                                if (ack_sf_sec == 0U) {
                                    s_rate_switch_exec_ms = rx_now_ms + rem_ms + 1000U;
                                } else if (ack_sf_sec == 1U) {
                                    s_rate_switch_exec_ms = rx_now_ms + rem_ms;
                                } else {
                                    s_rate_switch_exec_ms = rx_now_ms;
                                }
                            } else {
                                s_rate_switch_exec_ms = rx_now_ms + ack.switch_delay_ms;
                            }
                            Serial.printf("[E220 LORA RX ACK] Rate switch confirmed! (Req #%u Status:%u Delay:%ums)\n",
                                          ack.req_id, ack.status, ack.switch_delay_ms);
                        }
                    } else if (!s_rate_switch_ack_received) {
                        s_rate_switch_requested = false;
                        e220_stats.rate_switch_pending = false;
                    }
                }
            }

            // Consume frame from rx_stage
            if (rx_stage_len > tot_len) {
                memmove(rx_stage, rx_stage + tot_len, rx_stage_len - tot_len);
                rx_stage_len -= tot_len;
            } else {
                rx_stage_len = 0;
            }
            continue;
        }

        // 0b. Intercept Late Mode 0 RSSI Replies (strictly within 250ms of an issued query):
        //     4-byte E220-900T30S reply: 0xC1 0x00 <noise>=0x80..0xE4> <pkt>=0x80..0xF6>
        //     5-byte datasheet reply:    0xC1 0x00 0x02 <noise>=0x80..0xE4> <pkt>=0x80..0xF6>
        bool allow_late_rssi = (last_rssi_poll_ms > 0) && ((millis() - last_rssi_poll_ms) <= 250U);
        if (allow_late_rssi && rx_stage_len >= 2 && rx_stage[0] == 0xC1 && rx_stage[1] == 0x00) {
            if (rx_stage_len < 3) {
                break; // Wait for byte 2 (0x02 or >= 0x80)
            }
            bool is_5b_hdr = (rx_stage[2] == 0x02U);
            bool is_4b_hdr = (rx_stage[2] >= 0x80U && rx_stage[2] <= 0xE4U);
            if (!is_5b_hdr && !is_4b_hdr) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
            size_t need_len = is_5b_hdr ? 5U : 4U;
            if (rx_stage_len < need_len) {
                break; // Wait for remaining RSSI payload byte(s)
            }
            uint8_t noise_raw = is_5b_hdr ? rx_stage[3] : rx_stage[2];
            uint8_t pkt_raw   = is_5b_hdr ? rx_stage[4] : rx_stage[3];
            if (noise_raw < 0x80U || noise_raw > 0xE4U ||
                (pkt_raw != 0x00U && (pkt_raw < 0x80U || pkt_raw > 0xF6U))) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
            e220_record_rssi_reply(noise_raw, pkt_raw, millis());
            if (rx_stage_len > need_len) {
                memmove(rx_stage, rx_stage + need_len, rx_stage_len - need_len);
                rx_stage_len -= need_len;
            } else {
                rx_stage_len = 0;
            }
            continue;
        }

        // 1. Strip any leading non-0xD3 orphan bytes before UM982 COM2 ever sees them
        if (rx_stage[0] != 0xD3) {
            size_t sync_pos = 1;
            while (sync_pos < rx_stage_len) {
                if (rx_stage[sync_pos] == 0xD3) {
                    break;
                }
                if (sync_pos + 1 < rx_stage_len &&
                    rx_stage[sync_pos] == ROVER_TELEM_SYNC0 &&
                    rx_stage[sync_pos + 1] == ROVER_TELEM_SYNC1) {
                    break;
                }
                if (allow_late_rssi &&
                    sync_pos + 2 < rx_stage_len &&
                    rx_stage[sync_pos] == 0xC1 &&
                    rx_stage[sync_pos + 1] == 0x00 &&
                    (rx_stage[sync_pos + 2] == 0x02U ||
                     (rx_stage[sync_pos + 2] >= 0x80U && rx_stage[sync_pos + 2] <= 0xE4U))) {
                    break;
                }
                sync_pos++;
            }
            e220_stats.rtcm_orphan_bytes += (uint32_t)sync_pos;
            if (sync_pos < rx_stage_len) {
                memmove(rx_stage, rx_stage + sync_pos, rx_stage_len - sync_pos);
                rx_stage_len -= sync_pos;
            } else {
                rx_stage_len = 0;
                break;
            }
        }

        // Wait for at least 3-byte RTCM3 header (0xD3 + 10-bit length)
        if (rx_stage_len < 3) {
            break;
        }

        // 2. Validate 6 reserved zero bits in byte 1 and plausible payload length [6..1023]
        if ((rx_stage[1] & 0xFC) != 0x00) {
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            continue;
        }

        uint16_t payload_len = ((uint16_t)(rx_stage[1] & 0x03) << 8) | rx_stage[2];
        if (payload_len < 6 || payload_len > 1023) {
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            continue;
        }

        // 3. Validate 12-bit RTCM3 message type once 5 bytes are staged
        if (rx_stage_len >= 5) {
            uint16_t cand_type = ((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F);
            if (!rtcm_is_plausible_msg_type(cand_type)) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
        }

        size_t total_len = (size_t)payload_len + 6U;
        if (rx_stage_len < total_len) {
            // Valid header so far; wait for remaining subpacket bytes (up to RTCM_TRUNC_TIMEOUT_MS)
            break;
        }

        // 4. Full candidate frame staged -> verify 24-bit CRC24Q before touching UM982 COM1
        uint16_t msg_type = ((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F);
        uint32_t crc = rtcm_crc24q_compute(rx_stage, total_len);
        if (crc == 0) {
            // Verified intact RTCM3 frame -> forward atomically to UM982 COM1!
            um982_forward_rtcm(rx_stage, total_len);

            uint32_t ok_now = millis();
            rtcm_in_resync = false;
            s_last_valid_rtcm_ms = ok_now;
            e220_stats.rtcm_frames_ok++;
            e220_stats.rtcm_bytes_forwarded += (uint32_t)total_len;
            e220_stats.last_msg_type = msg_type;

            if (msg_type == 1005 || msg_type == 1006) {
                e220_stats.msg_1005_count++;
            } else if (msg_type >= 1071 && msg_type <= 1130) {
                e220_stats.msg_msm_count++;
                if (msg_type == 1074) {
                    e220_stats.msg_1074_count++;
                    last_1074_rx_ms = ok_now;
                } else if (msg_type == 1084) {
                    e220_stats.msg_1084_count++;
                } else if (msg_type == 1094) {
                    e220_stats.msg_1094_count++;
                }

                if (msg_type == 1074U || msg_type == 1084U || msg_type == 1094U || msg_type == 1124U) {
                    uint32_t rx_utc_sec = 0;
                    if (e220_get_superframe_clock(ok_now, &rx_utc_sec, NULL, NULL)) {
                        s_last_valid_msm_utc_sec = rx_utc_sec;
                    }
                }

                if ((s_active_air_rate > 10.0f && msg_type == 1094 &&
                     last_1074_rx_ms > 0 && (ok_now - last_1074_rx_ms) <= 850U) ||
                    (s_active_air_rate > 5.0f && s_active_air_rate <= 10.0f &&
                     (msg_type == 1084 || msg_type == 1094) &&
                     last_1074_rx_ms > 0 && (ok_now - last_1074_rx_ms) <= 850U)) {
                    last_terminal_msm_rx_ms = ok_now;
                }
            } else if (msg_type == 1230) {
                e220_stats.msg_1230_count++;
            }

            if (current_settings.enable_serial_logging) {
                Serial.printf("[RTCM OK -> COM1] Type:%u Frame:%uB | OK:%lu Err:%lu(Trunc:%lu) Orph:%luB\n",
                              msg_type,
                              (unsigned int)total_len,
                              (unsigned long)e220_stats.rtcm_frames_ok,
                              (unsigned long)e220_stats.rtcm_crc_errors,
                              (unsigned long)e220_stats.rtcm_trunc_drops,
                              (unsigned long)e220_stats.rtcm_orphan_bytes);
            }

            if (rx_stage_len > total_len) {
                memmove(rx_stage, rx_stage + total_len, rx_stage_len - total_len);
                rx_stage_len -= total_len;
            } else {
                rx_stage_len = 0;
            }
        } else {
            // Corrupted frame or truncated head followed immediately by a new 0xD3 frame!
            // Do NOT forward to UM982 COM1; shift 1 byte so any complete frame inside rx_stage is rescued.
            if (!rtcm_in_resync) {
                rtcm_in_resync = true;
                e220_stats.rtcm_crc_errors++;
                if (current_settings.enable_serial_logging) {
                    Serial.printf("[RTCM DROP CRC] Type:%u Expected:%uB CRC:0x%06lX (NOT sent to COM1) | OK:%lu Err:%lu\n",
                                  msg_type,
                                  (unsigned int)total_len,
                                  (unsigned long)crc,
                                  (unsigned long)e220_stats.rtcm_frames_ok,
                                  (unsigned long)e220_stats.rtcm_crc_errors);
                }
            }
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
        }
    }
}

static void e220_flush_burst_log(uint32_t ref_last_byte_ms)
{
    uint32_t gap = (last_burst_end_ms > 0 && ref_last_byte_ms >= last_burst_end_ms)
                   ? (ref_last_byte_ms - last_burst_end_ms)
                   : 0;
    uint32_t cur_edges  = isr_aux_fall_count;
    uint32_t edge_delta = cur_edges - last_burst_aux_edges;
    last_burst_aux_edges = cur_edges;

    if (current_settings.enable_serial_logging) {
        Serial.printf("[E220 RX BURST] %lu B (gap: %lu ms | AUX edges:+%lu low:%.2fms)\n",
                      (unsigned long)burst_bytes,
                      (unsigned long)gap,
                      (unsigned long)edge_delta,
                      (float)isr_aux_last_low_us / 1000.0f);
    }
    last_burst_end_ms = ref_last_byte_ms;
    burst_bytes = 0;
}

static void e220_ingest_rx_bytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (len == 0) {
        return;
    }

    if (burst_bytes > 0 && (now - burst_last_byte_ms >= 15)) {
        e220_flush_burst_log(burst_last_byte_ms);
    }

    burst_bytes += (uint32_t)len;
    burst_last_byte_ms = now;

    e220_stats.rx_bytes_total   += (uint32_t)len;
    e220_stats.last_rx_ms        = now;
    e220_stats.radio_online      = true;
    e220_stats.last_radio_rsp_ms = now;
    s_rssi_timeout_streak        = 0;
    if (!s_was_radio_online) {
        s_was_radio_online = true;
        Serial.println("[E220 LORA] Radio link ONLINE (UART RX activity detected).");
    }
    last_stage_rx_ms = now;

    // Preserve raw wire-tap to SD/FFat rtcm-base_*.txt for forensic analysis
    if (rtcm_logger_is_active()) {
        rtcm_logger_feed(data, len);
    }

    // Append into sliding-window Gatekeeper buffer (shed oldest orphan bytes if near capacity)
    if (len >= RTCM_STAGE_CAPACITY) {
        e220_stats.rtcm_orphan_bytes += (uint32_t)(rx_stage_len + (len - RTCM_STAGE_CAPACITY));
        memcpy(rx_stage, data + (len - RTCM_STAGE_CAPACITY), RTCM_STAGE_CAPACITY);
        rx_stage_len = RTCM_STAGE_CAPACITY;
    } else {
        if (rx_stage_len + len > RTCM_STAGE_CAPACITY) {
            size_t drop = (rx_stage_len + len) - RTCM_STAGE_CAPACITY;
            e220_stats.rtcm_orphan_bytes += (uint32_t)drop;
            memmove(rx_stage, rx_stage + drop, rx_stage_len - drop);
            rx_stage_len -= drop;
        }
        memcpy(rx_stage + rx_stage_len, data, len);
        rx_stage_len += len;
    }

    e220_process_rx_stage();
}

static void e220_record_rssi_reply(uint8_t noise_raw, uint8_t pkt_raw, uint32_t now)
{
    int16_t cand_noise_dbm = (int16_t)noise_raw - 256;
    int16_t cand_pkt_dbm   = (int16_t)pkt_raw - 256;
    // Reject ambient noise samples contaminated by active close-range RF carrier demodulation
    bool carrier_contaminated = (cand_noise_dbm > -65) &&
                                (cand_pkt_dbm > -65) &&
                                ((cand_noise_dbm - cand_pkt_dbm) >= -8) &&
                                ((cand_noise_dbm - cand_pkt_dbm) <= 8);
    if (noise_raw > 0x80U && noise_raw <= 0xE4U && !carrier_contaminated) {
        e220_stats.ambient_noise_dbm = cand_noise_dbm;
    }
    if (e220_stats.rx_bytes_total > 0 && pkt_raw >= 0x80U && pkt_raw <= 0xF6U) {
        e220_stats.last_pkt_rssi_dbm = cand_pkt_dbm;
        e220_stats.rssi_valid        = true;
    }
    e220_stats.radio_online      = true;
    e220_stats.last_radio_rsp_ms = now;
    s_rssi_timeout_streak        = 0;
    if (!s_was_radio_online) {
        s_was_radio_online = true;
        Serial.println("[E220 LORA] Radio link ONLINE (Mode 0 RSSI response verified).");
        if (!e220_stats.config_valid) {
            pending_cfg_read = true;
        }
    }
    rssi_query_active     = false;
    rssi_reply_len        = 0;
    isr_aux_fall_us       = 0;
    e220_suppress_aux_isr = false;
}

extern "C" void e220_poll_and_forward_rtcm(void)
{
    e220_stats.aux_pin_high    = (digitalRead(ROVER_PIN_E220_AUX) == HIGH);
    e220_stats.aux_fall_count  = isr_aux_fall_count;
    e220_stats.aux_last_low_us = isr_aux_last_low_us;

    uint32_t poll_now_ms = millis();
    if (e220_stats.aux_pin_high) {
        s_aux_low_since_ms = 0;
    } else if (s_aux_low_since_ms == 0) {
        s_aux_low_since_ms = (poll_now_ms == 0) ? 1U : poll_now_ms;
    }

    if (pending_cfg_write) {
        pending_cfg_write = false;
        e220_write_default_config_registers();
    }
    if (pending_cfg_read) {
        pending_cfg_read = false;
        e220_read_config_registers();
    }

    // 1. Intercept 4-byte (0xC1 0x00 <noise> <pkt>) or 5-byte (0xC1 0x00 0x02 <noise> <pkt>) Mode 0 RSSI reply
    if (rssi_query_active) {
        uint32_t now = millis();
        while (Serial0.available() > 0 && rssi_query_active) {
            uint8_t b = (uint8_t)Serial0.read();
            rssi_reply_buf[rssi_reply_len++] = b;

            bool prefix_mismatch =
                (rssi_reply_len == 1 && rssi_reply_buf[0] != 0xC1) ||
                (rssi_reply_len == 2 && rssi_reply_buf[1] != 0x00) ||
                (rssi_reply_len == 3 && rssi_reply_buf[2] != 0x02U &&
                 (rssi_reply_buf[2] < 0x80U || rssi_reply_buf[2] > 0xE4U)) ||
                (rssi_reply_len == 4 &&
                 ((rssi_reply_buf[2] == 0x02U && (rssi_reply_buf[3] < 0x80U || rssi_reply_buf[3] > 0xE4U)) ||
                  (rssi_reply_buf[2] != 0x02U && rssi_reply_buf[3] != 0x00U &&
                   (rssi_reply_buf[3] < 0x80U || rssi_reply_buf[3] > 0xF6U)))) ||
                (rssi_reply_len == 5 &&
                 rssi_reply_buf[4] != 0x00U && (rssi_reply_buf[4] < 0x80U || rssi_reply_buf[4] > 0xF6U));

            if (prefix_mismatch) {
                // An incoming LoRa packet arrived simultaneously -> cancel RSSI intercept and feed zero-loss to RTCM stage!
                rssi_query_active     = false;
                isr_aux_fall_us       = 0;
                e220_suppress_aux_isr = false;
                uint8_t saved_len     = rssi_reply_len;
                rssi_reply_len        = 0;
                e220_ingest_rx_bytes(rssi_reply_buf, saved_len, now);
                break;
            }

            if (rssi_reply_len == 4 && rssi_reply_buf[2] != 0x02U) {
                // 4-byte E220-900T30S silicon firmware reply: 0xC1 0x00 <noise_raw> <pkt_raw>
                e220_record_rssi_reply(rssi_reply_buf[2], rssi_reply_buf[3], now);
                break;
            }

            if (rssi_reply_len == 5) {
                // 5-byte datasheet reply: 0xC1 0x00 0x02 <noise_raw> <pkt_raw>
                e220_record_rssi_reply(rssi_reply_buf[3], rssi_reply_buf[4], now);
                break;
            }
        }

        if (rssi_query_active) {
            uint32_t q_now_ms = millis();
            uint32_t q_sf_ms  = 0;
            bool     has_q_sf = e220_get_superframe_clock(q_now_ms, NULL, NULL, &q_sf_ms);
            if ((q_now_ms - rssi_query_start_ms > E220_RSSI_REPLY_TIMEOUT_MS) ||
                (has_q_sf && q_sf_ms >= 990U)) {
                rssi_query_active     = false;
                isr_aux_fall_us       = 0;
                e220_suppress_aux_isr = false;
                if (rssi_reply_len > 0 && rssi_reply_buf[0] != 0xC1) {
                    uint8_t saved_len = rssi_reply_len;
                    rssi_reply_len    = 0;
                    e220_ingest_rx_bytes(rssi_reply_buf, saved_len, q_now_ms);
                } else {
                    rssi_reply_len = 0;
                    if (s_rssi_timeout_streak < 255U) {
                        s_rssi_timeout_streak++;
                    }
                    uint32_t rx_silence_ms = (e220_stats.last_rx_ms > 0) ? (q_now_ms - e220_stats.last_rx_ms) : 0xFFFFFFFFU;
                    if (s_rssi_timeout_streak >= 3U && rx_silence_ms > 10000U) {
                        if (e220_stats.radio_online) {
                            Serial.println("[E220 LORA] WARNING: Radio link OFFLINE (3 consecutive Mode 0 poll timeouts & >10s RX silence).");
                        }
                        e220_stats.radio_online = false;
                        e220_stats.rssi_valid   = false;
                        s_was_radio_online      = false;
                    }
                }
            }
        }
    }

    // 2. Drain all available bytes in Serial0 FIFO into the Store-and-Verify RTCM3 Gatekeeper
    uint8_t buf[256];
    int avail;
    while (!rssi_query_active && (avail = Serial0.available()) > 0) {
        if (avail > (int)sizeof(buf)) avail = (int)sizeof(buf);
        int n = Serial0.readBytes((char *)buf, avail);
        if (n <= 0) break;
        e220_ingest_rx_bytes(buf, (size_t)n, millis());
    }

    uint32_t now = millis();

    if (s_aux_low_since_ms > 0 &&
        (now - s_aux_low_since_ms > 3000U) &&
        (e220_stats.last_rx_ms == 0 || (now - e220_stats.last_rx_ms > 3000U))) {
        if (e220_stats.radio_online) {
            Serial.println("[E220 LORA] WARNING: Radio link OFFLINE (AUX held LOW > 3.0s with no UART RX).");
        }
        e220_stats.radio_online = false;
        e220_stats.rssi_valid   = false;
        s_was_radio_online      = false;
    }

    // 3. Flush burst diagnostic log when UART line goes idle for >= 15 ms
    if (burst_bytes > 0 && (now - burst_last_byte_ms >= 15)) {
        e220_flush_burst_log(burst_last_byte_ms);
    }

    // 4. Mid-frame truncation timeout: if tail subpacket did not arrive within rate-adaptive timeout,
    //    quarantine the broken head (never forward to UM982 COM1) and rescan remaining bytes
    //    so any smaller valid frame sitting behind it in rx_stage is rescued.
    uint32_t cur_trunc_timeout_ms = e220_get_trunc_timeout_ms();
    if (rx_stage_len > 0 && (now - last_stage_rx_ms > cur_trunc_timeout_ms)) {
        bool logged_trunc = false;
        while (rx_stage_len > 0) {
            if (!logged_trunc && !rtcm_in_resync) {
                uint16_t payload_len = (rx_stage_len >= 3)
                                       ? (((uint16_t)(rx_stage[1] & 0x03) << 8) | rx_stage[2])
                                       : 0;
                uint16_t msg_type    = (rx_stage_len >= 5)
                                       ? (((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F))
                                       : 0;
                e220_stats.rtcm_crc_errors++;
                e220_stats.rtcm_trunc_drops++;
                logged_trunc = true;
                if (current_settings.enable_serial_logging) {
                    Serial.printf("[RTCM DROP TRUNC] Type:%u Got:%u/%uB after %ums (NOT sent to COM1) | OK:%lu Err:%lu(Trunc:%lu)\n",
                                  msg_type,
                                  (unsigned int)rx_stage_len,
                                  (unsigned int)(payload_len + 6U),
                                  (unsigned int)cur_trunc_timeout_ms,
                                  (unsigned long)e220_stats.rtcm_frames_ok,
                                  (unsigned long)e220_stats.rtcm_crc_errors,
                                  (unsigned long)e220_stats.rtcm_trunc_drops);
                }
            }
            rtcm_in_resync = true;
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            e220_process_rx_stage();
        }
        rtcm_in_resync = false;
    }

    // 5. Synchronized Mode 0 Ambient Noise & Last Packet RSSI query:
    //    - Suppressed while an air-rate switch request/ACK handshake, pending Mode 3 switch/rollback,
    //      or single-subpacket RF TX is actively in flight on the LLCC68.
    //    - In Second 0: poll strictly after uplink RF transmission finishes (810..895ms).
    //    - In Seconds 1..4: poll inside the GPS-disciplined end-of-second quiet micro-slot (855..895ms)
    //      while Base enforces an 850ms downlink TX cutoff, giving 1.0 Hz updates with zero collisions.
    uint32_t sf_utc_sec = 0;
    uint32_t sf_sec     = 0;
    uint32_t sf_ms      = 0;
    bool     has_sf     = e220_get_superframe_clock(now, &sf_utc_sec, &sf_sec, &sf_ms);
    uint32_t ul_block_start_ms = (s_active_air_rate <= 5.0f) ? 440U : ((s_active_air_rate <= 10.0f) ? 610U : 670U);
    bool     in_sec0_tx_block  = has_sf && (sf_sec == 0U) && (sf_ms >= ul_block_start_ms) && (sf_ms <= 860U) &&
                                 (s_last_uplink_utc_sec != sf_utc_sec);
    bool     tx_air_busy     = (s_tx_air_busy_until_ms > 0) && ((int32_t)(now - s_tx_air_busy_until_ms) < 0);
    bool     switch_exec_due = s_rate_switch_ack_received && ((int32_t)(now - s_rate_switch_exec_ms) >= 0);
    bool     watchdog_due    = s_switched_requires_watchdog && (now >= s_rate_switch_done_ms) && ((now - s_rate_switch_done_ms) > 60000U);

    if (!s_rate_switch_requested &&
        !rssi_query_active &&
        !in_sec0_tx_block &&
        !tx_air_busy &&
        !switch_exec_due &&
        !watchdog_due &&
        (!e220_stats.config_valid || ((e220_stats.cfg_reg1 & 0x20) != 0)) &&
        rx_stage_len == 0 &&
        burst_bytes == 0 &&
        (burst_last_byte_ms == 0 || (now - burst_last_byte_ms >= 15U)) &&
        (now - last_rssi_poll_ms >= E220_RSSI_POLL_INTERVAL_MS) &&
        digitalRead(ROVER_PIN_E220_AUX) == HIGH &&
        Serial0.available() == 0) {

        // Consolidated Architecture 2B RSSI polling: run once per macro-cycle in Second 0
        // right after uplink TX completes so Seconds 1..4 remain 100% unbroken downlink.
        bool post_uplink_gap = has_sf && (sf_sec == 0U) &&
                               (s_last_uplink_utc_sec == sf_utc_sec) &&
                               (s_last_rssi_poll_utc_sec != sf_utc_sec) &&
                               (sf_ms >= 680U) && (sf_ms <= 850U);

        bool unsynced_gap    = !has_sf &&
                               (s_active_air_rate > 5.0f) &&
                               (last_terminal_msm_rx_ms > 0) &&
                               (now - last_terminal_msm_rx_ms >= 15U) &&
                               (now - last_terminal_msm_rx_ms <= 45U) &&
                               (now - last_rssi_poll_ms >= 1800U);

        bool idle_link_probe = (e220_stats.last_rx_ms == 0 || (now - e220_stats.last_rx_ms > 3000U)) &&
                               (!has_sf || ((sf_sec != 0U || sf_ms < 390U) && sf_ms <= 810U)) &&
                               (now - last_rssi_poll_ms >= 2000U);

        if (post_uplink_gap || unsynced_gap || idle_link_probe) {
            static const uint8_t kRssiCmd[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
            last_rssi_poll_ms       = now;
            if (has_sf) {
                s_last_rssi_poll_utc_sec = sf_utc_sec;
            }
            rssi_query_start_ms     = now;
            rssi_reply_len          = 0;
            e220_suppress_aux_isr   = true;
            rssi_query_active       = true;
            Serial0.write(kRssiCmd, sizeof(kRssiCmd));
        }
    }

    // 6. Bi-Directional LoRa Uplink & Air-Rate Negotiation Scheduler (GPS-Disciplined 5s Superframe TDMA)
    // Transmits ONLY during the GPS-disciplined Second 0 quiet/uplink slot (max 1 packet per Second 0),
    // provided AUX is HIGH, UART RX is idle, and no RSSI query is in flight.
    if (!rssi_query_active &&
        rx_stage_len == 0 &&
        burst_bytes == 0 &&
        digitalRead(ROVER_PIN_E220_AUX) == HIGH &&
        Serial0.available() == 0) {

        bool uplink_slot_ready = has_sf &&
                                 e220_is_uplink_slot_active(now) &&
                                 (sf_utc_sec != s_last_uplink_utc_sec) &&
                                 (s_last_uplink_tx_ms == 0 || (now - s_last_uplink_tx_ms >= 2000U));

        // A. Priority 1: Transmit Pending Air-Rate Switch Request (12s timeout covers 2 full 5s superframes)
        if (s_rate_switch_requested && !s_rate_switch_ack_received) {
            if (s_rate_switch_req_start_ms > 0 && (now - s_rate_switch_req_start_ms > 12000U)) {
                // Request timeout after 12 seconds of no ACK -> abort and clear pending flag
                Serial.printf("[E220 LORA] Air-rate switch request to %.1fk timed out (no ACK) -> Aborting.\n", s_requested_rate_kbps);
                s_rate_switch_requested        = false;
                e220_stats.rate_switch_pending = false;
                s_rate_switch_req_start_ms     = 0;
            } else if (uplink_slot_ready) {
                uint8_t air_code = ROVER_AIR_CODE_19_2K;
                if (s_requested_rate_kbps < 3.0f)       air_code = ROVER_AIR_CODE_2_4K;
                else if (s_requested_rate_kbps < 7.0f)  air_code = ROVER_AIR_CODE_4_8K;
                else if (s_requested_rate_kbps < 14.0f) air_code = ROVER_AIR_CODE_9_6K;
                else if (s_requested_rate_kbps < 28.0f) air_code = ROVER_AIR_CODE_19_2K;
                else if (s_requested_rate_kbps < 50.0f) air_code = ROVER_AIR_CODE_38_4K;
                else                                    air_code = ROVER_AIR_CODE_62_5K;

                uint8_t chan = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
                uint8_t tx_buf[ROVER_CMD_SWITCH_FRAME_LEN];
                size_t n = rover_build_rate_switch_req(tx_buf, sizeof(tx_buf), air_code, chan, s_rate_switch_req_id);
                if (n > 0) {
                    e220_suppress_aux_isr = true;
                    Serial0.write(tx_buf, n);
                    Serial0.flush();
                    uint32_t tx_start_ms = millis();
                    s_tx_air_busy_until_ms = tx_start_ms + e220_estimate_tx_air_ms(n, s_active_air_rate);
                    e220_wait_tx_complete(25, 300);
                    now = millis();
                    isr_aux_fall_us = 0;
                    e220_suppress_aux_isr = false;
                    s_rate_switch_req_tx_ms = now;
                    s_last_uplink_tx_ms     = now;
                    s_last_uplink_utc_sec   = sf_utc_sec;
                    Serial.printf("[E220 LORA TX] Sent Rate Switch Req #%u -> air_code 0x%02X (%.1fk)\n",
                                  s_rate_switch_req_id, air_code, s_requested_rate_kbps);
                }
            }
        }
        // B. Priority 2: Periodic Uplink Telemetry (Every 5 seconds in GPS-disciplined Second 0 slot)
        else if (s_has_telemetry_state &&
                 !s_rate_switch_requested &&
                 uplink_slot_ready) {

            RoverUIData_t telem_state;
            portENTER_CRITICAL(&s_telem_mux);
            telem_state = s_latest_telemetry_state;
            portEXIT_CRITICAL(&s_telem_mux);

            uint8_t tx_buf[ROVER_TELEM_FRAME_LEN];
            size_t n = rover_build_telemetry_packet(tx_buf, sizeof(tx_buf), &telem_state, s_uplink_seq);
            if (n > 0) {
                e220_suppress_aux_isr = true;
                Serial0.write(tx_buf, n);
                Serial0.flush();
                uint32_t tx_start_ms = millis();
                s_tx_air_busy_until_ms = tx_start_ms + e220_estimate_tx_air_ms(n, s_active_air_rate);
                e220_wait_tx_complete(25, 300);
                now = millis();
                isr_aux_fall_us = 0;
                e220_suppress_aux_isr = false;
                s_last_uplink_tx_ms   = now;
                s_last_uplink_utc_sec = sf_utc_sec;
                s_uplink_seq++;
                e220_stats.uplink_packets_sent++;
                if (current_settings.enable_serial_logging) {
                    char pwr_str[32];
                    if (telem_state.vbus_present) {
                        snprintf(pwr_str, sizeof(pwr_str), "USB:%.2fV%s",
                                 telem_state.vbus_voltage,
                                 telem_state.is_charging ? "+" : "");
                    } else {
                        snprintf(pwr_str, sizeof(pwr_str), "Bat:%u%%(%.2fV)",
                                 telem_state.battery_percent,
                                 telem_state.battery_voltage);
                    }
                    Serial.printf("[E220 LORA TX #%u] Uplink %uB (FW:%s Fix:%d SLV:%d Tip:%.7f,%.7f,%.2fm BL:%.1fcm P:%+.1fdeg %s)\n",
                                  s_uplink_seq,
                                  (unsigned)n,
                                  ROVER_FIRMWARE_VERSION,
                                  telem_state.fix_type,
                                  telem_state.slave_fix_type,
                                  telem_state.tip_lat,
                                  telem_state.tip_lon,
                                  telem_state.tip_height_m,
                                  telem_state.baseline_length_m * 100.0f,
                                  telem_state.baseline_pitch_deg,
                                  pwr_str);
                }
            }
        }
    }

    // 7. Execute Scheduled Air-Rate Reconfiguration synchronously at Second 2 boundary
    if (s_rate_switch_ack_received && ((int32_t)(now - s_rate_switch_exec_ms) >= 0)) {
        s_rate_switch_ack_received = false;
        s_rate_switch_requested    = false;
        e220_stats.rate_switch_pending = false;

        const float prev_rate = s_active_air_rate;
        Serial.printf("[E220 LORA SWITCH] Reconfiguring E220 registers %.1f -> %.1f kbps...\n",
                      prev_rate, s_requested_rate_kbps);
        current_settings.e220_air_rate_kbps = s_requested_rate_kbps;
        if (e220_write_default_config_registers()) {
            s_fallback_air_rate = prev_rate;
            s_active_air_rate   = s_requested_rate_kbps;
            e220_stats.active_air_rate_kbps = s_active_air_rate;
            e220_stats.uplink_rate_switches++;
            now = millis();
            s_rate_switch_done_ms = now;
            s_switched_requires_watchdog = true;
            mark_settings_dirty();
            Serial.printf("[E220 TDMA] Switched to %.1f kbps (fallback: %.1f kbps). 60s watchdog armed.\n",
                          s_active_air_rate, s_fallback_air_rate);
        } else {
            s_fallback_air_rate = prev_rate;
            e220_stats.active_air_rate_kbps = s_active_air_rate;
            now = millis();
            s_rate_switch_done_ms = now;
            s_switched_requires_watchdog = (fabsf(s_active_air_rate - prev_rate) >= 0.2f);
            mark_settings_dirty();
            Serial.printf("[E220 TDMA] ERROR: Mode 3 write failed during switch to %.1f kbps! Hardware at %.1f kbps.\n",
                          s_requested_rate_kbps, s_active_air_rate);
        }
    }

    // 8. 60-Second Watchdog Rollback Guard
    if (s_switched_requires_watchdog) {
        if (e220_stats.rtcm_frames_ok > 0 && ((int32_t)(s_last_valid_rtcm_ms - s_rate_switch_done_ms) > 0)) {
            // Received valid RTCM strictly AFTER the switch completed -> switch confirmed!
            s_switched_requires_watchdog = false;
            s_fallback_air_rate = s_active_air_rate;
            Serial.printf("[E220 TDMA] Post-switch RTCM verified on %.1fk; rollback watchdog disarmed.\n", s_active_air_rate);
        } else if (now >= s_rate_switch_done_ms && (now - s_rate_switch_done_ms > 60000U)) {
            s_switched_requires_watchdog = false;
            const float target_fallback = (s_fallback_air_rate >= 2.0f && s_fallback_air_rate <= 65.0f) ? s_fallback_air_rate : 19.2f;
            Serial.printf("[E220 LORA WATCHDOG] 60s silence -> Rolling back %.1fk -> %.1fk fallback!\n",
                          s_active_air_rate, target_fallback);
            current_settings.e220_air_rate_kbps = target_fallback;
            if (e220_write_default_config_registers()) {
                s_active_air_rate = target_fallback;
            }
            e220_stats.active_air_rate_kbps = s_active_air_rate;
            s_rate_switch_requested = false;
            e220_stats.rate_switch_pending = false;
            mark_settings_dirty();
        }
    }

    e220_stats.aux_fall_count  = isr_aux_fall_count;
    e220_stats.aux_last_low_us = isr_aux_last_low_us;
}

extern "C" void e220_feed_telemetry_state(const RoverUIData_t *state)
{
    if (state) {
        portENTER_CRITICAL(&s_telem_mux);
        s_latest_telemetry_state = *state;
        s_has_telemetry_state    = true;
        portEXIT_CRITICAL(&s_telem_mux);
    }
}

extern "C" bool e220_request_air_rate_switch(float target_rate_kbps)
{
    if (target_rate_kbps < 2.0f || target_rate_kbps > 65.0f) {
        return false;
    }
    // Reject only once the over-the-air switch request has been transmitted in Second 0
    if (s_rate_switch_requested && (s_rate_switch_req_tx_ms > 0 || s_rate_switch_ack_received)) {
        return false;
    }
    // If rapid cycling brings target back to the currently active rate before Second 0 TX, cancel queued switch
    if (fabsf(target_rate_kbps - s_active_air_rate) < 0.2f) {
        if (s_rate_switch_requested) {
            s_rate_switch_requested        = false;
            s_requested_rate_kbps          = s_active_air_rate;
            e220_stats.rate_switch_pending = false;
            s_rate_switch_req_start_ms     = 0;
            Serial.printf("[E220 TDMA] Queued air-rate switch cancelled (cycled back to %.1fk).\n", s_active_air_rate);
            return true;
        }
        return false;
    }
    s_requested_rate_kbps      = target_rate_kbps;
    s_rate_switch_req_id++;
    if (s_rate_switch_req_id == 0) s_rate_switch_req_id = 1;
    s_rate_switch_requested    = true;
    s_rate_switch_ack_received = false;
    s_rate_switch_req_tx_ms    = 0;
    s_rate_switch_req_start_ms = millis();
    e220_stats.rate_switch_pending = true;
    Serial.printf("[E220 LORA] Queued air-rate switch request -> %.1fk (Req #%u)\n",
                  target_rate_kbps, s_rate_switch_req_id);
    return true;
}

extern "C" bool e220_is_rate_switch_pending(void)
{
    return e220_stats.rate_switch_pending;
}

extern "C" float e220_get_active_air_rate(void)
{
    return s_active_air_rate;
}

extern "C" float e220_get_requested_air_rate(void)
{
    return s_requested_rate_kbps;
}

extern "C" void e220_get_stats(E220LinkStats_t *stats_out)
{
    if (stats_out) {
        e220_stats.aux_fall_count  = isr_aux_fall_count;
        e220_stats.aux_last_low_us = isr_aux_last_low_us;
        *stats_out = e220_stats;
    }
}
