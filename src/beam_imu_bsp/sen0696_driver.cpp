#include "sen0696_driver.h"

#include <math.h>
#include <string.h>

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../i2c_bsp/i2c_bsp.h"

// Verified I2C register map from DFRobot_Multi_DOF_IMU.h (16-bit LE register addresses)
namespace {

constexpr uint16_t kRegVid              = 0x0000U;
constexpr uint16_t kRegPid              = 0x0001U;
constexpr uint16_t kRegFwVersion        = 0x0002U;
constexpr uint16_t kRegSensorsMode      = 0x0005U;
constexpr uint16_t kRegAccRangeConf     = 0x0006U;
constexpr uint16_t kRegGyrRangeConf     = 0x0007U;
constexpr uint16_t kRegAccDataX         = 0x0010U; // 12 B: Accel X/Y/Z (int16) + Gyro X/Y/Z (int16)
constexpr uint16_t kRegMagDataXLowWord  = 0x0016U; // 12 B: Mag X/Y/Z (int32, 0.01 uT/LSB)
constexpr uint16_t kRegPressDataLowWord = 0x001CU; //  4 B: Pressure (int32, Pa)

constexpr uint16_t kModeNormal          = 0x0002U; // eNormalMode
constexpr uint16_t kAccelRange2G        = 0x0000U; // +/-2g
constexpr uint16_t kGyroRange250Dps     = 0x0001U; // +/-250 deg/s

constexpr float    kAccelRangeG         = 2.0f;
constexpr float    kGyroRangeDps        = 250.0f;
constexpr int      kI2cXferTimeoutMs    = 50;
constexpr uint32_t kTwoPhaseDelayMs     = 10U; // Bridge MCU needs >= 10 ms between addr write and read
constexpr uint8_t  kReconfigFailMod     = 10U; // Re-apply sensor mode every 10 consecutive poll errors

sen0696_state_t s_state = {};

int16_t parse_s16_le(const uint8_t *buf)
{
    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return (int16_t)raw;
}

uint16_t parse_u16_le(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

int32_t parse_s32_le(const uint8_t *buf)
{
    uint32_t raw = (uint32_t)buf[0]
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16)
                 | ((uint32_t)buf[3] << 24);
    return (int32_t)raw;
}

bool sen0696_write_reg16(uint16_t reg16, uint16_t val16)
{
    uint8_t tx[4] = {
        (uint8_t)(reg16 & 0xFFU),
        (uint8_t)((reg16 >> 8) & 0xFFU),
        (uint8_t)(val16 & 0xFFU),
        (uint8_t)((val16 >> 8) & 0xFFU),
    };

    if (!i2c_bsp_lock(100)) {
        return false;
    }
    if (beam_imu_dev_handle == nullptr) {
        i2c_bsp_unlock();
        return false;
    }
    const esp_err_t err = i2c_master_transmit(beam_imu_dev_handle, tx, sizeof(tx), kI2cXferTimeoutMs);
    i2c_bsp_unlock();
    return (err == ESP_OK);
}

bool sen0696_read_reg(uint16_t reg16, uint8_t *out, size_t len)
{
    if (out == nullptr || len == 0U) {
        return false;
    }

    uint8_t reg_addr[2] = {
        (uint8_t)(reg16 & 0xFFU),
        (uint8_t)((reg16 >> 8) & 0xFFU),
    };

    // Phase 1: send 16-bit LE register address under I2C0 lock
    if (!i2c_bsp_lock(100)) {
        return false;
    }
    if (beam_imu_dev_handle == nullptr) {
        i2c_bsp_unlock();
        return false;
    }
    esp_err_t err = i2c_master_transmit(beam_imu_dev_handle, reg_addr, sizeof(reg_addr), kI2cXferTimeoutMs);
    i2c_bsp_unlock();
    if (err != ESP_OK) {
        return false;
    }

    // Phase 2: release I2C0 lock while SEN0696 bridge MCU prepares data so Touch/QMI8658 stay unblocked.
    // Only SEN0696 driver code ever addresses 0x4A..0x4D.
    vTaskDelay(pdMS_TO_TICKS(kTwoPhaseDelayMs));

    // Phase 3: briefly reacquire lock to read payload bytes (< 0.5 ms @ 400 kHz)
    if (!i2c_bsp_lock(100)) {
        return false;
    }
    if (beam_imu_dev_handle == nullptr) {
        i2c_bsp_unlock();
        return false;
    }
    err = i2c_master_receive(beam_imu_dev_handle, out, len, kI2cXferTimeoutMs);
    i2c_bsp_unlock();
    return (err == ESP_OK);
}

bool sen0696_read_u16(uint16_t reg16, uint16_t *out_val)
{
    uint8_t rx[2] = {0};
    if (!sen0696_read_reg(reg16, rx, sizeof(rx))) {
        return false;
    }
    *out_val = parse_u16_le(rx);
    return true;
}

bool sen0696_configure_device(void)
{
    const bool ok_mode = sen0696_write_reg16(kRegSensorsMode, kModeNormal);
    vTaskDelay(pdMS_TO_TICKS(25));
    const bool ok_acc  = sen0696_write_reg16(kRegAccRangeConf, kAccelRange2G);
    vTaskDelay(pdMS_TO_TICKS(10));
    const bool ok_gyr  = sen0696_write_reg16(kRegGyrRangeConf, kGyroRange250Dps);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ok_mode && ok_acc && ok_gyr;
}

void sen0696_read_blocks(bool allow_press_retry, sen0696_state_t *st)
{
    st->accel_gyro_valid = false;
    st->mag_valid        = false;
    st->press_valid      = false;

    uint8_t imu_raw[12] = {0};
    if (sen0696_read_reg(kRegAccDataX, imu_raw, sizeof(imu_raw))) {
        const float acc_scale = kAccelRangeG / 32768.0f;
        const float gyr_scale = kGyroRangeDps / 32768.0f;
        for (int i = 0; i < 3; ++i) {
            st->accel_g[i]  = (float)parse_s16_le(&imu_raw[i * 2]) * acc_scale;
            st->gyro_dps[i] = (float)parse_s16_le(&imu_raw[6 + (i * 2)]) * gyr_scale;
        }
        st->accel_gyro_valid = true;
    }

    uint8_t mag_raw[12] = {0};
    if (sen0696_read_reg(kRegMagDataXLowWord, mag_raw, sizeof(mag_raw))) {
        for (int i = 0; i < 3; ++i) {
            st->mag_ut[i] = (float)parse_s32_le(&mag_raw[i * 4]) * 0.01f;
        }
        st->mag_norm_ut = sqrtf((st->mag_ut[0] * st->mag_ut[0]) +
                                (st->mag_ut[1] * st->mag_ut[1]) +
                                (st->mag_ut[2] * st->mag_ut[2]));
        st->mag_valid = true;
    }

    uint8_t press_raw[4] = {0};
    if (sen0696_read_reg(kRegPressDataLowWord, press_raw, sizeof(press_raw))) {
        int32_t press_pa = parse_s32_le(press_raw);
        if (press_pa <= 0 && allow_press_retry) {
            // Allow one extra barometer conversion cycle immediately after cold wakeup
            vTaskDelay(pdMS_TO_TICKS(50));
            if (sen0696_read_reg(kRegPressDataLowWord, press_raw, sizeof(press_raw))) {
                press_pa = parse_s32_le(press_raw);
            }
        }
        if (press_pa > 0) {
            st->pressure_pa  = (float)press_pa;
            st->pressure_hpa = st->pressure_pa * 0.01f;
            st->altitude_m   = 44307.7f * (1.0f - powf(st->pressure_pa / 101325.0f, 0.190284f));
            st->press_valid  = true;
        }
    }
}

void copy_state_out(sen0696_state_t *out_state)
{
    if (out_state != nullptr) {
        *out_state = s_state;
    }
}

} // namespace

uint8_t sen0696_detect_i2c_addr(uint8_t addr_min, uint8_t addr_max)
{
    if (addr_min > addr_max || addr_max > 0x77U) {
        return 0U;
    }

    for (uint8_t addr = addr_min; addr <= addr_max; ++addr) {
        if (i2c_bsp_probe_optional_addr(addr)) {
            return addr;
        }
    }
    return 0U;
}

bool sen0696_init(uint8_t addr_min, uint8_t addr_max, sen0696_state_t *out_state)
{
    memset(&s_state, 0, sizeof(s_state));

    if (user_i2c_port0_handle == nullptr) {
        copy_state_out(out_state);
        return false;
    }

    const uint8_t addr = sen0696_detect_i2c_addr(addr_min, addr_max);
    if (addr == 0U) {
        copy_state_out(out_state);
        return false;
    }

    s_state.i2c_addr = addr;

    if (!i2c_bsp_attach_beam_imu(addr)) {
        copy_state_out(out_state);
        return false;
    }

    const bool ok_vid = sen0696_read_u16(kRegVid, &s_state.vid);
    const bool ok_pid = sen0696_read_u16(kRegPid, &s_state.pid);
    (void)sen0696_read_u16(kRegFwVersion, &s_state.fw_version);

    s_state.id_valid = ok_vid && ok_pid && (s_state.pid >= SEN0696_MIN_10DOF_PID);
    if (!s_state.id_valid) {
        i2c_bsp_detach_beam_imu();
        copy_state_out(out_state);
        return false;
    }

    s_state.present    = true;
    s_state.configured = sen0696_configure_device();

    // Allow BMI323 + BMM350 + BMP581 ODR conversion cycles after wakeup before first sample
    vTaskDelay(pdMS_TO_TICKS(120));
    sen0696_read_blocks(true, &s_state);

    s_state.valid = s_state.accel_gyro_valid && s_state.mag_valid && s_state.press_valid;
    if (s_state.valid) {
        s_state.last_sample_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }

    copy_state_out(out_state);
    return s_state.valid;
}

bool sen0696_is_present(void)
{
    return s_state.present;
}

bool sen0696_poll(sen0696_state_t *out_state)
{
    if (!s_state.present) {
        copy_state_out(out_state);
        return false;
    }

    sen0696_read_blocks(false, &s_state);
    if (s_state.accel_gyro_valid && s_state.mag_valid && s_state.press_valid) {
        s_state.valid          = true;
        s_state.consec_fails   = 0U;
        s_state.last_sample_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    } else {
        s_state.valid = false;
        if (s_state.consec_fails < 255U) {
            s_state.consec_fails++;
        }
        // Re-apply configuration gently on sustained errors in case SEN0696 reset to sleep mode
        if ((s_state.consec_fails % kReconfigFailMod) == 0U) {
            s_state.configured = sen0696_configure_device();
        }
    }

    copy_state_out(out_state);
    return s_state.valid;
}
