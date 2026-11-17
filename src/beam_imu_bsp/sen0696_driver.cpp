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
constexpr uint32_t kI2cClockHz          = 400000U;
constexpr uint32_t kSclWaitUs           = 20000U;
constexpr int      kI2cXferTimeoutMs    = 50;
constexpr uint32_t kTwoPhaseDelayMs     = 10U; // Bridge MCU needs >= 10 ms between addr write and read

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

bool sen0696_write_reg16(i2c_master_dev_handle_t dev, uint16_t reg16, uint16_t val16)
{
    uint8_t tx[4] = {
        (uint8_t)(reg16 & 0xFFU),
        (uint8_t)((reg16 >> 8) & 0xFFU),
        (uint8_t)(val16 & 0xFFU),
        (uint8_t)((val16 >> 8) & 0xFFU),
    };

    if (!i2c_bsp_lock(200)) {
        return false;
    }
    esp_err_t err = i2c_master_transmit(dev, tx, sizeof(tx), kI2cXferTimeoutMs);
    i2c_bsp_unlock();
    return (err == ESP_OK);
}

bool sen0696_read_reg(i2c_master_dev_handle_t dev, uint16_t reg16, uint8_t *out, size_t len)
{
    if (out == nullptr || len == 0U) {
        return false;
    }

    uint8_t reg_addr[2] = {
        (uint8_t)(reg16 & 0xFFU),
        (uint8_t)((reg16 >> 8) & 0xFFU),
    };

    // Hold i2c_bsp_lock across the STOP -> 10 ms bridge preparation gap -> READ phase
    // so no concurrent bus activity interrupts the two-phase SEN0696 transaction.
    if (!i2c_bsp_lock(200)) {
        return false;
    }

    esp_err_t err = i2c_master_transmit(dev, reg_addr, sizeof(reg_addr), kI2cXferTimeoutMs);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(kTwoPhaseDelayMs));
        err = i2c_master_receive(dev, out, len, kI2cXferTimeoutMs);
    }

    i2c_bsp_unlock();
    return (err == ESP_OK);
}

bool sen0696_read_u16(i2c_master_dev_handle_t dev, uint16_t reg16, uint16_t *out_val)
{
    uint8_t rx[2] = {0};
    if (!sen0696_read_reg(dev, reg16, rx, sizeof(rx))) {
        return false;
    }
    *out_val = parse_u16_le(rx);
    return true;
}

void sen0696_read_sensors(i2c_master_dev_handle_t dev, sen0696_boot_report_t *report)
{
    uint8_t imu_raw[12] = {0};
    if (sen0696_read_reg(dev, kRegAccDataX, imu_raw, sizeof(imu_raw))) {
        const float acc_scale = kAccelRangeG / 32768.0f;
        const float gyr_scale = kGyroRangeDps / 32768.0f;
        for (int i = 0; i < 3; ++i) {
            report->accel_g[i]  = (float)parse_s16_le(&imu_raw[i * 2]) * acc_scale;
            report->gyro_dps[i] = (float)parse_s16_le(&imu_raw[6 + (i * 2)]) * gyr_scale;
        }
        report->accel_gyro_valid = true;
    }

    uint8_t mag_raw[12] = {0};
    if (sen0696_read_reg(dev, kRegMagDataXLowWord, mag_raw, sizeof(mag_raw))) {
        for (int i = 0; i < 3; ++i) {
            report->mag_ut[i] = (float)parse_s32_le(&mag_raw[i * 4]) * 0.01f;
        }
        report->mag_norm_ut = sqrtf((report->mag_ut[0] * report->mag_ut[0]) +
                                    (report->mag_ut[1] * report->mag_ut[1]) +
                                    (report->mag_ut[2] * report->mag_ut[2]));
        report->mag_valid = true;
    }

    uint8_t press_raw[4] = {0};
    if (sen0696_read_reg(dev, kRegPressDataLowWord, press_raw, sizeof(press_raw))) {
        int32_t press_pa = parse_s32_le(press_raw);
        if (press_pa <= 0) {
            // Allow one extra barometer conversion cycle if polled immediately after wakeup
            vTaskDelay(pdMS_TO_TICKS(50));
            if (sen0696_read_reg(dev, kRegPressDataLowWord, press_raw, sizeof(press_raw))) {
                press_pa = parse_s32_le(press_raw);
            }
        }
        if (press_pa > 0) {
            report->pressure_pa  = (float)press_pa;
            report->pressure_hpa = report->pressure_pa * 0.01f;
            report->altitude_m   = 44307.7f * (1.0f - powf(report->pressure_pa / 101325.0f, 0.190284f));
            report->press_valid  = true;
        }
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

bool sen0696_probe_and_read_once(uint8_t addr_min, uint8_t addr_max, sen0696_boot_report_t *report)
{
    if (report == nullptr) {
        return false;
    }
    memset(report, 0, sizeof(*report));

    if (user_i2c_port0_handle == nullptr) {
        return false;
    }

    const uint8_t addr = sen0696_detect_i2c_addr(addr_min, addr_max);
    if (addr == 0U) {
        return false;
    }

    report->found    = true;
    report->i2c_addr = addr;

    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = addr;
    dev_cfg.scl_speed_hz    = kI2cClockHz;
    dev_cfg.scl_wait_us     = kSclWaitUs;

    i2c_master_dev_handle_t dev = nullptr;
    if (!i2c_bsp_lock(200)) {
        return false;
    }
    const esp_err_t add_err = i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &dev);
    i2c_bsp_unlock();

    if (add_err != ESP_OK || dev == nullptr) {
        return false;
    }

    bool ok_vid = sen0696_read_u16(dev, kRegVid, &report->vid);
    bool ok_pid = sen0696_read_u16(dev, kRegPid, &report->pid);
    (void)sen0696_read_u16(dev, kRegFwVersion, &report->fw_version);

    report->id_valid = ok_vid && ok_pid && (report->pid >= SEN0696_MIN_10DOF_PID);

    if (report->id_valid) {
        const bool ok_mode = sen0696_write_reg16(dev, kRegSensorsMode, kModeNormal);
        vTaskDelay(pdMS_TO_TICKS(25));
        const bool ok_acc  = sen0696_write_reg16(dev, kRegAccRangeConf, kAccelRange2G);
        vTaskDelay(pdMS_TO_TICKS(10));
        const bool ok_gyr  = sen0696_write_reg16(dev, kRegGyrRangeConf, kGyroRange250Dps);
        vTaskDelay(pdMS_TO_TICKS(10));

        report->configured = ok_mode && ok_acc && ok_gyr;

        // Allow BMI323 + BMM350 + BMP581 ODR conversion cycles after wakeup before sampling
        vTaskDelay(pdMS_TO_TICKS(120));
        sen0696_read_sensors(dev, report);
    }

    if (i2c_bsp_lock(200)) {
        (void)i2c_master_bus_rm_device(dev);
        i2c_bsp_unlock();
    }

    return report->id_valid && report->accel_gyro_valid && report->mag_valid && report->press_valid;
}
