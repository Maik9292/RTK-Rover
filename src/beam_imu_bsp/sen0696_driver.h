#ifndef SEN0696_DRIVER_H
#define SEN0696_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

// Default DFRobot Gravity SEN0696 (BMI323 + BMM350 + BMP581) I2C address range (A1/A0 DIP)
#define SEN0696_I2C_ADDR_DEFAULT     0x4AU
#define SEN0696_I2C_ADDR_MAX         0x4DU

// Verified DFRobot_Multi_DOF_IMU I2C identity constants
#define SEN0696_EXPECTED_VID         0x3343U
#define SEN0696_MIN_10DOF_PID        0x0010U

/**
 * @brief One-shot power-up verification and sensor snapshot for DFRobot SEN0696 (10DOF).
 */
typedef struct {
    bool     found;            // True if any address in [addr_min..addr_max] ACKed on I2C0
    bool     id_valid;         // True if PID >= 0x0010 (10DOF compatible)
    bool     configured;       // True if Normal mode + +/-2g + +/-250dps writes succeeded
    bool     accel_gyro_valid; // True if 12-byte IMU block read succeeded
    bool     mag_valid;        // True if 12-byte magnetometer block read succeeded
    bool     press_valid;      // True if 4-byte barometer block read succeeded
    uint8_t  i2c_addr;         // Detected 7-bit I2C address (0 if not found)
    uint16_t vid;              // REG_I2C_VID (0x0000), expected 0x3343 (DFRobot)
    uint16_t pid;              // REG_I2C_PID (0x0001), expected >= 0x0010 (10DOF)
    uint16_t fw_version;       // REG_I2C_FW_VERSION (0x0002)
    float    accel_g[3];       // Accelerometer X, Y, Z in g (+/-2g scale)
    float    gyro_dps[3];      // Gyroscope X, Y, Z in deg/s (+/-250dps scale)
    float    mag_ut[3];        // Magnetometer X, Y, Z in uT (0.01 uT/LSB)
    float    mag_norm_ut;      // Magnetic field magnitude |B| in uT
    float    pressure_pa;      // Barometric pressure in Pa (1 Pa/LSB)
    float    pressure_hpa;     // Barometric pressure in hPa
    float    altitude_m;       // Standard-atmosphere altitude estimate (P0 = 101325 Pa)
} sen0696_boot_report_t;

/**
 * @brief Quietly probes [addr_min..addr_max] on shared I2C0 without tripping bus recovery.
 * @return First responding 7-bit I2C address, or 0 if no SEN0696 responded.
 */
uint8_t sen0696_detect_i2c_addr(uint8_t addr_min, uint8_t addr_max);

/**
 * @brief Performs a one-shot boot verification and initial 10DOF read on I2C0.
 *
 * Uses a temporary ESP-IDF i2c_master_dev_handle_t attached to user_i2c_port0_handle
 * under i2c_bsp_lock(), configures normal mode (+/-2g, +/-250dps), waits for first
 * conversion settling, reads Accel/Gyro/Mag/Pressure, and detaches cleanly.
 * Note: DFRobot's SEN0696 onboard I2C bridge firmware (regs 0x0000..0x0021) does not
 * expose an external temperature register over I2C.
 *
 * @param addr_min Minimum 7-bit I2C address to probe (e.g., 0x4A).
 * @param addr_max Maximum 7-bit I2C address to probe (e.g., 0x4D).
 * @param report   Output report struct (must be non-NULL).
 * @return true if device was found, verified 10DOF PID, and all active sensors read cleanly.
 */
bool sen0696_probe_and_read_once(uint8_t addr_min, uint8_t addr_max, sen0696_boot_report_t *report);

#endif // SEN0696_DRIVER_H
