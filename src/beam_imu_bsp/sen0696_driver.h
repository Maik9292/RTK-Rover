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
 * @brief Persistent state and latest physical readings for DFRobot SEN0696 (10DOF IMU).
 */
typedef struct {
    bool     present;          // True if detected on I2C0 and verified PID >= 0x0010 at boot
    bool     id_valid;         // True if REG_I2C_PID >= 0x0010
    bool     configured;       // True if Normal mode + +/-2g + +/-250dps writes succeeded
    bool     valid;            // True if the latest sample read all 3 blocks cleanly
    bool     accel_gyro_valid; // True if latest 12-byte IMU block read passed sanity checks
    bool     mag_valid;        // True if latest 12-byte magnetometer block passed sanity checks
    bool     press_valid;      // True if latest 4-byte barometer block passed sanity checks
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
    uint8_t  consec_fails;     // Consecutive poll errors since last valid sample
    uint32_t last_sample_ms;   // FreeRTOS tick timestamp (ms) of last valid sample
} sen0696_state_t;

/**
 * @brief Quietly probes [addr_min..addr_max] on shared I2C0 without tripping bus recovery.
 * @return First responding 7-bit I2C address, or 0 if no SEN0696 responded.
 */
uint8_t sen0696_detect_i2c_addr(uint8_t addr_min, uint8_t addr_max);

/**
 * @brief Initializes DFRobot SEN0696 on I2C0 (uniform 400 kHz), attaches persistent handle, and reads initial sample.
 *
 * Configures Normal operating mode (+/-2g accel, +/-250dps gyro) and waits for first sensor
 * conversion settling. Note: DFRobot's SEN0696 onboard I2C bridge firmware (regs 0x0000..0x0021)
 * does not expose an external temperature register over I2C.
 */
bool sen0696_init(uint8_t addr_min, uint8_t addr_max, sen0696_state_t *out_state);

/**
 * @brief Returns true if SEN0696 was detected and verified at boot.
 */
bool sen0696_is_present(void);

/**
 * @brief Returns consecutive error count (useful for backing off poll cadence when disconnected).
 */
uint8_t sen0696_get_consec_fails(void);

/**
 * @brief Executes one staggered acquisition step (step 0=IMU, 1=Mag, 2=Baro/Finalize).
 *
 * Holds i2c_bsp_lock() atomically across each single block's [write -> 10 ms -> read]
 * sequence so Touch (0x3B) and QMI8658 (0x6B) never split an active SEN0696 transaction,
 * while capping max single-tick bus lock hold time to ~10.3 ms across 3 RoverUI ticks.
 *
 * @param step      0, 1, or 2.
 * @param out_state Optional snapshot updated cleanly after step 2 completes (may be NULL).
 * @return true after step 2 if all 3 sensor blocks succeeded on this cycle.
 */
bool sen0696_poll_step(uint8_t step, sen0696_state_t *out_state);

/**
 * @brief Convenience blocking call that executes steps 0, 1, and 2 sequentially.
 */
bool sen0696_poll(sen0696_state_t *out_state);

#endif // SEN0696_DRIVER_H
