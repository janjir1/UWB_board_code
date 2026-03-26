#ifndef IMU_SETUP_H
#define IMU_SETUP_H

#include "lsm6dsv_reg.h"

/* ================================================================
 * imu_setup.h
 * Central configuration for the LSM6DSV IMU.
 * Change values here only — never edit imu_init() directly.
 * ================================================================ */


/* ── Boot ────────────────────────────────────────────────────────── */

#define IMU_BOOT_TIME_MS        10      // sensor startup delay, do not go below 10


/* ── Accelerometer ───────────────────────────────────────────────── */

/* Output data rate — how often a new accel sample is produced
 * Must match IMU_FIFO_XL_BATCH below.
 *
 * Options (with I2C read time for 200ms FIFO window):
 *   LSM6DSV_ODR_AT_7Hz5   →  ~1–2 samples,  I2C Fast ~1.0ms  ← current
 *   LSM6DSV_ODR_AT_15Hz   →  ~3 samples,    I2C Fast ~1.5ms
 *   LSM6DSV_ODR_AT_30Hz   →  ~6 samples,    I2C Fast ~2.5ms
 *   LSM6DSV_ODR_AT_60Hz   →  ~12 samples,   I2C Fast ~4.5ms
 *   LSM6DSV_ODR_AT_120Hz  →  ~24 samples,   I2C Fast ~8.5ms  (SPI recommended)
 */
#define IMU_XL_ODR              LSM6DSV_ODR_AT_7Hz5

/* Full scale — maximum measurable acceleration
 *   LSM6DSV_2g   → 0.061 mg/LSB  best precision, clips above 2g  ← current
 *   LSM6DSV_4g   → 0.122 mg/LSB
 *   LSM6DSV_8g   → 0.244 mg/LSB
 *   LSM6DSV_16g  → 0.488 mg/LSB  use for high-impact / vibration
 */
#define IMU_XL_FULL_SCALE       LSM6DSV_2g

/* Low-pass filter 2 bandwidth — higher = less filtering, less phase lag
 * At low ODR use STRONG. At high ODR switch to LIGHT to preserve signal.
 *   LSM6DSV_XL_ULTRA_LIGHT   → ODR/2   minimal filtering
 *   LSM6DSV_XL_VERY_LIGHT    → ODR/4
 *   LSM6DSV_XL_LIGHT         → ODR/5
 *   LSM6DSV_XL_MEDIUM        → ODR/7
 *   LSM6DSV_XL_STRONG        → ODR/9   ← current (good for 7.5–30 Hz)
 *   LSM6DSV_XL_VERY_STRONG   → ODR/20
 */
#define IMU_XL_LP2_BW           LSM6DSV_XL_STRONG


/* ── Gyroscope ───────────────────────────────────────────────────── */

/* Output data rate — should match IMU_XL_ODR in most cases
 * Same options as accel ODR above.
 */
#define IMU_GY_ODR              LSM6DSV_ODR_AT_7Hz5

/* Full scale — maximum measurable angular rate
 *   LSM6DSV_125dps   → 4.375 mdps/LSB  highest precision
 *   LSM6DSV_250dps   → 8.75  mdps/LSB
 *   LSM6DSV_500dps   → 17.5  mdps/LSB  good for slow human motion
 *   LSM6DSV_1000dps  → 35    mdps/LSB
 *   LSM6DSV_2000dps  → 70    mdps/LSB  ← current (safe for any motion)
 *   LSM6DSV_4000dps  → 140   mdps/LSB  motorsport / robotics
 */
#define IMU_GY_FULL_SCALE       LSM6DSV_2000dps

/* Low-pass filter 1 bandwidth — keep ULTRA_LIGHT for integration.
 * Heavier settings introduce phase lag that corrupts quaternion output.
 *   LSM6DSV_GY_ULTRA_LIGHT   → widest BW, minimal lag  ← current
 *   LSM6DSV_GY_VERY_LIGHT
 *   LSM6DSV_GY_LIGHT
 *   LSM6DSV_GY_MEDIUM
 *   LSM6DSV_GY_STRONG        → narrowest BW, most lag
 */
#define IMU_GY_LP1_BW           LSM6DSV_GY_ULTRA_LIGHT


/* ── SFLP (Sensor Fusion Low Power) ──────────────────────────────── */

/* SFLP output data rate — independent of accel/gyro ODR
 * Minimum recommended: 2× your main ODR.
 *   LSM6DSV_SFLP_15Hz   ← current
 *   LSM6DSV_SFLP_30Hz
 *   LSM6DSV_SFLP_60Hz
 *   LSM6DSV_SFLP_120Hz
 */
#define IMU_SFLP_ODR            LSM6DSV_SFLP_15Hz

/* Which SFLP outputs to batch into FIFO */
#define IMU_SFLP_BATCH_QUAT     PROPERTY_ENABLE   // game rotation quaternion x,y,z
#define IMU_SFLP_BATCH_GRAVITY  PROPERTY_ENABLE   // gravity vector — needed for accel compensation
#define IMU_SFLP_BATCH_GBIAS    PROPERTY_DISABLE  // gyro bias — enable if you want raw bias values


/* ── FIFO batching ───────────────────────────────────────────────── */

/* Must match IMU_XL_ODR / IMU_GY_ODR above.
 * Accel options: LSM6DSV_XL_NOT_BATCHED, LSM6DSV_XL_BATCHED_AT_7Hz5 ... _AT_240Hz
 * Gyro options:  LSM6DSV_GY_NOT_BATCHED, LSM6DSV_GY_BATCHED_AT_7Hz5 ... _AT_240Hz
 */
#define IMU_FIFO_XL_BATCH       LSM6DSV_XL_BATCHED_AT_7Hz5
#define IMU_FIFO_GY_BATCH       LSM6DSV_GY_BATCHED_AT_7Hz5

/* Timestamp decimation — how often a timestamp entry appears in FIFO
 * relative to the accel/gyro batch rate.
 *   LSM6DSV_TMSTMP_DEC_1   → every sample     ← current
 *   LSM6DSV_TMSTMP_DEC_8   → every 8 samples
 *   LSM6DSV_TMSTMP_DEC_32  → every 32 samples
 */
#define IMU_FIFO_TS_DEC         LSM6DSV_TMSTMP_DEC_1

/* Maximum samples to read per FIFO drain.
 * At 7.5 Hz over 200ms: 2 accel + 2 gyro + 3 quat + 3 gravity + 2 ts = ~12 entries
 * Set higher when increasing ODR.
 *   7.5  Hz → 16
 *   15   Hz → 32
 *   52   Hz → 64
 *   104  Hz → 128
 */
#define IMU_MAX_SAMPLES         16

#endif /* IMU_SETUP_H */