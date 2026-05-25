#ifndef IMU_H
#define IMU_H

#include "imu_setup.h"
#include "main.h"

/** @brief One IMU FIFO sample — accel (raw int16) paired with SFLP gravity (f16). */
typedef struct {
    int16_t  accel_raw[3];       /**< Raw accelerometer counts; convert with lsm6dsv_from_fs2_to_mg(). */
    uint16_t gravity_raw[3];     /**< SFLP gravity vector in f16 (x, y, z). */
    uint32_t timestamp_ticks;    /**< LSM6DSV hardware timestamp; 25 µs per tick. */
} imu_sample_t;

/** @brief Batch of samples drained from the IMU FIFO in one call to @ref imu_read_fifo. */
typedef struct {
    imu_sample_t samples[IMU_MAX_SAMPLES]; /**< Sample array. */
    uint16_t     count;                    /**< Number of valid entries in @c samples. */
} imu_fifo_data_t;

/**
 * @brief Convert a 16-bit half-precision float to single precision.
 *
 * Delegates to the ST driver's @c lsm6dsv_from_f16_to_f32() which returns
 * the bit pattern as a uint32; this helper reinterprets it as a float via
 * a union to avoid strict-aliasing undefined behaviour.
 *
 * @param h  Half-precision value.
 * @return   Single-precision equivalent.
 */
static inline float f16_to_f32(uint16_t h)
{
    union { float f; uint32_t u; } conv;
    conv.u = lsm6dsv_from_f16_to_f32(h);
    return conv.f;
}

/** @brief Accelerometer X axis in m/s² (raw → mg → m/s²). */
#define IMU_ACCEL_X_MS2(s) (lsm6dsv_from_fs2_to_mg((s)->accel_raw[0]) * 0.00981f)
/** @brief Accelerometer Y axis in m/s². */
#define IMU_ACCEL_Y_MS2(s) (lsm6dsv_from_fs2_to_mg((s)->accel_raw[1]) * 0.00981f)
/** @brief Accelerometer Z axis in m/s². */
#define IMU_ACCEL_Z_MS2(s) (lsm6dsv_from_fs2_to_mg((s)->accel_raw[2]) * 0.00981f)

/** @brief SFLP gravity X component in m/s² (f16 → mg → m/s²). */
#define IMU_GRAV_X(s) (lsm6dsv_from_sflp_to_mg((s)->gravity_raw[0]) * 0.00981f)
/** @brief SFLP gravity Y component in m/s². */
#define IMU_GRAV_Y(s) (lsm6dsv_from_sflp_to_mg((s)->gravity_raw[1]) * 0.00981f)
/** @brief SFLP gravity Z component in m/s². */
#define IMU_GRAV_Z(s) (lsm6dsv_from_sflp_to_mg((s)->gravity_raw[2]) * 0.00981f)

void imu_init(void);
void imu_read_fifo(imu_fifo_data_t *fifo);

void imu_integrate(const imu_fifo_data_t *fifo,
                   float    *out_pitch_rad,
                   float    *out_speed_horiz,
                   float    *out_vel_z,
                   uint32_t *out_ticks_elapsed);

void imu_calibrate(void);

#ifdef UWB_DEBUG
void imu_print_results(float    pitch_rad,
                       float    speed_horiz,
                       float    vel_z,
                       uint32_t ticks_elapsed);
#endif /* UWB_DEBUG */

#endif /* IMU_H */