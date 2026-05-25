#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS task entry point for the IMU pipeline.
 *
 * Called by @c osThreadNew() in @c main.c. Runs the LSM6DSV self-test,
 * initialises the IMU, calibrates the accelerometer bias, signals the
 * ranging task that it is ready, then loops waiting for ranging-task
 * trigger flags.
 *
 * @param argument  Unused; required by the CMSIS-RTOS2 task signature.
 */
void StartAcc(void *argument);

/**
 * @brief Read back the most recent IMU integration results.
 *
 * All three outputs are updated atomically from the module-static variables
 * written by @ref StartAcc after each @ref imu_integrate call.
 *
 * @param[out] out_pitch_rad     Pitch angle in radians.
 * @param[out] out_speed_horiz   Horizontal speed magnitude in m/s (>= 0).
 * @param[out] out_vel_z         Vertical velocity in m/s (signed).
 */
void imu_get_results(float *out_pitch_rad,
                     float *out_speed_horiz,
                     float *out_vel_z);

#ifdef __cplusplus
}
#endif