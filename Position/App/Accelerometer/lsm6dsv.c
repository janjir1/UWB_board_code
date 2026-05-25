#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cmsis_os2.h"
#include "lsm6dsv_ST_example.h"

#include "main.h"
#include "../Generic/my_print.h"
#include "stm32l4xx_hal.h"
#include "imu.h"

/* -- Module-static IMU results — written by StartAcc, read by imu_get_results -- */
static float    pitch_rad    = 0.0f;
static float    speed_horiz  = 0.0f;
static float    vel_z        = 0.0f;
static uint32_t ticks_elapsed = 0;

/**
 * @brief FreeRTOS task entry point for the IMU pipeline.
 *
 * Startup sequence:
 * 1. LSM6DSV self-test — task deleted on failure.
 * 2. @ref imu_init — configures FIFO, filters, ODR.
 * 3. 200 ms settle delay.
 * 4. @ref imu_calibrate — collects accelerometer bias.
 * 5. Signals the ranging task (flag 0x01) that the IMU is ready.
 *
 * Loop (triggered by ranging task flag 0x01):
 * - Drains the FIFO, integrates velocity and pitch.
 * - Signals the ranging task (flag 0x02) that results are ready.
 *
 * @param argument  Unused; required by the CMSIS-RTOS2 task signature.
 */
void StartAcc(void *argument)
{
    bool passed = SelfTest();
    if (passed) {
        mprintf("LSM6DSV self test passed\r\n");
    } else {
        mprintf("LSM6DSV self test failed\r\n");
        vTaskDelete(NULL);
        while (1) {}
    }

    imu_init();
    osDelay(200);
    imu_calibrate();

    osThreadFlagsSet(RangingHandle, 0x01);

    while (1) {
        osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);

        imu_fifo_data_t fifo;
        imu_read_fifo(&fifo);
        imu_integrate(&fifo, &pitch_rad, &speed_horiz, &vel_z, &ticks_elapsed);

        osThreadFlagsSet(RangingHandle, 0x02);

#ifdef UWB_DEBUG
        imu_print_results(pitch_rad, speed_horiz, vel_z, ticks_elapsed);
#endif
    }

    vTaskDelete(NULL);
    while (1) {}
}

/**
 * @brief Read back the most recent IMU integration results.
 *
 * @param[out] out_pitch_rad    Pitch angle in radians.
 * @param[out] out_speed_horiz  Horizontal speed magnitude in m/s.
 * @param[out] out_vel_z        Vertical velocity in m/s.
 */
void imu_get_results(float *out_pitch_rad,
                     float *out_speed_horiz,
                     float *out_vel_z)
{
    *out_pitch_rad   = pitch_rad;
    *out_speed_horiz = speed_horiz;
    *out_vel_z       = vel_z;
}