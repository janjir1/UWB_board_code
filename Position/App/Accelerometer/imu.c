#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "lsm6dsv_reg.h"
#include "lsm6dsv_platform.h"
#include "../Generic/my_print.h"
#include "imu_setup.h"
#include "lsm6dsv_ST_example.h"
#include "imu.h"

#define    BOOT_TIME            10 //ms

stmdev_ctx_t dev_ctx;
static lsm6dsv_filt_settling_mask_t filt_settling_mask;

static uint32_t last_ts   = 0;

static float s_accel_bias[3] = {0.0f, 0.0f, 0.0f};
static bool  s_calibrated    = false;

void imu_calibrate(void)
{
    float acc[3] = {0.0f, 0.0f, 0.0f};
    uint8_t n    = 0;

    /* Flush any stale FIFO entries first */
    lsm6dsv_fifo_status_t status;
    lsm6dsv_fifo_status_get(&dev_ctx, &status);
    for (uint16_t i = 0; i < status.fifo_level; i++) {
        lsm6dsv_fifo_out_raw_t dummy;
        lsm6dsv_fifo_out_raw_get(&dev_ctx, &dummy);
    }

    mprintf("[IMU] Calibrating bias (%d samples)...", IMU_CAL_SAMPLES);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    while (n < IMU_CAL_SAMPLES)
    {
        platform_delay(100 / IMU_CAL_ODR_HZ);  /* wait ~1 ODR period */

        imu_fifo_data_t fifo;
        imu_read_fifo(&fifo);

        for (uint16_t i = 0; i < fifo.count; i++)
        {
            const imu_sample_t *s = &fifo.samples[i];

            float ax = IMU_ACCEL_X_MS2(s) - IMU_GRAV_X(s);
            float ay = IMU_ACCEL_Y_MS2(s) - IMU_GRAV_Y(s);
            float az = IMU_ACCEL_Z_MS2(s) - IMU_GRAV_Z(s);

            acc[0] += ax;
            acc[1] += ay;
            acc[2] += az;
            n++;

            if (n >= IMU_CAL_SAMPLES) break;
        }
    }

    s_accel_bias[0] = acc[0] / IMU_CAL_SAMPLES;
    s_accel_bias[1] = acc[1] / IMU_CAL_SAMPLES;
    s_accel_bias[2] = acc[2] / IMU_CAL_SAMPLES;
    s_calibrated    = true;

    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    
    mprintf("[IMU] Bias: [%.4f  %.4f  %.4f] m/s²",
            (double)s_accel_bias[0],
            (double)s_accel_bias[1],
            (double)s_accel_bias[2]);
}

void imu_init(void)
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;
    dev_ctx.handle    = &hi2c1;

    platform_delay(IMU_BOOT_TIME_MS);

    uint8_t whoamI = 0;
    lsm6dsv_device_id_get(&dev_ctx, &whoamI);
    
    if (whoamI != LSM6DSV_ID){
       mprintf("ERROR: lsm6dsv device id not detected");
        vTaskDelete( NULL );
        while(1) {
            osDelay(5000);
         }
    }

    lsm6dsv_sw_por(&dev_ctx);
    platform_delay(IMU_BOOT_TIME_MS);  /* wait for reset to complete */

    lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_xl_full_scale_set(&dev_ctx, IMU_XL_FULL_SCALE);
    lsm6dsv_gy_full_scale_set(&dev_ctx, IMU_GY_FULL_SCALE);

    lsm6dsv_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    lsm6dsv_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&dev_ctx, IMU_GY_LP1_BW);
    lsm6dsv_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&dev_ctx, IMU_XL_LP2_BW);

    lsm6dsv_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_fifo_xl_batch_set(&dev_ctx, IMU_FIFO_XL_BATCH);
    lsm6dsv_fifo_timestamp_batch_set(&dev_ctx, IMU_FIFO_TS_DEC);

    /* Batch SFLP into FIFO */
    lsm6dsv_fifo_sflp_batch_set(&dev_ctx, (lsm6dsv_fifo_sflp_raw_t){
        .game_rotation = 0,
        .gravity       = 1,
        .gbias         = 0
    });

    /* Start FIFO streaming */
    lsm6dsv_fifo_mode_set(&dev_ctx, LSM6DSV_STREAM_MODE);

    /* Set ODRs AFTER FIFO mode */
    lsm6dsv_xl_data_rate_set(&dev_ctx, IMU_XL_ODR);
    lsm6dsv_gy_data_rate_set(&dev_ctx, IMU_GY_ODR);
    lsm6dsv_sflp_data_rate_set(&dev_ctx, IMU_SFLP_ODR);

    /* Enable SFLP LAST */
    lsm6dsv_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
}

void imu_read_fifo(imu_fifo_data_t *fifo)
{
    fifo->count = 0;
    memset(fifo->samples, 0, sizeof(fifo->samples));

    lsm6dsv_fifo_status_t status;
    lsm6dsv_fifo_status_get(&dev_ctx, &status);
    uint16_t num = status.fifo_level;

    int16_t idx = -1;

    for (uint16_t i = 0; i < num; i++)
    {
        lsm6dsv_fifo_out_raw_t entry;
        lsm6dsv_fifo_out_raw_get(&dev_ctx, &entry);

        switch (entry.tag)
        {
            case LSM6DSV_TIMESTAMP_TAG:
                idx++;
                if (idx >= IMU_MAX_SAMPLES) { idx--; break; }
                memcpy(&fifo->samples[idx].timestamp_ticks, entry.data, sizeof(uint32_t));
                fifo->count = idx + 1;
                break;

            case LSM6DSV_XL_NC_TAG:
                if (idx < 0) break;
                memcpy(fifo->samples[idx].accel_raw, entry.data, 6);
                break;

            case LSM6DSV_SFLP_GRAVITY_VECTOR_TAG: 
                if (idx < 0) break;
                memcpy(fifo->samples[idx].gravity_raw, entry.data, 6);
                break;

            default:
                break;
        }
    }
}

void imu_integrate(const imu_fifo_data_t *fifo,
                   float *out_pitch_rad,
                   float *out_speed_horiz,
                   float *out_vel_z,
                   uint32_t *out_ticks_elapsed)
{
    float vel[3] = {0.0f, 0.0f, 0.0f};

    uint32_t first_ts = 0;
    uint32_t final_ts = 0;

    for (uint16_t i = 0; i < fifo->count; i++)
    {
        const imu_sample_t *s = &fifo->samples[i];

        if (last_ts == 0)
        {
            last_ts = s->timestamp_ticks;
            continue;
        }

        float dt = (float)(s->timestamp_ticks - last_ts) * 25e-6f;
        last_ts = s->timestamp_ticks;

        if (dt <= 0.0f || dt > 1.0f)
            continue;

        /* Track window boundaries */
        if (first_ts == 0) first_ts = s->timestamp_ticks;
        final_ts = s->timestamp_ticks;

        /* Gravity-compensated body-frame acceleration */
        float ax_b = IMU_ACCEL_X_MS2(s) - IMU_GRAV_X(s) - s_accel_bias[0];
        float ay_b = IMU_ACCEL_Y_MS2(s) - IMU_GRAV_Y(s) - s_accel_bias[1];
        float az_b = IMU_ACCEL_Z_MS2(s) - IMU_GRAV_Z(s) - s_accel_bias[2];

        float gx = IMU_GRAV_X(s);
        float gy = IMU_GRAV_Y(s);
        float gz = IMU_GRAV_Z(s);
        float gmag = sqrtf(gx*gx + gy*gy + gz*gz);
        if (gmag < 1e-3f) continue;

        *out_pitch_rad = acosf(fmaxf(-1.0f, fminf(1.0f, gz / gmag)));

        vel[0] += ax_b * dt;
        vel[1] += ay_b * dt;
        vel[2] += az_b * dt;

        float vel_z  = -(vel[0]*gx + vel[1]*gy + vel[2]*gz) / gmag;
        float v_sq   = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
        float v_h_sq = fmaxf(0.0f, v_sq - vel_z * vel_z);

        *out_vel_z       = vel_z;
        *out_speed_horiz = sqrtf(v_h_sq);
    }

    *out_ticks_elapsed = (first_ts == 0) ? 0 : (final_ts - first_ts);
}

#ifdef UWB_DEBUG

void imu_print_results(float pitch_rad,
                       float speed_horiz,
                       float vel_z,
                       uint32_t ticks_elapsed)
{
    /* Latest quaternion — last sample in the drain */
    mprintf("Pitch:    %6.2f deg\r\n",  (double)(pitch_rad * 57.2957795f));
    mprintf("Vel Z:    %8.5f m/s\r\n",  (double)vel_z);
    mprintf("Speed H:  %8.5f m/s\r\n",  (double)speed_horiz);
    mprintf("dt ticks: %lu  (%.1f ms)\r\n",
           (unsigned long)ticks_elapsed,
           (double)(ticks_elapsed * 0.025f));
}

#endif