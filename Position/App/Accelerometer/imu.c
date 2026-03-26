#include <string.h>
#include <stdio.h>
#include "lsm6dsv_reg.h"
#include "lsm6dsv_platform.h"
#include "../Generic/my_print.h"
#include "imu_setup.h"
#include "lsm6dsv_ST_example.h"
#include "imu.h"

#define    BOOT_TIME            10 //ms

stmdev_ctx_t dev_ctx;
static lsm6dsv_filt_settling_mask_t filt_settling_mask;

void imu_init(void)
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;
    dev_ctx.handle    = &hi2c1;

    platform_delay(IMU_BOOT_TIME_MS);

    uint8_t whoamI = 0;
    lsm6dsv_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LSM6DSV_ID)
        while (1);

    lsm6dsv_sw_por(&dev_ctx);
    lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    lsm6dsv_xl_data_rate_set(&dev_ctx, IMU_XL_ODR);
    lsm6dsv_gy_data_rate_set(&dev_ctx, IMU_GY_ODR);
    lsm6dsv_xl_full_scale_set(&dev_ctx, IMU_XL_FULL_SCALE);
    lsm6dsv_gy_full_scale_set(&dev_ctx, IMU_GY_FULL_SCALE);

    filt_settling_mask.drdy   = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g  = PROPERTY_ENABLE;
    lsm6dsv_filt_settling_mask_set(&dev_ctx, filt_settling_mask);

    lsm6dsv_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_filt_gy_lp1_bandwidth_set(&dev_ctx, IMU_GY_LP1_BW);
    lsm6dsv_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_filt_xl_lp2_bandwidth_set(&dev_ctx, IMU_XL_LP2_BW);

    lsm6dsv_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv_sflp_data_rate_set(&dev_ctx, IMU_SFLP_ODR);
    lsm6dsv_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

    lsm6dsv_fifo_xl_batch_set(&dev_ctx, IMU_FIFO_XL_BATCH);
    lsm6dsv_fifo_gy_batch_set(&dev_ctx, IMU_FIFO_GY_BATCH);
    lsm6dsv_fifo_timestamp_batch_set(&dev_ctx, IMU_FIFO_TS_DEC);
    lsm6dsv_fifo_sflp_batch_set(&dev_ctx, (lsm6dsv_fifo_sflp_raw_t){
        .game_rotation = IMU_SFLP_BATCH_QUAT,
        .gravity       = IMU_SFLP_BATCH_GRAVITY,
        .gbias         = IMU_SFLP_BATCH_GBIAS
    });

    lsm6dsv_fifo_mode_set(&dev_ctx, LSM6DSV_STREAM_MODE);
}

void imu_read_fifo(imu_fifo_data_t *fifo)
{
    static uint32_t last_ts = 0;

    fifo->count = 0;
    memset(fifo->samples, 0, sizeof(fifo->samples));

    lsm6dsv_fifo_status_t fifo_status;
    lsm6dsv_fifo_status_get(&dev_ctx, &fifo_status);
    uint16_t num_entries = fifo_status.fifo_level;

    uint16_t idx = 0;

    for (uint16_t i = 0; i < num_entries; i++)
    {
        if (idx >= IMU_MAX_SAMPLES)
            break;

        lsm6dsv_fifo_out_raw_t entry;
        lsm6dsv_fifo_out_raw_get(&dev_ctx, &entry);

        switch (entry.tag)
        {
            case LSM6DSV_XL_NC_TAG:
            {
                int16_t *raw = (int16_t *)entry.data;
                fifo->samples[idx].accel_raw[0] = raw[0];
                fifo->samples[idx].accel_raw[1] = raw[1];
                fifo->samples[idx].accel_raw[2] = raw[2];
                break;
            }

            case LSM6DSV_GY_NC_TAG:
            {
                int16_t *raw = (int16_t *)entry.data;
                fifo->samples[idx].gyro_raw[0] = raw[0];
                fifo->samples[idx].gyro_raw[1] = raw[1];
                fifo->samples[idx].gyro_raw[2] = raw[2];
                break;
            }

            case LSM6DSV_SFLP_GAME_ROTATION_VECTOR_TAG:
            {
                uint16_t *raw = (uint16_t *)entry.data;
                fifo->samples[idx].quat_raw[0] = raw[0];
                fifo->samples[idx].quat_raw[1] = raw[1];
                fifo->samples[idx].quat_raw[2] = raw[2];
                break;
            }

            case LSM6DSV_SFLP_GRAVITY_VECTOR_TAG:
            {
                uint16_t *raw = (uint16_t *)entry.data;
                fifo->samples[idx].gravity_raw[0] = raw[0];
                fifo->samples[idx].gravity_raw[1] = raw[1];
                fifo->samples[idx].gravity_raw[2] = raw[2];
                break;
            }

            case LSM6DSV_TIMESTAMP_TAG:
            {
                memcpy(&fifo->samples[idx].timestamp_ticks,
                       entry.data, sizeof(uint32_t));
                if (last_ts != 0)
                    fifo->samples[idx].dt_s =
                        (fifo->samples[idx].timestamp_ticks - last_ts) * 25e-6f;
                else
                    fifo->samples[idx].dt_s = 0.0f;
                last_ts = fifo->samples[idx].timestamp_ticks;
                idx++;
                fifo->count = idx;
                break;
            }

            default:
                break;
        }
    }
}