#include <math.h>
#include <string.h>
#include "cmsis_os2.h"
#include "lsm6dsv_reg.h"
#include "lsm6dsv_platform.h"
#include "../Generic/my_print.h"
#include "imu_setup.h"
#include "lsm6dsv_ST_example.h"
#include "imu.h"
#include "main.h"

/* -- LSM6DSV register address for FIFO burst read -------------------------
 * FIFO_DATA_OUT_TAG = 0x78. The driver's read_reg callback auto-increments
 * the address on multi-byte transfers (SPI: bit7 set; I2C: always on).
 * CS is asserted once and all (num × 7) bytes are clocked in one transaction.
 * ------------------------------------------------------------------------ */
#define LSM6DSV_FIFO_DATA_OUT_TAG_ADDR 0x78U

stmdev_ctx_t dev_ctx;
static lsm6dsv_filt_settling_mask_t filt_settling_mask;

static uint32_t last_ts        = 0;
static float    s_accel_bias[3] = {0.0f, 0.0f, 0.0f};
static bool     s_calibrated   = false;

/* ============================================================================
 * Internal helpers
 * ============================================================================ */

/**
 * @brief Burst-read @p num FIFO entries in a single bus transaction.
 *
 * Calls @c dev_ctx.read_reg with a length of @p num × 7, which triggers
 * a multi-byte (DMA) transfer in the platform layer
 * (see @c lsm6dsv_platform.c). On SPI (V1.1 boards) CS is asserted once
 * and bit7 of the address selects hardware auto-increment. On I2C, the
 * controller handles repeated-start automatically.
 *
 * @param buf  Destination buffer; must be at least @p num × 7 bytes.
 * @param num  Number of 7-byte FIFO entries to read.
 * @return     Bytes actually read (@p num × 7), or 0 on bus error.
 */
static uint16_t imu_fifo_burst_read(uint8_t *buf, uint16_t num)
{
    if (num == 0 || buf == NULL) return 0;

    uint16_t len    = num * 7U;
    int32_t  ret    = dev_ctx.read_reg(dev_ctx.handle,
                                       LSM6DSV_FIFO_DATA_OUT_TAG_ADDR,
                                       buf, len);
    return (ret == 0) ? len : 0;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief Collect and average accelerometer bias over @c IMU_CAL_SAMPLES samples.
 *
 * The device must be stationary and level during calibration. The function
 * signals the calibration window to the operator via LED_R:
 * - LED toggles ON — 2-second settling window (place the device flat).
 * - LED toggles OFF — 1-second final settling period.
 * - LED toggles ON during sampling, then OFF on completion.
 *
 * The FIFO is flushed via a bypass/stream cycle before sampling to discard
 * any frames that arrived before the device was stationary. @c last_ts is
 * reset so @ref imu_integrate does not use a stale reference after calibration.
 *
 * Gravity is subtracted sample-by-sample using the SFLP gravity vector,
 * so the bias estimate is independent of sensor orientation.
 *
 * Results are stored in @c s_accel_bias[] and @c s_calibrated is set to true.
 */
void imu_calibrate(void)
{
    float    acc[3] = {0.0f, 0.0f, 0.0f};
    uint16_t n      = 0;

    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    osDelay(2000);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    osDelay(1000);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

    /* Flush stale FIFO instantly via bypass — no buffer, no bus reads */
    lsm6dsv_fifo_mode_set(&dev_ctx, LSM6DSV_BYPASS_MODE);
    lsm6dsv_fifo_mode_set(&dev_ctx, LSM6DSV_STREAM_MODE);

    /* Reset last_ts so imu_integrate() does not use a stale reference */
    last_ts = 0;

    mprintf("[IMU] Calibrating bias (%d samples)...", IMU_CAL_SAMPLES);

    while (n < IMU_CAL_SAMPLES) {
        platform_delay(1000u / IMU_CAL_ODR_HZ + 1u);

        imu_fifo_data_t fifo;
        imu_read_fifo(&fifo);

        for (uint16_t i = 0; i < fifo.count; i++) {
            const imu_sample_t *s = &fifo.samples[i];

            acc[0] += IMU_ACCEL_X_MS2(s) - IMU_GRAV_X(s);
            acc[1] += IMU_ACCEL_Y_MS2(s) - IMU_GRAV_Y(s);
            acc[2] += IMU_ACCEL_Z_MS2(s) - IMU_GRAV_Z(s);
            n++;

            if (n >= IMU_CAL_SAMPLES) break;
        }
    }

    s_accel_bias[0] = acc[0] / IMU_CAL_SAMPLES;
    s_accel_bias[1] = acc[1] / IMU_CAL_SAMPLES;
    s_accel_bias[2] = acc[2] / IMU_CAL_SAMPLES;
    s_calibrated    = true;

    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    mprintf("[IMU] Bias: [%.4f %.4f %.4f] m/s²",
            (double)s_accel_bias[0],
            (double)s_accel_bias[1],
            (double)s_accel_bias[2]);
}

/**
 * @brief Initialise the LSM6DSV IMU and configure it for FIFO-based operation.
 *
 * Sets up the ST driver context (bus handle selected by @c UWB_BOARD_V1_1),
 * verifies the WHO_AM_I register, applies a soft reset, then configures:
 * - Full-scale ranges and low-pass filters for accel and gyro.
 * - Hardware timestamp at @c IMU_FIFO_TS_DEC decimation (25 µs/tick).
 * - FIFO in STREAM mode: XL batched, gyro bypassed (feeds SFLP directly),
 *   gravity vector output enabled.
 * - Output data rates for XL, gyro, and SFLP.
 *
 * Calls @c vTaskDelete(NULL) and loops if the device ID is not detected,
 * preventing silent misconfiguration.
 */
void imu_init(void)
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;

#ifdef UWB_BOARD_V1_1
    static lsm6dsv_spi_handle_t spi_handle = {
        .hspi    = &hspi2,
        .cs_port = SPI2_CS_GPIO_Port,
        .cs_pin  = SPI2_CS_Pin,
    };
    dev_ctx.handle = &spi_handle;
#else
    dev_ctx.handle = &hi2c1;
#endif

    platform_delay(IMU_BOOT_TIME_MS);

    uint8_t whoamI = 0;
    lsm6dsv_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != LSM6DSV_ID) {
        mprintf("ERROR: lsm6dsv device id not detected");
        vTaskDelete(NULL);
        while (1) { osDelay(5000); }
    }

    lsm6dsv_sw_por(&dev_ctx);
    platform_delay(IMU_BOOT_TIME_MS);

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
    /* Gyro is NOT batched into FIFO — it feeds SFLP on-chip directly */
    lsm6dsv_fifo_timestamp_batch_set(&dev_ctx, IMU_FIFO_TS_DEC);

    lsm6dsv_fifo_sflp_batch_set(&dev_ctx, (lsm6dsv_fifo_sflp_raw_t){
        .game_rotation = 0,
        .gravity       = 1,
        .gbias         = 0
    });

    lsm6dsv_fifo_mode_set(&dev_ctx, LSM6DSV_STREAM_MODE);

    lsm6dsv_xl_data_rate_set(&dev_ctx,   IMU_XL_ODR);
    lsm6dsv_gy_data_rate_set(&dev_ctx,   IMU_GY_ODR);
    lsm6dsv_sflp_data_rate_set(&dev_ctx, IMU_SFLP_ODR);

    lsm6dsv_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);
}

/**
 * @brief Drain the IMU FIFO into @p fifo in a single burst bus transaction.
 *
 * **FIFO layout** at 120 Hz XL = 120 Hz SFLP (1:1 ratio) with
 * @c TMSTMP_DEC_8:
 * @code
 *   [TIMESTAMP] [XL]×8 [GRAVITY]×8
 * @endcode
 * One timestamp entry appears per group of 8 paired XL + GRAVITY entries.
 *
 * **Parsing strategy:**
 * 1. Read FIFO level (2 register bytes — fast).
 * 2. Burst-read all @c num×7 bytes in one SPI/I2C transaction.
 * 3. Walk the raw buffer, pairing each @c XL_NC_TAG entry with the
 *    immediately following @c SFLP_GRAVITY_VECTOR_TAG entry.
 * 4. Timestamps are back-filled uniformly across the 8-sample group
 *    once the closing timestamp arrives.
 *
 * @param[out] fifo  Output struct; @c count and @c samples are populated.
 *                   Must not be NULL.
 */
void imu_read_fifo(imu_fifo_data_t *fifo)
{
    fifo->count = 0;
    memset(fifo->samples, 0, sizeof(fifo->samples));

    /* -- 1. Read FIFO level (2 bytes over bus) -- */
    lsm6dsv_fifo_status_t status;
    lsm6dsv_fifo_status_get(&dev_ctx, &status);
    uint16_t num = status.fifo_level;

    if (num == 0) return;

    /* Clamp to buffer capacity */
    if (num > IMU_FIFO_RAW_BUF_ENTRIES)
        num = IMU_FIFO_RAW_BUF_ENTRIES;

    /* -- 2. Burst-read all FIFO bytes in ONE transaction -- */
    static uint8_t raw[IMU_FIFO_RAW_BUF_ENTRIES * 7U]; /* static — lives in .bss, not stack */
    uint16_t bytes_read = imu_fifo_burst_read(raw, num);
    if (bytes_read == 0) return;

    /* -- 3. Parse raw buffer -- */

    /* XL and SFLP are both 120 Hz (1:1), so each XL entry is immediately
     * followed by a GRAVITY entry in FIFO order. Timestamps appear every
     * 8 samples (TMSTMP_DEC_8); dt is interpolated uniformly per group. */

    uint32_t group_ts_start   = last_ts;
    uint32_t group_ts_end     = 0;
    uint16_t group_xl_count   = 0;

    int16_t pending_accel[3]  = {0};
    bool    pending_accel_valid = false;

    uint16_t sample_idx = 0;

    for (uint16_t i = 0; i < num; i++) {
        const uint8_t *entry = &raw[i * 7U];
        uint8_t tag = (entry[0] >> 3) & 0x1FU; /* bits [7:3] */

        switch (tag) {
            case LSM6DSV_TIMESTAMP_TAG:
            {
                uint32_t ts;
                memcpy(&ts, &entry[1], sizeof(uint32_t));

                if (group_xl_count > 0 && group_ts_end != 0) {
                    /* Back-fill: distribute elapsed ticks evenly across the group */
                    uint32_t ticks_per = (group_ts_end - group_ts_start) / group_xl_count;
                    for (uint16_t k = sample_idx - group_xl_count; k < sample_idx; k++) {
                        fifo->samples[k].timestamp_ticks =
                            group_ts_start + ticks_per * (k - (sample_idx - group_xl_count) + 1U);
                    }
                }

                group_ts_start  = ts;
                group_ts_end    = ts;
                group_xl_count  = 0;
                break;
            }

            case LSM6DSV_XL_NC_TAG:
                memcpy(pending_accel, &entry[1], 6);
                pending_accel_valid = true;
                break;

            case LSM6DSV_SFLP_GRAVITY_VECTOR_TAG:
            {
                if (!pending_accel_valid)         break;
                if (sample_idx >= IMU_MAX_SAMPLES) break;

                imu_sample_t *s = &fifo->samples[sample_idx];
                memcpy(s->accel_raw,   pending_accel, 6);
                memcpy(s->gravity_raw, &entry[1],     6);

                /* Timestamp placeholder — back-filled when the next TS arrives.
                 * Assign group_ts_start now so imu_integrate() never sees zero. */
                s->timestamp_ticks  = group_ts_start;

                pending_accel_valid = false;
                group_xl_count++;
                sample_idx++;
                fifo->count = sample_idx;
                break;
            }

            default:
                break;
        }
    }

    /* Update last_ts for next call */
    if (fifo->count > 0)
        last_ts = fifo->samples[fifo->count - 1].timestamp_ticks;
}

/**
 * @brief Integrate a FIFO batch into velocity and pitch estimates.
 *
 * Pitch is computed once from the last sample's SFLP gravity vector
 * (gravity changes slowly; per-sample recomputation wastes cycles). The
 * velocity integration loop is kept tight — no transcendental calls inside.
 *
 * Z velocity is projected along the gravity axis; horizontal speed is the
 * magnitude of the residual after subtracting the vertical component.
 * Both @c last_ts (module-static) and @c s_accel_bias[] are applied.
 *
 * Samples with @c dt <= 0 or @c dt > 1 s are skipped to guard against
 * timestamp glitches or first-sample stale-reference conditions.
 *
 * @param[in]  fifo               Populated FIFO batch from @ref imu_read_fifo.
 * @param[out] out_pitch_rad      Pitch angle in radians (angle from vertical).
 * @param[out] out_speed_horiz    Horizontal speed magnitude in m/s (>= 0).
 * @param[out] out_vel_z          Vertical velocity in m/s (signed).
 * @param[out] out_ticks_elapsed  Hardware ticks spanned by this batch (0 if empty).
 */
void imu_integrate(const imu_fifo_data_t *fifo,
                   float    *out_pitch_rad,
                   float    *out_speed_horiz,
                   float    *out_vel_z,
                   uint32_t *out_ticks_elapsed)
{
    float    vel[3]    = {0.0f, 0.0f, 0.0f};
    uint32_t first_ts  = 0;
    uint32_t final_ts  = 0;

    float gx_last = 0.0f, gy_last = 0.0f, gz_last = 0.0f, gmag_last = 0.0f;

    /* Compute pitch once from the last sample's gravity vector */
    if (fifo->count > 0) {
        const imu_sample_t *last = &fifo->samples[fifo->count - 1];
        gx_last   = IMU_GRAV_X(last);
        gy_last   = IMU_GRAV_Y(last);
        gz_last   = IMU_GRAV_Z(last);
        gmag_last = sqrtf(gx_last * gx_last + gy_last * gy_last + gz_last * gz_last);

        if (gmag_last > 1e-3f)
            *out_pitch_rad = acosf(fmaxf(-1.0f, fminf(1.0f, gz_last / gmag_last)));
    }

    /* -- Velocity integration loop (tight, no transcendental calls) -- */
    for (uint16_t i = 0; i < fifo->count; i++) {
        const imu_sample_t *s = &fifo->samples[i];

        if (last_ts == 0) {
            last_ts = s->timestamp_ticks;
            continue;
        }

        float dt = (float)(s->timestamp_ticks - last_ts) * 25e-6f;
        last_ts  = s->timestamp_ticks;

        if (dt <= 0.0f || dt > 1.0f) continue;

        if (first_ts == 0) first_ts = s->timestamp_ticks;
        final_ts = s->timestamp_ticks;

        float ax_b = IMU_ACCEL_X_MS2(s) - IMU_GRAV_X(s) - s_accel_bias[0];
        float ay_b = IMU_ACCEL_Y_MS2(s) - IMU_GRAV_Y(s) - s_accel_bias[1];
        float az_b = IMU_ACCEL_Z_MS2(s) - IMU_GRAV_Z(s) - s_accel_bias[2];

        vel[0] += ax_b * dt;
        vel[1] += ay_b * dt;
        vel[2] += az_b * dt;
    }

    /* -- Project velocity using last gravity vector -- */
    if (gmag_last > 1e-3f) {
        float vel_z  = -(vel[0] * gx_last + vel[1] * gy_last + vel[2] * gz_last) / gmag_last;
        float v_sq   = vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2];
        float v_h_sq = fmaxf(0.0f, v_sq - vel_z * vel_z);

        *out_vel_z        = vel_z;
        *out_speed_horiz  = sqrtf(v_h_sq);
    } else {
        *out_vel_z       = 0.0f;
        *out_speed_horiz = 0.0f;
    }

    *out_ticks_elapsed = (first_ts == 0) ? 0U : (final_ts - first_ts);
}

/* ============================================================================
 * Debug output (compiled only when UWB_DEBUG is defined)
 * ============================================================================ */

#ifdef UWB_DEBUG
/**
 * @brief Print IMU integration results to the debug UART.
 *
 * @param pitch_rad      Pitch angle in radians.
 * @param speed_horiz    Horizontal speed in m/s.
 * @param vel_z          Vertical velocity in m/s.
 * @param ticks_elapsed  Hardware tick span of the processed batch.
 */
void imu_print_results(float    pitch_rad,
                       float    speed_horiz,
                       float    vel_z,
                       uint32_t ticks_elapsed)
{
    mprintf("Pitch:    %6.2f deg\r\n",  (double)(pitch_rad * 57.2957795f));
    mprintf("Vel Z:    %8.5f m/s\r\n",  (double)vel_z);
    mprintf("Speed H:  %8.5f m/s\r\n",  (double)speed_horiz);
    mprintf("dt ticks: %lu (%.1f ms)\r\n",
            (unsigned long)ticks_elapsed,
            (double)(ticks_elapsed * 0.025f));
}
#endif /* UWB_DEBUG */