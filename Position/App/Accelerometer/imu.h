#ifndef IMU_H
#define IMU_H

#include "imu_setup.h"
#include "main.h"

typedef struct {
    int16_t  accel_raw[3];    // raw int16, convert with lsm6dsv_from_fs2_to_mg()
    uint16_t gravity_raw[3];  // f16 x,y,z
    uint32_t timestamp_ticks; // 25µs per tick
} imu_sample_t;

typedef struct {
    imu_sample_t samples[IMU_MAX_SAMPLES];
    uint16_t     count;
} imu_fifo_data_t;

static inline float f16_to_f32(uint16_t h)
{
    union { float f; uint32_t u; } conv;
    conv.u = lsm6dsv_from_f16_to_f32(h);
    return conv.f;
}

#define IMU_ACCEL_X_MS2(s)  (lsm6dsv_from_fs2_to_mg((s)->accel_raw[0]) * 0.00981f)
#define IMU_ACCEL_Y_MS2(s)  (lsm6dsv_from_fs2_to_mg((s)->accel_raw[1]) * 0.00981f)
#define IMU_ACCEL_Z_MS2(s)  (lsm6dsv_from_fs2_to_mg((s)->accel_raw[2]) * 0.00981f)

#define IMU_GRAV_X(s)  (lsm6dsv_from_sflp_to_mg((s)->gravity_raw[0]) * 0.00981f)
#define IMU_GRAV_Y(s)  (lsm6dsv_from_sflp_to_mg((s)->gravity_raw[1]) * 0.00981f)
#define IMU_GRAV_Z(s)  (lsm6dsv_from_sflp_to_mg((s)->gravity_raw[2]) * 0.00981f)


void imu_init(void);
void imu_read_fifo(imu_fifo_data_t *fifo);

void imu_integrate(const imu_fifo_data_t *fifo,
                   float *out_pitch_rad,
                   float *out_speed_horiz,
                   float *out_vel_z,
                   uint32_t *out_ticks_elapsed);

void imu_calibrate(void);

#ifdef UWB_DEBUG               
void imu_print_results(float pitch_rad,
                       float speed_horiz,
                       float vel_z,
                       uint32_t ticks_elapsed);
#endif //UWB_debug
#endif