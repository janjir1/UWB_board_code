#ifndef IMU_H
#define IMU_H

#include "imu_setup.h"

typedef struct {
    int16_t  accel_raw[3];    // raw int16, convert with lsm6dsv_from_fs2_to_mg()
    int16_t  gyro_raw[3];     // raw int16, convert with lsm6dsv_from_fs2000_to_mdps()
    uint16_t quat_raw[3];     // f16 x,y,z — w reconstructed on use
    uint16_t gravity_raw[3];  // f16 x,y,z
    uint32_t timestamp_ticks; // 25µs per tick
    float    dt_s;            // keep as float — computed once, used many times
} imu_sample_t;

typedef struct {
    imu_sample_t samples[IMU_MAX_SAMPLES];
    uint16_t     count;
} imu_fifo_data_t;

#define IMU_ACCEL_X_MS2(s)  (lsm6dsv_from_fs2_to_mg((s)->accel_raw[0]) * 0.00981f)
#define IMU_ACCEL_Y_MS2(s)  (lsm6dsv_from_fs2_to_mg((s)->accel_raw[1]) * 0.00981f)
#define IMU_ACCEL_Z_MS2(s)  (lsm6dsv_from_fs2_to_mg((s)->accel_raw[2]) * 0.00981f)

#define IMU_GYRO_X_DPS(s)   (lsm6dsv_from_fs2000_to_mdps((s)->gyro_raw[0]) * 0.001f)
#define IMU_GYRO_Y_DPS(s)   (lsm6dsv_from_fs2000_to_mdps((s)->gyro_raw[1]) * 0.001f)
#define IMU_GYRO_Z_DPS(s)   (lsm6dsv_from_fs2000_to_mdps((s)->gyro_raw[2]) * 0.001f)

#define IMU_QUAT_X(s)       lsm6dsv_from_f16_to_f32((s)->quat_raw[0])
#define IMU_QUAT_Y(s)       lsm6dsv_from_f16_to_f32((s)->quat_raw[1])
#define IMU_QUAT_Z(s)       lsm6dsv_from_f16_to_f32((s)->quat_raw[2])

#define IMU_GRAV_X(s)       lsm6dsv_from_f16_to_f32((s)->gravity_raw[0])
#define IMU_GRAV_Y(s)       lsm6dsv_from_f16_to_f32((s)->gravity_raw[1])
#define IMU_GRAV_Z(s)       lsm6dsv_from_f16_to_f32((s)->gravity_raw[2])


void imu_init(void);
void imu_read_fifo(imu_fifo_data_t *fifo);

#endif