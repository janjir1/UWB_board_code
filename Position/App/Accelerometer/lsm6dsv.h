#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS task entry point — called by osThreadNew() in main.c */
void StartAcc(void *argument);
void imu_get_results(float *out_pitch_rad, float *out_speed_horiz, float *out_vel_z);
#ifdef __cplusplus
}
#endif
