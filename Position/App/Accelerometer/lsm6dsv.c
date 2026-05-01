#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "cmsis_os2.h"
#include "lsm6dsv_ST_example.h"

#include "main.h"
#include "../Generic/my_print.h"
#include "stm32l4xx_hal.h"
#include "imu.h"

    float pitch_rad = 0, speed_horiz = 0, vel_z = 0;
    uint32_t ticks_elapsed = 0;

// THE CRITICAL PART: Extern "C" wrapper for the task function
void StartAcc(void *argument) {
    // This function is called by FreeRTOS from main.c
    
    bool passed = SelfTest();
    if (passed) {
        mprintf("LSM6DSV self test passed\r\n");
    } else {
        mprintf("LSM6DSV self test failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }

    //lsm6dsv_read_data_polling();

    imu_init();
    osDelay(200);
    imu_calibrate();

    osThreadFlagsSet(RangingHandle, 0x01);


    while(1){
        osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
        mprintf("Starting IMU task\r\n");
        imu_fifo_data_t fifo;
        imu_read_fifo(&fifo);
        imu_integrate(&fifo, &pitch_rad, &speed_horiz, &vel_z, &ticks_elapsed);
        osThreadFlagsSet(RangingHandle, 0x02);
        mprintf("IMU data ready\r\n");
        //imu_print_results(pitch_rad, speed_horiz, vel_z, ticks_elapsed);
    }
    vTaskDelete( NULL );
    while(1) { } 
}

void imu_get_results(float *out_pitch_rad, float *out_speed_horiz, float *out_vel_z)
{
    *out_pitch_rad     = pitch_rad;
    *out_speed_horiz   = speed_horiz;
    *out_vel_z        = vel_z;

}