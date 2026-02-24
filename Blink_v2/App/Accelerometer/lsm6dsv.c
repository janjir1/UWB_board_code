#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "lsm6dsv_ST_example.h"

#include "main.h"
#include "../Generic/my_print.h"
#include "stm32l4xx_hal.h"



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
    
    vTaskDelete( NULL );
    while(1) { } 
}

