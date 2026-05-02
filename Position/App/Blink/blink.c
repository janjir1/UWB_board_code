#include "main.h"
#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdio.h>
#include <string.h>
#include "../Generic/my_print.h"



void StartBlink(void *argument) {
    // This function is called by FreeRTOS from main.c
    
    // You can use C++ objects here!
    int i = 0;
    while (1) {
        HAL_IWDG_Refresh(&hiwdg); // set t 3 seconds
        #ifdef UWB_BOARD_V1_1
            mprintf("Hello from Board V1.1! Count: %d\r\n", i);
        #else
            mprintf("Hello from Board V1.0! Count: %d\r\n", i);
        #endif
        
        i++;
        osDelay(1000);
    }
    
    // If run() returns, the task must delete itself
    vTaskDelete( NULL );
    while(1) { } 
}
