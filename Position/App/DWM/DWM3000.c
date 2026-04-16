#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include "queue.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "dw3000_deca_regs.h"
#include "deca_device_api.h"
#include "main.h"
#include "../Generic/my_print.h"
#include "stm32l4xx_hal.h"

#include "DWM3000_setup.h"
#include "DWM3000.h"
#include "DWM3000_driver.h"
#include "../UWB_app/uwb_exchange.h"
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"

#define U64_HI(x)  ((uint32_t)((x) >> 32))
#define U64_LO(x)  ((uint32_t)((x) & 0xFFFFFFFFU))
#define MPRINT_TS(label, val) \
    mprintf("UWB_eTWR_TEST - %-20s: 0x%08lX%08lX\r\n", (label), U64_HI(val), U64_LO(val))



void StartRangingTask(void *argument) {
    mprintf("Starting DWM3000 task\r\n");
    bool passed = dwm_init();;
    if (passed) {
        mprintf("DWM3000 initialized successfully\r\n");
    } else {
        mprintf("DWM3000 initialization failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(200);

    passed = dwm_selftest();
    if (passed) {
        mprintf("DWM3000 self test passed\r\n");
    } else {
        mprintf("DWM3000 self test failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(200);

    passed = dwm_configure();
    if (passed) {
        mprintf("DWM3000 configured successfully\r\n");
    } else {
        mprintf("DWM3000 configure failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(200);

    /*
    if (dwm_get_addr() == 0x506B) {
        dwm_tx_continuous_delayed();   // device A transmits
    } else {
        dwm_rx_continuous_sleep();

    }
        */

    //msg_run_tests();

    network_init(dwm_get_addr());

    osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);
    uint8_t timer = 0;
    while(1){
        mprintf("Starting sync\r\n");
        HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
        uwb_sync_result_t result_sync = uwb_sync();
        mprintf("Sync result: %d\r\n", result_sync);
        uwb_etwr_result_t result_etwr = uwb_extended_twr(result_sync);
        distance_calculate(result_etwr);
        uint32_t sleep_time = uwb_share (result_etwr, DEEP_SLEEP); //TODO if output is 0 set to last times sleep_time
        HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
        if(timer++ > 10) {
            timer = 0;
            // Heap — most important, run periodically
            mprintf("[DIAG] heap free=%u  min_ever=%u\r\n",
                    xPortGetFreeHeapSize(),
                    xPortGetMinimumEverFreeHeapSize());

            // Stack watermark of the calling task
            mprintf("[DIAG] stack HWM=%u words\r\n",
                    uxTaskGetStackHighWaterMark(NULL));
        }
        osDelay(sleep_time);
    }
    

    vTaskDelete( NULL );
    while(1) { } 
}



