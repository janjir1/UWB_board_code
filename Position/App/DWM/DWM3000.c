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

/**
 * @brief Watchdog to catch hardware TX lockups and RTOS sleep underflows.
 * * @param result_sync  The return value from uwb_sync()
 * @param result_etwr  The return value from uwb_extended_twr()
 * @param sleep_time   The requested sleep time from uwb_share()
 * @return Safely clamped sleep time for osDelay()
 */
uint32_t tx_err_watchdog(uwb_sync_result_t result_sync, 
                                     uwb_etwr_result_t result_etwr, 
                                     uint32_t sleep_time)
{
    /* Static counter persists across loop iterations */
    static uint8_t tx_fail_count = 0;

    /* ------------------------------------------------------------------
     * GUARD 1: DWM3000 Hardware TX Failure Watchdog
     * ------------------------------------------------------------------ */
    if (result_sync == UWB_SYNC_TX_FAILED || result_etwr == UWB_TWR_TX_FAILED || sleep_time > 5000) {
        tx_fail_count++;
        if (tx_fail_count >= 10) {  /* 10 consecutive TX failures */
            mprintf("ERROR: Consecutive TX failures! Re-initializing DWM3000...\r\n");
            mprintf("WARNING: Invalid sleep time %lu detected. Defaulting to DEEP_SLEEP.\r\n", sleep_time);
       
            bool passed = dwm_init();
            passed &= dwm_configure();
            
            if (!passed) {
                mprintf("ERROR: DWM3000 re-initialization failed!\r\n");
            } else {
                mprintf("SUCCESS: DWM3000 re-initialized.\r\n");
            }
            tx_fail_count = 0; /* Reset counter after recovery attempt */
            return DEEP_SLEEP - 50;
        }
    } else {
        /* If we succeed or fail for a normal RF reason (like TIMEOUT), reset the counter */
        tx_fail_count = 0; 
    }

    return sleep_time;
}

void StartRangingTask(void *argument) {
    mprintf("Starting DWM3000 task\r\n");
    bool passed = dwm_init();
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
    //uint8_t timer = 0;
    while(1){
        dwm_wakeup();
        mprintf("Starting sync\r\n");
        HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
        uwb_sync_result_t result_sync = uwb_sync();
        mprintf("Sync result: %d\r\n", result_sync);
        uwb_etwr_result_t result_etwr = uwb_extended_twr(result_sync);
        distance_calculate(result_etwr);
        uint32_t sleep_time = uwb_share (result_etwr, DEEP_SLEEP); 
        sleep_time = tx_err_watchdog(result_sync, result_etwr, sleep_time);
        HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
        dwm_sleep();
        osDelay(sleep_time);
    }
    

    vTaskDelete( NULL );
    while(1) { } 
}



