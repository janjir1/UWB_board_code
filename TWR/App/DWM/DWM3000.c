#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include "queue.h"
#include <stdbool.h>
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
    osDelay(1000);

    passed = dwm_selftest();
    if (passed) {
        mprintf("DWM3000 self test passed\r\n");
    } else {
        mprintf("DWM3000 self test failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(1000);

    passed = dwm_configure();
    if (passed) {
        mprintf("DWM3000 configured successfully\r\n");
    } else {
        mprintf("DWM3000 configure failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(1000);

    /*
    if (dwm_get_addr() == 0x506B) {
        dwm_tx_continuous_delayed();   // device A transmits
    } else {
        dwm_rx_continuous_sleep();

    }
        */

    //msg_run_tests();

    network_init(dwm_get_addr());



    uint8_t seq = 0;
    while(1){
        mprintf("Starting sync\r\n");
        uwb_sync_result_t result = uwb_sync(seq++);
        mprintf("Sync result: %d\r\n", result);
        uwb_twr_test(seq, result);

        osDelay(500);
        
    
    }
    

    vTaskDelete( NULL );
    while(1) { } 
}



