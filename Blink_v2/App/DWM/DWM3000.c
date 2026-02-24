#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "deca_device_api.h"
#include "main.h"
#include "../Generic/my_print.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1; 

static bool dwm_selfTest(void);
static void dwm_init(void);

// THE CRITICAL PART: Extern "C" wrapper for the task function
void StartDWM(void *argument) {
    // This function is called by FreeRTOS from main.c


    dwm_init();

    bool passed = dwm_selfTest();
    if (passed) {
        mprintf("DWM3000 self test passed\r\n");
    } else {
        mprintf("DWM3000 self test failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    vTaskDelete( NULL );
    while(1) { } 
}

static bool dwm_selfTest(void) {

    uint32_t dev_id = dwt_readdevid();

    if (dev_id != 0xDECA0302UL) {
        dwm_init();
        osDelay(10);  
    }
    if (dev_id != 0xDECA0302UL) {
        return false; 
    }

    uint8_t xtal = dwt_getxtaltrim();
    if (xtal == 0) {
        return false;
    }

    dwt_configeventcounters(1);  // clear & enable

    uint8_t tx_msg[] = {0xC5, 0x00, 0x01, 0x02, 0x03, 0x04};
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
    dwt_writetxfctrl(sizeof(tx_msg) + 2, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    osDelay(2);  // give time for frame to finish

    dwt_deviceentcnts_t cnt;
    dwt_readeventcounters(&cnt);
    if (cnt.TXF < 1) {
        Error_Handler();
        return false;
    }
    return true;
}

static void dwm_init(void) {

    dw3000_hw_init();
    dw3000_hw_reset();
    osDelay(10);

    extern const struct dwt_probe_s dw3000_probe_interf;
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        Error_Handler();
    }

    // Poll until chip is in IDLE_RC — do NOT skip this
    uint32_t timeout = 1000;
    while (!dwt_checkidlerc()) {
        osDelay(1);
        if (--timeout == 0) {
            mprintf("Idle_RC timeout");
            Error_Handler();          // chip never became ready
        }
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        Error_Handler();
    }

    dw3000_spi_speed_fast();
}

