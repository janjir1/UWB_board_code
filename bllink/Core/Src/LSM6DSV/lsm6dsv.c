#include "cmsis_os.h" // or "FreeRTOS.h" depending on your setup
#include <stdio.h>
#include <string.h>

#include "LSM6DSV/lsm6dsv_ST_example.h"

#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "deca_device_api.h"
#include "main.h"
#include "my_print.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1; 

#define DW_CS_GPIO    ChipSelect_GPIO_Port
#define DW_CS_PIN     ChipSelect_Pin
#define DW_RESET_GPIO DWM_RSTN_GPIO_Port
#define DW_RESET_PIN  DWM_RSTN_Pin

void dw3000_spi_test(void)
{
    uint8_t tx[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    /* Make sure RESET is released (high) */
    HAL_GPIO_WritePin(DW_RESET_GPIO, DW_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(5);

    /* CS low → transfer → CS high */
    HAL_GPIO_WritePin(DW_CS_GPIO, DW_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 100);
    HAL_GPIO_WritePin(DW_CS_GPIO, DW_CS_PIN, GPIO_PIN_SET);

    /* Assemble device ID from rx[1..4] (little-endian, rx[0] is garbage) */
    uint32_t devId = ((uint32_t)rx[4] << 24)
                   | ((uint32_t)rx[3] << 16)
                   | ((uint32_t)rx[2] <<  8)
                   | ((uint32_t)rx[1]);

    if (devId == 0xDECA0302UL) {
        /* Success — LED toggle or UART print here */
        mprintf("DW3000 SPI test passed! Device ID: 0x%08X\n", (unsigned int)devId);
    }

}



// THE CRITICAL PART: Extern "C" wrapper for the task function
void StartAcc(void *argument) {
    // This function is called by FreeRTOS from main.c
    
    SelfTest();
    //HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_RESET);
    //dw3000_spi_test();

    dw3000_hw_init();
    dw3000_hw_reset();
    osDelay(10);                      // wait for IDLE_RC after reset release

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

    // Switch to fast SPI only AFTER successful initialise
    dw3000_spi_speed_fast();

    uint32_t dev_id = dwt_readdevid();
        if (dev_id == 0xDECA0302UL) {
        /* Success — LED toggle or UART print here */
        mprintf("DW3000 SPI test passed! Device ID: 0x%08X\n", (unsigned int)dev_id);
    }
    

    //lsm6dsv_read_data_polling();
    
    
    osThreadTerminate(NULL);
}

