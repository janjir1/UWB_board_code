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
#include "../Calculations/ekf.h"
#include "uart.h"
#include "power.h"
#include "sleep.h"

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

    if (sleep_time > 5000) {
        mprintf("WARNING: Invalid sleep time %lu detected. Defaulting to DEEP_SLEEP.\r\n", sleep_time);
        return DEEP_SLEEP - 50;
    }

    return sleep_time;
}

void low_battery_check(void){
    const BatteryStatus_t *s = power_get_status();

    bool bat_too_low = (s->battery_voltage_V > 0.5f)
                    && (s->battery_voltage_V < 3.50f)
                    && !s->usb_connected;

    if (bat_too_low)
    {
        dwm_sleep();

        for (int i = 0; i < 3; i++)
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            osDelay(200);
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            osDelay(200);
        }

        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
        HAL_PWREx_EnterSHUTDOWNMode();
    }

    return;
}

void StartRangingTask(void *argument) {

    power_read();
    low_battery_check();

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

    network_init(dwm_get_addr());

    bool ok = uart_boot_start();
    mprintf("uart_boot_start -> %d\r\n", ok ? 1 : 0);
    
    osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);

    boot_config_t boot_cfg = uart_boot_config_read();

    mprintf("boot_cfg.valid = %d\r\n", boot_cfg.valid ? 1 : 0);
    mprintf("boot_cfg.charge_only = %d\r\n", boot_cfg.charge_only ? 1 : 0);


    ekf_init(boot_cfg.hints, 4);
    uart_rst_arm();

    uint8_t counter = 0;

    if (!boot_cfg.charge_only){
        while(1){
            
            mprintf("Starting sync\r\n");
            HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
            uwb_sync_result_t result_sync = uwb_sync();
            mprintf("Sync result: %d\r\n", result_sync);

            if (result_sync == UWB_SYNC_UNEXPECTED_MASTER){
                HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
                continue;
            } 

            uwb_etwr_result_t result_etwr = uwb_extended_twr(result_sync);

            if ((result_etwr == UWB_TWR_UNEXPECTED_MASTER) || (result_etwr == UWB_TWR_TIMEOUT)){
                HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);
                continue;
            } 

            distance_calculate(result_etwr);

            uint32_t sleep_time = uwb_share (result_etwr, DEEP_SLEEP); 
            sleep_time = tx_err_watchdog(result_sync, result_etwr, sleep_time);

            HAL_GPIO_TogglePin(LED_W_GPIO_Port, LED_W_Pin);

            uint32_t t_start = osKernelGetTickCount();

            dwm_sleep();

            counter++;

            
            if (counter % 50 == 0) {
                power_read();
                uart_print_power();
                low_battery_check();
                counter = 0;
            }

            if (boot_cfg.valid){
                ekf_step(0, 0); //TODO, setup IMU inputs
                network_print_positions();

            }

            bool rst_requested = uart_periodic_poll();
            if (rst_requested){
                mprintf("UART requested reset\r\n");
                osDelay(100);
                NVIC_SystemReset();
            }

            uint32_t elapsed = osKernelGetTickCount() - t_start;

            if (elapsed < sleep_time){
                uint32_t remaining = sleep_time - elapsed;
                 //osDelay(remaining);

                 if (get_usb_ready())
                {
                    osDelay(remaining);
                }
                else
                {
                    
                    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
                    osDelay(10); //finish what needs to be done
                    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);

                    sleep_stop1_ms(remaining-10);
                    //osDelay(remaining);
                }

            }
                

            dwm_wakeup();
        }


    } else {
        dwm_sleep();
        while(1) {
            bool rst_requested = uart_periodic_poll();
            if (rst_requested){
                mprintf("UART requested reset\r\n");
                osDelay(100);        // let RST-ACK finish transmitting before reset
                NVIC_SystemReset();
            }
            osDelay(1000);
        }
    }
    

    vTaskDelete( NULL );
    while(1) { } 
}



