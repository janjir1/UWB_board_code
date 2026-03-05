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
#include "DWM3000_RxTx.h"

extern SPI_HandleTypeDef hspi1; 

static bool dwm_selfTest(void);
static bool dwm_init(void);
static uint16_t dwm_configure(void);


void StartDWM(void *argument) {

    bool passed = dwm_init();;
    if (passed) {
        mprintf("DWM3000 initialized successfully\r\n");
    } else {
        mprintf("DWM3000 initialization failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(1000);

    passed = dwm_selfTest();
    if (passed) {
        mprintf("DWM3000 self test passed\r\n");
    } else {
        mprintf("DWM3000 self test failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(1000);

    uint16_t addr = dwm_configure();
    if (addr != 0) {
        mprintf("DWM3000 configured successfully\r\n");
    } else {
        mprintf("DWM3000 configure failed\r\n");
        vTaskDelete( NULL );
        while(1) { } 
    }
    osDelay(1000);

    if (addr == 0x506B) {
        dwm_tx_test();   // device A transmits
    } else {
        dwm_frame_t result;
        while(1){
            osDelay(900);
            dwm_rx(&result, 200);
            if (result.type == DWM_RX_TIMEOUT) {
                mprintf("Frame timeout\r\n");
            }
            else if (result.type == DWM_RX_OK) {
                mprintf("Frame received: %d\r\n", result.type);
            }
            else if (result.type == DWM_RX_ERR) {
                mprintf("Frame error: 0x%08lX\r\n", result.status);
            
            }
        }

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
        return false;      
    }
    return true;
}

static bool dwm_init(void) {

    dw3000_hw_init();
    dw3000_hw_reset();
    osDelay(10);

    extern const struct dwt_probe_s dw3000_probe_interf;
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        return false;
    }

    // Poll until chip is in IDLE_RC — do NOT skip this
    uint32_t timeout = 1000;
    while (!dwt_checkidlerc()) {
        osDelay(1);
        if (--timeout == 0) {
            mprintf("Idle_RC timeout");
            return false;
         // chip never became ready
        }
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        return false;
    }

    dw3000_spi_speed_fast();
    return true;
}

static uint16_t dwm_configure(void) {

    static const dwt_config_t config = {
        .chan            = DWM_UWB_CHANNEL,
        .txPreambLength  = DWM_UWB_PREAMBLE_LEN,
        .rxPAC           = DWM_UWB_PAC, 
        .txCode          = DWM_UWB_TX_CODE, 
        .rxCode          = DWM_UWB_RX_CODE,
        .sfdType         = DWM_UWB_SFD_TYPE,  
        .dataRate        = DWM_UWB_DATA_RATE,
        .phrMode         = DWM_UWB_PHR_MODE,
        .phrRate         = DWM_UWB_PHR_RATE,
        .sfdTO           = DWM_UWB_SFD_TO, 
        .stsMode         = DWM_UWB_STS_MODE,
        .stsLength       = DWM_UWB_STS_LENGTH,
        .pdoaMode        = DWM_UWB_PDOA_MODE,
    };

    static const dwt_txconfig_t tx_config = {
        .PGdly   = TX_RF_PGdly,
        .power   = TX_RF_Power,
        .PGcount = TX_RF_PGcount,
    };

    dwt_forcetrxoff();

    int err = dwt_configure((dwt_config_t *)&config);
    if (err != DWT_SUCCESS) {
        mprintf("Configuration failed: %d", err);
        return 0;
    }

    dwt_configuretxrf((dwt_txconfig_t *)&tx_config);

    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);

    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint16_t addr = (uint16_t)((uid0 ^ uid1) & 0xFFFF);

    dwt_setpanid(0xDECA);
    dwt_setaddress16(addr);
    mprintf("Short addr: 0x%04X\r\n", addr);

    //configure for intrupts

    

    static dwt_callbacks_s callbacks = {
    .cbTxDone  = NULL,
    .cbRxOk    = cb_rx_ok,
    .cbRxTo    = cb_rx_to,
    .cbRxErr   = cb_rx_err,
    .cbSPIErr  = NULL,
    .cbSPIRdy  = NULL,
    };

    dwt_setcallbacks(&callbacks);

    dwt_setinterrupt(
        DWT_INT_RXFCG_BIT_MASK  |   // RX good frame → cb_rx_ok
        DWT_INT_RXFCE_BIT_MASK   |   // FCS/CRC error → cb_rx_err
        DWT_INT_RXPHE_BIT_MASK   |   // PHR error → cb_rx_err
        DWT_INT_RXFSL_BIT_MASK   |   // frame sync loss → cb_rx_err
        DWT_INT_RXSTO_BIT_MASK  |   // SFD timeout → cb_rx_err
        DWT_INT_RXFTO_BIT_MASK,     // frame wait timeout → cb_rx_to
        0,
        DWT_ENABLE_INT
    );

    rx_queue = xQueueCreate(8, sizeof(dwm_raw_frame_t));

    return addr;
}



