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
#include "DWM3000_RxTx.h"

#include <inttypes.h>

// ─── Queues ───────────────────────────────────────────────────────────────────

QueueHandle_t rx_queue = NULL;
QueueHandle_t tx_queue = NULL;
QueueHandle_t wakeup_queue = NULL;

// ─── Constants ────────────────────────────────────────────────────────────────

#define DWM_CRC_LEN          2U      // DW3000 auto-appends 2-byte CRC
#define DWM_RX_TIMEOUT_1S    976562U // ~1s in UWB symbol ticks (~1µs each)
#define DWM_ANT_DELAY        16385U

// ─── Internal helpers ─────────────────────────────────────────────────────────

static inline uint64_t ts_buf_to_u64(const uint8_t buf[5])
{
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++)
        ts |= ((uint64_t)buf[i]) << (i * 8);
    return ts;
}

static void log_rx_error(uint32_t status)
{
    if      (status & DWT_INT_RXPHE_BIT_MASK) mprintf("RX ERR: PHR error\r\n");
    else if (status & DWT_INT_RXFCE_BIT_MASK) mprintf("RX ERR: CRC mismatch\r\n");
    else if (status & DWT_INT_RXFSL_BIT_MASK) mprintf("RX ERR: Frame sync loss\r\n");
    else if (status & DWT_INT_RXSTO_BIT_MASK) mprintf("RX ERR: SFD timeout\r\n");
    else                                       mprintf("RX ERR: unknown 0x%08lX\r\n", status);
}

// ─── ISR Callbacks ────────────────────────────────────────────────────────────

void cb_rx_ok(const dwt_cb_data_t *cb_data)
{
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_OK, .status = cb_data->status };

    frame.len = cb_data->datalength - DWM_CRC_LEN;
    dwt_readrxdata(frame.data, frame.len, 0);
    dwt_readrxtimestamp(frame.ts_buf, DWT_COMPAT_NONE);
    dwt_readdiagnostics_acc(&frame.rx_diag, DWT_ACC_IDX_IP_M);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

void cb_rx_err(const dwt_cb_data_t *cb_data)
{
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_ERR, .status = cb_data->status };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

void cb_rx_to(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_TIMEOUT, .status = 0 };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

void cb_tx_done(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    uint8_t ts_buf[5] = {0};
    dwt_readtxtimestamp(ts_buf);
    uint64_t ts_64 = ts_buf_to_u64(ts_buf);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(tx_queue, &ts_64, &woken);
    portYIELD_FROM_ISR(woken);
}

void cb_spi_rdy(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    uint8_t rdy = 1;

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(wakeup_queue, &rdy, &woken);
    portYIELD_FROM_ISR(woken);
}


// ─── RX ───────────────────────────────────────────────────────────────────────

void dwm_rx(dwm_rx_frame_t *result, uint32_t timeout_ms)
{
    dwt_setrxtimeout(timeout_ms * 1000U);
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    dwm_rx_raw_frame_t frame;
    xQueueReceive(rx_queue, &frame, portMAX_DELAY);
    dwt_forcetrxoff();

    result->type   = frame.type;
    result->status = frame.status;

    if (frame.type == DWM_RX_ERR)  { log_rx_error(frame.status); return; }
    if (frame.type != DWM_RX_OK)   { return; }

    result->len          = frame.len;
    result->rx_timestamp = ts_buf_to_u64(frame.ts_buf);
    memcpy(result->data, frame.data, frame.len);

    if (dwt_calculate_rssi(&frame.rx_diag, DWT_ACC_IDX_IP_M, &result->rssi_q8) != DWT_SUCCESS)
        mprintf("RSSI calc failed\r\n");

    if (dwt_calculate_first_path_power(&frame.rx_diag, DWT_ACC_IDX_IP_M, &result->fp_q8) != DWT_SUCCESS)
        mprintf("FP power calc failed\r\n");
}


// ─── TX ───────────────────────────────────────────────────────────────────────

dwm_tx_event_type_t dwm_tx(dwm_tx_frame_t *frame)
{
    dwt_forcetrxoff();

    dwt_writetxdata(frame->len + DWM_CRC_LEN, frame->data, 0);
    dwt_writetxfctrl(frame->len + DWM_CRC_LEN, 0, 1);

    xQueueReset(tx_queue);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR)
        return DWM_TX_LATE;

    uint64_t ts = 0;
    if (xQueueReceive(tx_queue, &ts, pdMS_TO_TICKS(10)) != pdTRUE)
        return DWM_TX_ERROR;

    frame->tx_timestamp = ts;
    return DWM_TX_OK;
}

   

void dwm_tx_continuous(void)
{
    // Frame layout: [0xDE][0xAD][counter:1][prev_ts:5]["HELLOWORLD":10]
    uint8_t msg[18];
    msg[0] = 0xDE;
    msg[1] = 0xAD;
    memcpy(&msg[8], "HELLOWORLD", 10);

    dwm_tx_frame_t frame = { .data = msg, .len = sizeof(msg), .tx_timestamp = 0 };
    uint8_t counter = 0;

    while (true) {
        msg[2] = counter;
        memcpy(&msg[3], &frame.tx_timestamp, 5);  // embed previous TX timestamp

        dwm_wakeup();
        dwm_tx_event_type_t result = dwm_tx(&frame);
        dwm_sleep();

        switch (result) {
        case DWM_TX_OK:
            mprintf("[%3d] Sent TS: %08lX%02X\r\n",
                counter,
                (uint32_t)(frame.tx_timestamp >> 8),
                (uint8_t) (frame.tx_timestamp & 0xFF));
            break;
        case DWM_TX_LATE:  mprintf("[%3d] TX LATE\r\n",  counter); break;
        case DWM_TX_ERROR: mprintf("[%3d] TX ERROR\r\n", counter); break;
        }

        counter++;
        osDelay(1000);
    }
}

// ─── Sleep / Wakeup ───────────────────────────────────────────────────────────

void dwm_sleep(void)
{
    dwt_configuresleep(
        DWT_CONFIG | DWT_PGFCAL,    // save config to AON; run PGF cal on wakeup
        DWT_WAKE_WUP | DWT_SLP_EN   // wake on WAKEUP pin; enable sleep
    );
    dwt_entersleep(DWT_DW_IDLE);    // chip is in IDLE_PLL — must match actual state
}

void dwm_wakeup(void)
{
    xQueueReset(wakeup_queue);

    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_SET);

    uint8_t rdy = 0;
    if (xQueueReceive(wakeup_queue, &rdy, pdMS_TO_TICKS(10)) != pdTRUE)
        mprintf("DW3000 wakeup timeout!\r\n");

    osDelay(1);
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_RESET);

    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(DWM_EXTI_Pin);
    dwt_restoreconfig(1);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    osDelay(2);  // PLL stabilisation — guaranteed minimum after restoreconfig(1)

    dwt_setrxantennadelay(DWM_ANT_DELAY);
    dwt_settxantennadelay(DWM_ANT_DELAY);
    dwt_setinterrupt(DWM_IRQ_MASK, 0, DWT_ENABLE_INT);
}


