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

QueueHandle_t rx_queue = NULL;
QueueHandle_t tx_queue = NULL;

void dwm_tx_test(void) {
    uint8_t msg[] = {0xC5, 0x00, 'H', 'e', 'l', 'l', 'o'};

    while (true) {
        dwt_configeventcounters(1);          // clear & enable

        dwt_writetxdata(sizeof(msg), msg, 0);
        dwt_writetxfctrl(sizeof(msg) + 2, 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        osDelay(2);                          // wait for frame to complete

        dwt_deviceentcnts_t cnt;
        dwt_readeventcounters(&cnt);
        mprintf("TX frames sent: %u\r\n", cnt.TXF);
        osDelay(1000);
    }
}


void cb_rx_ok(const dwt_cb_data_t *cb_data) {
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_OK, .status = cb_data->status };

    frame.len = cb_data->datalength - 2;  // strip CRC
    dwt_readrxdata(frame.data, frame.len, 0);

    dwt_readrxtimestamp(frame.ts_buf, DWT_COMPAT_NONE);

    dwt_readdiagnostics_acc(&frame.rx_diag, DWT_ACC_IDX_IP_M);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void cb_rx_err(const dwt_cb_data_t *cb_data) {

    dwm_rx_raw_frame_t frame = { .type = DWM_RX_ERR, .status = cb_data->status };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

void cb_rx_to(const dwt_cb_data_t *cb_data) {
    
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_TIMEOUT, .status = 0 };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

void cb_tx_done(const dwt_cb_data_t *cb_data)
{
    uint64_t ts_64 = 0;

    uint8_t ts[5] = {0};
    dwt_readtxtimestamp(ts);
    memcpy(&ts_64, ts, 5);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(tx_queue, &ts_64, &woken);
    portYIELD_FROM_ISR(woken);
}

void dwm_rx_continuous(void) {
    dwm_rx_raw_frame_t frame;

    dwt_setrxtimeout(976562);
    dwt_setpreambledetecttimeout(0); 
    
    dwt_rxenable(0);

    while (true) {
            if (xQueueReceive(rx_queue, &frame, portMAX_DELAY) != pdTRUE) continue;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            switch (frame.type) {

                case DWM_RX_OK:{

                    int16_t rssi_q8 = 0;
                    int16_t fp_q8   = 0;

                    int err =dwt_calculate_rssi(&frame.rx_diag, DWT_ACC_IDX_IP_M, &rssi_q8);
                    if (err != DWT_SUCCESS) {
                        mprintf("Failed to calculate RSSI: %d\r\n", err);
                    }
                    err = dwt_calculate_first_path_power(&frame.rx_diag, DWT_ACC_IDX_IP_M, &fp_q8);
                    if (err != DWT_SUCCESS) {
                        mprintf("Failed to calculate First Path Power: %d\r\n", err);
                    }

                    mprintf("RX OK %u bytes: ", frame.len);
                    for (uint16_t i = 0; i < frame.len; i++) {
                        mprintf("%c", (frame.data[i] >= 32 && frame.data[i] < 127)
                                    ? frame.data[i] : '.');
                    }
                    mprintf("\r\n");

                    uint64_t rx_ts = 0;
                    for (int i = 0; i < 5; i++) {
                        rx_ts |= ((uint64_t)frame.ts_buf[i]) << (i * 8);
                    }

                    mprintf("  Timestamp : %08lX%08lX ticks\r\n",
                        (uint32_t)(rx_ts >> 32),
                        (uint32_t)(rx_ts & 0xFFFFFFFFUL));
                    mprintf("  RSSI       : %.1f dBm\r\n", rssi_q8 / 256.0f);
                    mprintf("  First path : %.1f dBm\r\n", fp_q8 / 256.0f);
                    break;
                }

                case DWM_RX_ERR:
                    if      (frame.status & DWT_INT_RXPHE_BIT_MASK) mprintf("RX ERR: PHR error\r\n");
                    else if (frame.status & DWT_INT_RXFCE_BIT_MASK) mprintf("RX ERR: CRC mismatch\r\n");
                    else if (frame.status & DWT_INT_RXFSL_BIT_MASK) mprintf("RX ERR: Frame sync loss\r\n");
                    else if (frame.status & DWT_INT_RXSTO_BIT_MASK) mprintf("RX ERR: SFD timeout\r\n");
                    else mprintf("RX ERR: unknown 0x%08lX\r\n", frame.status);
                    break;

                case DWM_RX_TIMEOUT:
                    mprintf("RX timeout - re-armed\r\n");
                    break;
            }
        }
}

void dwm_rx(dwm_rx_frame_t *result, uint32_t timeout_ms) {

    dwt_setrxtimeout(timeout_ms * 1000);
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    dwm_rx_raw_frame_t frame;
    // Wait forever — DW3000 will always eventually post DWM_RX_TIMEOUT
    xQueueReceive(rx_queue, &frame, portMAX_DELAY);
    dwt_forcetrxoff();

    result->type   = frame.type;
    result->status = frame.status;
    
    switch (frame.type) {
        case DWM_RX_OK:{
            // Copy data
            result->len = frame.len;
            memcpy(result->data, frame.data, frame.len);

            // Convert timestamp — bit shifts only, safe and fast
            result->rx_timestamp = 0;
            for (int i = 0; i < 5; i++) {
                result->rx_timestamp |= ((uint64_t)frame.ts_buf[i]) << (i * 8);
            }

            // Copy raw diag — caller computes RSSI/FP outside
            int err =dwt_calculate_rssi(&frame.rx_diag, DWT_ACC_IDX_IP_M, &result->rssi_q8);
            if (err != DWT_SUCCESS) {
                mprintf("Failed to calculate RSSI: %d\r\n", err);
            }
            err = dwt_calculate_first_path_power(&frame.rx_diag, DWT_ACC_IDX_IP_M, &result->fp_q8);
            if (err != DWT_SUCCESS) {
                mprintf("Failed to calculate First Path Power: %d\r\n", err);
            }
            break;
        }
        case DWM_RX_ERR:
            if      (frame.status & DWT_INT_RXPHE_BIT_MASK) mprintf("RX ERR: PHR error\r\n");
            else if (frame.status & DWT_INT_RXFCE_BIT_MASK) mprintf("RX ERR: CRC mismatch\r\n");
            else if (frame.status & DWT_INT_RXFSL_BIT_MASK) mprintf("RX ERR: Frame sync loss\r\n");
            else if (frame.status & DWT_INT_RXSTO_BIT_MASK) mprintf("RX ERR: SFD timeout\r\n");
            else mprintf("RX ERR: unknown 0x%08lX\r\n", frame.status);
            break;

        case DWM_RX_TIMEOUT:
            break;
    }
}

dwm_tx_event_type_t dwm_tx(dwm_tx_frame_t *frame)
{
     dwt_forcetrxoff();

    // Write payload before firing
    dwt_writetxdata(frame->len + 2, frame->data, 0);
    dwt_writetxfctrl(frame->len + 2, 0, 1);

    xQueueReset(tx_queue);  // flush stale events

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR) return DWM_TX_LATE;

    // Block until TX done callback fires and posts the timestamp
    uint64_t ts = 0;
    if (xQueueReceive(tx_queue, &ts, pdMS_TO_TICKS(10)) != pdTRUE) {
        return DWM_TX_ERROR;
    }

    frame->tx_timestamp = ts;
    return DWM_TX_OK;

}


    /*
    uint32_t tx_time = frame->tx_time;
    if (frame->tx_time == 0){
            tx_time = dwt_readsystimestamphi32() + TX_DELAY_500US;
    }

    if (frame->embed_timestamp) {
        uint64_t tx_ts = ((uint64_t)tx_time << 8) + dwt_gettxantennadelay();
        mprintf("TX_TS: %08lX%02X\r\n",
            (uint32_t)(tx_ts >> 8),    // high 32 bits
            (uint8_t) (tx_ts & 0xFF)   // low 8 bits
        );
        memcpy(&frame->data[frame->ts_position], &tx_ts, 5);
    }
    */

    

void dwm_tx_continuous(void)
{
    uint8_t msg[18];  // 0xDE 0xAD + counter(1) + timestamp(5) + "HELLOWORLD"(10)
    const char *text = "HELLOWORLD";

    msg[0] = 0xDE;
    msg[1] = 0xAD;
    // msg[2]    = counter
    // msg[3..7] = previous TX timestamp (5 bytes)
    // msg[8..17] = "HELLOWORLD"
    memcpy(&msg[8], text, 10);

    dwm_tx_frame_t frame = {
        .data         = msg,
        .len          = sizeof(msg),
        .tx_timestamp = 0,
    };

    uint8_t counter = 0;

    while (true)
    {
        // Embed counter and previous timestamp into message
        msg[2] = counter;
        memcpy(&msg[3], &frame.tx_timestamp, 5);  // previous frame's timestamp

        dwm_tx_event_type_t result = dwm_tx(&frame);  // frame.tx_timestamp updated after send

        if (result == DWM_TX_OK) {
            mprintf("[%3d] Sent TS: %08lX%02X\r\n",
                counter,
                (uint32_t)(frame.tx_timestamp >> 8),
                (uint8_t) (frame.tx_timestamp & 0xFF));
        } else if (result == DWM_TX_LATE) {
            mprintf("[%3d] TX LATE\r\n", counter);
        } else {
            mprintf("[%3d] TX ERROR\r\n", counter);
        }

        counter++;
        osDelay(1000);
    }
}