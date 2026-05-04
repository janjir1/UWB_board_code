#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "my_print.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "task.h"           // taskENTER_CRITICAL / taskEXIT_CRITICAL
#include "FreeRTOS.h"
#include "semphr.h"

#define PRINT_QUEUE_DEPTH   50
#define PRINT_BUF_SIZE      128

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct {
    uint32_t tick;
    uint16_t len;
    char     data[PRINT_BUF_SIZE];
} print_msg_t;

static osMessageQueueId_t s_print_queue  = NULL;
static SemaphoreHandle_t  s_cdc_tx_done  = NULL;   // given by Tx-complete ISR

/* -----------------------------------------------------------------------
 * Call from usbd_cdc_if.c  CDC_TransmitCplt_FS callback (ISR context):
 *
 *   extern void PrintTask_TxCpltCallback(void);
 *
 *   static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
 *   {
 *       PrintTask_TxCpltCallback();
 *       return USBD_OK;
 *   }
 * --------------------------------------------------------------------- */
void PrintTask_TxCpltCallback(void)
{
    if (s_cdc_tx_done == NULL) return;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_cdc_tx_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* -----------------------------------------------------------------------
 * Public printf-style API — callable from any task or ISR-deferred context.
 * Returns silently if the queue is full (non-blocking).
 * --------------------------------------------------------------------- */
void mprintf(const char *format, ...)
{
    if (s_print_queue == NULL) return;

    print_msg_t msg;
    va_list args;
    va_start(args, format);
    int len = vsnprintf(msg.data, sizeof(msg.data), format, args);
    va_end(args);

    if (len <= 0) return;

    msg.tick = osKernelGetTickCount();
    msg.len  = (uint16_t)(len < (int)sizeof(msg.data) ? len : (int)sizeof(msg.data) - 1);

    osMessageQueuePut(s_print_queue, &msg, 0, 0);  // non-blocking, drop if full
}

/* -----------------------------------------------------------------------
 * Low-priority print task — yields whenever possible.
 * --------------------------------------------------------------------- */
void PrintTask(void *arg)
{
    static uint8_t cdc_tx_buf[12 + PRINT_BUF_SIZE];  // prefix + payload

    s_print_queue = osMessageQueueNew(PRINT_QUEUE_DEPTH, sizeof(print_msg_t), NULL);
    configASSERT(s_print_queue != NULL);

    s_cdc_tx_done = xSemaphoreCreateBinary();
    configASSERT(s_cdc_tx_done != NULL);
    xSemaphoreGive(s_cdc_tx_done);  // first send is immediately available

    print_msg_t msg;

    for (;;)
    {
        /* Block here — yields CPU to every other task until a message arrives */
        if (osMessageQueueGet(s_print_queue, &msg, 0, osWaitForever) != osOK)
            continue;

        /* ── STlink / UART ────────────────────────────────────────────── */
#if (PRINT_MODE == STlink || PRINT_MODE == Both)
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
            HAL_UART_Transmit(&huart1, (uint8_t *)msg.data, msg.len, 100);
#endif

        /* ── USB CDC ──────────────────────────────────────────────────── */
#if (PRINT_MODE == USB_CDC || PRINT_MODE == Both)
        {
            /* Format once into the transmit buffer */
            int plen = snprintf((char *)cdc_tx_buf, 12, "[%8lu] ", msg.tick);
            memcpy(cdc_tx_buf + plen, msg.data, msg.len);
            uint16_t total = (uint16_t)(plen + msg.len);

            /*
             * Wait for the previous USB transfer to complete.
             * Yields CPU — other tasks run freely during this wait.
             * 50 ms timeout guards against USB stack stall / disconnect.
             */
            if (xSemaphoreTake(s_cdc_tx_done, pdMS_TO_TICKS(50)) != pdTRUE)
                continue;   // USB stalled or disconnected — drop this message

            /*
             * Atomic check-and-claim:
             *   - Short critical section (only a few instructions).
             *   - USB IRQ is masked only for this tiny window, NOT during
             *     the actual DMA/PMA transfer, so the USB stack is never
             *     starved.
             *   - Claiming TxState = 1 here prevents a concurrent disconnect
             *     IRQ from tearing down the endpoint between our check and
             *     the transmit call.
             */
            uint8_t usb_ready = 0;
            taskENTER_CRITICAL();
            {
                USBD_CDC_HandleTypeDef *hcdc =
                    (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

                if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
                    hcdc != NULL &&
                    hcdc->TxState == 0)
                {
                    usb_ready = 1;          // ← read-only check, no write
                }
            }
            taskEXIT_CRITICAL();

            if (usb_ready)
            {
                if (CDC_Transmit_FS(cdc_tx_buf, total) != USBD_OK)
                    xSemaphoreGive(s_cdc_tx_done);  // rejected → restore token
                // on USBD_OK: TransmitCplt_FS gives the semaphore back
            }
            else
            {
                xSemaphoreGive(s_cdc_tx_done);      // USB not ready → restore token
            }
        }
#endif
    }
}