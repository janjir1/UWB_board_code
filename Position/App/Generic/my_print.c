#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "my_print.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "task.h"       /* taskENTER_CRITICAL / taskEXIT_CRITICAL */
#include "FreeRTOS.h"
#include "semphr.h"

/** @brief Maximum number of pending log messages in the print queue. */
#define PRINT_QUEUE_DEPTH  50
/** @brief Maximum formatted string length per message (bytes, including NUL). */
#define PRINT_BUF_SIZE     128

extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief One queued print message — tick-stamped at enqueue time.
 *
 * Kept small so the full queue fits in a predictable amount of heap.
 */
typedef struct {
    uint32_t tick;              /**< RTOS tick count at time of @ref mprintf call. */
    uint16_t len;               /**< Valid byte count in @c data (without NUL). */
    char     data[PRINT_BUF_SIZE]; /**< Formatted string. */
} print_msg_t;

static osMessageQueueId_t s_print_queue = NULL;
static SemaphoreHandle_t  s_cdc_tx_done = NULL; /**< Given by the USB Tx-complete ISR. */

/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * @brief USB CDC Tx-complete callback — call from the ISR context.
 *
 * Wire this into @c usbd_cdc_if.c as follows:
 * @code
 * extern void PrintTask_TxCpltCallback(void);
 *
 * static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
 * {
 *     PrintTask_TxCpltCallback();
 *     return USBD_OK;
 * }
 * @endcode
 *
 * Releases the @c s_cdc_tx_done binary semaphore so @ref PrintTask can
 * proceed with the next queued message. Safe to call from ISR context.
 */
void PrintTask_TxCpltCallback(void)
{
    if (s_cdc_tx_done == NULL) return;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_cdc_tx_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief printf-style logging function — callable from any task context.
 *
 * Formats the message into a @c print_msg_t and enqueues it non-blocking.
 * If the queue is full the message is silently dropped; the caller is never
 * blocked or delayed. Safe to call before @ref PrintTask has started, in
 * which case messages are also silently dropped.
 *
 * The tick count is captured at call time so log timestamps reflect when
 * the event occurred, not when @ref PrintTask eventually drains the queue.
 *
 * @param format  printf-compatible format string.
 * @param ...     Format arguments.
 */
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

    osMessageQueuePut(s_print_queue, &msg, 0, 0); /* non-blocking — drop if full */
}

/**
 * @brief Low-priority FreeRTOS print task — drains the message queue.
 *
 * Must be created exactly once via @c osThreadNew() before the scheduler
 * starts. Initialises the message queue and the @c s_cdc_tx_done semaphore
 * internally so both are guaranteed to exist before any other task calls
 * @ref mprintf.
 *
 * **Output paths** (selected at build time by @c PRINT_MODE):
 * - @c STlink / @c Both — transmits raw message bytes over UART1 (blocking,
 *   100 ms timeout). Suitable for low-rate debug output.
 * - @c USB_CDC / @c Both — prepends an 8-digit tick prefix, then transmits
 *   over USB CDC with a flow-control semaphore:
 *   - Waits up to 50 ms for the previous transfer to complete; drops the
 *     message on timeout (USB stall or disconnect).
 *   - A short critical section gates the @c TxState check to prevent a
 *     concurrent disconnect IRQ from tearing down the endpoint between the
 *     check and the transmit call. The USB stack is never held across the
 *     actual DMA/PMA transfer.
 *   - On @c USBD_OK: @ref PrintTask_TxCpltCallback gives the semaphore back.
 *   - On rejection or USB-not-ready: the semaphore token is restored
 *     immediately so the next message is not blocked.
 *
 * @param arg  Unused; required by CMSIS-RTOS2 task signature.
 */
void PrintTask(void *arg)
{
    static uint8_t cdc_tx_buf[12 + PRINT_BUF_SIZE]; /* prefix + payload */

    s_print_queue = osMessageQueueNew(PRINT_QUEUE_DEPTH, sizeof(print_msg_t), NULL);
    configASSERT(s_print_queue != NULL);

    s_cdc_tx_done = xSemaphoreCreateBinary();
    configASSERT(s_cdc_tx_done != NULL);
    xSemaphoreGive(s_cdc_tx_done); /* first send is immediately available */

    print_msg_t msg;

    for (;;) {
        /* Block here — yields CPU to every other task until a message arrives */
        if (osMessageQueueGet(s_print_queue, &msg, 0, osWaitForever) != osOK)
            continue;

        /* -- STlink / UART ------------------------------------------------ */
#if (PRINT_MODE == STlink || PRINT_MODE == Both)
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
            HAL_UART_Transmit(&huart1, (uint8_t *)msg.data, msg.len, 100);
#endif

        /* -- USB CDC ------------------------------------------------------ */
#if (PRINT_MODE == USB_CDC || PRINT_MODE == Both)
        {
            /* Format once into the transmit buffer */
            int      plen  = snprintf((char *)cdc_tx_buf, 12, "[%8lu] ", msg.tick);
            memcpy(cdc_tx_buf + plen, msg.data, msg.len);
            uint16_t total = (uint16_t)(plen + msg.len);

            /* Wait for the previous USB transfer to complete.
             * Yields CPU — other tasks run freely during this wait.
             * 50 ms timeout guards against USB stack stall / disconnect. */
            if (xSemaphoreTake(s_cdc_tx_done, pdMS_TO_TICKS(50)) != pdTRUE)
                continue; /* USB stalled or disconnected — drop this message */

            /* Atomic check-and-claim:
             * - Short critical section (only a few instructions).
             * - USB IRQ masked only for this window, NOT during the DMA/PMA
             *   transfer, so the USB stack is never starved.
             * - Guards against a concurrent disconnect IRQ tearing down the
             *   endpoint between our TxState check and the transmit call. */
            uint8_t usb_ready = 0;
            taskENTER_CRITICAL();
            {
                USBD_CDC_HandleTypeDef *hcdc =
                    (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

                if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
                    hcdc != NULL &&
                    hcdc->TxState == 0)
                {
                    usb_ready = 1; /* read-only check — TxState not modified here */
                }
            }
            taskEXIT_CRITICAL();

            if (usb_ready) {
                if (CDC_Transmit_FS(cdc_tx_buf, total) != USBD_OK)
                    xSemaphoreGive(s_cdc_tx_done); /* rejected → restore token */
                /* on USBD_OK: PrintTask_TxCpltCallback gives the semaphore back */
            } else {
                xSemaphoreGive(s_cdc_tx_done); /* USB not ready → restore token */
            }
        }
#endif
    }
}

/**
 * @brief Return true if the USB device is in the configured (active) state.
 *
 * Intended as a quick pre-flight check before sending large amounts of data.
 * Does not guarantee that a subsequent @ref mprintf will succeed — the device
 * may disconnect between this call and the actual transmit.
 *
 * @return true if @c hUsbDeviceFS is @c USBD_STATE_CONFIGURED, false otherwise.
 */
bool get_usb_ready(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}