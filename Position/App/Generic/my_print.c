#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "main.h"   // To get huart1 definition
#include "my_print.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"


#define PRINT_QUEUE_DEPTH   32
#define PRINT_BUF_SIZE      128
extern USBD_HandleTypeDef hUsbDeviceFS;


typedef struct {
    uint32_t tick;                  // raw, unformatted
    uint16_t len;
    char     data[PRINT_BUF_SIZE];
} print_msg_t;

static osMessageQueueId_t s_print_queue = NULL;
/*
static void ensure_queue_init(void)
{
    if (s_print_queue == NULL) {
        s_print_queue = osMessageQueueNew(PRINT_QUEUE_DEPTH,
                                          sizeof(print_msg_t), NULL);
    }
}
*/

void mprintf(const char *format, ...)
{
    print_msg_t msg;

    va_list args;
    va_start(args, format);
    int len = vsnprintf(msg.data, sizeof(msg.data), format, args);
    va_end(args);

    if (len <= 0) return;

    msg.tick = osKernelGetTickCount();   // cheap, no formatting
    msg.len  = (uint16_t)(len < sizeof(msg.data) ? len : sizeof(msg.data) - 1);

    osMessageQueuePut(s_print_queue, &msg, 0, 0);
}

/* Low-priority task — blocking here is fine */
void PrintTask(void *arg)
{
    static uint8_t cdc_tx_buf[PRINT_BUF_SIZE];

    s_print_queue = osMessageQueueNew(PRINT_QUEUE_DEPTH, sizeof(print_msg_t), NULL);
    configASSERT(s_print_queue != NULL);

    print_msg_t msg;
    for (;;) {
        if (osMessageQueueGet(s_print_queue, &msg, 0, osWaitForever) != osOK)
            continue;

#if (PRINT_MODE == STlink || PRINT_MODE == Both)
    // HAL_UART_Transmit blocks 100ms even with nothing connected
    // Only transmit if UART is ready
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
        HAL_UART_Transmit(&huart1, (uint8_t *)msg.data, msg.len, 100);
#endif

#if (PRINT_MODE == USB_CDC || PRINT_MODE == Both)
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
        hUsbDeviceFS.pClassData != NULL)                   // ← NULL check
    {
        uint8_t prefix[12];
        int plen = snprintf((char *)prefix, sizeof(prefix), "[%8lu] ", msg.tick);
        memcpy(cdc_tx_buf,         prefix,   plen);
        memcpy(cdc_tx_buf + plen,  msg.data, msg.len);
        uint16_t total = plen + msg.len;

        /* Wrap in critical section so USB disconnect can't tear down
         * endpoints between our state check and the actual transmit */
        taskENTER_CRITICAL();
        uint8_t usb_ok = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
                          hUsbDeviceFS.pClassData != NULL);
        if (usb_ok) CDC_Transmit_FS(cdc_tx_buf, total);
        taskEXIT_CRITICAL();
    }
#endif
    }
}