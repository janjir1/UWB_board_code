#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "main.h"   // To get huart1 definition
#include "my_print.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"


#define PRINT_QUEUE_DEPTH   32
#define PRINT_BUF_SIZE      128

typedef struct {
    char data[PRINT_BUF_SIZE];
    uint16_t len;
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
    //ensure_queue_init();

    print_msg_t msg;
    va_list args;
    va_start(args, format);
    int len = vsnprintf(msg.data, PRINT_BUF_SIZE, format, args);
    va_end(args);

    if (len <= 0) return;
    msg.len = (uint16_t)(len < PRINT_BUF_SIZE ? len : PRINT_BUF_SIZE - 1);

    /* Non-blocking put — drop message if queue is full rather than stalling caller */
    osMessageQueuePut(s_print_queue, &msg, 0, 0);
}

/* Low-priority task — blocking here is fine */
void PrintTask(void *arg)
{
    (void)arg;
    s_print_queue = osMessageQueueNew(PRINT_QUEUE_DEPTH, sizeof(print_msg_t), NULL);

    print_msg_t msg;
    for (;;) {
        if (osMessageQueueGet(s_print_queue, &msg, 0, osWaitForever) != osOK)
            continue;

#if (PRINT_MODE == STlink || PRINT_MODE == Both)
        HAL_UART_Transmit(&huart1, (uint8_t *)msg.data, msg.len, 100);
#endif

#if (PRINT_MODE == USB_CDC || PRINT_MODE == Both)
        uint32_t t0 = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t *)msg.data, msg.len) == USBD_BUSY) {
            if (HAL_GetTick() - t0 > 5) break;
            osDelay(1);   /* yield while waiting for USB host poll */
        }
#endif
    }
}