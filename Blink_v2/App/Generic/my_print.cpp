#include <cstdio>   // For vsnprintf
#include <cstdarg>  // For va_list
#include <cstring>  // For strlen
#include "main.h"   // To get huart1 definition
#include "my_print.h"
#include "usbd_cdc_if.h"


void mprintf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    int len =vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Send via UART (blocking mode, 100ms timeout)
    if (PRINT_MODE == STlink || PRINT_MODE == Both) {
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
    }
    
    if (PRINT_MODE == USB_CDC || PRINT_MODE == Both) {
        if (len > 0) {
            uint32_t timeout = HAL_GetTick();
            while (CDC_Transmit_FS((uint8_t*)buffer, (uint16_t)len) == USBD_BUSY) {
                if (HAL_GetTick() - timeout > 10) break;
            }
        }
    }
}
