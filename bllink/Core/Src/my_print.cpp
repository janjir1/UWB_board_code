#include <cstdio>   // For vsnprintf
#include <cstdarg>  // For va_list
#include <cstring>  // For strlen
#include "main.h"   // To get huart1 definition
#include "my_print.h"

void mprintf(const char *format, ...)
{
    char buffer[128]; // Adjust size as needed
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args); // Format the string
    va_end(args);

    // Send via UART (blocking mode, 100ms timeout)
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}
