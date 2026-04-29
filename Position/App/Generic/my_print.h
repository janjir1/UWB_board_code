#ifndef MY_PRINTF_H
#define MY_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

#define STlink 1
#define USB_CDC 2
#define Both 3

// ---- Change this to select output method ----
#ifdef UWB_BOARD_V1_1
    #define PRINT_MODE USB_CDC
#else
    #define PRINT_MODE Both
#endif


// Function Prototype
void mprintf(const char *format, ...);
void PrintTask(void *arg);

#ifdef __cplusplus
}
#endif

#endif 
