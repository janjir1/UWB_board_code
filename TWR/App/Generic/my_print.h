#ifndef MY_PRINTF_H
#define MY_PRINTF_H

#ifdef __cplusplus
extern "C" {
#endif

#define STlink 1
#define USB_CDC 2
#define Both 3

// ---- Change this to select output method ----
#define PRINT_MODE Both

#include "main.h"

// Function Prototype
void mprintf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif 
