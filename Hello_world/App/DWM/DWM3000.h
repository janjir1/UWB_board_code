#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/* FreeRTOS task entry point — called by osThreadNew() in main.c */
void StartDWM(void *argument);

#ifdef __cplusplus
}
#endif
