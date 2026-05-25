#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>
#include "../Generic/my_print.h"

/**
 * @brief FreeRTOS watchdog-refresh task.
 *
 * Feeds the IWDG every 1 000 ms. The IWDG timeout is configured for 3 seconds
 * in CubeMX, so this task must not be starved for longer than that or the
 * device will reset.
 *
 * Also serves as a 1 Hz heartbeat indicator: when @c UWB_DEBUG is defined,
 * a counter value and board variant are printed each cycle.
 *
 * @param argument  Unused; required by the CMSIS-RTOS2 task signature.
 */
void StartBlink(void *argument)
{
#ifdef UWB_DEBUG
    int i = 0;
#endif

    while (1) {
        HAL_IWDG_Refresh(&hiwdg); /* IWDG timeout configured to 3 seconds */

#ifdef UWB_DEBUG
#ifdef UWB_BOARD_V1_1
        mprintf("Hello from Board V1.1! Count: %d\r\n", i);
#else
        mprintf("Hello from Board V1.0! Count: %d\r\n", i);
#endif
        i++;
#endif

        osDelay(1000);
    }

    vTaskDelete(NULL);
    while (1) {}
}