#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

extern LPTIM_HandleTypeDef hlptim1;

/* Set in the LPTIM1 callback when timeout expires */
static volatile uint8_t s_lptim1_woke = 0;

static uint16_t lptim1_arr_from_ms(uint32_t ms)
{
    uint32_t ticks = (ms * 32768U) / 1000U;   // LSE = 32768 Hz

    if (ticks == 0U) {
        ticks = 1U;
    }
    if (ticks > 0x10000UL) {
        ticks = 0x10000UL;                    // 16-bit counter limit
    }

    return (uint16_t)(ticks - 1U);
}


/* HAL callback */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim == &hlptim1) {
        s_lptim1_woke = 1U;
    }
}

void sleep_stop1_ms(uint32_t ms)
{

    uint16_t arr = lptim1_arr_from_ms(ms);
    s_lptim1_woke = 0U;

    /* Clear any stale flags */
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_ARRM | LPTIM_FLAG_ARROK |
                                     LPTIM_FLAG_CMPM | LPTIM_FLAG_CMPOK);

    /* Start one-shot timeout interrupt.
       Period can be max, timeout value is our wakeup point. */
    if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 0xFFFFU, arr) != HAL_OK) {
        return;
    }

    HAL_SuspendTick();

    /* Enter Stop1, wake on LPTIM1 interrupt */
    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    /* Back from Stop1 */
    SystemClock_Config();
    HAL_ResumeTick();

    HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);

    ReinitAdc();
}