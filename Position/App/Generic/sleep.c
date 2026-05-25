#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

extern LPTIM_HandleTypeDef hlptim1;

/** @brief Set to 1 by the LPTIM1 auto-reload-match ISR when the timeout expires. */
static volatile uint8_t s_lptim1_woke = 0;

/**
 * @brief Convert a millisecond duration to an LPTIM1 compare (timeout) register value.
 *
 * Assumes the LPTIM1 clock source is the LSE at 32 768 Hz.
 * The result is clamped to [0, 0xFFFE] so it always fits the 16-bit counter
 * and is never equal to the ARR value (0xFFFF), which would prevent the
 * compare match from firing.
 *
 * @param ms  Desired timeout in milliseconds.
 * @return    LPTIM1 compare register value (ARR - 1 of the timeout period).
 */
static uint16_t lptim1_arr_from_ms(uint32_t ms)
{
    uint32_t ticks = (ms * 32768U) / 1000U; /* LSE = 32 768 Hz */

    if (ticks == 0U)        ticks = 1U;
    if (ticks > 0x10000UL)  ticks = 0x10000UL; /* 16-bit counter limit */

    return (uint16_t)(ticks - 1U);
}

/**
 * @brief LPTIM1 auto-reload-match HAL callback — fires from ISR context.
 *
 * Sets @c s_lptim1_woke to signal @ref sleep_stop1_ms that the requested
 * timeout has elapsed. Ignores callbacks from any other LPTIM instance.
 *
 * @param hlptim  LPTIM handle that triggered the callback.
 */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim == &hlptim1) {
        s_lptim1_woke = 1U;
    }
}

/**
 * @brief Enter STM32L4 Stop1 mode for @p ms milliseconds, then resume.
 *
 * Uses LPTIM1 in one-shot timeout mode as the wakeup source. The sequence is:
 * 1. Compute the LPTIM1 compare value from @p ms (@ref lptim1_arr_from_ms).
 * 2. Clear any stale LPTIM1 flags.
 * 3. Start the one-shot timeout interrupt (ARR = 0xFFFF, compare = computed value).
 * 4. Suspend the SysTick and enter Stop1 via WFI.
 * 5. On wakeup: restore the system clock, resume SysTick, stop the LPTIM, and
 *    reinitialise the ADC (clocks are reconfigured by @c SystemClock_Config).
 *
 * @note Must not be called from ISR context or while holding any RTOS mutex.
 *       FreeRTOS tick is suspended for the full sleep duration.
 *
 * @param ms  Sleep duration in milliseconds (clamped to ~2 s by the 16-bit counter).
 */
void sleep_stop1_ms(uint32_t ms)
{
    uint16_t arr = lptim1_arr_from_ms(ms);
    s_lptim1_woke = 0U;

    /* Clear any stale LPTIM1 flags before arming */
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_ARRM | LPTIM_FLAG_ARROK |
                                      LPTIM_FLAG_CMPM | LPTIM_FLAG_CMPOK);

    /* One-shot timeout: ARR = 0xFFFF (max period), compare = arr (wakeup point).
     * The interrupt fires when the counter reaches the compare value. */
    if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 0xFFFFU, arr) != HAL_OK) {
        return;
    }

    HAL_SuspendTick();

    /* Enter Stop1 — CPU halts here until the LPTIM1 compare-match IRQ fires */
    HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

    /* Stop1 exit: clocks stopped, must reconfigure before any peripheral access */
    SystemClock_Config();
    HAL_ResumeTick();

    HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);

    ReinitAdc();
}