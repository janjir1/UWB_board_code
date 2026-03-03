/*
 * dw3000_hw.c — STM32L4 + FreeRTOS HW platform port for DW3000 decadriver
 *
 * Implements GPIO control: RESET, WAKEUP, and IRQ pins.
 *
 * Fill in the pin definitions below to match your CubeMX project.
 * CubeMX generates _GPIO_Port / _Pin constants in main.h automatically
 * when you label pins in the pinout view.
 *
 * Chip-select CS pin is only needed for the CS-wakeup fallback
 * (when DW3000_HAS_WAKEUP_PIN is not defined).
 */

#include "main.h"
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../../dwt_uwb_driver/deca_device_api.h"
#include "../dw3000_hw.h"
#include "../dw3000_spi.h"

/* -----------------------------------------------------------------------
 * Pin definitions — adapt to your CubeMX pin labels
 *
 * Example if you label pins in CubeMX as:
 *   DW3000_RESET  → generates DW3000_RESET_GPIO_Port / DW3000_RESET_Pin
 *   DW3000_IRQ    → generates DW3000_IRQ_GPIO_Port   / DW3000_IRQ_Pin
 *   DW3000_WAKEUP → generates DW3000_WAKEUP_GPIO_Port / DW3000_WAKEUP_Pin
 *   DW3000_CS     → generates DW3000_CS_GPIO_Port    / DW3000_CS_Pin
 *
 * DW3000_IRQ_EXTI_IRQn: the IRQn for the EXTI line of your IRQ pin.
 *   Pin 0     → EXTI0_IRQn
 *   Pin 1     → EXTI1_IRQn
 *   Pin 2     → EXTI2_IRQn
 *   Pin 3     → EXTI3_IRQn
 *   Pin 4     → EXTI4_IRQn
 *   Pins 5-9  → EXTI9_5_IRQn
 *   Pins 10-15→ EXTI15_10_IRQn
 *
 * Uncomment DW3000_HAS_WAKEUP_PIN if you have a dedicated WAKEUP pin wired.
 * If not, the CS pin is toggled for wakeup instead.
 * --------------------------------------------------------------------- */

#define DW3000_RESET_GPIO_Port   DWM_RSTN_GPIO_Port
#define DW3000_RESET_Pin         DWM_RSTN_Pin

#define DW3000_IRQ_GPIO_Port     SYS_WKUP1_DWM_GPIO_Port /* e.g. GPIOB */
#define DW3000_IRQ_Pin           SYS_WKUP1_DWM_Pin           /* e.g. GPIO_PIN_5 */
#define DW3000_IRQ_EXTI_IRQn     EXTI1_IRQn   /* e.g. EXTI9_5_IRQn */

/* #define DW3000_HAS_WAKEUP_PIN */
#ifdef DW3000_HAS_WAKEUP_PIN
#define DW3000_WAKEUP_GPIO_Port  /* e.g. GPIOC */
#define DW3000_WAKEUP_Pin        /* e.g. GPIO_PIN_3 */
#else
/* CS-based wakeup fallback — only needed if no WAKEUP pin */
#define DW3000_CS_GPIO_Port      ChipSelect_GPIO_Port/* e.g. GPIOA */
#define DW3000_CS_Pin            ChipSelect_Pin            /* e.g. GPIO_PIN_4 */
#endif

/* -----------------------------------------------------------------------
 * Logging — maps to printf by default; replace with your logger if needed
 * --------------------------------------------------------------------- */
#include <stdio.h>
#define LOG_INF(fmt, ...) printf("[DW3000 INF] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) printf("[DW3000 ERR] " fmt "\r\n", ##__VA_ARGS__)

/* -----------------------------------------------------------------------
 * Internal sub-millisecond busy-wait using DWT cycle counter.
 * Used only within this file for wakeup pulse timing.
 * deca_usleep_init() in deca_port.c enables the counter at startup —
 * if called here before that, this function self-enables it.
 * --------------------------------------------------------------------- */
static void hw_delay_us(uint32_t us)
{
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT       = 0U;
        DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks) { /* busy-wait */ }
}

/* -----------------------------------------------------------------------
 * dw3000_hw_init
 *
 * Configures RESET as input (hi-Z, DW3000 drives it open-drain),
 * waits up to 1 s for the device to come out of reset,
 * configures WAKEUP as output-low, then inits SPI.
 * --------------------------------------------------------------------- */
int dw3000_hw_init(void)
{
    LOG_INF("HW init");

    /* RESET — configure as input, no pull (DW3000 open-drain) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = DW3000_RESET_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DW3000_RESET_GPIO_Port, &gpio);

    /* Wait for RESET to go high (device ready), timeout 1 s */
    uint32_t tick = HAL_GetTick();
    while (HAL_GPIO_ReadPin(DW3000_RESET_GPIO_Port, DW3000_RESET_Pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - tick) > 1000U) {
            LOG_ERR("did not come out of reset");
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

#ifdef DW3000_HAS_WAKEUP_PIN
    /* WAKEUP — output, initially low */
    gpio.Pin   = DW3000_WAKEUP_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW3000_WAKEUP_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(DW3000_WAKEUP_GPIO_Port, DW3000_WAKEUP_Pin, GPIO_PIN_RESET);
#endif

    return dw3000_spi_init();
}

/* -----------------------------------------------------------------------
 * dw3000_hw_fini — release all GPIOs and SPI
 * --------------------------------------------------------------------- */
void dw3000_hw_fini(void)
{
    LOG_INF("HW fini");

    /* Disable and de-init IRQ EXTI */
    HAL_NVIC_DisableIRQ(DW3000_IRQ_EXTI_IRQn);
    HAL_GPIO_DeInit(DW3000_IRQ_GPIO_Port, DW3000_IRQ_Pin);

    HAL_GPIO_DeInit(DW3000_RESET_GPIO_Port, DW3000_RESET_Pin);

#ifdef DW3000_HAS_WAKEUP_PIN
    HAL_GPIO_DeInit(DW3000_WAKEUP_GPIO_Port, DW3000_WAKEUP_Pin);
#endif

    dw3000_spi_fini();
}

/* -----------------------------------------------------------------------
 * dw3000_hw_reset — pulse RESET low, then release to hi-Z
 * --------------------------------------------------------------------- */
void dw3000_hw_reset(void)
{
    LOG_INF("HW reset");

    GPIO_InitTypeDef gpio = {0};

    /* Drive RESET low (assert reset) */
    gpio.Pin   = DW3000_RESET_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW3000_RESET_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(DW3000_RESET_GPIO_Port, DW3000_RESET_Pin, GPIO_PIN_RESET);

    vTaskDelay(pdMS_TO_TICKS(1)); /* API guide says 10 ns min; 1 ms is safe */

    /* Release RESET — reconfigure as input (hi-Z) so DW3000 drives it */
    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DW3000_RESET_GPIO_Port, &gpio);

    vTaskDelay(pdMS_TO_TICKS(2)); /* Allow DW3000 to come out of reset */
}

/* -----------------------------------------------------------------------
 * IRQ — EXTI on rising edge, internal pull-down
 *
 * The ISR loops while the pin stays high to drain all pending events
 * before re-enabling the EXTI, matching the original nRF behaviour.
 *
 * If you configured the EXTI in CubeMX (recommended), CubeMX generates
 * MX_GPIO_Init() which sets up the EXTI line and NVIC priority.
 * In that case, remove the manual GPIO_Init block in dw3000_hw_init_interrupt()
 * and only keep the HAL_NVIC_EnableIRQ() call.
 * --------------------------------------------------------------------- */

static void dw3000_isr(void)
{
    while (HAL_GPIO_ReadPin(DW3000_IRQ_GPIO_Port, DW3000_IRQ_Pin) == GPIO_PIN_SET) {
        dwt_isr();
    }
}

/*
 * Override the HAL EXTI callback.
 * If other peripherals also use EXTI, guard with the pin check shown below
 * and add your other EXTI handlers inside the same callback.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DW3000_IRQ_Pin) {
        dw3000_isr();
    }
}

int dw3000_hw_init_interrupt(void)
{
    /*
     * Configure IRQ pin: rising-edge EXTI, pull-down.
     * Skip this block if CubeMX already configures this pin in MX_GPIO_Init().
     */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = DW3000_IRQ_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DW3000_IRQ_GPIO_Port, &gpio);

    /*
     * IRQ priority MUST be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
     * (typically numeric value 5) so FreeRTOS ISR-safe APIs can be called
     * from the EXTI handler if you ever use task notifications from it.
     */
    HAL_NVIC_SetPriority(DW3000_IRQ_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DW3000_IRQ_EXTI_IRQn);

    return 0;
}

/* -----------------------------------------------------------------------
 * Interrupt mask/unmask — manipulates the EXTI IMR1 bit directly.
 *
 * Using EXTI->IMR1 (not NVIC) so that only the DW3000 EXTI line is
 * masked, not the entire IRQ vector (which may be shared on pins 5-9
 * or 10-15). Safe to call from both task and ISR context.
 * --------------------------------------------------------------------- */

void dw3000_hw_interrupt_enable(void)
{
    EXTI->IMR1 |= DW3000_IRQ_Pin;
}

void dw3000_hw_interrupt_disable(void)
{
    EXTI->IMR1 &= ~((uint32_t)DW3000_IRQ_Pin);
}

bool dw3000_hw_interrupt_is_enabled(void)
{
    return (EXTI->IMR1 & DW3000_IRQ_Pin) != 0U;
}

/* -----------------------------------------------------------------------
 * Wakeup — toggle WAKEUP pin (500 µs high pulse) or CS pin as fallback
 * --------------------------------------------------------------------- */

void dw3000_hw_wakeup(void)
{
#ifdef DW3000_HAS_WAKEUP_PIN
    LOG_INF("WAKEUP PIN");
    HAL_GPIO_WritePin(DW3000_WAKEUP_GPIO_Port, DW3000_WAKEUP_Pin, GPIO_PIN_SET);
    hw_delay_us(500U);
    HAL_GPIO_WritePin(DW3000_WAKEUP_GPIO_Port, DW3000_WAKEUP_Pin, GPIO_PIN_RESET);
#else
    LOG_INF("WAKEUP CS");
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_RESET);
    hw_delay_us(500U);
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_SET);
#endif
    vTaskDelay(pdMS_TO_TICKS(1));
}

void dw3000_hw_wakeup_pin_low(void)
{
#ifdef DW3000_HAS_WAKEUP_PIN
    HAL_GPIO_WritePin(DW3000_WAKEUP_GPIO_Port, DW3000_WAKEUP_Pin, GPIO_PIN_RESET);
#endif
}
