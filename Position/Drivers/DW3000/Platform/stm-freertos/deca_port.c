#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../../dwt_uwb_driver/deca_version.h"

#ifdef DW3000_DRIVER_VERSION /* == 0x040000 */
#include "deca_device_api.h"
#else
#include "../../dwt_uwb_driver/deca_interface.h"
#endif

#include "../dw3000_hw.h"
#include "../dw3000_spi.h"

/* -----------------------------------------------------------------------
 * Wakeup
 * --------------------------------------------------------------------- */

void wakeup_device_with_io(void)
{
    /* Used only if dwt_wakeup_ic() is called; easier to call directly. */
    dw3000_hw_wakeup();
}

/* -----------------------------------------------------------------------
 * Critical section / IRQ mutex
 *
 * Protects against the DW3000 EXTI IRQ firing mid-transaction.
 * Delegates to dw3000_hw.c so it is safe from both task and ISR context.
 * Does NOT use a FreeRTOS mutex — a mutex would deadlock if called from
 * an ISR, and the core may call these from interrupt context.
 * --------------------------------------------------------------------- */

decaIrqStatus_t decamutexon(void)
{
    bool s = dw3000_hw_interrupt_is_enabled();
    if (s) {
        dw3000_hw_interrupt_disable();
    }
    return (decaIrqStatus_t)s;
}

void decamutexoff(decaIrqStatus_t s)
{
    if (s) {
        dw3000_hw_interrupt_enable();
    }
}

/* -----------------------------------------------------------------------
 * Delays
 * --------------------------------------------------------------------- */

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

/*
 * Enable the ARM Cortex-M4 DWT cycle counter.
 * Call this once from your init task before dwt_initialise().
 * Safe to call multiple times (idempotent).
 */
void deca_usleep_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
 * Sub-millisecond busy-wait using the DWT cycle counter.
 * vTaskDelay() has 1 ms minimum resolution and cannot be used here.
 * The counter is self-initialised on first call as a safety net,
 * but call deca_usleep_init() explicitly in your startup code.
 */
void deca_usleep(unsigned long time_us)
{
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        deca_usleep_init();
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = time_us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks) {
        /* busy-wait */
    }
}

/* -----------------------------------------------------------------------
 * SPI function table and probe struct (driver-version-dependent)
 * --------------------------------------------------------------------- */

#ifdef DW3000_DRIVER_VERSION /* == 0x040000 */

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength,   uint8_t *readBuffer)
{
    return dw3000_spi_read(headerLength, headerBuffer,
                           readLength,   readBuffer);
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint16_t bodyLength,   const uint8_t *bodyBuffer)
{
    return dw3000_spi_write(headerLength, headerBuffer,
                            bodyLength,   bodyBuffer);
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                      uint16_t bodyLength,   const uint8_t *bodyBuffer,
                      uint8_t  crc8)
{
    return dw3000_spi_write_crc(headerLength, headerBuffer,
                                bodyLength,   bodyBuffer, crc8);
}

#elif DRIVER_VERSION_HEX >= 0x080202

static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi       = dw3000_spi_read,
    .writetospi        = dw3000_spi_write,
    .writetospiwithcrc = dw3000_spi_write_crc,
    .setslowrate       = dw3000_spi_speed_slow,
    .setfastrate       = dw3000_spi_speed_fast,
};

#if defined(DW3000_CHIP_DW3720)
extern const struct dwt_driver_s dw3720_driver;
#else
extern const struct dwt_driver_s dw3000_driver;
#endif

const struct dwt_driver_s *tmp_ptr[] = {
#if defined(DW3000_CHIP_DW3720)
    &dw3720_driver,
#else
    &dw3000_driver,
#endif
};

const struct dwt_probe_s dw3000_probe_interf = {
    .dw                    = NULL,
    .spi                   = (void *)&dw3000_spi_fct,
    .wakeup_device_with_io = dw3000_hw_wakeup,
    .driver_list           = (struct dwt_driver_s **)tmp_ptr,
    .dw_driver_num         = 1,
};

#elif DRIVER_VERSION_HEX >= 0x060007

static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi       = dw3000_spi_read,
    .writetospi        = dw3000_spi_write,
    .writetospiwithcrc = dw3000_spi_write_crc,
    .setslowrate       = dw3000_spi_speed_slow,
    .setfastrate       = dw3000_spi_speed_fast,
};

const struct dwt_probe_s dw3000_probe_interf = {
    .dw                    = NULL,
    .spi                   = (void *)&dw3000_spi_fct,
    .wakeup_device_with_io = dw3000_hw_wakeup,
};

#endif
