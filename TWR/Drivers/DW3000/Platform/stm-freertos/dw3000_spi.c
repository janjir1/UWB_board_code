/*
 * dw3000_spi.c — STM32L4 + FreeRTOS SPI platform port for DW3000 decadriver
 *
 * Uses STM32 HAL SPI in blocking (polling) mode.
 * CS is managed manually — hardware NSS is NOT used.
 *
 * The SPI peripheral is assumed to be already initialised by CubeMX
 * (MX_SPIx_Init in spi.c). This file only manages the CS pin and
 * handles transfers.
 *
 * CubeMX SPI settings required:
 *   Mode:       Full-Duplex Master
 *   CPOL/CPHA:  Low / 1-Edge  (Mode 0)
 *   Data size:  8-bit, MSB first
 *   NSS:        Software (disabled — we drive CS manually)
 *
 * Prescaler reference — set SLOW/FAST below to match your PCLK:
 *   80 MHz:  /32 = 2.5 MHz (slow),  /4 = 20 MHz (fast)
 *   48 MHz:  /16 = 3.0 MHz (slow),  /2 = 24 MHz (fast)
 *   16 MHz:  /8  = 2.0 MHz (slow),  /2 =  8 MHz (fast)
 *
 * DW3000 requires ≤3 MHz until dwt_initialise() completes, max 38 MHz after.
 */

#include "main.h"
#include "stm32l4xx_hal.h"

#include "../../dwt_uwb_driver/deca_device_api.h"
#include "../dw3000_spi.h"

/* -----------------------------------------------------------------------
 * Configuration — adapt to your CubeMX project
 * --------------------------------------------------------------------- */

/*
 * SPI handle: set to the CubeMX-generated variable name.
 * Declared in spi.c/main.c; extern'd here via the generated spi.h/main.h.
 * Example: hspi1 if you enabled SPI1 in CubeMX.
 */
#define DW3000_SPI_HANDLE       hspi1
extern SPI_HandleTypeDef        DW3000_SPI_HANDLE;

/*
 * CS pin — must match the defines in dw3000_hw.c.
 * Tip: put both in a shared dw3000_config.h to avoid duplication.
 */
#define DW3000_CS_GPIO_Port     ChipSelect_GPIO_Port/* e.g. GPIOA */
#define DW3000_CS_Pin           ChipSelect_Pin            /* e.g. GPIO_PIN_4 */

/* SPI baud rate prescaler — see table above */
#define DW3000_SPI_PRESCALER_SLOW   SPI_BAUDRATEPRESCALER_32
#define DW3000_SPI_PRESCALER_FAST   SPI_BAUDRATEPRESCALER_4

/* HAL transfer timeout in ms — increase if you see HAL_TIMEOUT errors */
#define DW3000_SPI_TIMEOUT_MS       100U

/* -----------------------------------------------------------------------
 * Logging
 * --------------------------------------------------------------------- */
#include <stdio.h>
#define LOG_INF(fmt, ...) printf("[DW3000 INF] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) printf("[DW3000 ERR] " fmt "\r\n", ##__VA_ARGS__)

/* -----------------------------------------------------------------------
 * SPI trace (optional) — define DW3000_SPI_TRACE to enable.
 * Implement dw3000_spi_trace_in() in dw3000_spi_trace.c.
 * --------------------------------------------------------------------- */
#if defined(DW3000_SPI_TRACE)
extern void dw3000_spi_trace_in(bool rw, const uint8_t *headerBuffer,
                                 uint16_t headerLength,
                                 const uint8_t *bodyBuffer,
                                 uint16_t bodyLength);
#endif

/* -----------------------------------------------------------------------
 * dw3000_spi_init
 *
 * The SPI peripheral itself is already running (CubeMX MX_SPIx_Init).
 * We only need to bring the CS pin to a safe known-high state.
 * Set CS high BEFORE configuring as output to prevent a glitch.
 * --------------------------------------------------------------------- */
int dw3000_spi_init(void)
{
    LOG_INF("SPI init");

    /* Drive CS high before switching pin direction — prevents bus glitch */
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = DW3000_CS_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW3000_CS_GPIO_Port, &gpio);

    /* Write again after init to be certain */
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_SET);

    return 0;
}

void dw3000_spi_fini(void)
{
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_DeInit(DW3000_CS_GPIO_Port, DW3000_CS_Pin);
}

/* -----------------------------------------------------------------------
 * Speed control
 *
 * The decadriver core calls setslowrate() before dwt_initialise() and
 * setfastrate() once the device is confirmed alive.
 *
 * STM32 ref manual: BR[2:0] bits must only be changed while SPE=0.
 * We wait for the bus to go idle, disable SPE, change BR, re-enable SPE,
 * then update the HAL Init struct so any future HAL_SPI_Init() call is
 * consistent.
 * --------------------------------------------------------------------- */
static void spi_set_prescaler(uint32_t prescaler)
{
    /* Wait for any ongoing transfer to complete */
    while (DW3000_SPI_HANDLE.Instance->SR & SPI_SR_BSY) { /* spin */ }

    CLEAR_BIT(DW3000_SPI_HANDLE.Instance->CR1, SPI_CR1_SPE);
    MODIFY_REG(DW3000_SPI_HANDLE.Instance->CR1, SPI_CR1_BR_Msk, prescaler);
    SET_BIT(DW3000_SPI_HANDLE.Instance->CR1, SPI_CR1_SPE);

    /* Keep HAL state struct in sync */
    DW3000_SPI_HANDLE.Init.BaudRatePrescaler = prescaler;
}

void dw3000_spi_speed_slow(void)
{
    spi_set_prescaler(DW3000_SPI_PRESCALER_SLOW);
}

void dw3000_spi_speed_fast(void)
{
    spi_set_prescaler(DW3000_SPI_PRESCALER_FAST);
}

/* -----------------------------------------------------------------------
 * Write: TX header then TX body with CS held low across both phases.
 *
 * decamutexon/off disables the DW3000 EXTI IRQ for the entire transaction,
 * preventing dwt_isr() from starting a nested SPI transfer mid-operation.
 * This matches the original nRF design and is sufficient for single-task
 * DW3000 access. If multiple tasks share the driver, add a FreeRTOS mutex
 * around the decamutexon..decamutexoff region — but do NOT take a mutex
 * from ISR context (i.e. not inside dwt_isr()).
 * --------------------------------------------------------------------- */
int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t *headerBuffer,
                          uint16_t bodyLength,   const uint8_t *bodyBuffer)
{
    decaIrqStatus_t stat = decamutexon();

#if defined(DW3000_SPI_TRACE)
    dw3000_spi_trace_in(false, headerBuffer, headerLength,
                        bodyBuffer, bodyLength);
#endif

    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&DW3000_SPI_HANDLE,
                                              (uint8_t *)headerBuffer,
                                              headerLength,
                                              DW3000_SPI_TIMEOUT_MS);
    if (ret != HAL_OK) {
        LOG_ERR("SPI write header error (%d)", ret);
        goto exit;
    }

    if (bodyLength > 0U) {
        ret = HAL_SPI_Transmit(&DW3000_SPI_HANDLE,
                               (uint8_t *)bodyBuffer,
                               bodyLength,
                               DW3000_SPI_TIMEOUT_MS);
        if (ret != HAL_OK) {
            LOG_ERR("SPI write body error (%d)", ret);
        }
    }

exit:
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_SET);
    decamutexoff(stat);
    return (ret == HAL_OK) ? DWT_SUCCESS : DWT_ERROR;
}

/* -----------------------------------------------------------------------
 * Read: TX header then RX body with CS held low across both phases.
 *
 * HAL_SPI_Receive clocks out 0xFF dummy bytes on MOSI while receiving —
 * the DW3000 ignores MOSI during the data phase, so this is correct.
 * --------------------------------------------------------------------- */
int32_t dw3000_spi_read(uint16_t headerLength, uint8_t *headerBuffer,
                         uint16_t readLength,   uint8_t *readBuffer)
{
    decaIrqStatus_t stat = decamutexon();

    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&DW3000_SPI_HANDLE,
                                              headerBuffer,
                                              headerLength,
                                              DW3000_SPI_TIMEOUT_MS);
    if (ret != HAL_OK) {
        LOG_ERR("SPI read header error (%d)", ret);
        goto exit;
    }

    ret = HAL_SPI_Receive(&DW3000_SPI_HANDLE,
                           readBuffer,
                           readLength,
                           DW3000_SPI_TIMEOUT_MS);
    if (ret != HAL_OK) {
        LOG_ERR("SPI read body error (%d)", ret);
    }

#if defined(DW3000_SPI_TRACE)
    dw3000_spi_trace_in(true, headerBuffer, headerLength,
                        readBuffer, readLength);
#endif

exit:
    HAL_GPIO_WritePin(DW3000_CS_GPIO_Port, DW3000_CS_Pin, GPIO_PIN_SET);
    decamutexoff(stat);
    return (ret == HAL_OK) ? DWT_SUCCESS : DWT_ERROR;
}

/* -----------------------------------------------------------------------
 * CRC write — not implemented (matches original behaviour)
 * --------------------------------------------------------------------- */
int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t *headerBuffer,
                               uint16_t bodyLength,   const uint8_t *bodyBuffer,
                               uint8_t  crc8)
{
    (void)headerLength; (void)headerBuffer;
    (void)bodyLength;   (void)bodyBuffer;
    (void)crc8;
    LOG_ERR("WRITE WITH CRC NOT IMPLEMENTED!");
    return DWT_ERROR;
}
