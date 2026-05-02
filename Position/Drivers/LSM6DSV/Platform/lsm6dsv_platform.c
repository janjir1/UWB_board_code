#include "lsm6dsv_platform.h"
#include "../lsm6dsv/lsm6dsv_reg.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

#ifdef UWB_BOARD_V1_1


static inline void cs_low (lsm6dsv_spi_handle_t *h)
{
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(lsm6dsv_spi_handle_t *h)
{
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin, GPIO_PIN_SET);
}

#endif /* LSM6DSV_USE_SPI */

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                       uint16_t len)
{
#ifdef UWB_BOARD_V1_1

    lsm6dsv_spi_handle_t *h = (lsm6dsv_spi_handle_t *)handle;

    /* LSM6DSV SPI frame: bit7 = 0 → write, bits[6:0] = register address */
    uint8_t tx_reg = reg & 0x7Fu;

    cs_low(h);
    HAL_SPI_Transmit(h->hspi, &tx_reg, 1,    1000);
    HAL_SPI_Transmit(h->hspi, (uint8_t *)bufp, len, 1000);
    cs_high(h);

#else  /* I2C */

    HAL_I2C_Mem_Write(handle, LSM6DSV_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, 1000);

#endif /* LSM6DSV_USE_SPI */

    return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                      uint16_t len)
{
#ifdef UWB_BOARD_V1_1

    lsm6dsv_spi_handle_t *h = (lsm6dsv_spi_handle_t *)handle;

    /* LSM6DSV SPI frame: bit7 = 1 → read, bits[6:0] = register address */
    uint8_t tx_reg = reg | 0x80u;

    cs_low(h);
    HAL_SPI_Transmit(h->hspi, &tx_reg, 1,   1000);
    HAL_SPI_Receive (h->hspi, bufp,    len, 1000);
    cs_high(h);

#else  /* I2C */

    HAL_I2C_Mem_Read(handle, LSM6DSV_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

#endif /* LSM6DSV_USE_SPI */

    return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if !defined(UWB_BOARD_V1_1)
  HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
#endif

}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
  osDelay(ms);

}

/*
 * @brief  platform specific initialization (platform dependent)
 */
void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}