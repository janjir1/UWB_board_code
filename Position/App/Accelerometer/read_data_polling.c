/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 +
 * - NUCLEO_F401RE +
 * - DISCOVERY_SPC584B +
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsv_reg.h"
#include "lsm6dsv_platform.h"
#include "../Generic/my_print.h"

#include "lsm6dsv_ST_example.h"



/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv_filt_settling_mask_t filt_settling_mask;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Main Example --------------------------------------------------------------*/
void lsm6dsv_read_data_polling(void)
{
  lsm6dsv_all_sources_t all_sources;
  stmdev_ctx_t dev_ctx;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;

  /* Init test platform */
  platform_init();

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_7Hz5);
  lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_15Hz);
  /* Set full scale */
  lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_2g);
  lsm6dsv_gy_full_scale_set(&dev_ctx, LSM6DSV_2000dps);
  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV_GY_ULTRA_LIGHT);
  lsm6dsv_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV_XL_STRONG);

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read output only if new xl value is available */
    lsm6dsv_all_sources_get(&dev_ctx, &all_sources);

    if (all_sources.drdy_xl) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dsv_from_fs2_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dsv_from_fs2_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dsv_from_fs2_to_mg(data_raw_acceleration[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (all_sources.drdy_gy) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6dsv_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
        lsm6dsv_from_fs2000_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] =
        lsm6dsv_from_fs2000_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] =
        lsm6dsv_from_fs2000_to_mdps(data_raw_angular_rate[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    
     if (all_sources.drdy_temp) {
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6dsv_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6dsv_from_lsb_to_celsius(
                           data_raw_temperature);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "Temperature [degC]:%6.2f\r\n", temperature_degC);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    } 

    platform_delay(100);

  }
}
