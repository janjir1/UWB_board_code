/*
 ******************************************************************************
 * @file    lsm6dsv_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements the self test procedure.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
 * - STEVAL_MKI109V3 + STEVAL-MKI196V1
 * - NUCLEO_F401RE + X_NUCLEO_IKS01A3
 * - DISCOVERY_SPC584B + STEVAL-MKI196V1
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */


/* Includes ------------------------------------------------------------------*/

#include "lsm6dsv_reg.h"
#include "lsm6dsv_platform.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "../Generic/my_print.h"

#include "lsm6dsv_ST_example.h"


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME      100

/* Self test limits. */
#define    MIN_ST_LIMIT_mg        50.0f
#define    MAX_ST_LIMIT_mg      1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

uint8_t tx_buffer[1000];

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Main Example --------------------------------------------------------------*/
bool SelfTest(void)
{
  lsm6dsv_all_sources_t all_sources;
  int16_t data_raw[3];
  stmdev_ctx_t dev_ctx;
  float_t val_st_off[3];
  float_t val_st_on[3];
  float_t test_val[3];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t i;
  uint8_t j;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;

  /* Init test platform */
  platform_init();

  //platform_delay(5000);

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6dsv_device_id_get(&dev_ctx, &whoamI);



  if (whoamI != LSM6DSV_ID)
    return false;

  /* Restore default configuration */
  lsm6dsv_sw_por(&dev_ctx);

  /* Enable Block Data Update */
  lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_60Hz);
  /* Set full scale */
  lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_4g);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
  } while (!all_sources.drdy_xl);

  /* Read dummy data and discard it */
  lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
    } while (!all_sources.drdy_xl);

    /* Read data and accumulate the mg value */
    lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsv_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsv_xl_self_test_set(&dev_ctx, LSM6DSV_XL_ST_NEGATIVE);
  //lsm6dsv_xl_self_test_set(&dev_ctx, LSM6DSV_XL_ST_POSITIVE);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
  } while (!all_sources.drdy_xl);

  /* Read dummy data and discard it */
  lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
    } while (!all_sources.drdy_xl);

    /* Read data and accumulate the mg value */
    lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsv_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsv_xl_self_test_set(&dev_ctx, LSM6DSV_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_OFF);
  /*
   * Gyroscope Self Test
   */
  /* Set Output Data Rate */
  lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_240Hz);
  /* Set full scale */
  lsm6dsv_gy_full_scale_set(&dev_ctx, LSM6DSV_2000dps);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
  } while (!all_sources.drdy_gy);

  /* Read dummy data and discard it */
  lsm6dsv_angular_rate_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
    } while (!all_sources.drdy_gy);
    /* Read data and accumulate the mg value */
    lsm6dsv_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsv_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsv_gy_self_test_set(&dev_ctx, LSM6DSV_GY_ST_POSITIVE);
  //lsm6dsv_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(100);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv_all_sources_get(&dev_ctx, &all_sources);
    } while (!all_sources.drdy_gy);

    /* Read data and accumulate the mg value */
    lsm6dsv_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsv_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mdps > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsv_gy_self_test_set(&dev_ctx, LSM6DSV_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_OFF);

  if (st_result == ST_PASS) {
    return true;
  }

  else {
    return false;
  }

}
