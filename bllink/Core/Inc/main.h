/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYS_WKUP2_ACC_Pin GPIO_PIN_13
#define SYS_WKUP2_ACC_GPIO_Port GPIOC
#define DWM_WAKEUP_Pin GPIO_PIN_0
#define DWM_WAKEUP_GPIO_Port GPIOH
#define DWM_RSTN_Pin GPIO_PIN_1
#define DWM_RSTN_GPIO_Port GPIOH
#define SYS_WKUP1_DWM_Pin GPIO_PIN_0
#define SYS_WKUP1_DWM_GPIO_Port GPIOA
#define ChipSelect_Pin GPIO_PIN_1
#define ChipSelect_GPIO_Port GPIOA
#define USART2_TX_Master_Pin GPIO_PIN_2
#define USART2_TX_Master_GPIO_Port GPIOA
#define USART_RX_Master_Pin GPIO_PIN_3
#define USART_RX_Master_GPIO_Port GPIOA
#define USART2_CK_Master_Pin GPIO_PIN_4
#define USART2_CK_Master_GPIO_Port GPIOA
#define Vbat_sense_Pin GPIO_PIN_5
#define Vbat_sense_GPIO_Port GPIOA
#define USB_CC2_A_Sense_Pin GPIO_PIN_6
#define USB_CC2_A_Sense_GPIO_Port GPIOA
#define USB_CC1_A_Sense_Pin GPIO_PIN_7
#define USB_CC1_A_Sense_GPIO_Port GPIOA
#define PB0_Pin GPIO_PIN_0
#define PB0_GPIO_Port GPIOB
#define USART1_TX_DEBUG_Pin GPIO_PIN_9
#define USART1_TX_DEBUG_GPIO_Port GPIOA
#define USART1_RX_Debug_Pin GPIO_PIN_10
#define USART1_RX_Debug_GPIO_Port GPIOA
#define Debug_SWDIO_Pin GPIO_PIN_13
#define Debug_SWDIO_GPIO_Port GPIOA
#define Debug_SWCLK_Pin GPIO_PIN_14
#define Debug_SWCLK_GPIO_Port GPIOA
#define SPI1_SCK_DWM_Pin GPIO_PIN_3
#define SPI1_SCK_DWM_GPIO_Port GPIOB
#define SPI1_MISO_DWM_Pin GPIO_PIN_4
#define SPI1_MISO_DWM_GPIO_Port GPIOB
#define SPI1_MOSI_DWM_Pin GPIO_PIN_5
#define SPI1_MOSI_DWM_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOB
#define LED_W_Pin GPIO_PIN_7
#define LED_W_GPIO_Port GPIOB
#define BOOT_button_Pin GPIO_PIN_3
#define BOOT_button_GPIO_Port GPIOH
#define I2C1_SCL_ACC_Pin GPIO_PIN_8
#define I2C1_SCL_ACC_GPIO_Port GPIOB
#define I2C1_SDA_ACC_Pin GPIO_PIN_9
#define I2C1_SDA_ACC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
