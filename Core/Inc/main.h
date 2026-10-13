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
#include "stm32f4xx_hal.h"

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
/* HEART_LED_Pin/HEART_LED_GPIO_Port：PB15 接心率 LED_EN（Core H5-5 → Back H2-5），旧名为 BLE_LED。
 * 蓝牙 KT_LED 接原理图 KT6328A1 第 2 脚及指示灯，与 PB15 不相连。 */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_EN_Pin GPIO_PIN_3
#define POWER_EN_GPIO_Port GPIOA
#define WAKE_Pin GPIO_PIN_4
#define WAKE_GPIO_Port GPIOA
#define WAKE_EXTI_IRQn EXTI4_IRQn
#define KEY_1_Pin GPIO_PIN_5
#define KEY_1_GPIO_Port GPIOA
#define LCD_BLK_Pin GPIO_PIN_0
#define LCD_BLK_GPIO_Port GPIOB
#define MPU6050_INT_Pin GPIO_PIN_12
#define MPU6050_INT_GPIO_Port GPIOB
#define MPU6050_INT_EXTI_IRQn EXTI15_10_IRQn
#define SENSOR_SDA_Pin GPIO_PIN_13
#define SENSOR_SDA_GPIO_Port GPIOB
#define SENSOR_SCL_Pin GPIO_PIN_14
#define SENSOR_SCL_GPIO_Port GPIOB
#define HEART_LED_Pin GPIO_PIN_15
#define HEART_LED_GPIO_Port GPIOB
#define BLE_EN_Pin GPIO_PIN_8
#define BLE_EN_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_9
#define BLE_TX_GPIO_Port GPIOA
#define BLE_RX_Pin GPIO_PIN_10
#define BLE_RX_GPIO_Port GPIOA
#define EEPROM_SDA_Pin GPIO_PIN_11
#define EEPROM_SDA_GPIO_Port GPIOA
#define EEPROM_SCL_Pin GPIO_PIN_12
#define EEPROM_SCL_GPIO_Port GPIOA
#define TP_RST_Pin GPIO_PIN_15
#define TP_RST_GPIO_Port GPIOA
#define LCD_CLK_Pin GPIO_PIN_3
#define LCD_CLK_GPIO_Port GPIOB
#define TP_SDA_Pin GPIO_PIN_4
#define TP_SDA_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_5
#define LCD_SDA_GPIO_Port GPIOB
#define TP_SCL_Pin GPIO_PIN_6
#define TP_SCL_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_9
#define LCD_DC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
