/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
/* PB15（HEART_LED，旧名 BLE_LED）控制心率 LED_EN；推挽输出，初始化低电平关闭外部 LED。 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, POWER_EN_Pin|EEPROM_SDA_Pin|EEPROM_SCL_Pin|TP_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WDG_EN_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SENSOR_SDA_Pin|SENSOR_SCL_Pin|HEART_LED_Pin
                          |TP_SDA_Pin|TP_SCL_Pin|LCD_CS_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : POWER_EN_Pin */
  GPIO_InitStruct.Pin = POWER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WAKE_Pin */
  GPIO_InitStruct.Pin = WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_1_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_1_GPIO_Port, &GPIO_InitStruct);

  /* WDI在正式启用前保持高阻，避免非A版TPS3823在上电复位期间收到跳变而锁存。 */
  GPIO_InitStruct.Pin = WDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WDI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WDG_EN_Pin SENSOR_SDA_Pin SENSOR_SCL_Pin
                           TP_SDA_Pin TP_SCL_Pin */
  GPIO_InitStruct.Pin = WDG_EN_Pin|SENSOR_SDA_Pin|SENSOR_SCL_Pin
                          |TP_SDA_Pin|TP_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU6050_INT_Pin */
  GPIO_InitStruct.Pin = MPU6050_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HEART_LED_Pin LCD_CS_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = HEART_LED_Pin|LCD_CS_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_EN_Pin */
  GPIO_InitStruct.Pin = BLE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EEPROM_SDA_Pin EEPROM_SCL_Pin */
  GPIO_InitStruct.Pin = EEPROM_SDA_Pin|EEPROM_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_RST_Pin */
  GPIO_InitStruct.Pin = TP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
