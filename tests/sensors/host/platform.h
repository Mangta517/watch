#ifndef SENSOR_HOST_PLATFORM_H
#define SENSOR_HOST_PLATFORM_H
#include <stdint.h>
#define __MAIN_H
#define MPU6050_INT_Pin 4096U
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void HAL_Delay(uint32_t ms);
void Error_Handler(void);
#endif
