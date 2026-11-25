#ifndef SENSOR_HOST_PLATFORM_H
#define SENSOR_HOST_PLATFORM_H
#include <stdint.h>
#define __MAIN_H
#define MPU6050_INT_Pin 4096U
typedef struct
{
    uint32_t IMR, PR;
} sensor_host_exti_struct;
extern sensor_host_exti_struct sensor_host_exti;
#define EXTI (&sensor_host_exti)
#define __HAL_GPIO_EXTI_CLEAR_IT(pin) (EXTI->PR &= ~(uint32_t)(pin))
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
void Error_Handler(void);
#endif
