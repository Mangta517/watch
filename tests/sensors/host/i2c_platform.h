#ifndef SENSOR_HOST_I2C_PLATFORM_H
#define SENSOR_HOST_I2C_PLATFORM_H

#include <stdint.h>

#define GPIO_MODE_OUTPUT_OD          2U
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)

void host_i2c_nop(void);
#define __NOP() host_i2c_nop()

#endif
