/* Preincluded only in the desktop harness; firmware uses the real main.h. */
#ifndef UI_HOST_PLATFORM_H
#define UI_HOST_PLATFORM_H
#include <stdint.h>
#define __MAIN_H
#define KEY_1_GPIO_Port ((void *)0)
#define KEY_1_Pin       32U
#define WAKE_GPIO_Port  ((void *)0)
#define WAKE_Pin        16U
typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;
typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
uint32_t __get_IPSR(void);
uint32_t HAL_GetTick(void);
#endif
