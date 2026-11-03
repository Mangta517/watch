/* 引导入口主机测试最小HAL桩：仅满足boot_main.c引用的GPIO/时钟/延时符号。 */
#ifndef ENTRY_HOST_MAIN_H
#define ENTRY_HOST_MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum
{
    GPIO_PIN_RESET,
    GPIO_PIN_SET
} GPIO_PinState;

typedef struct
{
    uint32_t dummy;
} GPIO_TypeDef;

#define KEY_1_GPIO_Port ((GPIO_TypeDef *)0)
#define KEY_1_Pin       (1U << 5)
#define __HAL_FLASH_DATA_CACHE_DISABLE() ((void)0)
#define __WFI()                          entry_wfi()

void entry_wfi(void);
HAL_StatusTypeDef HAL_Init(void);
void HAL_IncTick(void);
void NVIC_SystemReset(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

#endif
