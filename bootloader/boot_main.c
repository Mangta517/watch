#include "main.h"
#include "boot_platform.h"
#include "app_boot_select.h"

/* PA5(KEY_1)空闲为高，复位后直接返回0不延时；按下(低)才每10ms轮询，
 * 连续按住满2000ms返回1，任何提前松开返回0。tick为无符号，跨回绕仍正确。 */
static uint8_t boot_recovery_requested(void)
{
    uint32_t start;

    if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) != GPIO_PIN_RESET)
    {
        return 0;
    }
    start = HAL_GetTick();
    for (;;)
    {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) != GPIO_PIN_RESET)
        {
            return 0;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= 2000)
        {
            return 1;
        }
    }
}

int main(void)
{
    uint32_t target;

    boot_platform_gpio_init();
    HAL_Init();
    __HAL_FLASH_DATA_CACHE_DISABLE();
    boot_platform_clock_init();
    target = app_boot_select_target(boot_recovery_requested());
    if (target != 0)
    {
        boot_platform_jump(target);
    }
    /* 保底向量缺失/损坏时不跳转未知代码，保留PA3供电等待SWD维护。 */
    for (;;)
    {
        __WFI();
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    NVIC_SystemReset();
    for (;;)
    {
    }
}
