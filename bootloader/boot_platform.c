#include "main.h"
#include "boot_platform.h"

void boot_platform_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET);
    gpio.Pin = POWER_EN_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(POWER_EN_GPIO_Port, &gpio);

    /* 升级不运行手表任务，屏蔽外部看门狗RST，WDI保持高阻。 */
    HAL_GPIO_WritePin(WDG_EN_GPIO_Port, WDG_EN_Pin, GPIO_PIN_SET);
    gpio.Pin = WDG_EN_Pin;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WDG_EN_GPIO_Port, &gpio);
    gpio.Pin = WDI_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);

    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);
    gpio.Pin = BLE_EN_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(BLE_EN_GPIO_Port, &gpio);
    gpio.Pin = KEY_1_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_1_GPIO_Port, &gpio);
}

void boot_platform_clock_init(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    oscillator.PLL.PLLM = 8;
    oscillator.PLL.PLLN = 100;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV2;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 修改MSP后直接BX，不经过C函数栈帧退出；目标从Reset_Handler开始。 */
__asm void boot_jump(uint32_t stack, uint32_t reset)
{
    MSR MSP, r0
    MOVS r0, #0
    MSR CONTROL, r0
    MSR BASEPRI, r0
    ISB
    CPSIE I
    BX r1
}

void boot_platform_jump(uint32_t address)
{
    uint32_t stack = *(volatile uint32_t *)address;
    uint32_t reset = *(volatile uint32_t *)(address + 4);
    uint32_t i;

    /* 保留PA3供电，不调用会复位GPIO的HAL_DeInit。 */
    (void)HAL_RCC_DeInit();
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (i = 0; i < 8; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();
    SCB->VTOR = address;
    __DSB();
    __ISB();
    boot_jump(stack, reset);
}
