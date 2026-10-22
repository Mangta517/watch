#include "main.h"
#include "usart.h"
#include "kt6368a.h"
#include "app_bootloader.h"

static DMA_HandleTypeDef hdma_usart1_rx;

static void boot_watchdog_start(void)
{
    /* 仅升级分支使用；256分频、4095重装，LSI容差下约22～62秒。 */
    WRITE_REG(IWDG->KR, 0xCCCCU);
    WRITE_REG(IWDG->KR, 0x5555U);
    WRITE_REG(IWDG->PR, 6U);
    WRITE_REG(IWDG->RLR, 0x0FFFU);
    while (IWDG->SR != 0U)
    {
        /* 寄存器无法同步时不喂狗，由IWDG复位。 */
    }
    WRITE_REG(IWDG->KR, 0xAAAAU);
}

static void boot_gpio_init(void)
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

static void boot_clock_init(void)
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

static void boot_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

/* 修改MSP后直接BX，不经过C函数栈帧退出；APP从Reset_Handler开始。 */
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

void int_bootloader_jump_to_app(void)
{
    uint32_t stack;
    uint32_t reset;
    uint32_t i;
    if (int_bootloader_app_is_valid() == 0U)
    {
        return;
    }
    stack = *(volatile uint32_t *)APP_START_ADDR;
    reset = *(volatile uint32_t *)(APP_START_ADDR + 4U);
    int_bootloader_stop();
    (void)HAL_UART_DeInit(&huart1);
    (void)HAL_DMA_DeInit(&hdma_usart1_rx);
    (void)HAL_RCC_DeInit();
    /* 不调用HAL_DeInit：AHB复位会释放PA3，导致TPS电源保持失效。 */
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (i = 0; i < 8U; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();
    SCB->VTOR = APP_START_ADDR;
    __DSB();
    __ISB();
    boot_jump(stack, reset);
}

int main(void)
{
    boot_gpio_init(); /* 最先保持PA3供电。 */
    HAL_Init();
    __HAL_FLASH_DATA_CACHE_DISABLE(); /* 回读必须访问Flash实际内容。 */
    if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) != GPIO_PIN_RESET && int_bootloader_app_is_valid() != 0U)
    {
        /* 普通启动不初始化蓝牙，APP继续使用自己的启动顺序。 */
        __disable_irq();
        SCB->VTOR = APP_START_ADDR;
        __DSB();
        __ISB();
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
        boot_jump(*(volatile uint32_t *)APP_START_ADDR, *(volatile uint32_t *)(APP_START_ADDR + 4U));
    }
    boot_watchdog_start();
    boot_clock_init();
    MX_USART1_UART_Init();
    boot_dma_init();
    if (kt6368a_init() != HAL_OK)
    {
        Error_Handler();
    }
    app_bootloader_init();
    for (;;)
    {
        app_bootloader_work();
        /* 接收超时由状态机处理；IWDG覆盖主循环停滞。 */
        WRITE_REG(IWDG->KR, 0xAAAAU);
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void Error_Handler(void)
{
    NVIC_SystemReset();
    for (;;)
    {
    }
}
