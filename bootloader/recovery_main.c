#include "main.h"
#include "usart.h"
#include "kt6368a.h"
#include "app_bootloader.h"
#include "boot_platform.h"

static DMA_HandleTypeDef hdma_usart1_rx;

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

void int_bootloader_jump_to_app(void)
{
    if (int_bootloader_app_is_valid() == 0)
    {
        return;
    }
    int_bootloader_stop();
    (void)HAL_UART_DeInit(&huart1);
    (void)HAL_DMA_DeInit(&hdma_usart1_rx);
    boot_platform_jump(APP_START_ADDR);
}

int main(void)
{
    boot_platform_gpio_init();
    SCB->VTOR = RECOVERY_START_ADDR;
    __DSB();
    __ISB();
    HAL_Init();
    __HAL_FLASH_DATA_CACHE_DISABLE();
    boot_platform_clock_init();
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
