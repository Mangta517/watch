#include "main.h"
#include "usart.h"
#include "kt6368a.h"
#include "app_bootloader.h"
#include "boot_platform.h"

/* 独立保底(Recovery)固件入口：常驻 0x08008000 起的扇区 2..3，由主引导根据启动请求、
 * 镜像与升级标志选择进入。本程序不跑手表任务，只通过 USART1+循环 DMA 接收上位机下发的
 * APP 镜像分块写 Flash，校验通过后再跳回 APP；协议解析与调度在 application/app_bootloader.c。 */

static DMA_HandleTypeDef hdma_usart1_rx;

/* USART1_RX 走 DMA2_Stream5/Channel4。用 DMA_CIRCULAR 把字节连续写入
 * int_bootloader.c 的 8KiB 双半区环形缓冲，半满/全满中断只发布进度，收流期间
 * 无需每半区重启；FIFO 关闭、按字节搬运、高优先级。 */
static void boot_dma_init(void)
{
    /* 打开 DMA2 时钟，后续才能配置 DMA2_Stream5 寄存器。 */
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* 选用 DMA2_Stream5 作为 USART1_RX 的搬运通道。 */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    /* 为USART1_RX选择Channel4请求。 */
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    /* 传输方向为外设到内存，即 UART 数据寄存器搬到接收缓冲。 */
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    /* 外设地址固定为 USART1->DR，不做地址自增。 */
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    /* 内存地址每搬一个字节就自增，依次填满环形缓冲。 */
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    /* 外设侧按字节宽度访问。 */
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    /* 内存侧同样按字节宽度写入。 */
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    /* 循环模式：搬满缓冲后自动回到起点继续接收。 */
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    /* 接收实时性优先，使用高优先级。 */
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    /* 关闭 FIFO，直接模式逐字节搬运。 */
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    /* 写入 DMA 配置，失败则复位。 */
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        /* DMA 不可用时无法收流，进入统一错误处理。 */
        Error_Handler();
    }
    /* 关联接收DMA句柄，供HAL UART的DMA接收使用。 */
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
    /* DMA2_Stream5 中断设为最高抢占/子优先级。 */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    /* 使能 DMA2_Stream5 中断，半满/全满时进入回调更新进度。 */
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

/* 升级成功后尝试跳转 APP，APP向量与EEPROM有效标志检查不通过则返回；
 * 跳转前先停 DMA、反初始化 UART/DMA 释放本程序启用的外设，再交给
 * boot_platform_jump 关其余中断并搬 MSP 后 BX 到 APP 复位入口。 */
void int_bootloader_jump_to_app(void)
{
    /* 先校验APP向量与EEPROM有效标志，不满足就留在保底，不跳转。 */
    if (int_bootloader_app_is_valid() == 0)
    {
        /* APP不满足启动条件，返回调用方，由其按跳转失败处理。 */
        return;
    }
    /* 停止本程序启动的 UART 接收与 DMA 传输。 */
    int_bootloader_stop();
    /* 反初始化 USART1，撤销本程序对外设的配置。 */
    (void)HAL_UART_DeInit(&huart1);
    /* 反初始化 DMA2_Stream5，释放接收通道。 */
    (void)HAL_DMA_DeInit(&hdma_usart1_rx);
    /* 切换到 APP 向量表并跳到 APP 复位入口，不再返回。 */
    boot_platform_jump(APP_START_ADDR);
}

int main(void)
{
    /* 先配置 PA3 供电、看门狗屏蔽、BLE_EN 和 KEY_1 输入。 */
    boot_platform_gpio_init();
    /* 进入保底后再次将 VTOR 设为自身起址，在 HAL 初始化启动 SysTick 前
     * 确认后续异常使用保底向量表。 */
    SCB->VTOR = RECOVERY_START_ADDR;
    /* 同步 VTOR 写入并刷新指令流，使后续异常取新向量表。 */
    __DSB();
    __ISB();
    /* 初始化 HAL 与 1ms SysTick 时基。 */
    HAL_Init();
    /* 关闭数据缓存，避免读到旧缓存内容。 */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    /* 建立引导阶段 100MHz 系统时钟。 */
    boot_platform_clock_init();
    /* 初始化 USART1 串口外设，用于接收上位机固件数据。 */
    MX_USART1_UART_Init();
    /* 配置 USART1_RX 的 DMA2_Stream5 循环接收。 */
    boot_dma_init();
    /* 初始化 KT6368A 蓝牙模块；失败则复位重来。 */
    if (kt6368a_init() != HAL_OK)
    {
        /* 模块初始化失败，进入统一错误处理。 */
        Error_Handler();
    }
    /* 初始化升级协议状态与接收缓冲。 */
    app_bootloader_init();
    /* 裸机轮询循环，无 RTOS；每轮推进 DMA 半区搬运、帧解析与写 Flash。 */
    for (;;)
    {
        /* 每轮检查 DMA 进度、解析帧并按需写 Flash。 */
        app_bootloader_work();
    }
}

void SysTick_Handler(void)
{
    /* 每个 1ms SysTick 把 HAL 毫秒计数加一。 */
    HAL_IncTick();
}

/* UART 错误由 HAL 回调记录；DMA 半满/全满经 HAL 回调链更新接收进度，
 * 接收空闲由主循环按字节总量和静默时长判断。 */
void USART1_IRQHandler(void)
{
    /* 交给 HAL 统一处理 USART1 的接收/错误中断标志。 */
    HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream5_IRQHandler(void)
{
    /* 交给 HAL 处理 DMA 半满、全满与错误标志，并调用对应回调。 */
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/* 任一初始化失败即复位，由主引导再次据 EEPROM 标志决定回保底或跳 APP。 */
void Error_Handler(void)
{
    /* 触发内核复位，重新走主引导的启动选择。 */
    NVIC_SystemReset();
    /* 复位生效前保持空转，不再执行后续代码。 */
    for (;;)
    {
    }
}
