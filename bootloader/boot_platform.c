#include "main.h"
#include "boot_platform.h"

/* 主引导与独立保底共用的板级起步例程：保电、屏蔽外部看门狗、内部 HSI 时钟，
 * 以及把 CPU 完整交接到目标镜像复位入口的跳转流程。跳转前先清理本程序启用
 * 的中断与 SysTick，避免切到目标向量后仍跑在旧调度上。 */

/* PA3 供电使能置高保持自锁；WDG_EN 置高屏蔽 TPS3823 复位线，WDI 保持高阻不喂狗
 * （升级期间不跑手表任务，无需喂狗）；BLE_EN 拉低关闭蓝牙；KEY_1 上拉输入用于
 * 长按进保底的判定。 */
void boot_platform_gpio_init(void)
{
    /* GPIO 配置模板，先整体清零，避免结构体里残留未显式赋值的字段。 */
    GPIO_InitTypeDef gpio = {0};
    /* 打开 GPIOA 与 GPIOB 时钟，下面两组端口的寄存器才能写入。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* 先把 PA3 输出锁存置高，再切成推挽输出，使能后立即保持供电。 */
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET);
    /* 本轮只配置 PA3 一个引脚。 */
    gpio.Pin = POWER_EN_Pin;
    /* PA3 设为推挽输出，用来驱动电源使能。 */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    /* 输出脚内部下拉，未驱动时保持低电平。 */
    gpio.Pull = GPIO_PULLDOWN;
    /* 电源使能是静态电平，低速档即可。 */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    /* 按以上字段初始化 PA3。 */
    HAL_GPIO_Init(POWER_EN_GPIO_Port, &gpio);

    /* 升级不运行手表任务，屏蔽外部看门狗RST，WDI保持高阻。 */
    /* 先把 WDG_EN 置高，屏蔽 TPS3823 的复位输出。 */
    HAL_GPIO_WritePin(WDG_EN_GPIO_Port, WDG_EN_Pin, GPIO_PIN_SET);
    /* 复用模板，改配 WDG_EN 引脚。 */
    gpio.Pin = WDG_EN_Pin;
    /* WDG_EN 不需要内部上下拉。 */
    gpio.Pull = GPIO_NOPULL;
    /* 保持推挽输出模式，把 WDG_EN 初始化为输出。 */
    HAL_GPIO_Init(WDG_EN_GPIO_Port, &gpio);
    /* 同一模板切到 WDI 引脚。 */
    gpio.Pin = WDI_Pin;
    /* WDI 改成输入，呈高阻，不主动喂狗。 */
    gpio.Mode = GPIO_MODE_INPUT;
    /* 按输入模式初始化 WDI。 */
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);

    /* 先把 BLE_EN 输出拉低，关闭蓝牙供电/使能。 */
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);
    /* 模板切到 BLE_EN 引脚。 */
    gpio.Pin = BLE_EN_Pin;
    /* BLE_EN 恢复为推挽输出。 */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    /* 初始化 BLE_EN 输出。 */
    HAL_GPIO_Init(BLE_EN_GPIO_Port, &gpio);
    /* 模板切到 KEY_1 引脚。 */
    gpio.Pin = KEY_1_Pin;
    /* KEY_1 配置为输入，读取按键电平。 */
    gpio.Mode = GPIO_MODE_INPUT;
    /* 打开内部上拉，按键未按下时读到高电平。 */
    gpio.Pull = GPIO_PULLUP;
    /* 完成 KEY_1 输入初始化。 */
    HAL_GPIO_Init(KEY_1_GPIO_Port, &gpio);
}

/* 引导阶段用内部 HSI 作 PLL 源（不依赖外部 HSE）：
 * 16MHz/8×100/2 = 100MHz 系统时钟，APB1 二分频，Flash 3 等待周期。
 * APP 运行时会自行重配为 64MHz，这里只为引导/保底提供一个确定时钟。 */
void boot_platform_clock_init(void)
{
    /* 振荡器配置模板，先清零再逐项填写 HSI 与 PLL 参数。 */
    RCC_OscInitTypeDef oscillator = {0};
    /* 总线分频与系统时钟源配置模板，先清零。 */
    RCC_ClkInitTypeDef clock = {0};
    /* 打开 PWR 时钟，才能修改电压调节档位。 */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* 选择 Scale1 电压档，对应 100MHz 系统时钟所需供电。 */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* 本轮只配置 HSI 振荡器。 */
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    /* 打开内部 16MHz HSI。 */
    oscillator.HSIState = RCC_HSI_ON;
    /* 使用出厂校准值，不做额外 HSI 校准。 */
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    /* 打开 PLL，作为系统时钟源。 */
    oscillator.PLL.PLLState = RCC_PLL_ON;
    /* PLL 输入选择 HSI。 */
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    /* HSI 16MHz 先除以 8，得到 2MHz 的 PLL 输入。 */
    oscillator.PLL.PLLM = 8;
    /* 2MHz 乘以 100，得到 200MHz 的 VCO 频率。 */
    oscillator.PLL.PLLN = 100;
    /* VCO 再二分频，得到 100MHz 的系统时钟。 */
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    /* PLLQ 设为 4，即 VCO 200MHz 再四分频得到 50MHz。 */
    oscillator.PLL.PLLQ = 4;
    /* 写入振荡器与 PLL 配置，失败则复位。 */
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        /* 时钟树无法建立，进入统一错误处理。 */
        Error_Handler();
    }
    /* 同时配置 HCLK、SYSCLK、PCLK1、PCLK2 四类总线时钟。 */
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    /* 系统时钟切到 PLL 输出。 */
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    /* AHB 不分频，HCLK 保持 100MHz。 */
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    /* APB1 二分频，得到 50MHz。 */
    clock.APB1CLKDivider = RCC_HCLK_DIV2;
    /* APB2 不分频，保持 100MHz。 */
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    /* 按 100MHz 设置 Flash 3 个等待周期，并写入总线分频配置。 */
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_3) != HAL_OK)
    {
        /* 总线时钟切换失败，进入统一错误处理。 */
        Error_Handler();
    }
}

/* 修改MSP后直接BX，不经过C函数栈帧退出；目标从Reset_Handler开始。
 * 入口按 AAPCS 传入 r0=目标栈顶、r1=目标复位入口：先把 r0 写入 MSP，
 * 再清 CONTROL/BASEPRI 并开中断，最后 BX r1，此后不再返回本程序。 */
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

/* address 处前两字为目标向量表：[0]=初始 MSP，[1]=复位处理函数。调用方须先校验
 * 向量有效并自行反初始化它额外启用的 UART/DMA。 */
void boot_platform_jump(uint32_t address)
{
    /* 取目标向量表第 0 个字，作为目标镜像的初始主栈指针 MSP。 */
    uint32_t stack = *(volatile uint32_t *)address;
    /* 取目标向量表第 1 个字，作为目标镜像的复位处理函数入口。 */
    uint32_t reset = *(volatile uint32_t *)(address + 4);
    /* NVIC 使能/挂起寄存器的循环下标，共 8 组，覆盖全部中断线。 */
    uint32_t i;

    /* 保留PA3供电，不调用会复位GPIO的HAL_DeInit。 */
    /* 复位 HAL 维护的 RCC 状态，但不反初始化 GPIO，PA3 供电保持。 */
    (void)HAL_RCC_DeInit();
    /* 屏蔽可屏蔽中断，停止 SysTick 并清除 NVIC 使能/挂起及 PendSV 挂起，
     * 减少旧中断状态被带入目标镜像。 */
    /* 先关全局可屏蔽中断，避免清理过程中被中断打断。 */
    __disable_irq();
    /* 关闭 SysTick 计数，并清零重装值与当前值。 */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    /* 逐组清除 8 个 NVIC 使能寄存器与挂起寄存器中的全部中断。 */
    for (i = 0; i < 8; ++i)
    {
        /* 写 1 到 ICER 关闭该组全部中断使能。 */
        NVIC->ICER[i] = 0xFFFFFFFFU;
        /* 写 1 到 ICPR 清除该组全部已挂起中断。 */
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    /* 单独清除 SysTick 与 PendSV 的挂起位，防止跳转后立刻进旧异常。 */
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    /* 目标镜像自带时钟/缓存配置，这里只关指令缓存，避免跳过去后取指用到旧缓存。 */
    /* 关闭指令缓存并复位其状态，使目标镜像从 Flash 重新取指。 */
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_RESET();
    /* 设置目标向量表基址，并用 DSB/ISB 同步该更新；随后通过目标复位地址跳转。 */
    /* 把 VTOR 指向目标镜像的向量表起址。 */
    SCB->VTOR = address;
    /* 等待之前的总线写与寄存器更新完成，再同步指令流。 */
    __DSB();
    __ISB();
    /* 用目标栈顶和复位入口执行汇编跳转，不再返回本函数。 */
    boot_jump(stack, reset);
}
