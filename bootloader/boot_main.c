#include "main.h"
#include "boot_platform.h"
#include "app_boot_select.h"

/* PA5(KEY_1)空闲为高，复位后直接返回0不延时；按下(低)才每10ms轮询，
 * 连续按住满2000ms返回1，任何提前松开返回0。tick为无符号，跨回绕仍正确。 */
static uint8_t boot_recovery_requested(void)
{
    /* 保存按下时刻的毫秒 tick，用于后面计算已按住时长。 */
    uint32_t start;

    /* KEY_1 空闲为高：未按下就直接返回 0，不启动长按计时。 */
    if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) != GPIO_PIN_RESET)
    {
        /* 本次上电不请求进入保底。 */
        return 0;
    }
    /* 记录当前 tick 作为长按计时的起点。 */
    start = HAL_GetTick();
    /* 已确认按下，进入轮询，直到松开或按满 2000ms。 */
    for (;;)
    {
        /* 每轮等待 10ms 再采样，形成固定的按键轮询间隔。 */
        HAL_Delay(10);
        /* 读到高电平说明按键已松开，提前结束并取消保底请求。 */
        if (HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin) != GPIO_PIN_RESET)
        {
            /* 未按满时长，按普通上电处理。 */
            return 0;
        }
        /* 用无符号差值计算已按住毫秒数，达到 2000ms 即确认长按。 */
        if ((uint32_t)(HAL_GetTick() - start) >= 2000)
        {
            /* 请求进入保底固件。 */
            return 1;
        }
    }
}

int main(void)
{
    /* 保存 app_boot_select_target 选出的镜像起址，0 表示没有可跳转的镜像。 */
    uint32_t target;

    /* 先配置 PA3 供电使能、看门狗屏蔽线、BLE_EN 输出和 KEY_1 输入。 */
    boot_platform_gpio_init();
    /* 初始化 HAL 与 1ms SysTick，供 HAL_Delay/HAL_GetTick 使用。 */
    HAL_Init();
    /* 关闭 Flash 数据缓存。 */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    /* 用内部 HSI PLL 建立引导阶段的 100MHz 系统时钟。 */
    boot_platform_clock_init();
    /* 先采样 KEY_1 是否构成 2s 长按，再连同 APP 向量与 EEPROM 标志一起决定启动目标。 */
    target = app_boot_select_target(boot_recovery_requested());
    /* 选到了 APP 或保底镜像的起址才执行跳转。 */
    if (target != 0)
    {
        /* 切换到目标镜像的向量表并跳到它的复位入口。 */
        boot_platform_jump(target);
    }
    /* 保底向量缺失/损坏时不跳转未知代码，保留PA3供电等待SWD维护。 */
    for (;;)
    {
        /* 无镜像可跳时进入低功耗等待，由中断唤醒后继续空转。 */
        __WFI();
    }
}

void SysTick_Handler(void)
{
    /* 每个 1ms SysTick 把 HAL 的毫秒计数加一。 */
    HAL_IncTick();
}

void Error_Handler(void)
{
    /* 触发内核复位，让主引导按 EEPROM 标志重新选择启动路径。 */
    NVIC_SystemReset();
    /* 复位生效前保持空转，不再执行后续代码。 */
    for (;;)
    {
    }
}
