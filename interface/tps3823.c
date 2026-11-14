#include "tps3823.h"
#include "main.h"
#include "platform_delay.h"

#define TPS3823_RESET_RELEASE_MS 301 /* 手册复位延时最大 300ms，留出时间裕量。 */

static uint8_t tps3823_enabled;
static uint8_t tps3823_stop_paused;

void tps3823_init(void)
{
    /* 上电初始化先保持看门狗断开：屏蔽复位输出并把 WDI 置高阻 */
    tps3823_disable();
}

void tps3823_disable(void)
{
    /* gpio 描述 WDI 引脚的新配置；先清零，避免未赋值字段带随机值 */
    GPIO_InitTypeDef gpio = {0};

    /* WDG_EN 输出高电平，断开 BL1551B 对外部复位线的接通 */
    HAL_GPIO_WritePin(WDG_EN_GPIO_Port, WDG_EN_Pin, GPIO_PIN_SET);
    /* 记录软件状态：后续 tps3823_feed 据此判断是否需要喂狗 */
    tps3823_enabled = 0;
    /* 已禁用时不存在暂停状态，清掉后resume不会误恢复输出。 */
    tps3823_stop_paused = 0;
    /* 仅屏蔽 RST 不会停止计时；WDI 高阻使内部自动喂狗，便于随后安全接通。 */
    gpio.Pin = WDI_Pin;
    /* 把 WDI 切成输入，呈高阻，不再由 MCU 主动驱动 */
    gpio.Mode = GPIO_MODE_INPUT;
    /* 不加上下拉，引脚电平交由外部电路决定 */
    gpio.Pull = GPIO_NOPULL;
    /* 写入寄存器，完成 WDI 高阻配置 */
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);
}

void tps3823_enable(void)
{
    /* gpio 描述 WDI 引脚的输出配置；先清零保证字段确定 */
    GPIO_InitTypeDef gpio = {0};

    /* 已经处于使能状态时直接返回，不重复走延时和接通流程 */
    if (tps3823_enabled != 0)
    {
        return;
    }

    /* 先回到断开态：复位线屏蔽、WDI 高阻、软件标志清零 */
    tps3823_disable();
    /* 等待正常上电复位延时；非A版若已被WDI跳变锁存，必须断电，等待不能解锁。 */
    platform_delay_min_ms(TPS3823_RESET_RELEASE_MS);

    /* 先把 WDI 预置为低电平，切输出后即驱动低，避免切换瞬间出现高脉冲 */
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    /* 指定要配置的是 WDI 引脚 */
    gpio.Pin = WDI_Pin;
    /* 推挽输出：由 MCU 主动驱动 WDI 产生喂狗所需的高低电平 */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    /* 不启用内部上下拉，输出电平不受内部电阻影响 */
    gpio.Pull = GPIO_NOPULL;
    /* 喂狗脉冲间隔远大于引脚翻转时间，低速档即可 */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    /* 写入寄存器，把 WDI 切成输出 */
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);
    /* 置使能标志，之后 tps3823_feed 才会真正输出脉冲 */
    tps3823_enabled = 1;
    /* 新接通后不可能处于暂停态，清掉以允许后续长STOP重新暂停。 */
    tps3823_stop_paused = 0;
    /* 接通复位线前先输出一次喂狗脉冲。 */
    tps3823_feed();
    /* WDG_EN 拉低，接通 BL1551B，让看门狗复位可以作用到系统 */
    HAL_GPIO_WritePin(WDG_EN_GPIO_Port, WDG_EN_Pin, GPIO_PIN_RESET);
}

void tps3823_feed(void)
{
    /* i 为产生高电平脉宽的 NOP 循环计数 */
    uint8_t i;
    /* 仅在使能状态下喂狗；禁用时保持 WDI 高阻，不做任何电平动作 */
    if (tps3823_enabled != 0)
    {
        /* 每次均产生有效下降沿；64MHz下16个NOP的高脉宽超过手册100ns。 */
        HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
        /* 空转 16 次，把 WDI 高电平维持足够长的脉宽 */
        for (i = 0; i < 16; ++i)
        {
            /* 单条空操作，仅消耗 CPU 周期，不改变任何寄存器 */
            __NOP();
        }
        /* 拉回低电平，形成本次喂狗的下降沿，重置芯片内部计时 */
        HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    }
}

void tps3823_stop_pause(void)
{
    /* gpio 描述暂停期间 WDI 的高阻配置；先清零保证字段确定 */
    GPIO_InitTypeDef gpio = {0};

    /* 未启用时没有需要暂停的超时检测；重复暂停也直接返回，不重复改GPIO。 */
    if (tps3823_enabled == 0 || tps3823_stop_paused != 0)
    {
        return;
    }
    /* WDI高阻让TPS3823内部产生脉冲，停止超时检测；PB1/WDG_EN保持低，欠压复位仍接通。 */
    gpio.Pin = WDI_Pin;
    /* 把 WDI 切成无上下拉输入，停止MCU侧下降沿驱动 */
    gpio.Mode = GPIO_MODE_INPUT;
    /* 不加上下拉，避免内部电阻影响芯片内部喂狗脉冲 */
    gpio.Pull = GPIO_NOPULL;
    /* 写入寄存器，完成暂停配置 */
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);
    /* 记录暂停已实际执行，resume只在这种情况下恢复输出。 */
    tps3823_stop_paused = 1;
}

void tps3823_stop_resume(void)
{
    /* gpio 描述恢复后的 WDI 推挽输出配置；先清零保证字段确定 */
    GPIO_InitTypeDef gpio = {0};

    /* 只有已启用且确实暂停过才需要恢复；其它情况保持现状。 */
    if (tps3823_enabled == 0 || tps3823_stop_paused == 0)
    {
        return;
    }
    /* 先预置低电平，切输出后不产生额外高脉冲。 */
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    /* 指定要恢复的是 WDI 引脚 */
    gpio.Pin = WDI_Pin;
    /* 恢复推挽输出，由MCU继续周期性喂狗 */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    /* 不启用内部上下拉 */
    gpio.Pull = GPIO_NOPULL;
    /* 喂狗脉冲间隔远大于引脚翻转时间，低速档即可 */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    /* 写入寄存器，把 WDI 切回输出 */
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);
    /* 清除暂停状态，重复resume不会再产生脉冲。 */
    tps3823_stop_paused = 0;
    /* 立即补一次喂狗，重新开始芯片超时计时。 */
    tps3823_feed();
}
