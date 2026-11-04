/* 执行真实驱动；核对低有效接通、WDI 高阻释放、任务/调度器前等待和重复启停。 */
#include "tps3823.h"
#include "main.h"
#include "task.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
static GPIO_TypeDef host_gpio_c;
#define GPIOC (&host_gpio_c)
#define GPIO_MODE_IT_RISING 0x10110000U
#define GPIO_PULLDOWN 2U
#define EXTI4_IRQn 4
#define EXTI15_10_IRQn 15
#include "../../Core/Src/gpio.c"

GPIO_TypeDef host_gpio_a, host_gpio_b;
static uint16_t analog_a, analog_b, analog_c;
static uint32_t wdi_mode = GPIO_MODE_INPUT;
static GPIO_PinState gate = GPIO_PIN_SET;
static GPIO_PinState wdi;
static unsigned int falling_edges, pulse_nops, waits;
/* 暂停/恢复期间的WDI方向切换次数，用于核对幂等与不产生额外脉冲。 */
static unsigned int pause_transitions, resume_transitions;
static uint8_t reset_released;
static int scheduler = taskSCHEDULER_RUNNING;

int xTaskGetSchedulerState(void)
{
    return scheduler;
}

static void wait_for_reset(uint32_t ms)
{
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT);
    assert(ms >= 301U);
    ++waits;
    reset_released = 1;
}

void HAL_Delay(uint32_t ms)
{
    assert(scheduler != taskSCHEDULER_RUNNING);
    wait_for_reset(ms);
}

void vTaskDelay(TickType_t ticks)
{
    assert(scheduler == taskSCHEDULER_RUNNING);
    wait_for_reset(ticks);
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    if (gpio->Mode == GPIO_MODE_ANALOG)
    {
        assert(gpio->Pull == GPIO_NOPULL);
        if (port == GPIOA)
        {
            analog_a |= gpio->Pin;
        }
        else if (port == GPIOB)
        {
            analog_b |= gpio->Pin;
        }
        else
        {
            assert(port == GPIOC);
            analog_c |= gpio->Pin;
        }
        return;
    }
    if (port == POWER_EN_GPIO_Port && gpio->Pin == POWER_EN_Pin)
    {
        assert(gpio->Pull == GPIO_NOPULL && gpio->Mode == GPIO_MODE_OUTPUT_PP);
    }
    if (port != WDI_GPIO_Port || (gpio->Pin & WDI_Pin) == 0)
    {
        return;
    }
    assert(gpio->Pull == GPIO_NOPULL);
    /* 看门狗已接通（PB1/WDG_EN为低）时的WDI方向切换只可能来自stop_pause/stop_resume：
     * 它们不重复等待复位延时、不动PB1、也不清除已释放复位的标记。 */
    if (gate == GPIO_PIN_RESET)
    {
        assert(reset_released != 0);
        if (gpio->Mode == GPIO_MODE_INPUT)
        {
            /* 暂停：仅把WDI切成高阻输入，输入配置不带输出速度字段。 */
            assert(wdi_mode == GPIO_MODE_OUTPUT_PP);
            ++pause_transitions;
        }
        else
        {
            /* 恢复：先预置低电平再切输出，切换瞬间不出现额外高脉冲。 */
            assert(gpio->Mode == GPIO_MODE_OUTPUT_PP);
            assert(gpio->Speed == GPIO_SPEED_FREQ_LOW);
            assert(wdi == GPIO_PIN_RESET);
            ++resume_transitions;
        }
        wdi_mode = gpio->Mode;
        return;
    }
    assert(gate == GPIO_PIN_SET);
    if (gpio->Mode == GPIO_MODE_OUTPUT_PP)
    {
        assert(reset_released != 0 && gpio->Speed == GPIO_SPEED_FREQ_LOW);
        assert(wdi == GPIO_PIN_RESET);
    }
    else
    {
        assert(gpio->Mode == GPIO_MODE_INPUT);
        reset_released = 0;
    }
    wdi_mode = gpio->Mode;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port != GPIOB)
    {
        return;
    }
    if ((pin & WDG_EN_Pin) != 0)
    {
        if (state == GPIO_PIN_RESET)
        {
            assert(reset_released != 0 && wdi_mode == GPIO_MODE_OUTPUT_PP);
            assert(wdi == GPIO_PIN_RESET && falling_edges != 0);
        }
        gate = state;
    }
    if ((pin & WDI_Pin) != 0)
    {
        if (wdi_mode == GPIO_MODE_OUTPUT_PP)
        {
            assert(reset_released != 0);
            if (state == GPIO_PIN_SET)
            {
                assert(wdi == GPIO_PIN_RESET);
                pulse_nops = 0;
            }
            else
            {
                assert(wdi == GPIO_PIN_SET && pulse_nops >= 16U);
                ++falling_edges;
            }
        }
        wdi = state;
    }
}

void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    (void)irq;
    (void)priority;
    (void)subpriority;
}

void HAL_NVIC_EnableIRQ(int irq)
{
    (void)irq;
}

void host_i2c_nop(void)
{
    assert(wdi_mode == GPIO_MODE_OUTPUT_PP && wdi == GPIO_PIN_SET);
    ++pulse_nops;
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "before_scheduler") == 0)
    {
        scheduler = taskSCHEDULER_NOT_STARTED;
    }
    else if (argc > 1 && strcmp(argv[1], "suspended") == 0)
    {
        scheduler = taskSCHEDULER_SUSPENDED;
    }
    /* 覆盖真实CubeMX前缀，防止驱动初始化之前已在RESET低期间驱动WDI。 */
    MX_GPIO_Init();
    assert(analog_a == (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7));
    assert(analog_b == GPIO_PIN_10 && analog_c == GPIO_PIN_13);
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT && falling_edges == 0);
    tps3823_init();
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT && waits == 0);
    tps3823_feed();
    assert(falling_edges == 0);
    tps3823_enable();
    assert(gate == GPIO_PIN_RESET && waits == 1 && falling_edges == 1);
    tps3823_enable();
    assert(waits == 1 && falling_edges == 1);
    tps3823_feed();
    assert(wdi == GPIO_PIN_RESET && falling_edges == 2);
    tps3823_feed();
    assert(wdi == GPIO_PIN_RESET && falling_edges == 3);
    tps3823_disable();
    tps3823_feed();
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT && falling_edges == 3);
    tps3823_enable();
    assert(gate == GPIO_PIN_RESET && waits == 2 && falling_edges == 4);

    /* 真实长STOP的暂停/恢复：只切换WDI方向，PB1/WDG_EN保持低（欠压复位仍接通），
     * 不重复301ms复位等待，也不产生多余脉冲；重复调用必须是幂等的。 */
    {
        unsigned int falls_before = falling_edges;
        unsigned int waits_before = waits;

        tps3823_stop_pause();
        assert(gate == GPIO_PIN_RESET && wdi_mode == GPIO_MODE_INPUT);
        assert(pause_transitions == 1U && resume_transitions == 0U);
        assert(falling_edges == falls_before && waits == waits_before);
        /* 重复暂停不再改GPIO，也没有任何引脚动作。 */
        tps3823_stop_pause();
        assert(pause_transitions == 1U && wdi_mode == GPIO_MODE_INPUT);
        assert(gate == GPIO_PIN_RESET && falling_edges == falls_before && waits == waits_before);
        tps3823_stop_resume();
        assert(gate == GPIO_PIN_RESET && wdi_mode == GPIO_MODE_OUTPUT_PP && wdi == GPIO_PIN_RESET);
        assert(resume_transitions == 1U && pause_transitions == 1U);
        /* 恢复只补一次喂狗：一次下降沿，且没有额外的复位等待。 */
        assert(falling_edges == falls_before + 1U && waits == waits_before);
        /* 重复恢复是空操作，不产生第二个脉冲。 */
        tps3823_stop_resume();
        assert(resume_transitions == 1U && falling_edges == falls_before + 1U);
        assert(gate == GPIO_PIN_RESET && wdi_mode == GPIO_MODE_OUTPUT_PP && waits == waits_before);
        /* 再次暂停并恢复：两个计数各加一次，引脚动作仍只有一次喂狗下降沿。 */
        tps3823_stop_pause();
        assert(pause_transitions == 2U && wdi_mode == GPIO_MODE_INPUT && gate == GPIO_PIN_RESET);
        tps3823_stop_resume();
        assert(resume_transitions == 2U && wdi_mode == GPIO_MODE_OUTPUT_PP);
        assert(falling_edges == falls_before + 2U && waits == waits_before);
    }

    /* 断开之后暂停/恢复都必须是空操作：不恢复输出、不喂狗、不动PB1。 */
    {
        unsigned int falls_before = falling_edges;
        unsigned int switches_before = pause_transitions + resume_transitions;
        unsigned int waits_before = waits;

        tps3823_disable();
        assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT);
        tps3823_stop_pause();
        tps3823_stop_resume();
        assert(pause_transitions + resume_transitions == switches_before);
        assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT);
        assert(falling_edges == falls_before && waits == waits_before);
        tps3823_feed();
        assert(falling_edges == falls_before && wdi_mode == GPIO_MODE_INPUT);
    }
    puts("TPS3823: stop pause/resume only re-directs WDI, keeps PB1 (reset path) asserted, never re-waits 301ms, "
         "feeds exactly once on resume, is idempotent, and is inert after disable: PASS");
    puts("TPS3823: real boot GPIO keeps WDI high-impedance; gate polarity, reset wait, feed pulse and re-enable: PASS");
    return 0;
}
