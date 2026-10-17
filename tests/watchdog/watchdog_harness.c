/* 执行真实驱动；核对低有效接通、WDI 高阻释放、任务/调度器前等待和重复启停。 */
#include "tps3823.h"
#include "main.h"
#include "task.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef host_gpio_a, host_gpio_b;
static uint32_t wdi_mode = GPIO_MODE_OUTPUT_PP;
static GPIO_PinState gate = GPIO_PIN_SET;
static GPIO_PinState wdi;
static unsigned int toggles, waits;
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
    assert(port == GPIOB && gpio->Pin == GPIO_PIN_2 && gpio->Pull == GPIO_NOPULL);
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
    assert(port == GPIOB);
    if (pin == GPIO_PIN_1)
    {
        if (state == GPIO_PIN_RESET)
        {
            assert(reset_released != 0 && wdi_mode == GPIO_MODE_OUTPUT_PP);
            assert(wdi == GPIO_PIN_SET);
        }
        gate = state;
    }
    else
    {
        assert(pin == GPIO_PIN_2 && wdi_mode == GPIO_MODE_INPUT);
        wdi = state;
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    assert(port == GPIOB && pin == GPIO_PIN_2 && wdi_mode == GPIO_MODE_OUTPUT_PP);
    assert(reset_released != 0);
    if (wdi == GPIO_PIN_RESET)
    {
        wdi = GPIO_PIN_SET;
    }
    else
    {
        wdi = GPIO_PIN_RESET;
    }
    ++toggles;
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
    tps3823_init();
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT && waits == 0);
    tps3823_feed();
    assert(toggles == 0);
    tps3823_enable();
    assert(gate == GPIO_PIN_RESET && waits == 1 && toggles == 1);
    tps3823_enable();
    assert(waits == 1 && toggles == 1);
    tps3823_feed();
    assert(wdi == GPIO_PIN_RESET && toggles == 2);
    tps3823_feed();
    assert(wdi == GPIO_PIN_SET && toggles == 3);
    tps3823_disable();
    tps3823_feed();
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT && toggles == 3);
    tps3823_enable();
    assert(gate == GPIO_PIN_RESET && waits == 2 && toggles == 4);
    puts("TPS3823: pin polarity, high-impedance release, feed and re-enable: PASS");
    return 0;
}
