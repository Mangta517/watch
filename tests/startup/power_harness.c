/* 执行main.c的真实启动前缀，模拟SW2松手后仍由PA3保持供电。 */
#include "main.h"
#include <assert.h>
#include <stdio.h>

#define GPIO_PULLDOWN                2U
#define GPIO_SPEED_FREQ_LOW          0U
#define __HAL_RCC_GPIOA_CLK_ENABLE() enable_gpio_a()

GPIO_TypeDef host_gpio_a, host_gpio_b;
static unsigned int step;
static uint8_t button_pressed = 1;
static uint8_t output_high, output_enabled;

static void enable_gpio_a(void)
{
    assert(step++ == 0);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(step++ == 1 && port == POWER_EN_GPIO_Port && pin == POWER_EN_Pin && state == GPIO_PIN_SET);
    output_high = 1;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    assert(step++ == 2 && port == POWER_EN_GPIO_Port && config->Pin == POWER_EN_Pin);
    assert(output_high && config->Mode == GPIO_MODE_OUTPUT_PP);
    assert(config->Pull == GPIO_PULLDOWN && config->Speed == GPIO_SPEED_FREQ_LOW);
    output_enabled = 1;
}

static void HAL_Init(void)
{
    assert(step++ == 3);
    button_pressed = 0;
    assert(output_enabled && output_high); /* 松手后供电保持不依赖按钮。 */
}

static void watch_system_clock_config(void)
{
    assert(step++ == 4 && !button_pressed && output_enabled && output_high);
}

static void MX_GPIO_Init(void)
{
    assert(step++ == 5 && output_enabled && output_high);
}

#include "main_prefix.inc"

int main(void)
{
    boot_entry();
    assert(step == 6 && !button_pressed && output_high && output_enabled);
    puts("PASS: PA3 output preset before HAL/clock initialization, short-press power hold.");
    return 0;
}
