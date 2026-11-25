/* 执行SystemInit和main的启动前缀，模拟SW2松手后仍由PA3保持供电。 */
#include "main.h"
#include <assert.h>
#include <stdio.h>

#define STM32F411xE
#define RCC_AHB1ENR_GPIOAEN (1U << 0)
#define RCC_AHB1ENR_GPIOBEN (1U << 1)
#define GPIO_BSRR_BS3       (1U << 3)
#define GPIO_BSRR_BS1       (1U << 1)
#define GPIO_MODER_MODER3   (3U << 6)
#define GPIO_MODER_MODER3_0 (1U << 6)
#define GPIO_MODER_MODER1   (3U << 2)
#define GPIO_MODER_MODER1_0 (1U << 2)

static struct
{
    volatile uint32_t AHB1ENR;
} host_rcc = {0x100000};
static struct
{
    volatile uint32_t MODER, BSRR;
} startup_gpio_a = {0xA5A5A5A5, 0}, startup_gpio_b = {0x5A5A5A5A, 0};

#undef GPIOA
#undef GPIOB
#define GPIOA (&startup_gpio_a)
#define GPIOB (&startup_gpio_b)
#define RCC   (&host_rcc)

#include "system_init.inc"

static unsigned int step;
static uint8_t button_pressed = 1;
static unsigned int flash_powerdown_calls, regulator_calls, adc_clock_disables;

void host_adc_clock_set(uint8_t enabled)
{
    assert(enabled == 0);
    ++adc_clock_disables;
}
static struct
{
    uint32_t IMR;
} host_exti;
#define EXTI (&host_exti)

static void app_reset_init(void)
{
    assert(step++ == 0);
    assert(POWER_EN_GPIO_Port->BSRR == POWER_EN_Pin);
    assert(WDG_EN_GPIO_Port->BSRR == WDG_EN_Pin);
}

static void HAL_Init(void)
{
    assert(step++ == 1);
    button_pressed = 0;
    assert((GPIOA->MODER & GPIO_MODER_MODER3) == GPIO_MODER_MODER3_0);
}

static void watch_system_clock_config(void)
{
    assert(step++ == 2 && !button_pressed);
}

static void MX_GPIO_Init(void)
{
    assert(step++ == 3);
}

#include "main_prefix.inc"

static void HAL_PWREx_EnableFlashPowerDown(void)
{
    ++flash_powerdown_calls;
}

static void HAL_PWREx_EnableLowRegulatorLowVoltage(void)
{
    ++regulator_calls;
}

#include "main_power.inc"

int main(void)
{
    uint32_t original_imr = 0xA5A5FFFF;

    SystemInit();
    assert(RCC->AHB1ENR == (0x100000 | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN));
    assert(GPIOA->MODER == ((0xA5A5A5A5 & ~GPIO_MODER_MODER3) | GPIO_MODER_MODER3_0));
    assert(GPIOB->MODER == ((0x5A5A5A5A & ~GPIO_MODER_MODER1) | GPIO_MODER_MODER1_0));
    boot_entry();
    assert(step == 4 && !button_pressed);
    puts("PASS: SystemInit asserts PA3 and PB1 before main, preserves adjacent pins and retains early reset capture.");
    EXTI->IMR = original_imr;
    boot_power_config();
    assert((EXTI->IMR & MPU6050_INT_Pin) == 0);
    assert((EXTI->IMR | MPU6050_INT_Pin) == original_imr);
    assert((EXTI->IMR & WAKE_Pin) != 0);
    original_imr = EXTI->IMR;
    boot_power_config();
    assert(EXTI->IMR == original_imr);
    assert(flash_powerdown_calls == 2 && regulator_calls == 2 && adc_clock_disables == 2);
    puts("PASS: startup keeps MPU PB12 masked and preserves PA4, all other EXTI lines and an already-masked PB12.");
    return 0;
}
