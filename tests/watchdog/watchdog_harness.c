/* 执行真实驱动；核对低有效接通、WDI 高阻释放、任务/调度器前等待和重复启停。 */
#include "tps3823.h"
#include "main.h"
#include "task.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
static unsigned gpio_c_clock, gpio_h_clock;
#define __HAL_RCC_GPIOC_CLK_ENABLE() (gpio_c_clock = 1)
#define __HAL_RCC_GPIOC_CLK_DISABLE() (gpio_c_clock = 0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() (gpio_h_clock = 1)
#define __HAL_RCC_GPIOH_CLK_DISABLE() (gpio_h_clock = 0)
static GPIO_TypeDef host_gpio_c, host_gpio_h;
#define GPIOC (&host_gpio_c)
#define GPIOH (&host_gpio_h)
#define GPIO_MODE_IT_RISING 0x00110000U
#define GPIO_MODE_IT_FALLING 0x00210000U
#define GPIO_PULLDOWN 2U
#define EXTI4_IRQn 4
#define EXTI15_10_IRQn 15
#include "../../Core/Src/gpio.c"
#define __HAL_RCC_SYSCFG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)
#define PWR_PVDLEVEL_7 7
#define PWR_PVD_MODE_NORMAL 0
typedef struct
{
    uint32_t PVDLevel, Mode;
} PWR_PVDTypeDef;
static unsigned pvd_configured, pvd_enabled;
static void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *pvd)
{
    assert(pvd->PVDLevel == PWR_PVDLEVEL_7 && pvd->Mode == PWR_PVD_MODE_NORMAL);
    ++pvd_configured;
}
static void HAL_PWR_EnablePVD(void)
{
    assert(pvd_configured != 0);
    ++pvd_enabled;
}
#include "../../Core/Src/stm32f4xx_hal_msp.c"

GPIO_TypeDef host_gpio_a, host_gpio_b;
static uint16_t analog_a, analog_b, analog_c, analog_h;
static uint32_t primask;
static uint8_t charge_input, sensor_idle, mpu_exti_configured;
uint32_t __get_PRIMASK(void)
{
    return primask;
}
void __disable_irq(void)
{
    primask = 1;
}
void __set_PRIMASK(uint32_t mask)
{
    primask = mask;
}

static uint32_t wdi_mode = GPIO_MODE_INPUT;
static GPIO_PinState gate = GPIO_PIN_SET;
static GPIO_PinState wdi;
static unsigned int falling_edges, pulse_nops, waits;
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
    if (port == GPIOA && gpio->Pin == GPIO_PIN_2)
    {
        assert(gpio->Mode == GPIO_MODE_INPUT && gpio->Pull == GPIO_NOPULL);
        charge_input = 1;
    }
    if ((gpio->Pin & (SENSOR_SDA_Pin | SENSOR_SCL_Pin)) != 0 && port == GPIOB)
    {
        assert(gpio->Mode == GPIO_MODE_OUTPUT_OD && sensor_idle == 3);
    }
    if (port == MPU6050_INT_GPIO_Port && gpio->Pin == MPU6050_INT_Pin)
    {
        assert(gpio->Mode == GPIO_MODE_IT_FALLING && gpio->Pull == GPIO_NOPULL);
        mpu_exti_configured = 1;
    }
    if (port == WDI_GPIO_Port && gpio->Pin == WDI_Pin && waits != 0)
    {
        assert(primask == 1);
    }
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
        else if (port == GPIOC)
        {
            assert(gpio_c_clock != 0);
            analog_c |= gpio->Pin;
        }
        else
        {
            assert(port == GPIOH && gpio_h_clock != 0);
            analog_h |= gpio->Pin;
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
    if (port == GPIOB && (pin & (SENSOR_SDA_Pin | SENSOR_SCL_Pin)) != 0)
    {
        assert(state == GPIO_PIN_SET);
        sensor_idle = 3;
    }
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
    HAL_MspInit();
    assert(pvd_configured == 1 && pvd_enabled == 1);
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
    assert(analog_a == (GPIO_PIN_0 | GPIO_PIN_6 | GPIO_PIN_7) && charge_input != 0);
    assert(analog_b == GPIO_PIN_10 && analog_c == GPIO_PIN_13);
    assert(analog_h == (GPIO_PIN_0 | GPIO_PIN_1) && gpio_c_clock == 0 && gpio_h_clock == 0);
    assert(mpu_exti_configured != 0);
    assert(gate == GPIO_PIN_SET && wdi_mode == GPIO_MODE_INPUT && falling_edges == 0);
    tps3823_init();
    assert(primask == 0);
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

    puts("TPS3823: real boot GPIO keeps WDI high-impedance; gate polarity, reset wait, feed pulse and re-enable: PASS");
    return 0;
}
