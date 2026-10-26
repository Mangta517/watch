#include "tps3823.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define TPS3823_RESET_RELEASE_MS 301 /* 手册复位延时最大 300ms，留出时间裕量。 */

static uint8_t tps3823_enabled;

void tps3823_init(void)
{
    tps3823_disable();
}

void tps3823_disable(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(WDG_EN_GPIO_Port, WDG_EN_Pin, GPIO_PIN_SET);
    tps3823_enabled = 0;
    /* 仅屏蔽 RST 不会停止计时；WDI 高阻使内部自动喂狗，便于随后安全接通。 */
    gpio.Pin = WDI_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);
}

void tps3823_enable(void)
{
    GPIO_InitTypeDef gpio = {0};

    if (tps3823_enabled != 0)
    {
        return;
    }

    tps3823_disable();
    /* 等待正常上电复位延时；非A版若已被WDI跳变锁存，必须断电，等待不能解锁。 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(TPS3823_RESET_RELEASE_MS) + 1);
    }
    else
    {
        HAL_Delay(TPS3823_RESET_RELEASE_MS);
    }

    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    gpio.Pin = WDI_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WDI_GPIO_Port, &gpio);
    tps3823_enabled = 1;
    tps3823_feed();
    HAL_GPIO_WritePin(WDG_EN_GPIO_Port, WDG_EN_Pin, GPIO_PIN_RESET);
}

void tps3823_feed(void)
{
    uint8_t i;
    if (tps3823_enabled != 0)
    {
        /* 每次均产生有效下降沿；84MHz下16个NOP的高脉宽超过手册100ns。 */
        HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_SET);
        for (i = 0; i < 16; ++i)
        {
            __NOP();
        }
        HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
    }
}
