#include "platform_delay.h"
#include "main.h"
#if !defined(F411_BOOTLOADER)
#include "FreeRTOS.h"
#include "task.h"
#endif

void platform_delay_ms(uint32_t ms)
{
#if !defined(F411_BOOTLOADER)
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
#endif
    HAL_Delay(ms);
}

void platform_delay_min_ms(uint32_t ms)
{
#if !defined(F411_BOOTLOADER)
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms) + 1);
        return;
    }
#endif
    HAL_Delay(ms);
}
