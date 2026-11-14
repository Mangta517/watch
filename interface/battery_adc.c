#include "battery_adc.h"
#include "adc.h"

HAL_StatusTypeDef battery_adc_read_raw(uint32_t timeout_ms, uint16_t *raw)
{
    HAL_StatusTypeDef result;
    HAL_StatusTypeDef stop_result;

    if (raw == NULL)
    {
        return HAL_ERROR;
    }
    result = HAL_ADC_Start(&hadc1);
    if (result == HAL_OK)
    {
        /* ADC时钟8MHz、采样480周期，调用方提供有限轮询超时。 */
        result = HAL_ADC_PollForConversion(&hadc1, timeout_ms);
        if (result == HAL_OK)
        {
            *raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }
        stop_result = HAL_ADC_Stop(&hadc1);
        if (result == HAL_OK)
        {
            result = stop_result;
        }
    }
    return result;
}

uint8_t battery_adc_is_stop_idle(void)
{
    return (ADC1->CR2 & ADC_CR2_ADON) == 0;
}
