#include "battery_adc.h"
#include "adc.h"
#include "stm32f4xx_ll_adc.h"

static HAL_StatusTypeDef battery_adc_read_channel(uint32_t channel, uint32_t timeout_ms, uint16_t *raw)
{
    ADC_ChannelConfTypeDef config = {0};
    HAL_StatusTypeDef result;
    HAL_StatusTypeDef stop_result;
    volatile uint32_t delay;

    config.Channel = channel;
    config.Rank = 1;
    config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    result = HAL_ADC_ConfigChannel(&hadc1, &config);
    if (result != HAL_OK)
    {
        return result;
    }
    if (channel == ADC_CHANNEL_VREFINT)
    {
        /* TSVREFE由HAL开启；留足内部参考至少10us的稳定时间，不依赖毫秒节拍。 */
        for (delay = 0; delay < SystemCoreClock / 1000000 * LL_ADC_DELAY_VREFINT_STAB_US; ++delay)
        {
            __NOP();
        }
    }
    result = HAL_ADC_Start(&hadc1);
    if (result == HAL_OK)
    {
        /* ADC时钟10.5MHz、采样480周期，调用方提供有限轮询超时。 */
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

HAL_StatusTypeDef battery_adc_read_raw(uint32_t timeout_ms, uint16_t *raw, uint32_t *vdda_mv)
{
    uint16_t reference, battery;
    uint16_t calibration = *VREFINT_CAL_ADDR;
    uint32_t reference_enabled = ADC->CCR & ADC_CCR_TSVREFE;
    uint32_t supply_mv = 0;
    HAL_StatusTypeDef result;

    if (raw == NULL || vdda_mv == NULL || calibration == 0 || calibration == 0xFFFF)
    {
        return HAL_ERROR;
    }
    result = battery_adc_read_channel(ADC_CHANNEL_VREFINT, timeout_ms, &reference);
    if (result == HAL_OK)
    {
        if (reference == 0)
        {
            result = HAL_ERROR;
        }
        else
        {
            supply_mv = (VREFINT_CAL_VREF * (uint32_t)calibration + reference / 2) / reference;
            if (supply_mv < 1800 || supply_mv > 3600)
            {
                result = HAL_ERROR;
            }
            else
            {
                result = battery_adc_read_channel(ADC_CHANNEL_1, timeout_ms, &battery);
            }
        }
    }
    /* 恢复内部参考开关原状态，避免熄屏期间持续消耗参考源电流。 */
    if (reference_enabled == 0)
    {
        ADC->CCR &= ~ADC_CCR_TSVREFE;
    }
    if (result == HAL_OK)
    {
        *raw = battery;
        *vdda_mv = supply_mv;
    }
    return result;
}

uint8_t battery_adc_is_stop_idle(void)
{
    return (ADC1->CR2 & ADC_CR2_ADON) == 0;
}
