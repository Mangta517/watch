#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#include "main.h"

/* 依次采VREFINT和电池通道；全部成功才输出电池原始值与出厂校准反算的VDDA(mV)。 */
HAL_StatusTypeDef battery_adc_read_raw(uint32_t timeout_ms, uint16_t *raw, uint32_t *vdda_mv);
uint8_t battery_adc_is_stop_idle(void);

#endif
