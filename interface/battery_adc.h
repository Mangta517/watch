#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#include "main.h"

/* 保留HAL错误码供电池快照和诊断使用；成功时输出12位ADC原始值。 */
HAL_StatusTypeDef battery_adc_read_raw(uint32_t timeout_ms, uint16_t *raw);
uint8_t battery_adc_is_stop_idle(void);

#endif
