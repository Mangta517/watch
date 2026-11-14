/** 内部Flash sector 7用于电压与复位日志；APP与Bootloader均止于此地址。 */
#ifndef INT_BATTERY_LOG_H
#define INT_BATTERY_LOG_H

#include "main.h"

#include "boot_layout.h"

/* 写入前绑定；Flash忙等结束后、恢复中断前递交暂存字节。未绑定时不轮询UART。 */
void int_battery_log_set_rx_callbacks(uint8_t (*is_enabled)(void), void (*receive)(uint8_t byte, uint8_t errors));

void int_battery_log_read(uint32_t slot, uint8_t record[BATTERY_LOG_RECORD_BYTES]);
/* 0=写入并读回校验成功，1=参数、Flash操作或读回校验失败。 */
uint8_t int_battery_log_write(uint32_t slot, const uint8_t record[BATTERY_LOG_RECORD_BYTES]);
/* 仅用于显式清空后重启的维护流程，调用方先屏蔽外部看门狗和中断。 */
void int_battery_log_erase(void);

#endif
