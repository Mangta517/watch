/** 由原LVGL蓝牙上下文调用；命令必须以CR/LF结束，不消费其他模块的发送结果。 */
#ifndef APP_BATTERY_LOG_BLE_H
#define APP_BATTERY_LOG_BLE_H
#include <stdint.h>

void app_battery_log_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
void app_battery_log_ble_reset(void);
/* 返回1表示正在处理日志命令/发送，原心率波形须等待；断电后仍回收自己的在途结果。 */
uint8_t app_battery_log_ble_service(uint8_t ready, uint32_t now_ms);

#endif
