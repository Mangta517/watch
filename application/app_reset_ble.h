#ifndef APP_RESET_BLE_H
#define APP_RESET_BLE_H

#include <stdint.h>

/* 由LVGL蓝牙上下文串行调用；RESET/STATE/HEART,READ及RESET,HISTORY必须以CR/LF结束。 */
void app_reset_ble_reset(void);
void app_reset_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
/* 返回1时本次诊断仍占用发送；断电后也先回收自己的异步TX结果。 */
uint8_t app_reset_ble_service(uint8_t ready, uint32_t now_ms);

#endif
