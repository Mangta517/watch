#ifndef APP_OTA_H
#define APP_OTA_H

#include <stdint.h>

/* LVGL沿用唯一蓝牙接收入口；OTA,START必须以CR/LF结束。 */
void app_ota_reset_receive(void);
void app_ota_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
/* LVGL回收自己的异步回执，留发送窗口后复位；返回1表示升级请求仍在处理。 */
uint8_t app_ota_service(uint8_t ready, uint32_t now_ms);
/* 仅sensor调用；等待其他EEPROM保存结束，写入并读回Bootloader升级标志。 */
uint8_t app_ota_storage_service(uint8_t storage_available);

#endif
