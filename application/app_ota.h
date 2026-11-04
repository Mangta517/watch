#ifndef APP_OTA_H
#define APP_OTA_H

#include <stdint.h>

/* LVGL沿用唯一蓝牙接收入口；OTA,START必须以CR/LF结束。
 * 1秒无新字节丢弃残行；完整命令只置sensor侧请求，不在LVGL擦写。 */
void app_ota_reset_receive(void);
/* 拼接分包字节并识别OTA,START；overflow或非可打印字符使本行作废。 */
void app_ota_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
/* LVGL回收自己的异步回执，留发送窗口后复位；返回1表示升级请求仍在处理。 */
uint8_t app_ota_service(uint8_t ready, uint32_t now_ms);
/* 仅sensor调用；存储可用且有请求时写入并读回升级标志。返回1表示标志已确认，
 * 调用方应停止后续持久化操作并等待复位；其余返回0。 */
uint8_t app_ota_storage_service(uint8_t storage_available);

#endif
