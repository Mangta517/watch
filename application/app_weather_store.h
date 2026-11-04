/** 手机天气掉电保存：EEPROM 0xD0/0xE0双槽各16字节轮换，0xC0仅读兼容旧记录，sensor独占硬件访问。 */
#ifndef APP_WEATHER_STORE_H
#define APP_WEATHER_STORE_H

#include <stdint.h>

/* sensor开机读取；LVGL可同时发布先到达的手机天气。 */
void app_weather_store_init(void);
/* LVGL每轮调用：恢复缓存并发布新手机帧，临界区内仅复制快照。 */
void app_weather_store_sync(void);
/* sensor分步写入/读回；返回1表示占用EEPROM，其他保存必须等到返回0。
 * storage_available=0时推迟新事务，已开始的写入继续收尾。 */
uint8_t app_weather_store_service(uint8_t storage_available);

#endif
