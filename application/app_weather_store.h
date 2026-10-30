/** 手机天气掉电保存：EEPROM 0xC0/0xD0各16字节，sensor独占硬件访问。 */
#ifndef APP_WEATHER_STORE_H
#define APP_WEATHER_STORE_H

#include <stdint.h>

/* sensor开机读取；LVGL可同时发布先到达的手机天气。 */
void app_weather_store_init(void);
/* LVGL每轮调用：恢复缓存并发布新手机帧，临界区内仅复制快照。 */
void app_weather_store_sync(void);
/* sensor分步写入/读回；返回1表示占用EEPROM，其他保存必须等到返回0。 */
uint8_t app_weather_store_service(uint8_t storage_available);

#endif
