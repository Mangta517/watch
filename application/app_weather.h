#ifndef APP_WEATHER_H
#define APP_WEATHER_H

#include <stdint.h>

typedef struct
{
    int16_t temperature_x10;
    uint8_t humidity_rh;
    uint8_t code;
    uint8_t valid;
    uint8_t cached;       /* EEPROM恢复的旧天气，更新时间未知；新手机帧清除此标记。 */
    uint32_t received_ms; /* 最近有效帧的LVGL毫秒tick；所有读写归显示任务。 */
} app_weather_struct;

/* 与蓝牙日志/校时共用接收数据，仅由LVGL任务调用，不另行读取UART。
 * ASCII协议：WEATHER,WMO天气码,摄氏温度乘10,相对湿度百分比\n。 */
void app_weather_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
void app_weather_service(uint32_t now_ms);
void app_weather_reset_receive(void); /* 不清除最后一份有效天气。 */
void app_weather_get_snapshot(app_weather_struct *snapshot);
/* LVGL启动时恢复已校验的记录；已有手机天气时不覆盖，不恢复上次开机tick。 */
void app_weather_restore(const app_weather_struct *snapshot);
const char *app_weather_description(uint8_t code);

#endif
