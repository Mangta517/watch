#ifndef APP_CLOCK_H
#define APP_CLOCK_H
#include <stdint.h>

#define APP_CLOCK_FRAME_IDLE_MS 200U

typedef enum
{
    APP_CLOCK_NONE,
    APP_CLOCK_SYNCED,
    APP_CLOCK_INVALID,
    APP_CLOCK_RTC_ERROR
} app_clock_result;

/* 与蓝牙显示共用一个接收入口，仅在LVGL任务调用。支持H:M / HH:MM。
 * CR/LF结束消息；无换行时以200ms接收空闲为结束，分段数据不会提前设置RTC。 */
void app_clock_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
void app_clock_service(uint32_t now_ms);
app_clock_result app_clock_get_result(void);
#endif
