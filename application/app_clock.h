#ifndef APP_CLOCK_H
#define APP_CLOCK_H
#include <stdint.h>

#define APP_CLOCK_FRAME_IDLE_MS 200

typedef enum
{
    APP_CLOCK_NONE,
    APP_CLOCK_SYNCED,
    APP_CLOCK_INVALID,
    APP_CLOCK_RTC_ERROR
} app_clock_result;

/* 与蓝牙显示共用一个接收入口，仅在LVGL任务调用。支持H:M / HH:MM / HH:MM:SS；省略秒时置零。
 * CR/LF结束消息；无换行时以200ms接收空闲为结束，分段数据不会提前设置RTC。 */
void app_clock_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
void app_clock_service(uint32_t now_ms);
void app_clock_reset_receive(void); /* 丢字节时仅废弃残帧，保留已完成的校时结果。 */
app_clock_result app_clock_get_result(void);
/* LVGL上下文读取合法校时消息计数；用于无连接状态脚时确认手机通信就绪。 */
uint32_t app_clock_get_received_count(void);
#endif
