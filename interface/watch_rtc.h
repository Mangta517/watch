/** RTC使用LSE 32768Hz；首帧显示后初始化，初始化及运行期读写均由LVGL任务执行。 */
#ifndef WATCH_RTC_H
#define WATCH_RTC_H
#include "main.h"

/* 32.768kHz亚秒计数：约30.5us，避免短休眠反复丢失整毫秒。 */
#define WATCH_RTC_SUBSECOND_HZ 32768

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} watch_time_struct;

typedef struct
{
    watch_time_struct time;
    uint32_t marker;
    HAL_StatusTypeDef read_status;
    uint8_t ready, lse_ready, synced;
} watch_rtc_status_struct;

HAL_StatusTypeDef watch_rtc_init(void);
/* LVGL循环等待LSE就绪，5s真实超时；就绪后复用原RTC配置/备份恢复。 */
void watch_rtc_init_begin(void);
HAL_StatusTypeDef watch_rtc_init_service(void);
uint8_t watch_rtc_init_is_pending(void);
HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time);
/* 0=设置成功，1=未就绪、时间非法或HAL设置失败。 */
uint8_t watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
uint8_t watch_rtc_is_synced(void);
/* LVGL上下文只读快照；不初始化RTC、不改备份标记或时间。 */
void watch_rtc_get_status(watch_rtc_status_struct *status);
/* 返回本秒已过的亚秒计数；未就绪返回UINT32_MAX。由屏蔽中断的IDLE休眠流程读取。 */
uint32_t watch_rtc_get_subsecond(void);
#endif
