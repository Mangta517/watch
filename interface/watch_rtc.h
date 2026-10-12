/** RTC使用LSE 32768Hz；初始化在调度器前，运行期仅由LVGL任务读写。 */
#ifndef WATCH_RTC_H
#define WATCH_RTC_H
#include "main.h"

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} watch_time_struct;

HAL_StatusTypeDef watch_rtc_init(void);
HAL_StatusTypeDef watch_rtc_get_time(watch_time_struct *time);
HAL_StatusTypeDef watch_rtc_set_time(uint8_t hours, uint8_t minutes);
uint8_t watch_rtc_is_synced(void);
#endif
