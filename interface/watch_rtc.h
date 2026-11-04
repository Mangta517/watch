/** RTC使用LSE 32768Hz；首帧显示后初始化，初始化及运行期读写均由LVGL任务执行。 */
#ifndef WATCH_RTC_H
#define WATCH_RTC_H
#include "main.h"

/* 32.768kHz亚秒计数：约30.5us，避免短休眠反复丢失整毫秒。 */
#define WATCH_RTC_SUBSECOND_HZ 32768
/* 一分钟内的递增计数模数：秒(0~59)*32768 + 本秒已过亚秒。 */
#define WATCH_RTC_COUNTER_MOD (60 * WATCH_RTC_SUBSECOND_HZ)

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
/* INITF/影子同步各按DWT限1ms并用次数兜底；0成功，1未配置、时间非法或RTC故障，故障后可重新校时。 */
uint8_t watch_rtc_set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);
uint8_t watch_rtc_is_synced(void);
/* LVGL上下文只读快照；不初始化RTC、不改备份标记或时间。 */
void watch_rtc_get_status(watch_rtc_status_struct *status);
/* 返回本秒已过的亚秒计数(0~32767)；未就绪返回UINT32_MAX。由屏蔽中断的IDLE休眠流程读取。 */
uint32_t watch_rtc_get_subsecond(void);
/* 返回一分钟内递增的32768Hz计数(0~WATCH_RTC_COUNTER_MOD-1)，由同一次时间/日期快照的秒与亚秒合成；
 * 未就绪返回UINT32_MAX。跨秒/跨分/午夜差值按WATCH_RTC_COUNTER_MOD取模，供1秒级tickless补时使用。 */
uint32_t watch_rtc_get_counter(void);
/* IDLE屏蔽中断后调用；按32768Hz计数安排提前唤醒，0成功、1失败。 */
uint8_t watch_rtc_start_wakeup(uint32_t subsecond_counts);
void watch_rtc_stop_wakeup(void);
/* STOP醒后有界等待影子同步，不依赖HAL tick；0成功、1失败。 */
uint8_t watch_rtc_sync_after_stop(void);
#endif
