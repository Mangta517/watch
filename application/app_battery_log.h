/** 每分钟追加滤波电压；sensor写Flash，LVGL通过互斥快照导出。 */
#ifndef APP_BATTERY_LOG_H
#define APP_BATTERY_LOG_H

#include <stdint.h>

typedef enum
{
    APP_BATTERY_LOG_STARTING,
    APP_BATTERY_LOG_READY,
    APP_BATTERY_LOG_FULL,
    APP_BATTERY_LOG_NEED_CLEAR,
    APP_BATTERY_LOG_ERROR
} app_battery_log_state;

typedef struct
{
    uint32_t slots;
    uint32_t samples;
    app_battery_log_state state;
} app_battery_log_status_struct;

typedef struct
{
    uint32_t seconds;    /* session记录中此字段是开机编号；普通记录中是开机秒数。 */
    uint16_t voltage_mv; /* 0xFFFF表示session，其余为滤波mV。 */
} app_battery_log_record_struct;

void app_battery_log_init(void);
void app_battery_log_service(uint32_t seconds);
/* 0=已取得状态快照，1=互斥锁忙。 */
uint8_t app_battery_log_get_status(app_battery_log_status_struct *status);
/* 0=读取成功，1=坏槽/越界，2=互斥锁忙；忙时导出不得跳过此槽。session来自前一个开机标记。 */
uint8_t app_battery_log_read(uint32_t slot, uint32_t session, app_battery_log_record_struct *record);
/* LVGL显式清空：屏蔽TPS3823、仅擦sector 7，成功失败均重启。 */
void app_battery_log_clear_and_reset(void);

#endif
