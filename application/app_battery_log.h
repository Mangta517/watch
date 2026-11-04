/**
 * @file app_battery_log.h
 * @brief 每分钟追加一条滤波电压；追加与扫描由sensor在锁内更新，LVGL经带锁接口导出。
 *        日志区为内部Flash sector 7 (0x08060000起，16384条8字节记录)，只能整扇区擦除，
 *        因此记录只追加；显式清空由LVGL取得同一日志锁后执行并重启。
 */
#ifndef APP_BATTERY_LOG_H
#define APP_BATTERY_LOG_H

#include <stdint.h>
#include "app_reset.h"

/* 0xFF00..0xFF0F存16个现场字，0xFF10为最后提交槽；0xFFFF是session标记，不复用于复位现场。 */
#define APP_BATTERY_LOG_RESET_TAG   0xFF00
#define APP_BATTERY_LOG_RESET_SLOTS 17

typedef enum
{
    APP_BATTERY_LOG_STARTING,   /* 扫描完成前的默认值，UI按"未就绪"处理。 */
    APP_BATTERY_LOG_READY,      /* 可继续追加电压与复位现场。 */
    APP_BATTERY_LOG_FULL,       /* 剩余槽位不足(含写满)，须显式清空后才恢复。 */
    APP_BATTERY_LOG_NEED_CLEAR, /* 首条不是有效session标记(旧固件/未知格式)，本固件不自动擦除。 */
    APP_BATTERY_LOG_ERROR       /* 日志锁未建立、Flash写失败或session计数溢出。 */
} app_battery_log_state;

typedef struct
{
    uint32_t slots;   /* 已占用槽数(含掉电坏槽)，等于下一条写入位置。 */
    uint32_t samples; /* 扫描/写盘累计的有效分钟电压条数。 */
    uint32_t resets;  /* 完整复位现场组数(每组17槽)。 */
    app_battery_log_state state;
} app_battery_log_status_struct;

typedef struct
{
    uint32_t seconds;    /* session记录中此字段是开机编号；普通记录中是开机秒数。 */
    uint16_t voltage_mv; /* 0xFFFF表示session，其余为滤波mV。 */
} app_battery_log_record_struct;

/* sensor启动时建日志锁并扫描Flash已有记录，推断开机编号、样本数与状态。 */
void app_battery_log_init(void);
/* 仅sensor任务循环调用：跨入新分钟且电压有效时追加，开机后首次符合条件时保存复位现场。 */
void app_battery_log_service(uint32_t seconds);
/* 0=已取得状态快照，1=互斥锁忙。 */
uint8_t app_battery_log_get_status(app_battery_log_status_struct *status);
/* 0=读取成功，1=坏槽/越界，2=互斥锁忙；忙时导出不得跳过此槽。
 * session 为调用方当前已知的开机编号(CRC按它校验)，遇到session标记后要换成其中的新编号继续。 */
uint8_t app_battery_log_read(uint32_t slot, uint32_t session, app_battery_log_record_struct *record);
/* 0=已取得读取结果，1=日志锁忙/未初始化；complete=1时info为完整现场，0表示无完整记录。 */
uint8_t app_battery_log_read_reset(uint32_t slot, uint32_t session, app_reset_info_struct *info, uint8_t *complete);
/* LVGL显式清空电压与复位历史：屏蔽TPS3823、仅擦sector 7，成功失败均重启。 */
void app_battery_log_clear_and_reset(void);

#endif
