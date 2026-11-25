/**
 * @file app_mag_store.h
 * @brief 磁力计校准记录；EEPROM 0x00/0x30两槽各48字节。
 */
#ifndef __APP_MAG_STORE_H
#define __APP_MAG_STORE_H

#include "common_mag_cal.h"

typedef struct
{
    common_mag_cal_params_struct params;
    uint8_t is_ellipsoid; /* MAG1的校正模长未归一到50μT。 */
} app_mag_calibration_struct;

typedef enum
{
    APP_MAG_STORE_OK,
    APP_MAG_STORE_EMPTY,
    APP_MAG_STORE_ERROR,
    APP_MAG_STORE_BUSY
} app_mag_store_result;

/* 仅sensor任务调用；EMPTY表示没有通过版本/校验/数值检查的记录。 */
app_mag_store_result app_mag_store_load(app_mag_calibration_struct *calibration);
/* begin仅复制数据，step每轮至多一笔短事务；BUSY时继续采样。
 * 旧同步入口save仅供兼容调用方，内部循环begin/step并延时等待。 */
app_mag_store_result app_mag_store_save_begin(const app_mag_calibration_struct *calibration);
app_mag_store_result app_mag_store_save_step(void);
app_mag_store_result app_mag_store_save(const app_mag_calibration_struct *calibration);

#endif
