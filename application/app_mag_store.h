/**
 * @file app_mag_store.h
 * @brief 磁力计校准记录：三轴偏置(μT)和比例系数；EEPROM前96字节保留给双副本。
 */
#ifndef __APP_MAG_STORE_H
#define __APP_MAG_STORE_H

#include <stdint.h>

typedef struct
{
    float offset_uT[3];
    float scale[3];
} app_mag_calibration_struct;

typedef enum
{
    APP_MAG_STORE_OK,
    APP_MAG_STORE_EMPTY,
    APP_MAG_STORE_ERROR
} app_mag_store_result;

/* 仅sensor任务调用；EMPTY表示没有通过版本/校验/数值检查的记录。 */
app_mag_store_result app_mag_store_load(app_mag_calibration_struct *calibration);
app_mag_store_result app_mag_store_save(const app_mag_calibration_struct *calibration);

#endif
