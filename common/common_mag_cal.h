#ifndef COMMON_MAG_CAL_H
#define COMMON_MAG_CAL_H

#include <stdint.h>

#define COMMON_MAG_CAL_MAX_POINTS 64
#define COMMON_MAG_CAL_NOMINAL_UT 50.0f

typedef struct
{
    float offset_uT[3];
    float soft_sym[6]; /* Wxx, Wyy, Wzz, Wxy, Wxz, Wyz */
    float dip_ref_deg;
    uint8_t dip_valid;
} common_mag_cal_params_struct;

typedef struct
{
    float mag_uT[3];
    float gravity[3]; /* 同一芯片坐标下的单位加速度，零向量表示无配对帧。 */
} common_mag_cal_point_struct;

typedef struct
{
    common_mag_cal_point_struct points[COMMON_MAG_CAL_MAX_POINTS];
    float min_uT[3], max_uT[3], last_uT[3];
    float closest_squared;
    uint16_t samples;
    uint8_t count, coverage_pct, have_last, closest_index;
} common_mag_cal_struct;

/* gap表示磁帧之间存在断采，跳过本次相邻帧跳变检查。 */
void common_mag_cal_collect(common_mag_cal_struct *cal, const float mag_uT[3], const float gravity[3], uint8_t gap);
/* 单任务调用，共用静态工作区。成功返回0，失败保留调用方系数。 */
uint8_t common_mag_cal_fit(const common_mag_cal_struct *cal, common_mag_cal_params_struct *params);
void common_mag_cal_apply(const common_mag_cal_params_struct *params, const float raw_uT[3], float corrected_uT[3]);
float common_mag_cal_dip_deg(const float mag_uT[3], const float gravity[3]);

#endif
