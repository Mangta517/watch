/**
  ******************************************************************************
  * @file    app_baro.h
  * @brief   应用层-SPL006 气压服务 (周期采样 + 温度补偿 + 最新数据快照)
  *          分层: application(app_task 的 sensor 任务兼管本传感器 → 本层提供服务),
  *          注意: 任务的创建与调度器启动不在本层 (见 app_task.h)。
  *          总线约定: SPL006 与 MPU6050 共用 SENSOR 总线, 任务级唯一 owner 是
  *          sensor 任务; 其他模块取气压一律走 App_Baro_GetSnapshot(),
  *          不得直调 spl006_read_* (与 app_imu.h 同款约束)。
  ******************************************************************************
***/
#ifndef __APP_BARO_H
#define __APP_BARO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define APP_BARO_HISTORY_POINTS    24
#define APP_BARO_HISTORY_PERIOD_MS 600000
#define APP_BARO_HISTORY_NONE      INT16_MIN

    typedef struct
    {
        int16_t altitude_m[APP_BARO_HISTORY_POINTS]; /* 由气压换算，最旧在左；缺测为NONE。 */
        uint32_t sequence;
    } app_baro_history_struct;

    /* 一次快照的三件套: 补偿后气压 + 两个原始计数 (原始值留给高度换算/滤波复用) */
    typedef struct
    {
        int32_t pressure_pa; /* 温度补偿后气压 (Pa) */
        float alt_m;         /* 离海平面高度 (m), 标准气压公式; 随天气漂移, 非真高 */
        int32_t press_raw;   /* 原始压力计数 (24bit 补码) */
        int32_t temp_raw;    /* 原始温度计数 (24bit 补码) */
    } app_baro_struct;

    /**
 * @brief 创建数据锁 (须在任务上下文调用, 调度器已启动; 同 App_IMU_Init)
 */
    void App_Baro_Init(void);

    /**
 * @brief 周期服务一轮: 读压力/温度原始值并补偿, 更新快照, 维护在线状态
 *        仅sensor调用：启动及每5分钟的约1.6s窗口结束后读取，SPORT暂停。
 */
    void App_Baro_Service(void);
    /* sensor在采样尝试完成后调用；十分钟一柱，失败保留空位，独立于页面生命周期。 */
    void app_baro_record_history(void);
    void app_baro_get_history(app_baro_history_struct *history);

    /**
 * @brief 取最新数据的一致快照 (加锁, 防多字段撕裂)
 * @param baro 输出快照, 不可为 NULL
 * @return 1=快照有效; 0=器件离线或首帧未完成
 */
    uint8_t App_Baro_GetSnapshot(app_baro_struct *baro);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BARO_H */
