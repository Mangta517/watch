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

#define APP_BARO_HISTORY_POINTS    24      /* 高度柱状历史点数，最旧在左 */
#define APP_BARO_HISTORY_PERIOD_MS 600000  /* 每柱间隔，毫秒(10分钟) */
#define APP_BARO_HISTORY_NONE      INT16_MIN /* 缺测柱标记：该十分钟没有有效读数 */

    typedef struct
    {
        int16_t altitude_m[APP_BARO_HISTORY_POINTS]; /* 由气压换算，最旧在左；缺测为NONE。 */
        uint32_t sequence;
    } app_baro_history_struct;

    typedef enum
    {
        APP_BARO_CAL_NONE,
        APP_BARO_CAL_PENDING,
        APP_BARO_CAL_APPLIED,
        APP_BARO_CAL_FAILED
    } app_baro_calibration_state;

    /* 气压快照: 补偿气压、换算海拔、两路原始计数，以及参考气压/温补/校准状态 */
    typedef struct
    {
        int32_t pressure_pa; /* 温度补偿后气压 (Pa) */
        float alt_m;         /* 气压海拔(m)，采用手机参考、环境温度和本次校准偏差。 */
        int32_t press_raw;   /* 原始压力计数 (24bit 补码) */
        int32_t temp_raw;    /* 原始温度计数 (24bit 补码) */
        uint8_t reference_ready; /* 已收到手机海平面气压，可开始已知海拔校准。 */
        uint8_t temperature_corrected; /* 1=使用手机环境温度；0=标准大气模型。 */
        app_baro_calibration_state calibration_state;
    } app_baro_struct;

    /* 硬件生命周期仅由sensor任务推进；set_sleep返回0成功、1失败，恢复结果由本模块记录诊断。 */
    void app_baro_init_hw(void);
    uint8_t app_baro_set_sleep(uint8_t sleeping);
    void app_baro_recover_step(uint32_t now_ms);

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
    /* 加锁取24点高度柱状历史的一致快照；NULL不处理。 */
    void app_baro_get_history(app_baro_history_struct *history);

    /**
 * @brief 取最新数据的一致快照 (加锁, 防多字段撕裂)
 * @param baro 输出快照, 不可为 NULL
 * @return 1=快照有效; 0=器件离线或首帧未完成
 */
    uint8_t App_Baro_GetSnapshot(app_baro_struct *baro);
    /* LVGL同步参考和天气温度(0.1摄氏度)；pressure_pa=0保留当前参考，温度无效时使用标准模型。 */
    void app_baro_set_reference(uint32_t pressure_pa, int16_t temperature_x10, uint8_t temperature_valid);
    /* LVGL提交已知海拔(-1000..9000m)，sensor用下一次新采样校准；0=受理、1=范围错误或尚未同步手机。 */
    uint8_t app_baro_calibrate_altitude(int16_t altitude_m);
    /* 查询是否有待处理的已知海拔校准请求: 1=等待下一次采样校准; 0=无。 */
    uint8_t app_baro_calibration_pending(void);
    /* 清除本次开机的校准偏差和待处理请求，参考气压及环境温度继续使用。 */
    void app_baro_reset_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BARO_H */
