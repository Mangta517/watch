/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   应用层-MPU6050 + LSM303DLHC 数据与姿态服务
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          App_IMU_Init()/App_IMU_Service() 由 sensor 任务体调用,
  *          App_IMU_GetSnapshot() 供 UI 等消费方在自己的任务上下文取数。
  *          并发模式: 单生产者(sensor 任务)写 → 互斥锁保护快照 → 消费者加锁取。
  *          不使用 MPU6050 片内温度: 突发读15字节包含状态+加速度+温度+陀螺仪
  *          (温度寄存器夹在中间一起读出, 但不参与显示), 省一次独立温度事务。
  ******************************************************************************
***/
#include "app_imu.h"
#include "app_diag.h"
#include "app_heart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h" /* xTaskGetTickCount / configTICK_RATE_HZ: dt 实测 */

#include "common_imu.h" /* Common_IMU_GetEulerAngle 四元数姿态融合 */
#include "app_mag_store.h"
#include "bl24c02.h"
#include <stdlib.h> /* abs: 静止判定三轴比较 */
#include <math.h>
#include <string.h>

/*********************
 *  参数
 *********************/
/* 连续 N 次读失败才判离线: 单次 NACK 可能只是总线瞬态,
 * 不宜立即翻 OFFLINE (设备端表现为数据闪断一下又恢复, 易误判为 bug) */
#define APP_IMU_OFFLINE_STREAK 3

/* dt 实测限幅 (秒): 解算积分步长宁可用错值的邻域也不要跳变,
 * 下限挡两拍合并 (tick 同值), 上限挡任务被长时间抢占后的积分暴走 */
#define APP_IMU_DT_MIN_S  0.002f
#define APP_IMU_DT_MAX_S  0.050f
#define APP_IMU_DT_INIT_S 0.010f /* 首拍兜底值 = 任务标称周期 */

/* ---- 陀螺仪残余零漂抑制 ----
 * 上电 zero_offset 只吃掉初始零偏；温漂仍可能引起姿态误差，
 * 磁参考失效时航向尤其容易漂移，故严格静止时继续做增量微调:
 *   静止判据: 三轴角速度均小于约0.18°/s，LSB门限随配置灵敏度换算
 *   先稳定 APP_IMU_STILL_SETTLE 拍 (~1s) 才开始累积, 防放下的瞬间被当静止
 *   累积 APP_IMU_TRIM_WINDOW 拍取平均 (抵消白噪声), 一次性喂 gyro_trim
 *   检测到任何转动立即复位全部计数 —— 宁可漏校不误校 */
#define APP_IMU_STILL_GYRO_LSB ((int16_t)(0.18f * MPU6050_GYRO_LSB_PER_DPS + 0.5f))
#define APP_IMU_STILL_SETTLE   100 /* 10ms×100 = 1s */
#define APP_IMU_TRIM_WINDOW    100 /* 10ms×100 = 1s 一轮平均 */

#define APP_MAG_POLL_MS         10
#define APP_MAG_MAX_AGE_MS      50
#define APP_MAG_CAL_SAMPLES     100
#define APP_MAG_CAL_MIN_SPAN_UT 20.0f

#define APP_MAG_CMD_START  1 /* 开始旋转采集 */
#define APP_MAG_CMD_FINISH 2 /* 检查覆盖范围并应用校准 */

/* 用户确认两颗器件同轴同向：校准后将磁场映射到 MPU6050 X/Y/Z。 */
static const uint8_t s_mag_axis[3] = {0, 1, 2};
static const float s_mag_sign[3] = {1.0f, 1.0f, 1.0f};

/**********************
 *  STATIC VARIABLES
 **********************/
static SemaphoreHandle_t s_imu_mux; /* 保护完整快照与校准命令，不包住I2C事务 */
static app_imu_struct s_imu_data;   /* 最新快照 (锁内赋值) */
static uint8_t s_online;            /* 1=快照有效 */
static uint8_t s_err_streak;        /* 连续读失败计数 (仅 Service 读写, 不跨任务) */

/* 姿态解算递推状态: 四元数在 common_imu 内部 static;
 * 只允许 sensor 任务触碰 (单生产者), 进快照前无并发问题 */
static Euler_struct s_euler = {0};

static lsm303dlhc_mag_struct s_mag;
static TickType_t s_mag_poll_tick, s_mag_sample_tick;
static uint8_t s_mag_valid;
static uint8_t s_mag_cal_request; /* 仅在快照锁内跨任务传递 */
static app_mag_cal_state s_mag_cal_state;
static uint8_t s_mag_calibrated;
static float s_mag_offset[3];
static float s_mag_scale[3] = {1.0f, 1.0f, 1.0f};
static float s_mag_min[3], s_mag_max[3];
static uint16_t s_mag_cal_samples;
static uint8_t s_mag_save_pending;
static app_mag_persist_state s_mag_persist_state;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void imu_lock(void);
static void imu_unlock(void);
static void imu_still_gyro_trim(const Accel_Gyro_struct *raw);

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 初始化失败交给 Error_Handler；首帧前的 UI 查询在 GetSnapshot 内返回空快照。 */
static void imu_lock(void)
{
    if (s_imu_mux != NULL)
    {
        xSemaphoreTake(s_imu_mux, portMAX_DELAY);
    }
}

static void imu_unlock(void)
{
    if (s_imu_mux != NULL)
    {
        xSemaphoreGive(s_imu_mux);
    }
}

/* 静止检测 + 陀螺仪零点微调: 仅 sensor 任务上下文调用 (单生产者, 无共享状态竞争) */
static void imu_still_gyro_trim(const Accel_Gyro_struct *raw)
{
    static uint8_t still_cnt = 0;
    static int32_t trim_sum_x = 0, trim_sum_y = 0, trim_sum_z = 0;
    static uint16_t trim_cnt = 0;

    if (s_mag_cal_state != APP_MAG_CAL_RUNNING && abs(raw->Gyro.Gyro_x) < APP_IMU_STILL_GYRO_LSB &&
        abs(raw->Gyro.Gyro_y) < APP_IMU_STILL_GYRO_LSB && abs(raw->Gyro.Gyro_z) < APP_IMU_STILL_GYRO_LSB)
    {
        if (still_cnt < APP_IMU_STILL_SETTLE)
        {
            still_cnt++;
        }
        if (still_cnt >= APP_IMU_STILL_SETTLE) /* 静止稳定 ~1s 后才开始累积 */
        {
            trim_sum_x += raw->Gyro.Gyro_x;
            trim_sum_y += raw->Gyro.Gyro_y;
            trim_sum_z += raw->Gyro.Gyro_z;
            trim_cnt++;
            if (trim_cnt >= APP_IMU_TRIM_WINDOW) /* 攒满一轮取平均, 抵消静止白噪声 */
            {
                my_mpu6050_gyro_trim((int16_t)(trim_sum_x / APP_IMU_TRIM_WINDOW), (int16_t)(trim_sum_y / APP_IMU_TRIM_WINDOW),
                                     (int16_t)(trim_sum_z / APP_IMU_TRIM_WINDOW));
                trim_sum_x = trim_sum_y = trim_sum_z = 0;
                trim_cnt = 0;
            }
        }
    }
    else
    {
        /* 检测到转动: 全部计数复位重来, 防止把真实动作当成偏置误吃掉 */
        still_cnt = 0;
        trim_sum_x = trim_sum_y = trim_sum_z = 0;
        trim_cnt = 0;
    }
}

/* 三轴旋转的 min/max 估计安装硬铁偏置和各轴比例；所有状态仅 sensor 写。 */
static void mag_cal_command(uint8_t command)
{
    uint8_t i;
    float radius[3], average;

    if (command == APP_MAG_CMD_START) /* 保留上次已生效的校准系数 */
    {
        s_mag_cal_samples = 0;
        s_mag_cal_state = APP_MAG_CAL_RUNNING;
    }
    else if (command == APP_MAG_CMD_FINISH && s_mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        s_mag_cal_state = APP_MAG_CAL_FAILED;
        if (s_mag_cal_samples < APP_MAG_CAL_SAMPLES)
        {
            return;
        }

        /* 各轴都覆盖足够幅度后，才允许更新整组系数。 */
        for (i = 0; i < 3; ++i)
        {
            radius[i] = (s_mag_max[i] - s_mag_min[i]) * 0.5f;
            if (radius[i] < APP_MAG_CAL_MIN_SPAN_UT * 0.5f)
            {
                return;
            }
        }
        average = (radius[0] + radius[1] + radius[2]) / 3.0f;
        for (i = 0; i < 3; ++i)
        {
            s_mag_offset[i] = (s_mag_max[i] + s_mag_min[i]) * 0.5f;
            s_mag_scale[i] = average / radius[i];
        }
        s_mag_calibrated = 1;
        s_mag_cal_state = APP_MAG_CAL_DONE;
        s_mag_save_pending = 1;
        s_mag_persist_state = APP_MAG_PERSIST_SAVING;
    }
}

static uint8_t mag_service(TickType_t now, float body_uT[3])
{
    lsm303dlhc_mag_result result;
    uint8_t i, command;
    float corrected[3];

    /* 取走UI命令并清零；后续采集和计算在快照锁外执行。 */
    imu_lock();
    command = s_mag_cal_request;
    s_mag_cal_request = 0;
    imu_unlock();
    mag_cal_command(command);

    if ((TickType_t)(now - s_mag_poll_tick) >= pdMS_TO_TICKS(APP_MAG_POLL_MS))
    {
        s_mag_poll_tick = now;
        uint32_t read_started = app_diag_clock_count();
        result = lsm303dlhc_mag_read(&s_mag);
        app_diag_sensor_record(APP_DIAG_MAG, (uint8_t)result, (uint32_t)result, read_started);
        if (result == LSM303DLHC_MAG_OK)
        {
            s_mag_sample_tick = xTaskGetTickCount();
            s_mag_valid = 1;
            if (s_mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                for (i = 0; i < 3; ++i)
                {
                    if (s_mag_cal_samples == 0 || s_mag.field_uT[i] < s_mag_min[i])
                    {
                        s_mag_min[i] = s_mag.field_uT[i];
                    }
                    if (s_mag_cal_samples == 0 || s_mag.field_uT[i] > s_mag_max[i])
                    {
                        s_mag_max[i] = s_mag.field_uT[i];
                    }
                }
                if (s_mag_cal_samples < UINT16_MAX)
                {
                    ++s_mag_cal_samples;
                }
            }
        }
        else if (result == LSM303DLHC_MAG_ERROR)
        {
            s_mag_valid = 0; /* NACK/溢出时立即停磁修正，不能继续喂旧帧 */
        }
    }
    if ((TickType_t)(xTaskGetTickCount() - s_mag_sample_tick) > pdMS_TO_TICKS(APP_MAG_MAX_AGE_MS))
    {
        s_mag_valid = 0;
    }

    /* 先在磁芯片坐标下去偏置/缩放，再映射到MPU6050机体坐标。 */
    for (i = 0; i < 3; ++i)
    {
        corrected[i] = (s_mag.field_uT[i] - s_mag_offset[i]) * s_mag_scale[i];
    }
    for (i = 0; i < 3; ++i)
    {
        body_uT[i] = corrected[s_mag_axis[i]] * s_mag_sign[i];
    }
    return s_mag_valid;
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_IMU_Init(void)
{
    app_mag_calibration_struct calibration;
    app_mag_store_result result;
    uint8_t i;
    /* mutex 在任务上下文创建 (与 app_display 的 LVGL 锁同理): 调度器未启动时
     * CM4F port 的临界区计数不真正解除 BASEPRI, 建锁留给任务里做才干净 */
    s_imu_mux = xSemaphoreCreateMutex();
    if (s_imu_mux == NULL)
    {
        Error_Handler();
    }
    Common_IMU_Reset();

    /* EEPROM与传感器均由sensor任务访问；读取在快照锁外执行。 */
    bl24c02_init();
    result = app_mag_store_load(&calibration);
    if (result == APP_MAG_STORE_OK)
    {
        for (i = 0; i < 3; ++i)
        {
            s_mag_offset[i] = calibration.offset_uT[i];
            s_mag_scale[i] = calibration.scale[i];
        }
        s_mag_calibrated = 1;
        s_mag_cal_state = APP_MAG_CAL_DONE;
        s_mag_persist_state = APP_MAG_PERSIST_RESTORED;
    }
    else
    {
        if (result == APP_MAG_STORE_ERROR)
        {
            s_mag_persist_state = APP_MAG_PERSIST_LOAD_FAILED;
        }
        else
        {
            s_mag_persist_state = APP_MAG_PERSIST_NONE;
        }
    }
}

void App_IMU_Service(void)
{
    Accel_Gyro_struct raw;
    TickType_t now = xTaskGetTickCount();
    static TickType_t s_last;
    static uint8_t s_first = 1;
    float dt, mag_body[3], field_squared;
    const float *mag_reference = NULL;
    uint8_t mag_online, mag_fused = 0;
    uint8_t axis;
    uint8_t read_result;
    mpu6050_cal_state cal_before, cal_after;
    uint32_t read_started;
    app_mag_fusion_state mag_fusion_state = APP_MAG_FUSION_IMU_NO_DATA;

    /* 1. 采样：同一sensor任务先取磁参考，再取MPU帧，避免总线竞争。 */
    mag_online = mag_service(now, mag_body);
    field_squared = mag_body[0] * mag_body[0] + mag_body[1] * mag_body[1] + mag_body[2] * mag_body[2];
    /* 15 字节突发读同时释放 MPU INT 锁存，无 ISR 总线竞争。 */
    read_started = app_diag_clock_count();
    read_result = my_mpu6050_get_data(&raw);
    if (read_result == 0)
    {
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_OK, read_result, read_started);
    }
    else if (read_result == MPU6050_DATA_NOT_READY)
    {
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_NOT_READY, read_result, read_started);
    }
    else
    {
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_ERROR, read_result, read_started);
    }
    cal_before = my_mpu6050_zero_offset_state();
    cal_after = my_mpu6050_zero_offset_step(&raw, read_result == 0, HAL_GetTick());
    app_diag_mpu_cal_record(my_mpu6050_zero_offset_diag());
    if (read_result == 0)
    {
        app_heart_motion_update(raw.Accel.Accel_x, raw.Accel.Accel_y, raw.Accel.Accel_z, HAL_GetTick());
        /* dt = 距上次有效更新的真实间隔 (flight 端同款约定: 实测+限幅, 不硬编码) */
        if (s_first)
        {
            s_first = 0;
            dt = APP_IMU_DT_INIT_S;
        }
        else
        {
            dt = (float)(now - s_last) / (float)configTICK_RATE_HZ;
            if (dt < APP_IMU_DT_MIN_S)
            {
                dt = APP_IMU_DT_MIN_S;
            }
            if (dt > APP_IMU_DT_MAX_S)
            {
                dt = APP_IMU_DT_MAX_S;
            }
        }
        s_last = now;
        if (cal_after == MPU6050_CAL_DONE && cal_before == MPU6050_CAL_AVERAGING)
        {
            Common_IMU_Reset();
        }
        /* 2. 处理：静止零偏微调，再做四元数融合；25～10000为5～100μT的平方。 */
        if (cal_before != MPU6050_CAL_WAIT_STILL && cal_before != MPU6050_CAL_AVERAGING)
        {
            imu_still_gyro_trim(&raw);
        }
        if (!mag_online)
        {
            mag_fusion_state = APP_MAG_FUSION_NO_DATA;
        }
        else if (s_mag_cal_state == APP_MAG_CAL_RUNNING)
        {
            mag_fusion_state = APP_MAG_FUSION_CALIBRATING;
        }
        else if (!(field_squared >= 25.0f && field_squared <= 10000.0f))
        {
            mag_fusion_state = APP_MAG_FUSION_FIELD_RANGE;
        }
        else
        {
            mag_reference = mag_body;
        }
        mag_fused = Common_IMU_GetEulerAngleWithMag(&raw, mag_reference, &s_euler, dt);
        if (mag_reference != NULL)
        {
            if (mag_fused)
            {
                mag_fusion_state = APP_MAG_FUSION_ACTIVE;
            }
            else
            {
                mag_fusion_state = APP_MAG_FUSION_HORIZONTAL_WEAK;
            }
        }

        s_err_streak = 0;
    }
    else if (read_result != MPU6050_DATA_NOT_READY && s_err_streak < APP_IMU_OFFLINE_STREAK)
    {
        ++s_err_streak;
    }
    /* 3. 发布：六轴、姿态和磁状态在同一保护区更新，UI只读一致快照。 */
    imu_lock();
    if (read_result == 0)
    {
        s_imu_data.raw = raw;
        s_imu_data.pitch_deg = s_euler.pitch;
        s_imu_data.roll_deg = s_euler.roll;
        s_imu_data.yaw_deg = s_euler.yaw;
        s_online = 1;
    }
    else if (s_err_streak >= APP_IMU_OFFLINE_STREAK ||
             (s_first == 0 && (TickType_t)(now - s_last) >= pdMS_TO_TICKS(50U)))
    {
        s_online = 0;
    }
    s_imu_data.mag = s_mag;
    memcpy(s_imu_data.mag_body_uT, mag_body, sizeof(mag_body));
    s_imu_data.mag_online = mag_online;
    s_imu_data.mag_fused = mag_fused;
    s_imu_data.mag_calibrated = s_mag_calibrated;
    s_imu_data.mag_cal_state = s_mag_cal_state;
    s_imu_data.mag_fusion_state = mag_fusion_state;
    s_imu_data.mag_cal_samples = s_mag_cal_samples;
    s_imu_data.mag_persist_state = s_mag_persist_state;
    for (axis = 0; axis < 3; ++axis)
    {
        if (s_mag_cal_samples)
        {
            s_imu_data.mag_cal_span_uT[axis] = s_mag_max[axis] - s_mag_min[axis];
        }
        else
        {
            s_imu_data.mag_cal_span_uT[axis] = 0.0f;
        }
    }
    imu_unlock();

    /* 一次成功校准只保存一次，采样/融合完成后写EEPROM，UI读取不被数据锁阻塞。 */
    if (s_mag_save_pending)
    {
        app_mag_calibration_struct calibration;
        app_mag_store_result result;
        s_mag_save_pending = 0;
        for (axis = 0; axis < 3; ++axis)
        {
            calibration.offset_uT[axis] = s_mag_offset[axis];
            calibration.scale[axis] = s_mag_scale[axis];
        }
        result = app_mag_store_save(&calibration);
        if (result == APP_MAG_STORE_OK)
        {
            s_mag_persist_state = APP_MAG_PERSIST_SAVED;
        }
        else
        {
            s_mag_persist_state = APP_MAG_PERSIST_SAVE_FAILED;
        }
        imu_lock();
        s_imu_data.mag_persist_state = s_mag_persist_state;
        imu_unlock();
    }
}

uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)
{
    uint8_t online;

    if (imu == NULL)
    {
        return 0;
    }

    if (s_imu_mux == NULL)
    {
        memset(imu, 0, sizeof(*imu));
        return 0;
    }
    imu_lock();
    *imu = s_imu_data;
    online = s_online;
    imu_unlock();

    return online;
}

void App_IMU_MagCalibrate(uint8_t start)
{
    if (s_imu_mux == NULL)
    {
        return;
    }
    imu_lock();
    if (start)
    {
        s_mag_cal_request = APP_MAG_CMD_START;
    }
    else
    {
        s_mag_cal_request = APP_MAG_CMD_FINISH;
    }
    imu_unlock();
}
