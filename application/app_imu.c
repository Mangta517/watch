/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   应用层-MPU6050 + LSM303DLHC 数据与姿态服务
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          App_IMU_Init()/App_IMU_Service() 由 sensor 任务体调用,
  *          App_IMU_GetSnapshot() 供 UI 等消费方在自己的任务上下文取数。
  *          并发模式: 单生产者(sensor 任务)写 → 互斥锁保护快照 → 消费者加锁取。
  *          MPU6050按使用状态50/25Hz轮询并递推四元数，欧拉角最多50Hz。
  ******************************************************************************
***/
#include "app_imu.h"
#include "app_diag.h"
#include "app_heart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h" /* 磁力计轮询与数据年龄使用RTOS tick */

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

/* 初始化断采门限50ms；运行时为当前周期的5倍，恢复首帧按标称周期积分。 */
#define APP_IMU_MAX_GAP_MS 50U
#define APP_IMU_EULER_PERIOD_MS 20U /* 每帧递推四元数，显示角度最多50Hz。 */

/* ---- 陀螺仪残余零漂抑制 ----
 * 上电 zero_offset 只吃掉初始零偏；温漂仍可能引起姿态误差，
 * 磁参考失效时航向尤其容易漂移，故严格静止时继续做增量微调:
 *   静止判据: 三轴角速度均小于约0.18°/s，LSB门限随配置灵敏度换算
 *   先稳定 APP_IMU_STILL_SETTLE 拍 (50/25Hz下约2/4s) 才开始累积
 *   累积 APP_IMU_TRIM_WINDOW 拍取平均 (抵消白噪声), 一次性喂 gyro_trim
 *   检测到任何转动立即复位全部计数 —— 宁可漏校不误校 */
#define APP_IMU_STILL_GYRO_LSB ((int16_t)(0.18f * MPU6050_GYRO_LSB_PER_DPS + 0.5f))
#define APP_IMU_STILL_SETTLE   100 /* 50/25Hz下约2/4s。 */
#define APP_IMU_TRIM_WINDOW    100 /* 保留100帧平均，低速时延长观察窗口。 */

/* 初始化兼容75Hz/10ms；运行时活动30Hz/40ms、后台7.5Hz/160ms。
 * 各档最多沿用120/400ms内的有效帧，超期停磁修正。
 * 连续N次读失败才判磁离线: 单次NACK可能只是总线瞬态，
 * 立即翻MAG NO DATA会让触摸/刷屏期间的数据看起来"消失"。 */
#define APP_MAG_POLL_MS         10
#define APP_MAG_MAX_AGE_MS      50
#define APP_MAG_CAL_SAMPLES     100
#define APP_MAG_CAL_MIN_SPAN_UT 20.0f
#define APP_MAG_OFFLINE_STREAK  3

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
static uint32_t s_last_sample_ms;   /* 最近有效MPU帧的HAL读取时刻 */
static uint8_t s_have_sample;      /* 是否可用上一帧时刻计算解算间隔 */
static uint32_t s_euler_ms;
static uint8_t s_have_euler;
static uint32_t s_sample_period_ms = MPU6050_SAMPLE_PERIOD_MS;
static uint32_t s_max_gap_ms = APP_IMU_MAX_GAP_MS;
static uint32_t s_mag_poll_ms = APP_MAG_POLL_MS;
static uint32_t s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;

/* 姿态解算递推状态: 四元数在 common_imu 内部 static;
 * 只允许 sensor 任务触碰 (单生产者), 进快照前无并发问题 */
static Euler_struct s_euler = {0};

static lsm303dlhc_mag_struct s_mag;
static TickType_t s_mag_poll_tick, s_mag_sample_tick;
static uint8_t s_mag_valid;
static uint8_t s_mag_err_streak;  /* 连续磁读失败计数 (仅 sensor 任务读写) */
static uint8_t s_mag_cal_request; /* 仅在快照锁内跨任务传递 */
static app_mag_cal_state s_mag_cal_state;
static uint8_t s_mag_calibrated;
static float s_mag_offset[3];
static float s_mag_scale[3] = {1.0f, 1.0f, 1.0f};
static float s_mag_min[3], s_mag_max[3];
static uint16_t s_mag_cal_samples;
static uint8_t s_mag_save_pending;
static uint8_t s_mag_saving;
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

    if ((TickType_t)(now - s_mag_poll_tick) >= pdMS_TO_TICKS(s_mag_poll_ms))
    {
        s_mag_poll_tick = now;
        uint32_t read_started = app_diag_clock_count();
        result = lsm303dlhc_mag_read(&s_mag);
        app_diag_sensor_record(APP_DIAG_MAG, (uint8_t)result, (uint32_t)result, read_started);
        if (result == LSM303DLHC_MAG_OK)
        {
            s_mag_err_streak = 0;
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
            if (s_mag_err_streak < APP_MAG_OFFLINE_STREAK)
            {
                ++s_mag_err_streak;
            }
            if (s_mag_err_streak >= APP_MAG_OFFLINE_STREAK)
            {
                s_mag_valid = 0; /* 连败才停磁修正，单次失败仍用最近一帧，由新鲜度门限兜底 */
            }
        }
    }
    if ((TickType_t)(xTaskGetTickCount() - s_mag_sample_tick) > pdMS_TO_TICKS(s_mag_max_age_ms))
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
    s_last_sample_ms = 0;
    s_have_euler = 0;
    s_have_sample = 0;
    s_online = 0;
    s_err_streak = 0;

    /* EEPROM与传感器均由sensor任务访问；读取在快照锁外执行。 */
    s_sample_period_ms = MPU6050_SAMPLE_PERIOD_MS;
    s_max_gap_ms = APP_IMU_MAX_GAP_MS;
    s_mag_poll_ms = APP_MAG_POLL_MS;
    s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;
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

uint32_t app_imu_set_sample_period(uint32_t period_ms)
{
    uint8_t calibrating;
    uint16_t mag_rate = 300U;
    imu_lock();
    calibrating = s_mag_cal_state == APP_MAG_CAL_RUNNING || s_mag_cal_request == APP_MAG_CMD_START;
    imu_unlock();
    if (calibrating != 0 || period_ms != 40U)
    {
        period_ms = 20U;
    }
    if (my_mpu6050_set_sample_period(period_ms) != 0)
    {
        period_ms = 20U; /* 重配置期间继续推进其他服务，不阻塞任务。 */
    }
    if (period_ms != s_sample_period_ms)
    {
        s_sample_period_ms = period_ms;
        s_max_gap_ms = period_ms * 5U;
        s_have_sample = s_have_euler = 0;
    }
    if (period_ms == 40U)
    {
        mag_rate = 75U;
    }
    if (lsm303dlhc_mag_set_rate(mag_rate) == 0)
    {
        s_mag_poll_ms = 40U;
        s_mag_max_age_ms = 120U;
        if (mag_rate == 75U)
        {
            s_mag_poll_ms = 160U;
            s_mag_max_age_ms = 400U;
        }
    }
    return period_ms;
}

void App_IMU_Service(void)
{
    Accel_Gyro_struct raw;
    TickType_t now = xTaskGetTickCount();
    float mag_body[3], field_squared, dt;
    const float *mag_reference = NULL;
    Euler_struct *euler_output = NULL;
    uint8_t mag_online, mag_fused = 0;
    uint8_t axis, read_result;
    uint32_t sample_ms, elapsed_ms, read_started;
    mpu6050_cal_state cal_before, cal_after;
    app_mag_fusion_state mag_fusion_state = APP_MAG_FUSION_IMU_NO_DATA;

    /* 1. 同一sensor任务先取磁参考，再以一笔事务轮询MPU最新帧。 */
    mag_online = mag_service(now, mag_body);
    field_squared = mag_body[0] * mag_body[0] + mag_body[1] * mag_body[1] + mag_body[2] * mag_body[2];
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
        sample_ms = my_mpu6050_sample_ms();
        elapsed_ms = s_sample_period_ms;
        if (s_have_sample != 0)
        {
            elapsed_ms = sample_ms - s_last_sample_ms;
            if (elapsed_ms == 0 || elapsed_ms >= s_max_gap_ms)
            {
                elapsed_ms = s_sample_period_ms;
            }
        }
        dt = (float)elapsed_ms / 1000.0f;
        s_last_sample_ms = sample_ms;
        s_have_sample = 1;
        app_heart_motion_update(raw.Accel.Accel_x, raw.Accel.Accel_y, raw.Accel.Accel_z, sample_ms);
        if (cal_after == MPU6050_CAL_DONE && cal_before == MPU6050_CAL_AVERAGING)
        {
            Common_IMU_Reset();
            s_have_euler = 0;
        }
        /* 2. 仅新帧参与零漂微调与姿态解算；短时调度抖动使用实际读帧间隔。 */
        if (cal_before != MPU6050_CAL_WAIT_STILL && cal_before != MPU6050_CAL_AVERAGING)
        {
            imu_still_gyro_trim(&raw);
        }
        /* 磁新鲜度统一使用RTOS tick；常规轮询不再与FIFO历史帧配对。 */
        if (!mag_online || (TickType_t)(xTaskGetTickCount() - s_mag_sample_tick) > pdMS_TO_TICKS(s_mag_max_age_ms))
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
        if (s_have_euler == 0 || (uint32_t)(sample_ms - s_euler_ms) >= APP_IMU_EULER_PERIOD_MS)
        {
            euler_output = &s_euler;
            s_euler_ms = sample_ms;
            s_have_euler = 1;
        }
        mag_fused = Common_IMU_GetEulerAngleWithMag(&raw, mag_reference, euler_output, dt);
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
        s_online = (uint32_t)(HAL_GetTick() - s_last_sample_ms) < s_max_gap_ms;
    }
    else if (s_err_streak >= APP_IMU_OFFLINE_STREAK ||
             (s_online != 0 && (uint32_t)(HAL_GetTick() - s_last_sample_ms) >= s_max_gap_ms))
    {
        s_online = 0;
        s_have_sample = 0;
        s_have_euler = 0;
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
}

uint8_t App_IMU_SaveService(void)
{
    app_mag_store_result result;
    uint8_t axis;
    if (s_mag_saving == 0)
    {
        app_mag_calibration_struct calibration;
        if (s_mag_save_pending == 0)
        {
            return 0;
        }
        s_mag_save_pending = 0;
        for (axis = 0; axis < 3; ++axis)
        {
            calibration.offset_uT[axis] = s_mag_offset[axis];
            calibration.scale[axis] = s_mag_scale[axis];
        }
        result = app_mag_store_save_begin(&calibration);
    }
    else
    {
        result = app_mag_store_save_step();
    }
    s_mag_saving = result == APP_MAG_STORE_BUSY;
    if (s_mag_saving == 0)
    {
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
    return s_mag_saving;
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
