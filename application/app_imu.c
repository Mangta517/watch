/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   应用层-MPU6050 + LSM303DLHC 活动与独立指南针服务
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          App_IMU_Init()/App_IMU_Service() 由 sensor 任务体调用,
  *          App_IMU_GetSnapshot() 供 UI 等消费方在自己的任务上下文取数。
  *          并发模式: 单生产者(sensor 任务)写 → 互斥锁保护快照 → 消费者加锁取。
 *          MPU6050仅计步/抬腕/心率参考，LSM303DLHC独立输出倾斜补偿指南针。
  ******************************************************************************
***/
#include "app_imu.h"
#include "mpu6050.h"
#include "lsm303dlhc.h"
#include "app_diag.h"
#include "app_heart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h" /* 磁力计轮询与数据年龄使用RTOS tick */

#include "common_activity.h"
#include "common_imu.h" /* 加速度倾斜补偿指南针。 */
#include "app_mag_store.h"
#include "app_battery_log.h"
#include "bl24c02.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*********************
 *  参数
 *********************/
/* 连续 N 次读失败才判离线: 单次 NACK 可能只是总线瞬态,
 * 不宜立即翻 OFFLINE (设备端表现为数据闪断一下又恢复, 易误判为 bug) */
#define APP_IMU_OFFLINE_STREAK 3
#define APP_IMU_MOT_IDLE_MS 6000
#define APP_IMU_MOT_FALLBACK_MG 45

#define APP_MAG_POLL_MS            40
#define APP_MAG_MAX_AGE_MS         250
#define APP_MAG_OFFLINE_STREAK     3
#define APP_MAG_LEGACY_MAX_UT      100.0f
#define APP_MAG_UNCAL_MAX_UT       450.0f
#define APP_MAG_GRAVITY_PAIR_MS    60
#define APP_MAG_CONTINUOUS_MS      100
#define APP_MAG_DEFAULT_DECLINATION_DEG (-2.0f)

#define APP_MAG_CMD_START  1 /* 开始旋转采集 */
#define APP_MAG_CMD_FINISH 2 /* 检查覆盖范围并应用校准 */

/* 实板核对：Back板LSM303DLHC芯片朝下(Z向手腕)，+X指向表盘6点，需绕Z转180°(-X,-Y,+Z)对齐12点。
 * 磁场与LSM303加速度共用同一套芯片轴向，映射保持同步以确保倾斜补偿一致。 */
static const uint8_t s_mag_axis[3] = {0, 1, 2};
static const float s_mag_sign[3] = {-1.0f, -1.0f, 1.0f};

/**********************
 *  STATIC VARIABLES
 **********************/
static SemaphoreHandle_t s_imu_mux; /* 保护完整快照与校准命令，不包住I2C事务 */
static app_imu_struct s_imu_data;   /* 最新快照 (锁内赋值) */
static uint8_t s_online;            /* 仅表示MPU加速度在线，指南针使用独立有效标志。 */
static uint8_t s_err_streak;        /* 连续读失败计数 (仅 Service 读写, 不跨任务) */
static uint32_t s_last_sample_ms;   /* 最近有效MPU帧的HAL读取时刻 */
static uint8_t s_have_sample;       /* 当前采样档是否已读取到有效帧。 */
static common_activity_struct s_activity;
static uint32_t s_sample_period_ms = MPU6050_SAMPLE_PERIOD_MS;
static uint8_t s_accel_reference_requested;
static uint32_t s_mag_poll_ms = APP_MAG_POLL_MS;
static uint32_t s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;
static float s_compass_gravity[3], s_compass_gravity_filt[3];
static TickType_t s_accel_sample_tick, s_accel_poll_tick;
static uint8_t s_compass_accel_valid, s_compass_accel_errors;
static uint8_t s_compass_filter_valid, s_compass_gravity_stable;
static float s_activity_accel[3];
static uint8_t s_activity_filter_valid;
static uint32_t s_activity_elapsed_ms;
static Accel_struct s_mot_accel;
static uint32_t s_mot_sample_ms;
static uint8_t s_mot_sample_valid;
static uint16_t s_activity_day, s_requested_day;
static uint8_t s_calendar_restored, s_day_bound;

static lsm303dlhc_mag_struct s_mag;
static TickType_t s_mag_poll_tick, s_mag_sample_tick;
static uint8_t s_mag_valid, s_mag_overflow;
static uint8_t s_mag_field_fault, s_mag_dip_fault, s_mag_clear_frames;
static uint8_t s_mag_err_streak;  /* 连续磁读失败计数 (仅 sensor 任务读写) */
static uint8_t s_mag_cal_request; /* 仅在快照锁内跨任务传递 */
static app_mag_cal_state s_mag_cal_state;
static uint8_t s_mag_calibrated;
static app_mag_calibration_struct s_mag_calibration = {.params.soft_sym = {1.0f, 1.0f, 1.0f}};
static common_mag_cal_struct s_mag_collect;
static float s_mag_declination_deg = APP_MAG_DEFAULT_DECLINATION_DEG; /* 快照锁保护。 */
static uint8_t s_mag_save_pending;
static uint8_t s_mag_saving;
static app_mag_persist_state s_mag_persist_state;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void imu_lock(void);
static void imu_unlock(void);

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 锁创建失败交给Error_Handler，快照中的各传感器独立标记有效性。 */
static void imu_lock(void)
{
    /* 互斥量未建成(初始化前或创建失败已走 Error_Handler)时直接跳过, 避免空句柄取锁。 */
    if (s_imu_mux != NULL)
    {
        xSemaphoreTake(s_imu_mux, portMAX_DELAY); /* 无限等待, 与同一把锁的释放成对使用。 */
    }
}

static void imu_unlock(void)
{
    /* 与 imu_lock 对称: 只有锁对象存在时才归还, 防止空句柄释放。 */
    if (s_imu_mux != NULL)
    {
        xSemaphoreGive(s_imu_mux);
    }
}

/* 校准直到拟合验收后才替换RAM系数并安排EEPROM写入。 */
static void mag_cal_command(uint8_t command)
{
    if (command == APP_MAG_CMD_START) /* 保留上次已生效的校准系数 */
    {
        memset(&s_mag_collect, 0, sizeof(s_mag_collect));
        s_mag_cal_state = APP_MAG_CAL_RUNNING;
    }
    else if (command == APP_MAG_CMD_FINISH && s_mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        /* 成功或失败结束都等待新磁帧，避免沿用校准前的缓存。 */
        s_mag_valid = 0;
        s_mag_poll_tick = xTaskGetTickCount() - pdMS_TO_TICKS(s_mag_poll_ms);
        s_mag_field_fault = s_mag_dip_fault = s_mag_clear_frames = 0;
        s_mag_cal_state = APP_MAG_CAL_FAILED;
        if (common_mag_cal_fit(&s_mag_collect, &s_mag_calibration.params) != 0)
        {
            return;
        }
        s_mag_calibration.is_ellipsoid = 1;
        s_mag_calibrated = 1; /* 已有可用校准系数。 */
        s_mag_cal_state = APP_MAG_CAL_DONE;
        s_mag_save_pending = 1; /* 请求 App_IMU_SaveService 写入 EEPROM。 */
        s_mag_persist_state = APP_MAG_PERSIST_SAVING;
    }
}

static const float *compass_paired_gravity(TickType_t now)
{
    if (s_compass_accel_valid && s_compass_gravity_stable &&
        (TickType_t)(now - s_accel_sample_tick) <= pdMS_TO_TICKS(APP_MAG_GRAVITY_PAIR_MS))
    {
        return s_compass_gravity;
    }
    return NULL;
}

static uint8_t mag_service(TickType_t now, float corrected[3], uint8_t read_mag, uint8_t *new_mag)
{
    lsm303dlhc_mag_result result;
    uint8_t command;
    /* 取走UI命令并清零；后续采集和计算在快照锁外执行。 */
    imu_lock();
    command = s_mag_cal_request;
    s_mag_cal_request = 0;
    imu_unlock();
    /* 在锁外执行命令: 启动清样本, 结束则检查覆盖范围并应用系数。 */
    mag_cal_command(command);

    if (read_mag != 0 && (TickType_t)(now - s_mag_poll_tick) >= pdMS_TO_TICKS(s_mag_poll_ms))
    {
        /* 到达轮询节拍: 记录本次时刻, 并在读取前打诊断时间戳。 */
        s_mag_poll_tick = now;
        uint32_t read_started = app_diag_clock_count();
        /* 发起一次磁读取, 把结果连同耗时起点记入诊断。 */
        result = lsm303dlhc_mag_read(&s_mag);
        app_diag_sensor_record(APP_DIAG_MAG, (uint8_t)result, (uint32_t)result, read_started);
        if (result == LSM303DLHC_MAG_OK)
        {
            uint8_t gap = !s_mag_valid || (TickType_t)(now - s_mag_sample_tick) > pdMS_TO_TICKS(APP_MAG_CONTINUOUS_MS);
            if (gap)
            {
                s_mag_clear_frames = 0;
            }
            *new_mag = 1;
            s_mag_err_streak = 0;
            s_mag_sample_tick = now;
            s_mag_valid = 1;
            s_mag_overflow = 0;
            if (s_mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                common_mag_cal_collect(&s_mag_collect, s_mag.field_uT, compass_paired_gravity(now), gap);
            }
        }
        else if (result == LSM303DLHC_MAG_OVERFLOW)
        {
            s_mag_valid = 0;
            s_mag_overflow = 1;
            s_mag_err_streak = s_mag_clear_frames = 0;
            s_mag_sample_tick = now;
            s_mag_field_fault = s_mag_calibration.is_ellipsoid;
        }
        else if (result == LSM303DLHC_MAG_ERROR)
        {
            s_mag_clear_frames = 0;
            /* 硬错误: 累加连续失败计数, 达到门限才判磁数据无效。 */
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
    if ((TickType_t)(now - s_mag_sample_tick) > pdMS_TO_TICKS(s_mag_max_age_ms))
    {
        s_mag_valid = 0; /* 最近有效帧超过年龄上限: 即使无新错误也停用磁数据。 */
        s_mag_overflow = 0;
    }

    common_mag_cal_apply(&s_mag_calibration.params, s_mag.field_uT, corrected);
    return s_mag_valid;
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void app_imu_init_hw(void)
{
    app_diag_sensor_init(APP_DIAG_MPU, my_mpu6050_init());
}

void app_imu_init_mag_hw(void)
{
    app_diag_sensor_init(APP_DIAG_MAG, lsm303dlhc_mag_init_mode(1));
    lsm303dlhc_accel_recover_begin();
}

uint8_t app_imu_recover_step(uint32_t now_ms)
{
    uint8_t result = my_mpu6050_recover_step(now_ms);
    app_diag_sensor_recovery(APP_DIAG_MPU, result);
    return result;
}

void app_imu_mag_recover_step(uint32_t now_ms)
{
    app_diag_sensor_recovery(APP_DIAG_MAG, lsm303dlhc_mag_recover_step(now_ms));
}

void app_imu_calibration_begin(uint32_t now_ms)
{
    my_mpu6050_zero_offset_begin(now_ms);
}

mpu6050_cal_state app_imu_calibration_progress(uint16_t *stable, uint16_t *average)
{
    my_mpu6050_zero_offset_progress(stable, average);
    return my_mpu6050_zero_offset_state();
}

void app_imu_restore_offset(const mpu6050_offset_struct *offset)
{
    my_mpu6050_restore_offset(offset);
}

void app_imu_get_offset(mpu6050_offset_struct *offset)
{
    my_mpu6050_get_offset(offset);
}

/* 建快照锁并复位活动状态；MPU6050、LSM303DLHC已由sensor启动流程初始化。
 * 磁校准记录在本函数内读取，成功则直接进入DONE/RESTORED，失败只记持久化状态。 */
void App_IMU_Init(void)
{
    app_mag_store_result result;
    /* mutex 在任务上下文创建 (与 app_display 的 LVGL 锁同理): 调度器未启动时
     * CM4F port 的临界区计数不真正解除 BASEPRI, 建锁留给任务里做才干净 */
    SemaphoreHandle_t mux = xSemaphoreCreateMutex();
    if (mux == NULL)
    {
        Error_Handler();
        return;
    }
    /* 活动检测和MPU读帧状态回到上电初值。 */
    memset(&s_activity, 0, sizeof(s_activity));
    s_last_sample_ms = 0;
    s_have_sample = 0;
    s_online = 0;
    s_err_streak = 0;
    s_imu_data.compass_valid = 0;

    /* EEPROM与传感器均由sensor任务访问；读取在快照锁外执行。 */
    /* 采样周期、加速度参考和磁新鲜度参数回到默认档。 */
    s_sample_period_ms = MPU6050_SAMPLE_PERIOD_MS;
    s_accel_reference_requested = 0;
    s_mag_poll_ms = APP_MAG_POLL_MS;
    s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;
    s_compass_accel_valid = s_compass_filter_valid = s_activity_filter_valid = 0;
    s_activity_elapsed_ms = 0;
    s_day_bound = 0;
    app_battery_log_steps_load(&s_activity_day, &s_activity.steps);
    s_imu_data.steps = s_activity.steps;
    /* 初始化EEPROM并读取上次保存的磁校准记录。 */
    bl24c02_init();
    result = app_mag_store_load(&s_mag_calibration);
    if (result == APP_MAG_STORE_OK)
    {
        s_mag_calibrated = 1;
        s_mag_cal_state = APP_MAG_CAL_DONE;
        s_mag_persist_state = APP_MAG_PERSIST_RESTORED;
    }
    else
    {
        /* 未读到记录: 区分读取错误与从未保存过, 供UI显示不同持久化状态。 */
        if (result == APP_MAG_STORE_ERROR)
        {
            s_mag_persist_state = APP_MAG_PERSIST_LOAD_FAILED;
        }
        else
        {
            s_mag_persist_state = APP_MAG_PERSIST_NONE;
        }
    }
    taskENTER_CRITICAL();
    s_imu_mux = mux;
    taskEXIT_CRITICAL();
}

/* 校准命令、采集或磁记录保存未完成时返回1；锁内读取，供sensor让出EEPROM。 */
uint8_t app_imu_is_calibrating(void)
{
    /* calibrating: 汇总校准运行、待处理命令、待保存与正在保存四种占用状态。 */
    uint8_t calibrating;
    /* 锁内读取, 与 sensor 任务对命令/保存标志的写入保持一致的判定。 */
    imu_lock();
    calibrating = s_mag_cal_state == APP_MAG_CAL_RUNNING || s_mag_cal_request != 0 || s_mag_save_pending != 0 || s_mag_saving != 0;
    imu_unlock();
    return calibrating; /* 1=仍有校准/保存事务, 0=EEPROM可让给其他用途。 */
}

uint8_t app_imu_mag_cal_active(void)
{
    uint8_t active;
    imu_lock();
    active = s_mag_cal_state == APP_MAG_CAL_RUNNING || s_mag_cal_request == APP_MAG_CMD_START;
    imu_unlock();
    return active;
}

/* 仅供旧维护调用方使用；日常熄屏保留MPU自主40Hz采样。 */
void app_imu_set_sleep(uint8_t sleeping)
{
    app_diag_sensor_init(APP_DIAG_MPU, my_mpu6050_set_sleep(sleeping));
    s_have_sample = 0;
    s_activity_filter_valid = 0;
}

void app_imu_set_accel_reference(uint8_t enabled)
{
    s_accel_reference_requested = enabled != 0;
}

uint8_t app_imu_can_enter_mot(uint32_t now_ms)
{
    return s_have_sample != 0 && s_err_streak == 0 && s_sample_period_ms == MPU6050_SAMPLE_PERIOD_MS &&
           (uint32_t)(now_ms - s_last_sample_ms) < 5 * MPU6050_SAMPLE_PERIOD_MS &&
           common_activity_is_mot_ready(&s_activity, now_ms, APP_IMU_MOT_IDLE_MS) != 0;
}

uint8_t app_imu_enter_mot(void)
{
    uint8_t result;
    s_mot_sample_valid = 0;
    result = my_mpu6050_enter_mot_sleep();
    app_diag_sensor_init(APP_DIAG_MPU, result);
    imu_lock();
    s_online = 0; /* 上次快照仍可读步数，原加速度不再标为实时。 */
    imu_unlock();
    return result;
}

void app_imu_exit_mot(uint8_t accel_reference)
{
    uint32_t period_ms = MPU6050_SAMPLE_PERIOD_MS;
    uint32_t now_ms = HAL_GetTick();
    uint8_t motion;

    /* EXTI唤醒时在切档前读取一次；轮询唤醒复用已读出的动作帧。 */
    if (s_mot_sample_valid == 0 && my_mpu6050_mot_woken() != 0 && my_mpu6050_read_mot(&s_mot_accel, &motion) == 0)
    {
        s_mot_sample_ms = now_ms;
        s_mot_sample_valid = 1;
    }
    if (s_mot_sample_valid != 0)
    {
        now_ms = s_mot_sample_ms;
    }
    common_activity_resume_from_mot(&s_activity, now_ms);
    if (s_mot_sample_valid != 0)
    {
        common_activity_update(&s_activity,
                               (float)s_mot_accel.Accel_x / MPU6050_ACCEL_LSB_PER_G,
                               (float)s_mot_accel.Accel_y / MPU6050_ACCEL_LSB_PER_G,
                               (float)s_mot_accel.Accel_z / MPU6050_ACCEL_LSB_PER_G, now_ms);
        s_mot_sample_valid = 0;
    }
    if (accel_reference != 0)
    {
        period_ms = MPU6050_REFERENCE_PERIOD_MS;
    }
    if (my_mpu6050_set_sample_mode(period_ms, accel_reference) == 0)
    {
        s_sample_period_ms = period_ms;
    }
    s_have_sample = 0;
    s_activity_filter_valid = 0;
    s_activity_elapsed_ms = 0;
}

/* 进档前的40Hz快照保留静止基准；检测到运动时保留本次读数供退出时计步。 */
uint8_t app_imu_mot_poll(void)
{
    const Accel_struct *baseline = &s_imu_data.raw.Accel;
    const int32_t threshold_lsb = APP_IMU_MOT_FALLBACK_MG * MPU6050_ACCEL_LSB_PER_G / 1000;
    Accel_struct accel;
    uint8_t motion;
    uint32_t now_ms = HAL_GetTick();

    if (my_mpu6050_read_mot(&accel, &motion) != 0)
    {
        return 1;
    }
    if (motion != 0 || abs((int32_t)accel.Accel_x - baseline->Accel_x) > threshold_lsb ||
        abs((int32_t)accel.Accel_y - baseline->Accel_y) > threshold_lsb ||
        abs((int32_t)accel.Accel_z - baseline->Accel_z) > threshold_lsb)
    {
        s_mot_accel = accel;
        s_mot_sample_ms = now_ms;
        s_mot_sample_valid = 1;
        return 1;
    }
    return 0;
}

uint8_t app_imu_mot_health_check(void)
{
    return my_mpu6050_mot_health_check();
}

uint32_t app_imu_set_sample_period(uint32_t period_ms)
{
    if (s_accel_reference_requested != 0)
    {
        period_ms = 10;
    }
    else
    {
        period_ms = MPU6050_SAMPLE_PERIOD_MS;
    }
    if (my_mpu6050_set_sample_mode(period_ms, s_accel_reference_requested) == 0 && period_ms != s_sample_period_ms)
    {
        s_sample_period_ms = period_ms;
        s_have_sample = 0;
    }
    return s_sample_period_ms;
}

uint32_t app_imu_wrist_sequence(void)
{
    return s_activity.wrist_sequence;
}

static void compass_filter_accel(const Accel_struct *raw, TickType_t now)
{
    float sample[3] = {raw->Accel_x, raw->Accel_y, raw->Accel_z};
    float norm = sqrtf(sample[0] * sample[0] + sample[1] * sample[1] + sample[2] * sample[2]);
    float deviation = fabsf(norm / MPU6050_ACCEL_LSB_PER_G - 1.0f);
    float alpha = 0.28f;
    uint8_t axis;
    s_compass_accel_valid = deviation <= 0.25f;
    s_compass_gravity_stable = deviation <= 0.15f;
    if (!s_compass_accel_valid)
    {
        return;
    }
    if (deviation > 0.10f)
    {
        alpha = 0.05f;
    }
    if (!s_compass_filter_valid)
    {
        alpha = 1.0f;
    }
    for (axis = 0; axis < 3; ++axis)
    {
        s_compass_gravity[axis] = sample[axis] / norm;
        s_compass_gravity_filt[axis] += alpha * (s_compass_gravity[axis] - s_compass_gravity_filt[axis]);
    }
    s_compass_filter_valid = 1;
    s_accel_sample_tick = now;
}

/* 重力与磁场经相同的刚体轴映射；滤波方向转换成航向算法沿用的1g计数。 */
static void compass_map_accel(Accel_struct *body)
{
    const float *f = s_compass_gravity_filt;
    float norm = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    float scale = 0.0f;
    if (norm >= 0.5f)
    {
        scale = MPU6050_ACCEL_LSB_PER_G / norm;
    }
    body->Accel_x = (int16_t)(f[s_mag_axis[0]] * s_mag_sign[0] * scale);
    body->Accel_y = (int16_t)(f[s_mag_axis[1]] * s_mag_sign[1] * scale);
    body->Accel_z = (int16_t)(f[s_mag_axis[2]] * s_mag_sign[2] * scale);
}

static app_compass_state compass_check_mag(TickType_t now, const float mag[3], float field_squared, uint8_t new_mag)
{
    uint8_t field_ok = field_squared >= 35.0f * 35.0f && field_squared <= 65.0f * 65.0f;
    if (new_mag)
    {
        const common_mag_cal_params_struct *params = &s_mag_calibration.params;
        const float *g = compass_paired_gravity(now);
        const float *f = s_compass_gravity_filt;
        uint8_t dip_checked = 0;
        float dip_error = 0.0f, dot;
        if (field_ok && params->dip_valid && g != NULL)
        {
            /* 快速翻腕时两传感器相位不同，重力未跟稳时暂停倾角判定。 */
            dot = g[0] * f[0] + g[1] * f[1] + g[2] * f[2];
            if (dot > 0.99f * sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]))
            {
                dip_error = fabsf(common_mag_cal_dip_deg(mag, g) - params->dip_ref_deg);
                dip_checked = 1;
            }
        }
        if (!field_ok)
        {
            s_mag_field_fault = 1;
        }
        if (dip_checked && dip_error > 20.0f)
        {
            s_mag_dip_fault = 1;
        }
        if (s_mag_field_fault || s_mag_dip_fault)
        {
            if (field_squared >= 38.0f * 38.0f && field_squared <= 62.0f * 62.0f &&
                ((dip_checked && dip_error < 16.0f) || (!dip_checked && !s_mag_dip_fault)))
            {
                if (++s_mag_clear_frames >= 3)
                {
                    s_mag_field_fault = s_mag_dip_fault = s_mag_clear_frames = 0;
                }
            }
            else
            {
                s_mag_clear_frames = 0;
            }
        }
    }
    if (s_mag_field_fault)
    {
        return APP_COMPASS_FIELD_RANGE;
    }
    if (s_mag_dip_fault)
    {
        return APP_COMPASS_INTERFERENCE;
    }
    return APP_COMPASS_READY;
}

void app_imu_set_declination(float deg)
{
    if (isfinite(deg))
    {
        imu_lock();
        s_mag_declination_deg = fmaxf(-45.0f, fminf(45.0f, deg));
        imu_unlock();
    }
}

/* 指南针的总线、恢复和采样单独调度，MPU故障不影响此链路。 */
app_compass_service_state app_imu_compass_service(uint8_t active, uint8_t read_sensors)
{
    TickType_t now = xTaskGetTickCount();
    Accel_struct raw_accel, body_accel;
    float corrected[3], body[3], heading = 0.0f, field_squared, max_field_uT = APP_MAG_UNCAL_MAX_UT;
    uint8_t axis, mag_recovery, accel_recovery, mag_ready, accel_ready, config_pending = 0;
    uint8_t online, valid = 0, new_mag = 0;
    uint8_t due = read_sensors != 0 && (TickType_t)(now - s_accel_poll_tick) >= pdMS_TO_TICKS(s_mag_poll_ms);
    lsm303dlhc_mag_result result;
    common_compass_result heading_result;
    app_compass_state state = APP_COMPASS_NO_MAG_DATA;

    mag_recovery = lsm303dlhc_mag_recover_step_mode(HAL_GetTick(), active == 0);
    app_diag_sensor_recovery(APP_DIAG_MAG, mag_recovery);
    accel_recovery = lsm303dlhc_accel_recover_step(HAL_GetTick());
    mag_ready = mag_recovery == 0;
    accel_ready = accel_recovery == 0;
    if (mag_ready != 0)
    {
        config_pending |= lsm303dlhc_mag_set_rate(300);
        config_pending |= lsm303dlhc_mag_set_sleep(active == 0);
    }
    else
    {
        s_mag_valid = s_mag_overflow = s_mag_clear_frames = 0;
    }
    if (accel_ready != 0)
    {
        config_pending |= lsm303dlhc_accel_set_sleep(active == 0);
    }
    else
    {
        s_compass_accel_valid = s_compass_filter_valid = 0;
    }
    if (active == 0)
    {
        s_mag_valid = s_mag_overflow = s_compass_accel_valid = s_compass_filter_valid = 0;
        s_mag_field_fault = s_mag_dip_fault = s_mag_clear_frames = 0;
    }
    if ((TickType_t)(now - s_accel_sample_tick) > pdMS_TO_TICKS(APP_MAG_MAX_AGE_MS))
    {
        s_compass_accel_valid = s_compass_filter_valid = 0;
    }
    /* 先更新加速度，再读磁场，校准和磁倾角检查使用时间最接近的原始重力方向。 */
    if (active != 0 && accel_ready != 0 && due != 0)
    {
        s_accel_poll_tick = now;
        result = lsm303dlhc_accel_read(&raw_accel);
        if (result == LSM303DLHC_MAG_OK)
        {
            s_compass_accel_errors = 0;
            compass_filter_accel(&raw_accel, now);
        }
        else if (result == LSM303DLHC_MAG_ERROR && ++s_compass_accel_errors >= APP_IMU_OFFLINE_STREAK)
        {
            s_compass_accel_valid = s_compass_filter_valid = 0;
            s_compass_accel_errors = APP_IMU_OFFLINE_STREAK;
        }
    }
    online = mag_service(now, corrected, active != 0 && mag_ready != 0 && read_sensors != 0, &new_mag);
    for (axis = 0; axis < 3; ++axis)
    {
        body[axis] = corrected[s_mag_axis[axis]] * s_mag_sign[axis];
    }
    field_squared = body[0] * body[0] + body[1] * body[1] + body[2] * body[2];
    if (s_mag_calibrated != 0)
    {
        max_field_uT = APP_MAG_LEGACY_MAX_UT;
    }
    if (s_mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        state = APP_COMPASS_CALIBRATING;
    }
    else if (s_mag_overflow)
    {
        state = APP_COMPASS_FIELD_RANGE;
    }
    else if (online != 0)
    {
        state = APP_COMPASS_READY;
        if (s_mag_calibration.is_ellipsoid)
        {
            state = compass_check_mag(now, corrected, field_squared, new_mag);
        }
        else if (!(field_squared >= 25.0f && field_squared <= max_field_uT * max_field_uT))
        {
            state = APP_COMPASS_FIELD_RANGE;
        }
        if (state == APP_COMPASS_READY)
        {
            state = APP_COMPASS_ACCEL_INVALID;
            if (s_compass_accel_valid != 0)
            {
                compass_map_accel(&body_accel);
                heading_result = common_imu_compass_heading_with_limit(&body_accel, body, max_field_uT, &heading);
                valid = heading_result == COMMON_COMPASS_OK;
                if (valid != 0)
                {
                    state = APP_COMPASS_READY;
                }
                else if (heading_result == COMMON_COMPASS_FIELD_RANGE)
                {
                    state = APP_COMPASS_FIELD_RANGE;
                }
                else if (heading_result == COMMON_COMPASS_HORIZONTAL_WEAK)
                {
                    state = APP_COMPASS_HORIZONTAL_WEAK;
                }
            }
        }
    }
    imu_lock();
    s_imu_data.mag = s_mag;
    for (axis = 0; axis < 3; ++axis)
    {
        s_imu_data.mag_body_uT[axis] = body[axis];
        s_imu_data.mag_cal_span_uT[axis] = 0.0f;
        if (s_mag_collect.samples != 0)
        {
            s_imu_data.mag_cal_span_uT[axis] = s_mag_collect.max_uT[axis] - s_mag_collect.min_uT[axis];
        }
    }
    s_imu_data.mag_declination_deg = 0.0f;
    if (s_mag_calibrated)
    {
        s_imu_data.mag_declination_deg = s_mag_declination_deg;
        if (valid)
        {
            heading += s_mag_declination_deg;
            if (heading < 0.0f)
            {
                heading += 360.0f;
            }
            else if (heading >= 360.0f)
            {
                heading -= 360.0f;
            }
        }
    }
    s_imu_data.mag_online = online;
    s_imu_data.compass_valid = valid;
    s_imu_data.compass_deg = heading;
    s_imu_data.compass_state = state;
    s_imu_data.mag_calibrated = s_mag_calibrated;
    s_imu_data.mag_cal_is_ellipsoid = s_mag_calibration.is_ellipsoid;
    s_imu_data.mag_cal_state = s_mag_cal_state;
    s_imu_data.mag_cal_samples = s_mag_collect.samples;
    s_imu_data.mag_cal_coverage_pct = s_mag_collect.coverage_pct;
    s_imu_data.mag_persist_state = s_mag_persist_state;
    imu_unlock();
    if (config_pending != 0 || mag_recovery == 2 || accel_recovery == 2)
    {
        return APP_COMPASS_SERVICE_BUSY;
    }
    if (mag_ready == 0 || accel_ready == 0)
    {
        return APP_COMPASS_SERVICE_RETRY;
    }
    return APP_COMPASS_SERVICE_IDLE;
}

/* 返回驱动读帧结果，调度器据NOT_READY保留待取帧。 */
uint8_t App_IMU_Service(uint8_t read_mag)
{
    Accel_Gyro_struct raw;
    float accel[3], alpha;
    uint32_t sample_ms, elapsed_ms = 10, started = app_diag_clock_count();
    uint8_t axis, result = my_mpu6050_get_data(&raw);
    (void)read_mag;
    if (result == 0)
    {
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_OK, result, started);
        sample_ms = my_mpu6050_sample_ms();
        accel[0] = (float)raw.Accel.Accel_x / MPU6050_ACCEL_LSB_PER_G;
        accel[1] = (float)raw.Accel.Accel_y / MPU6050_ACCEL_LSB_PER_G;
        accel[2] = (float)raw.Accel.Accel_z / MPU6050_ACCEL_LSB_PER_G;
        if (s_activity_filter_valid != 0)
        {
            elapsed_ms = sample_ms - s_last_sample_ms;
        }
        if (s_sample_period_ms == 10)
        {
            app_heart_motion_update(raw.Accel.Accel_x, raw.Accel.Accel_y, raw.Accel.Accel_z, sample_ms);
            if (elapsed_ms > COMMON_ACTIVITY_MAX_GAP_MS)
            {
                s_activity_filter_valid = 0;
            }
            if (s_activity_filter_valid == 0)
            {
                s_activity_elapsed_ms = 0;
                elapsed_ms = MPU6050_REFERENCE_PERIOD_MS;
            }
            s_activity_elapsed_ms += elapsed_ms;
            alpha = (float)elapsed_ms / (25.0f + (float)elapsed_ms);
            for (axis = 0; axis < 3; ++axis)
            {
                if (s_activity_filter_valid == 0)
                {
                    s_activity_accel[axis] = accel[axis];
                }
                else
                {
                    s_activity_accel[axis] += alpha * (accel[axis] - s_activity_accel[axis]);
                }
                accel[axis] = s_activity_accel[axis];
            }
        }
        else
        {
            /* 用40Hz末帧衔接下次100Hz滤波，切档时保留步态。 */
            for (axis = 0; axis < 3; ++axis)
            {
                s_activity_accel[axis] = accel[axis];
            }
            s_activity_elapsed_ms = 0;
        }
        s_activity_filter_valid = 1;
        if (s_sample_period_ms != 10 || s_activity.have_sample == 0 ||
            (s_activity_elapsed_ms >= MPU6050_SAMPLE_PERIOD_MS && (uint32_t)(sample_ms - s_activity.sample_ms) >= 15))
        {
            common_activity_update(&s_activity, accel[0], accel[1], accel[2], sample_ms);
            /* 100Hz档只在消费有效间隔后扣减余量，避免过密帧吞掉40Hz计步预算。 */
            s_activity_elapsed_ms %= MPU6050_SAMPLE_PERIOD_MS;
        }
        s_last_sample_ms = sample_ms;
        s_have_sample = 1;
        s_err_streak = 0;
    }
    else if (result == MPU6050_DATA_NOT_READY)
    {
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_NOT_READY, result, started);
    }
    else
    {
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_ERROR, result, started);
        if (s_err_streak < APP_IMU_OFFLINE_STREAK)
        {
            ++s_err_streak;
        }
    }
    imu_lock();
    if (result == 0)
    {
        s_imu_data.raw = raw;
    }
    s_online = s_have_sample != 0 && s_err_streak < APP_IMU_OFFLINE_STREAK &&
               (uint32_t)(HAL_GetTick() - s_last_sample_ms) < s_sample_period_ms * 5;
    s_imu_data.steps = s_activity.steps;
    s_imu_data.wrist_raise_sequence = s_activity.wrist_sequence;
    s_imu_data.wrist_raise_ms = s_activity.wrist_ms;
    imu_unlock();
    return result;
}

/* RTC读写保持LVGL唯一归属，sensor仅接收完整的日历请求。 */
void app_imu_set_day(uint16_t day, uint8_t calendar_restored)
{
    if (s_imu_mux == NULL || day == 0)
    {
        return;
    }
    imu_lock();
    s_requested_day = day;
    s_calendar_restored = calendar_restored;
    imu_unlock();
}

void app_imu_activity_service(void)
{
    uint16_t day;
    uint8_t restored;
    imu_lock();
    day = s_requested_day;
    restored = s_calendar_restored;
    imu_unlock();
    /* 已经结算过的日期回拨一日时保留步数，等RTC再次走到该日。 */
    if (day != 0 && !((s_day_bound != 0 || restored != 0) && day + 1 == s_activity_day))
    {
        if (s_activity_day != 0 && day != s_activity_day && (s_day_bound != 0 || restored != 0))
        {
            s_activity.steps = 0;
            /* 清掉昨日尚未回补的步态，保留重力基线与抬腕事件。 */
            s_activity.walking_run = s_activity.peak_pending = 0;
            s_activity.walking_confirmed = 0;
            s_activity.step_interval_ms = 0;
        }
        s_activity_day = day;
        s_day_bound = 1;
    }
    app_battery_log_steps_update(s_activity_day, s_activity.steps, HAL_GetTick());
    imu_lock();
    s_imu_data.steps = s_activity.steps;
    imu_unlock();
}

/* 仅sensor任务调用：有保存请求时启动磁校准记录写入，否则推进在途分步事务。
 * 结束(成功/失败)时把持久化状态发布进快照；返回1表示磁记录仍在保存。 */
uint8_t App_IMU_SaveService(void)
{
    app_mag_store_result result;
    if (s_mag_saving == 0)
    {
        if (s_mag_save_pending == 0)
        {
            return 0; /* 既无在途保存也无待保存请求: 无需推进。 */
        }
        s_mag_save_pending = 0; /* 取走请求, 本轮开始启动写入。 */
        result = app_mag_store_save_begin(&s_mag_calibration);
    }
    else
    {
        /* 已有在途事务: 推进一步分步写入。 */
        result = app_mag_store_save_step();
    }
    s_mag_saving = result == APP_MAG_STORE_BUSY; /* 仍为BUSY表示事务未结束。 */
    if (s_mag_saving == 0)
    {
        /* 事务结束: 按结果发布成功或失败, 并同步进快照供UI读取。 */
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
    return s_mag_saving; /* 1=磁记录仍在保存, 0=本轮无事务或已结束。 */
}

uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)
{
    /* online: 锁内取出的在线标志, 作为返回值; 快照本体复制到调用方缓冲区。 */
    uint8_t online;

    if (imu == NULL)
    {
        return 0; /* 无输出缓冲区: 不写任何数据, 按无效处理。 */
    }

    if (s_imu_mux == NULL)
    {
        /* 锁尚未创建: 没有可用快照, 清零输出并返回无效。 */
        memset(imu, 0, sizeof(*imu));
        return 0;
    }
    /* 锁内整份复制快照并取在线标志, 保证调用方看到同一时刻的一致数据。 */
    imu_lock();
    *imu = s_imu_data;
    online = s_online;
    imu_unlock();

    return online; /* 1=MPU加速度在线; 0=MPU加速度不可用，指南针和累计步数仍独立可读。 */
}

/* UI/任务发出请求即可：锁内只写命令，采集与系数更新由sensor任务执行。 */
void App_IMU_MagCalibrate(uint8_t start)
{
    if (s_imu_mux == NULL)
    {
        return; /* 锁未创建: 无法安全传递命令, 直接放弃本次请求。 */
    }
    imu_lock();
    if (start)
    {
        /* 开始请求: 置开始采集命令。 */
        s_mag_cal_request = APP_MAG_CMD_START;
    }
    else
    {
        /* 结束请求: 置检查覆盖并应用校准的命令。 */
        s_mag_cal_request = APP_MAG_CMD_FINISH;
    }
    imu_unlock();
}

/* 熄屏只取消尚未验收的采集，已验收的系数及EEPROM事务仍由保存服务推进。 */
void app_imu_cancel_mag_calibration(void)
{
    imu_lock();
    if (s_mag_cal_request != 0 || s_mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        s_mag_cal_request = 0;
        s_mag_cal_state = APP_MAG_CAL_FAILED;
        s_imu_data.mag_cal_state = APP_MAG_CAL_FAILED;
    }
    imu_unlock();
}
