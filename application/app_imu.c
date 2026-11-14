/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   应用层-MPU6050 + LSM303DLHC 数据与姿态服务
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          App_IMU_Init()/App_IMU_Service() 由 sensor 任务体调用,
  *          App_IMU_GetSnapshot() 供 UI 等消费方在自己的任务上下文取数。
  *          并发模式: 单生产者(sensor 任务)写 → 互斥锁保护快照 → 消费者加锁取。
 *          MPU6050后台5Hz仅采加速度；心率参考100Hz仅采加速度，姿态/校准才递推四元数。
  ******************************************************************************
***/
#include "app_imu.h"
#include "app_diag.h"
#include "app_heart.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h" /* 磁力计轮询与数据年龄使用RTOS tick */

#include "common_activity.h"
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
#define APP_IMU_MAX_GAP_MS      50
#define APP_IMU_EULER_PERIOD_MS 20 /* 普通档欧拉角最多50Hz；运动档同步改为10ms。 */

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
static uint8_t s_online;            /* 1=完整姿态快照在线；0仍可能保留加速度/步数等数据，纯加速度模式主动置0。 */
static uint8_t s_err_streak;        /* 连续读失败计数 (仅 Service 读写, 不跨任务) */
static uint32_t s_last_sample_ms;   /* 最近有效MPU帧的HAL读取时刻 */
static uint8_t s_have_sample;       /* 是否可用上一帧时刻计算解算间隔 */
static uint32_t s_euler_ms;
static uint32_t s_euler_period_ms = APP_IMU_EULER_PERIOD_MS;
static uint8_t s_have_euler;
static common_activity_struct s_activity;
static uint32_t s_sample_period_ms = MPU6050_SAMPLE_PERIOD_MS;
static uint8_t s_accel_reference_requested, s_accel_reference;
static uint32_t s_max_gap_ms = APP_IMU_MAX_GAP_MS;
static uint32_t s_mag_poll_ms = APP_MAG_POLL_MS;
static uint32_t s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;

/* 姿态解算递推状态: 四元数在 common_imu 内部 static;
 * 只允许 sensor 任务触碰 (单生产者), 进快照前无并发问题 */
static Euler_struct s_euler = {0};

static lsm303dlhc_mag_struct s_mag;
static TickType_t s_mag_poll_tick, s_mag_sample_tick;
static uint8_t s_mag_valid;
static uint8_t s_mag_sleep_pending; /* 模式写入失败时保留请求，由后续sensor轮次重试。 */
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

/* 静止检测 + 陀螺仪零点微调: 仅 sensor 任务上下文调用 (单生产者, 无共享状态竞争) */
static void imu_still_gyro_trim(const Accel_Gyro_struct *raw)
{
    /* 三个静态计数器跨调用保留: 稳定拍数、三轴角速度累加和、参与平均的样本数。 */
    static uint8_t still_cnt = 0;
    static int32_t trim_sum_x = 0, trim_sum_y = 0, trim_sum_z = 0;
    static uint16_t trim_cnt = 0;

    /* 校准期间不微调(避免旋转动作被当零漂); 三轴角速度绝对值都小于静止门限才认为静止。 */
    if (s_mag_cal_state != APP_MAG_CAL_RUNNING && abs(raw->Gyro.Gyro_x) < APP_IMU_STILL_GYRO_LSB &&
        abs(raw->Gyro.Gyro_y) < APP_IMU_STILL_GYRO_LSB && abs(raw->Gyro.Gyro_z) < APP_IMU_STILL_GYRO_LSB)
    {
        if (still_cnt < APP_IMU_STILL_SETTLE)
        {
            still_cnt++; /* 稳定拍数封顶, 避免无符号计数溢出。 */
        }
        if (still_cnt >= APP_IMU_STILL_SETTLE) /* 连续稳定达到100帧后才开始累积 */
        {
            /* 累加三轴原始陀螺值, 后续取平均作为零漂修正量。 */
            trim_sum_x += raw->Gyro.Gyro_x;
            trim_sum_y += raw->Gyro.Gyro_y;
            trim_sum_z += raw->Gyro.Gyro_z;
            trim_cnt++;
            if (trim_cnt >= APP_IMU_TRIM_WINDOW) /* 攒满一轮取平均, 抵消静止白噪声 */
            {
                /* 三轴平均值换算回int16后交给驱动更新零漂, 然后清空本轮累加。 */
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
    /* radius: 各轴 min/max 跨度的半宽(μT); average: 三轴半径平均值, 作为各轴归一化基准。 */
    float radius[3], average;

    if (command == APP_MAG_CMD_START) /* 保留上次已生效的校准系数 */
    {
        s_mag_cal_samples = 0; /* 重新开始采集, 样本计数与 min/max 更新从首帧起算。 */
        s_mag_cal_state = APP_MAG_CAL_RUNNING;
    }
    else if (command == APP_MAG_CMD_FINISH && s_mag_cal_state == APP_MAG_CAL_RUNNING)
    {
        /* 先置失败, 后续覆盖范围检查通过才改回 DONE。 */
        s_mag_cal_state = APP_MAG_CAL_FAILED;
        if (s_mag_cal_samples < APP_MAG_CAL_SAMPLES)
        {
            return; /* 采样数不足, 保持 FAILED。 */
        }

        /* 各轴都覆盖足够幅度后，才允许更新整组系数。 */
        for (i = 0; i < 3; ++i)
        {
            /* 该轴采集范围的一半。 */
            radius[i] = (s_mag_max[i] - s_mag_min[i]) * 0.5f;
            if (radius[i] < APP_MAG_CAL_MIN_SPAN_UT * 0.5f)
            {
                return; /* 任一轴覆盖不足, 整组系数都不更新, 保持 FAILED。 */
            }
        }
        average = (radius[0] + radius[1] + radius[2]) / 3.0f;
        for (i = 0; i < 3; ++i)
        {
            /* 偏置取 min/max 中点; 比例让该轴半径向三轴平均半径看齐。 */
            s_mag_offset[i] = (s_mag_max[i] + s_mag_min[i]) * 0.5f;
            s_mag_scale[i] = average / radius[i];
        }
        s_mag_calibrated = 1; /* 已有可用校准系数。 */
        s_mag_cal_state = APP_MAG_CAL_DONE;
        s_mag_save_pending = 1; /* 请求 App_IMU_SaveService 写入 EEPROM。 */
        s_mag_persist_state = APP_MAG_PERSIST_SAVING;
    }
}

static uint8_t mag_service(TickType_t now, float body_uT[3], uint8_t read_mag)
{
    /* result: 本次磁读取结果; i: 三轴下标; command: 本轮从共享请求位取走的校准命令。 */
    lsm303dlhc_mag_result result;
    uint8_t i, command;
    /* corrected: 去偏置并按比例缩放后的磁芯片坐标值(μT)。 */
    float corrected[3];

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
            /* 读取成功: 清连续失败计数, 刷新样本时刻并标记磁数据可用。 */
            s_mag_err_streak = 0;
            s_mag_sample_tick = xTaskGetTickCount();
            s_mag_valid = 1;
            if (s_mag_cal_state == APP_MAG_CAL_RUNNING)
            {
                /* 校准采集中: 逐轴扩展 min/max 覆盖范围。 */
                for (i = 0; i < 3; ++i)
                {
                    /* 首帧(samples==0)直接用当前值作为初始 min/max, 之后才比较。 */
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
                    ++s_mag_cal_samples; /* 计数封顶, 防止16位样本数回绕。 */
                }
            }
        }
        else if (result == LSM303DLHC_MAG_ERROR)
        {
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
    if ((TickType_t)(xTaskGetTickCount() - s_mag_sample_tick) > pdMS_TO_TICKS(s_mag_max_age_ms))
    {
        s_mag_valid = 0; /* 最近有效帧超过年龄上限: 即使无新错误也停用磁数据。 */
    }

    /* 先在磁芯片坐标下去偏置/缩放，再映射到MPU6050机体坐标。 */
    for (i = 0; i < 3; ++i)
    {
        /* 硬铁偏置扣除后再乘各轴比例, 得到校正后的芯片坐标磁场。 */
        corrected[i] = (s_mag.field_uT[i] - s_mag_offset[i]) * s_mag_scale[i];
    }
    for (i = 0; i < 3; ++i)
    {
        /* 按轴映射表重排并乘符号, 输出到 MPU6050 机体坐标(μT)。 */
        body_uT[i] = corrected[s_mag_axis[i]] * s_mag_sign[i];
    }
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
    app_diag_sensor_init(APP_DIAG_MAG, lsm303dlhc_mag_init());
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

/* 建快照锁并复位姿态/活动状态；MPU6050、LSM303DLHC已由sensor启动流程初始化。
 * 磁校准记录在本函数内读取，成功则直接进入DONE/RESTORED，失败只记持久化状态。 */
void App_IMU_Init(void)
{
    /* calibration: 从EEPROM读回的偏置/比例; result: 读取结果; i: 三轴下标。 */
    app_mag_calibration_struct calibration;
    app_mag_store_result result;
    uint8_t i;
    /* mutex 在任务上下文创建 (与 app_display 的 LVGL 锁同理): 调度器未启动时
     * CM4F port 的临界区计数不真正解除 BASEPRI, 建锁留给任务里做才干净 */
    s_imu_mux = xSemaphoreCreateMutex();
    if (s_imu_mux == NULL)
    {
        Error_Handler(); /* 互斥量创建失败: 进入错误处理, 后续不再继续初始化。 */
    }
    /* 姿态解算内部状态与活动检测状态回到上电初值。 */
    Common_IMU_Reset();
    memset(&s_activity, 0, sizeof(s_activity));
    s_last_sample_ms = 0;
    s_have_euler = 0;
    s_have_sample = 0;
    s_online = 0;
    s_err_streak = 0;

    /* EEPROM与传感器均由sensor任务访问；读取在快照锁外执行。 */
    /* 采样周期、加速度参考、欧拉角周期与磁新鲜度参数全部回到默认档。 */
    s_sample_period_ms = MPU6050_SAMPLE_PERIOD_MS;
    s_accel_reference_requested = s_accel_reference = 0;
    s_euler_period_ms = APP_IMU_EULER_PERIOD_MS;
    s_max_gap_ms = APP_IMU_MAX_GAP_MS;
    s_mag_poll_ms = APP_MAG_POLL_MS;
    s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;
    s_mag_sleep_pending = 0;
    /* 初始化EEPROM并读取上次保存的磁校准记录。 */
    bl24c02_init();
    result = app_mag_store_load(&calibration);
    if (result == APP_MAG_STORE_OK)
    {
        /* 读到有效记录: 逐轴恢复偏置与比例, 并把状态标为已校准/已恢复。 */
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

uint8_t app_imu_is_stationary(void)
{
    /* max_gap_ms: 与common_activity_update一致的断采上限，已静止放宽到1200ms。 */
    uint32_t max_gap_ms;
    /* s_activity 仅由 sensor 任务写、sensor 任务读，无需加锁。
     * 只有新鲜有效的加速度帧才允许放宽读节拍：无有效帧，或最近一帧已超过活动模块容忍的最大间隔
     * (离线、总线恢复中、任务被长时间抢占)，一律回到200ms重新取帧，再连续稳定约10秒才可慢待机。 */
    if (s_activity.stationary != 0)
    {
        /* 静止慢待机每秒一帧：允许多等一帧宽度的时间再加调度抖动。 */
        max_gap_ms = COMMON_ACTIVITY_STATIONARY_MAX_GAP_MS;
    }
    else
    {
        /* 尚未静止：沿用600ms快速连续性上限。 */
        max_gap_ms = COMMON_ACTIVITY_MAX_GAP_MS;
    }
    if (s_activity.have_sample == 0 ||
        (uint32_t)(HAL_GetTick() - s_activity.sample_ms) > max_gap_ms)
    {
        /* 无有效帧或最近帧已超时: 不能确认静止, 返回0让调用方保持正常读节拍。 */
        return 0;
    }
    return s_activity.stationary; /* 帧新鲜时返回活动模块最近一次的静止结论。 */
}

/* 仅sensor任务：切换MPU/LSM休眠，唤醒时丢弃旧时间基准与磁新鲜度，保留四元数。 */
void app_imu_set_sleep(uint8_t sleeping)
{
    /* 按目标休眠状态分别写MPU6050与LSM303DLHC, 并把写入结果记入诊断。 */
    app_diag_sensor_init(APP_DIAG_MPU, my_mpu6050_set_sleep(sleeping));
    app_diag_sensor_init(APP_DIAG_MAG, lsm303dlhc_mag_set_sleep(sleeping));
    if (sleeping == 0)
    {
        /* 间歇采样保留四元数；只丢弃旧时间基准，避免每次后台唤醒把航向重新拉回0。 */
        s_have_sample = s_have_euler = s_mag_valid = 0;
        /* 让磁轮询立即到期: 把上次轮询时刻回拨一个轮询周期。 */
        s_mag_poll_tick = xTaskGetTickCount() - pdMS_TO_TICKS(s_mag_poll_ms);
    }
}

/* 仅sensor任务：先记请求，下一轮set_sample_period据此决定心率参考模式。 */
void app_imu_set_accel_reference(uint8_t enabled)
{
    /* 只记录请求; 实际模式切换留给下一轮 set_sample_period 按校准状态统一处理。 */
    s_accel_reference_requested = enabled != 0;
}

/* 仅sensor任务：校准中或非法周期回落到20ms；合法值10/40/200ms切换MPU采样率，
 * 非纯加速度档再配置磁力计速率与新鲜度窗口。返回应用侧后续采用的调度周期；重配置失败时回落20ms继续恢复。 */
uint32_t app_imu_set_sample_period(uint32_t period_ms)
{
    /* calibrating: 校准是否占用; accel_reference: 本档是否心率参考模式;
     * accel_only: 本档是否纯加速度; previous_accel_only: 切换前是否纯加速度。 */
    uint8_t calibrating, accel_reference, accel_only, previous_accel_only;
    /* mag_rate: 本档要配置的磁力计输出率(0.1Hz), 默认300即30Hz。 */
    uint16_t mag_rate = 300;
    /* 锁内读取校准状态与待处理启动命令, 决定是否强制回落默认周期。 */
    imu_lock();
    calibrating = s_mag_cal_state == APP_MAG_CAL_RUNNING || s_mag_cal_request == APP_MAG_CMD_START;
    imu_unlock();
    if (calibrating != 0 || (period_ms != 10 && period_ms != 40 && period_ms != 200))
    {
        period_ms = 20; /* 校准中或非法周期: 回落到20ms普通档。 */
    }
    /* 只有请求了心率参考、未在校准且非200ms纯加速度档, 才启用加速度参考模式。 */
    accel_reference = s_accel_reference_requested != 0 && calibrating == 0 && period_ms != 200;
    if (my_mpu6050_set_sample_mode(period_ms, accel_reference) != 0)
    {
        period_ms = 20; /* 重配置期间继续推进其他服务，不阻塞任务。 */
    }
    /* 200ms档或心率参考档都只采加速度, 不运行磁融合与四元数。 */
    accel_only = period_ms == 200 || accel_reference != 0;
    previous_accel_only = s_sample_period_ms == 200 || s_accel_reference != 0;
    if (period_ms != s_sample_period_ms || accel_reference != s_accel_reference)
    {
        if (accel_only != previous_accel_only)
        {
            /* 纯加速度属性翻转: 请求切换磁力计休眠, 并作废当前磁有效与在线标记。 */
            s_mag_sleep_pending = 1;
            s_mag_valid = 0;
            imu_lock();
            s_online = 0;
            imu_unlock();
        }
        /* 记录新档位参数, 解算间隔上限取周期5倍, 并丢弃旧时间基准等待新帧。 */
        s_sample_period_ms = period_ms;
        s_accel_reference = accel_reference;
        s_max_gap_ms = period_ms * 5;
        s_have_sample = s_have_euler = 0;
    }
    if (s_mag_sleep_pending != 0)
    {
        /* 有待处理的休眠切换: 执行写入, 失败则保留请求由后续轮次重试。 */
        s_mag_sleep_pending = lsm303dlhc_mag_set_sleep(accel_only);
        app_diag_sensor_init(APP_DIAG_MAG, s_mag_sleep_pending);
    }
    if (accel_only != 0)
    {
        return period_ms; /* 磁力计保持休眠，加速度帧不参与四元数和陀螺仪零漂更新。 */
    }
    if (period_ms == 10)
    {
        /* 10ms运动档: 欧拉角同步10ms输出, 磁力计提到750(即75Hz)。 */
        s_euler_period_ms = 10;
        mag_rate = 750;
    }
    else
    {
        /* 其余档欧拉角回到默认20ms; 40ms档磁力计降到75(即7.5Hz)。 */
        s_euler_period_ms = APP_IMU_EULER_PERIOD_MS;
        if (period_ms == 40)
        {
            mag_rate = 75;
        }
    }
    if (lsm303dlhc_mag_set_rate(mag_rate) == 0)
    {
        /* 磁速率配置成功: 先按常规档设置轮询与新鲜度窗口。 */
        s_mag_poll_ms = 40;
        s_mag_max_age_ms = 120;
        if (mag_rate == 750)
        {
            /* 750(75Hz)档: 使用10ms轮询、50ms年龄窗口。 */
            s_mag_poll_ms = APP_MAG_POLL_MS;
            s_mag_max_age_ms = APP_MAG_MAX_AGE_MS;
        }
        else if (mag_rate == 75)
        {
            /* 75(7.5Hz)档: 轮询放慢到160ms, 年龄窗口放宽到400ms。 */
            s_mag_poll_ms = 160;
            s_mag_max_age_ms = 400;
        }
    }
    return period_ms; /* 返回应用侧后续采用的调度周期(ms)；重配置失败时回落20ms继续恢复。 */
}

/* 仅sensor任务调用：取磁参考→读MPU→零偏/活动/姿态更新→锁内发布一致快照。
 * 纯加速度档只发布加速度与步数，跳过四元数与磁融合；read_mag=0时本轮延后磁事务。 */
void App_IMU_Service(uint8_t read_mag)
{
    /* raw: 本拍MPU6050原始六轴帧; now: 本轮开始时的RTOS tick。 */
    Accel_Gyro_struct raw;
    TickType_t now = xTaskGetTickCount();
    /* mag_body: 映射到机体的磁场(μT); field_squared: 其模长平方, 用于量程门限; dt: 解算间隔(s)。 */
    float mag_body[3], field_squared, dt;
    /* mag_reference: 非空表示本拍允许磁融合; euler_output: 非空表示本拍刷新欧拉角。 */
    const float *mag_reference = NULL;
    Euler_struct *euler_output = NULL;
    /* mag_online: 磁数据当前是否可用; mag_fused: 本拍是否真正用了磁修正。 */
    uint8_t mag_online, mag_fused = 0;
    /* axis: 三轴循环下标; read_result: MPU读帧结果(0成功)。 */
    uint8_t axis, read_result;
    /* sample_ms: 本帧时间戳; elapsed_ms: 与上一帧间隔; read_started: 读取前诊断时间戳。 */
    uint32_t sample_ms, elapsed_ms, read_started;
    /* cal_before/cal_after: 零偏校准步骤调用前后的状态, 用于识别刚完成校准。 */
    mpu6050_cal_state cal_before, cal_after;
    /* mag_fusion_state: 本拍发布给UI的磁融合状态, 默认无数据。 */
    app_mag_fusion_state mag_fusion_state = APP_MAG_FUSION_IMU_NO_DATA;

    /* 1. 同一sensor任务先取磁参考，再以一笔事务轮询MPU最新帧。 */
    mag_online = 0;
    memset(mag_body, 0, sizeof(mag_body));
    if (s_sample_period_ms != 200 && s_accel_reference == 0)
    {
        /* 非纯加速度档才轮询磁力计; read_mag=0时本轮只取旧值, 不发起磁事务。 */
        mag_online = mag_service(now, mag_body, read_mag);
    }
    /* 先算磁场模长平方, 后面用25~10000μT²(即5~100μT)判断量程是否可信。 */
    field_squared = mag_body[0] * mag_body[0] + mag_body[1] * mag_body[1] + mag_body[2] * mag_body[2];
    read_started = app_diag_clock_count();
    read_result = my_mpu6050_get_data(&raw);
    if (read_result == 0)
    {
        /* 成功读到新帧。 */
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_OK, read_result, read_started);
    }
    else if (read_result == MPU6050_DATA_NOT_READY)
    {
        /* 新帧尚未就绪: 记为未就绪而非错误。 */
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_NOT_READY, read_result, read_started);
    }
    else
    {
        /* 其他结果按读取错误记录。 */
        app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_ERROR, read_result, read_started);
    }
    if (read_result == 0)
    {
        /* 仅新帧参与活动检测: 三轴LSB换算为g后连同帧时刻交给活动模块。 */
        common_activity_update(&s_activity, (float)raw.Accel.Accel_x / MPU6050_ACCEL_LSB_PER_G,
                               (float)raw.Accel.Accel_y / MPU6050_ACCEL_LSB_PER_G,
                               (float)raw.Accel.Accel_z / MPU6050_ACCEL_LSB_PER_G, my_mpu6050_sample_ms());
    }
    if (s_sample_period_ms == 200 || s_accel_reference != 0)
    {
        /* 纯加速度档: 只发布加速度、步数与抬腕, 不推进四元数与磁融合。 */
        if (read_result != 0)
        {
            /* 无新帧/离线不能确认稳定，取消静止并重启稳定计时，后台回到200ms尽快重获新鲜加速度。 */
            common_activity_stationary_cancel(&s_activity);
        }
        if (read_result == 0 && s_accel_reference != 0)
        {
            /* 心率参考档有新帧: 把原始加速度与帧时刻交给心率运动判定。 */
            app_heart_motion_update(raw.Accel.Accel_x, raw.Accel.Accel_y, raw.Accel.Accel_z, my_mpu6050_sample_ms());
        }
        imu_lock();
        if (read_result == 0)
        {
            s_imu_data.raw = raw; /* 有新帧才更新六轴原始值。 */
        }
        /* 无论有无新帧都同步步数与抬腕结果。 */
        s_imu_data.steps = s_activity.steps;
        s_imu_data.wrist_raise_sequence = s_activity.wrist_sequence;
        s_imu_data.wrist_raise_ms = s_activity.wrist_ms;
        /* 本档磁力计休眠: 清磁在线/融合标志并置无数据状态。 */
        s_imu_data.mag_online = s_imu_data.mag_fused = 0;
        s_imu_data.mag_fusion_state = APP_MAG_FUSION_NO_DATA;
        /* 清时间基准, 使离开本档后重新起算解算间隔。 */
        s_online = s_have_sample = s_have_euler = 0;
        imu_unlock();
        return;
    }
    /* 记录零偏校准步骤前的状态, 再推进一拍校准, 并把校准诊断写入记录。 */
    cal_before = my_mpu6050_zero_offset_state();
    cal_after = my_mpu6050_zero_offset_step(&raw, read_result == 0, HAL_GetTick());
    app_diag_mpu_cal_record(my_mpu6050_zero_offset_diag());
    if (read_result == 0)
    {
        /* 有新帧: 用帧时间戳计算与上一有效帧的间隔。 */
        sample_ms = my_mpu6050_sample_ms();
        elapsed_ms = s_sample_period_ms; /* 无上一帧时按标称周期积分。 */
        if (s_have_sample != 0)
        {
            elapsed_ms = sample_ms - s_last_sample_ms;
            if (elapsed_ms == 0 || elapsed_ms >= s_max_gap_ms)
            {
                elapsed_ms = s_sample_period_ms; /* 间隔异常(零或超断采门限)时回落标称周期。 */
            }
        }
        /* 间隔由ms换算为s, 供四元数递推使用。 */
        dt = (float)elapsed_ms / 1000.0f;
        s_last_sample_ms = sample_ms;
        s_have_sample = 1;
        /* 新帧参与心率运动判定, 使用该帧自己的时间戳。 */
        app_heart_motion_update(raw.Accel.Accel_x, raw.Accel.Accel_y, raw.Accel.Accel_z, sample_ms);
        if (cal_after == MPU6050_CAL_DONE && cal_before == MPU6050_CAL_AVERAGING)
        {
            /* 零偏校准刚在本拍完成: 复位姿态, 让解算从校准后的数据重新建立。 */
            Common_IMU_Reset();
            s_have_euler = 0;
        }
        /* 2. 仅新帧参与零漂微调与姿态解算；短时调度抖动使用实际读帧间隔。 */
        if (cal_before != MPU6050_CAL_WAIT_STILL && cal_before != MPU6050_CAL_AVERAGING)
        {
            /* 校准等待/取平均阶段不微调, 避免把校准动作当成静止零漂。 */
            imu_still_gyro_trim(&raw);
        }
        /* 磁新鲜度统一使用RTOS tick；常规轮询不再与FIFO历史帧配对。 */
        if (!mag_online || (TickType_t)(xTaskGetTickCount() - s_mag_sample_tick) > pdMS_TO_TICKS(s_mag_max_age_ms))
        {
            /* 磁不可用或最近样本超龄: 本拍不做磁修正。 */
            mag_fusion_state = APP_MAG_FUSION_NO_DATA;
        }
        else if (s_mag_cal_state == APP_MAG_CAL_RUNNING)
        {
            /* 校准采集中: 不把旋转采集的磁场用于航向。 */
            mag_fusion_state = APP_MAG_FUSION_CALIBRATING;
        }
        else if (!(field_squared >= 25.0f && field_squared <= 10000.0f))
        {
            /* 磁场模长平方不在5~100μT对应范围内: 判为量程异常, 不参与融合。 */
            mag_fusion_state = APP_MAG_FUSION_FIELD_RANGE;
        }
        else
        {
            /* 以上检查通过: 允许本拍把机体磁场作为磁参考。 */
            mag_reference = mag_body;
        }
        if (s_have_euler == 0 || (uint32_t)(sample_ms - s_euler_ms) >= s_euler_period_ms)
        {
            /* 首拍或到达欧拉角输出周期: 本拍刷新欧拉角并记录输出时刻。 */
            euler_output = &s_euler;
            s_euler_ms = sample_ms;
            s_have_euler = 1;
        }
        /* 推进一次四元数解算; 返回1表示本次实际用到了磁航向修正。 */
        mag_fused = Common_IMU_GetEulerAngleWithMag(&raw, mag_reference, euler_output, dt);
        if (mag_reference != NULL)
        {
            if (mag_fused)
            {
                /* 提供了磁参考且解算确实采用: 磁融合生效。 */
                mag_fusion_state = APP_MAG_FUSION_ACTIVE;
            }
            else
            {
                /* 提供了磁参考但水平分量不足等原因未采用: 标记水平磁场偏弱。 */
                mag_fusion_state = APP_MAG_FUSION_HORIZONTAL_WEAK;
            }
        }
        s_err_streak = 0; /* 成功读到帧, 连续失败计数清零。 */
    }
    else if (read_result != MPU6050_DATA_NOT_READY && s_err_streak < APP_IMU_OFFLINE_STREAK)
    {
        /* 非“未就绪”的失败才累加离线计数, 并封顶在门限。 */
        ++s_err_streak;
    }
    /* 3. 发布：六轴、姿态和磁状态在同一保护区更新，UI只读一致快照。 */
    imu_lock();
    if (read_result == 0)
    {
        /* 有新帧: 更新六轴与欧拉角, 并按最近帧年龄判定在线。 */
        s_imu_data.raw = raw;
        s_imu_data.pitch_deg = s_euler.pitch;
        s_imu_data.roll_deg = s_euler.roll;
        s_imu_data.yaw_deg = s_euler.yaw;
        s_online = (uint32_t)(HAL_GetTick() - s_last_sample_ms) < s_max_gap_ms;
    }
    else if (s_err_streak >= APP_IMU_OFFLINE_STREAK ||
             (s_online != 0 && (uint32_t)(HAL_GetTick() - s_last_sample_ms) >= s_max_gap_ms))
    {
        /* 连续失败达门限或已有快照超龄: 判离线并清时间基准。 */
        s_online = 0;
        s_have_sample = 0;
        s_have_euler = 0;
    }
    /* 以下字段无论本拍是否读到新帧都同步, 使快照与当前模块状态一致。 */
    s_imu_data.steps = s_activity.steps;
    s_imu_data.wrist_raise_sequence = s_activity.wrist_sequence;
    s_imu_data.wrist_raise_ms = s_activity.wrist_ms;
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
            /* 已有校准样本: 发布该轴当前 min/max 跨度, 供UI显示覆盖进度。 */
            s_imu_data.mag_cal_span_uT[axis] = s_mag_max[axis] - s_mag_min[axis];
        }
        else
        {
            /* 尚无样本: 跨度无意义, 置0。 */
            s_imu_data.mag_cal_span_uT[axis] = 0.0f;
        }
    }
    imu_unlock();
}

/* 仅sensor任务调用：有保存请求时启动磁校准记录写入，否则推进在途分步事务。
 * 结束(成功/失败)时把持久化状态发布进快照；返回1表示磁记录仍在保存。 */
uint8_t App_IMU_SaveService(void)
{
    /* result: 本轮保存事务的结果(可能为 BUSY); axis: 三轴下标。 */
    app_mag_store_result result;
    uint8_t axis;
    if (s_mag_saving == 0)
    {
        /* calibration: 本轮要写入EEPROM的偏置/比例副本, 仅无在途事务时在分支内构造。 */
        app_mag_calibration_struct calibration;
        if (s_mag_save_pending == 0)
        {
            return 0; /* 既无在途保存也无待保存请求: 无需推进。 */
        }
        s_mag_save_pending = 0; /* 取走请求, 本轮开始启动写入。 */
        for (axis = 0; axis < 3; ++axis)
        {
            /* 把当前生效的三轴偏置与比例装入待写记录。 */
            calibration.offset_uT[axis] = s_mag_offset[axis];
            calibration.scale[axis] = s_mag_scale[axis];
        }
        result = app_mag_store_save_begin(&calibration);
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

    return online; /* 1=完整姿态快照在线; 0=完整姿态不可用, 快照中的加速度/步数等数据仍可能有效。 */
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
