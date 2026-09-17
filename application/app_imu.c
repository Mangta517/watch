/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   应用层-MPU6050 数据服务实现
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          App_IMU_Init()/App_IMU_Service() 由 sensor 任务体调用,
  *          App_IMU_GetSnapshot() 供 UI 等消费方在自己的任务上下文取数。
  *          并发模式: 单生产者(sensor 任务)写 → 互斥锁保护快照 → 消费者加锁取。
  *          不读 MPU6050 片内温度: 突发读 14 字节一帧已含加速度+陀螺仪
  *          (温度寄存器夹在中间一起读出, 但不参与显示), 省一次独立温度事务。
  ******************************************************************************
***/
#include "app_imu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"         /* xTaskGetTickCount / configTICK_RATE_HZ: dt 实测 */

#include "common_imu.h"   /* Common_IMU_GetEulerAngle 四元数姿态融合 */
#include <stdlib.h>       /* abs: 静止判定三轴比较 */

/*********************
 *  参数
 *********************/
/* 连续 N 次读失败才判离线: 单次 NACK 可能只是被 EXTI ISR 抢一次的瞬态,
 * 不宜立即翻 OFFLINE (设备端表现为数据闪断一下又恢复, 易误判为 bug) */
#define APP_IMU_OFFLINE_STREAK  3

/* dt 实测限幅 (秒): 解算积分步长宁可用错值的邻域也不要跳变,
 * 下限挡两拍合并 (tick 同值), 上限挡任务被长时间抢占后的积分暴走 */
#define APP_IMU_DT_MIN_S  0.002f
#define APP_IMU_DT_MAX_S  0.050f
#define APP_IMU_DT_INIT_S 0.010f    /* 首拍兜底值 = 任务标称周期 */

/* ---- 陀螺仪残余零漂抑制 (策略逐条沿用 flight 端 flight_task 实测版) ----
 * 上电 zero_offset 只吃掉当时的初始零偏; 温漂会让零点慢慢挪, 偏航又是纯积分
 * 无绝对参考, 残余偏置会被积分放大成角度漂移。故静止时持续做增量微调:
 *   静止判据: 三轴 |读数| 均 < 50 LSB (±2000dps 下约 3°/s, 含静止噪声)
 *   先稳定 APP_IMU_STILL_SETTLE 拍 (~1s) 才开始累积, 防放下的瞬间被当静止
 *   累积 APP_IMU_TRIM_WINDOW 拍取平均 (抵消白噪声), 一次性喂 gyro_trim
 *   检测到任何转动立即复位全部计数 —— 宁可漏校不误校 */
#define APP_IMU_STILL_GYRO_LSB    50
#define APP_IMU_STILL_SETTLE      100    /* 10ms×100 = 1s */
#define APP_IMU_TRIM_WINDOW       100    /* 10ms×100 = 1s 一轮平均 */

/**********************
 *  STATIC VARIABLES
 **********************/
static SemaphoreHandle_t s_imu_mux;     /* 保护下面三项成组读写 */
static app_imu_struct    s_imu_data;    /* 最新快照 (锁内赋值) */
static uint8_t           s_online;      /* 1=快照有效 */
static uint8_t           s_err_streak;  /* 连续读失败计数 (仅 Service 读写, 不跨任务) */

/* 姿态解算递推状态: 四元数在 common_imu 内部 static, yaw 积分在这里累加;
 * 只允许 sensor 任务触碰 (单生产者), 进快照前无并发问题 */
static Euler_struct      s_euler = {0};

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void imu_lock(void);
static void imu_unlock(void);
static void imu_still_gyro_trim(const Accel_Gyro_struct *raw);

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 锁未创建成功时退化为无锁: 单生产者结构下最多读到一次旧快照, 不崩 */
static void imu_lock(void)
{
    if (s_imu_mux != NULL)
        xSemaphoreTake(s_imu_mux, portMAX_DELAY);
}

static void imu_unlock(void)
{
    if (s_imu_mux != NULL)
        xSemaphoreGive(s_imu_mux);
}

/* 静止检测 + 陀螺仪零点微调: 仅 sensor 任务上下文调用 (单生产者, 无共享状态竞争) */
static void imu_still_gyro_trim(const Accel_Gyro_struct *raw)
{
    static uint8_t  still_cnt = 0;
    static int32_t  trim_sum_x = 0, trim_sum_y = 0, trim_sum_z = 0;
    static uint16_t trim_cnt = 0;

    if (abs(raw->Gyro.Gyro_x) < APP_IMU_STILL_GYRO_LSB &&
        abs(raw->Gyro.Gyro_y) < APP_IMU_STILL_GYRO_LSB &&
        abs(raw->Gyro.Gyro_z) < APP_IMU_STILL_GYRO_LSB)
    {
        if (still_cnt < APP_IMU_STILL_SETTLE) still_cnt++;
        if (still_cnt >= APP_IMU_STILL_SETTLE)    /* 静止稳定 ~1s 后才开始累积 */
        {
            trim_sum_x += raw->Gyro.Gyro_x;
            trim_sum_y += raw->Gyro.Gyro_y;
            trim_sum_z += raw->Gyro.Gyro_z;
            trim_cnt++;
            if (trim_cnt >= APP_IMU_TRIM_WINDOW)  /* 攒满一轮取平均, 抵消静止白噪声 */
            {
                my_mpu6050_gyro_trim((int16_t)(trim_sum_x / APP_IMU_TRIM_WINDOW),
                                     (int16_t)(trim_sum_y / APP_IMU_TRIM_WINDOW),
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

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_IMU_Init(void)
{
    /* mutex 在任务上下文创建 (与 app_display 的 LVGL 锁同理): 调度器未启动时
     * CM4F port 的临界区计数不真正解除 BASEPRI, 建锁留给任务里做才干净 */
    s_imu_mux = xSemaphoreCreateMutex();
}

void App_IMU_Service(void)
{
    Accel_Gyro_struct raw;
    TickType_t now = xTaskGetTickCount();
    static TickType_t s_last;
    static uint8_t s_first = 1;
    float dt;

    /* dt = 距上次服务的真实间隔 (flight 端同款约定: 实测+限幅, 不硬编码) */
    if (s_first)
    {
        s_first = 0;
        dt = APP_IMU_DT_INIT_S;
    }
    else
    {
        dt = (float)(now - s_last) / (float)configTICK_RATE_HZ;
        if (dt < APP_IMU_DT_MIN_S) dt = APP_IMU_DT_MIN_S;
        if (dt > APP_IMU_DT_MAX_S) dt = APP_IMU_DT_MAX_S;
    }
    s_last = now;

    /* 14 字节突发读 (加速度+温度+陀螺仪), 软件 I2C 约 1.6ms 总线时间 */
    if (my_mpu6050_get_data(&raw) == 0)
    {
        imu_still_gyro_trim(&raw);
        Common_IMU_GetEulerAngle(&raw, &s_euler, dt);

        s_err_streak = 0;
        imu_lock();
        s_imu_data.raw       = raw;
        s_imu_data.pitch_deg = s_euler.pitch;
        s_imu_data.roll_deg  = s_euler.roll;
        s_imu_data.yaw_deg   = s_euler.yaw;
        s_online = 1;
        imu_unlock();
    }
    else if (++s_err_streak >= APP_IMU_OFFLINE_STREAK)
    {
        /* 保留最后一帧数值, 仅翻在线标志: 消费者据返回值决定显示 OFFLINE */
        imu_lock();
        s_online = 0;
        imu_unlock();
    }
}

uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)
{
    uint8_t online;

    if (imu == NULL)
        return 0;

    imu_lock();
    *imu   = s_imu_data;
    online = s_online;
    imu_unlock();

    return online;
}
