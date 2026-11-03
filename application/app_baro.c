/**
  ******************************************************************************
  * @file    app_baro.c
  * @brief   应用层-SPL006 气压服务实现
  *          本层不创建 FreeRTOS 任务、不启动调度器 —— 全部任务归口 app_task.c;
  *          与 app_imu.c 同款结构: 单生产者(sensor 任务)写 → 互斥锁保护快照 →
  *          消费者加锁取。
  ******************************************************************************
***/
#include "app_baro.h"
#include "app_diag.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "spl006.h"
#include <math.h>
#include <string.h>

/*********************
 *  参数
 *********************/
/* 连续 N 次总线失败才判离线: SPL006 与 MPU6050 共总线, 偶发瞬态不宜翻 OFFLINE */
#define APP_BARO_OFFLINE_STREAK 3

/* 标准大气压海平面参考 (Pa)。气压高度公式:
 *   h = 44330 * (1 - (P / 101325)^0.1902989)   [m]
 * 以固定 101325 为基准 => 得到的是"相对标准海平面"的气压高度,
 * 会随天气(实际海平面气压变化)整体漂移, 不是 GPS 真高; 手表足够,
 * 若要本地准, 用当地 QNH 替换此常量或做一次"当前置零"即可。 */
#define APP_BARO_SEA_LEVEL_PA 101325.0f
#define APP_BARO_ALT_SCALE    44330.0f
#define APP_BARO_ALT_POW      0.1902989f

/**********************
 *  STATIC VARIABLES
 **********************/
static SemaphoreHandle_t s_baro_mux; /* 保护快照 */
static app_baro_struct s_baro_data;  /* 最新数据 (锁内赋值) */
static uint8_t s_online;             /* 1=快照有效 */
static uint32_t s_sample_ms;
static uint8_t s_err_streak; /* 连续总线失败计数 (仅 Service 读写) */
static app_baro_history_struct s_history;
static uint32_t s_history_ms;
static uint8_t s_service_valid;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void baro_lock(void);
static void baro_unlock(void);

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 初始化必须取得快照锁；首帧前的查询只读取静态空快照。 */
static void baro_lock(void)
{
    if (s_baro_mux != NULL)
    {
        xSemaphoreTake(s_baro_mux, portMAX_DELAY);
    }
}

static void baro_unlock(void)
{
    if (s_baro_mux != NULL)
    {
        xSemaphoreGive(s_baro_mux);
    }
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_Baro_Init(void)
{
    uint16_t i;
    /* mutex 在任务上下文创建 (同 App_IMU_Init 的理由) */
    s_baro_mux = xSemaphoreCreateMutex();
    if (s_baro_mux == NULL)
    {
        Error_Handler();
        return;
    }
    s_sample_ms = HAL_GetTick();
    s_history.sequence = 0;
    for (i = 0; i < APP_BARO_HISTORY_POINTS; ++i)
    {
        s_history.altitude_m[i] = APP_BARO_HISTORY_NONE;
    }
}

void app_baro_record_history(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t slots = 1;
    uint32_t i;
    int16_t altitude = APP_BARO_HISTORY_NONE;
    if (s_history.sequence != 0 && (uint32_t)(now_ms - s_history_ms) < APP_BARO_HISTORY_PERIOD_MS)
    {
        return;
    }
    if (s_history.sequence == 0)
    {
        s_history_ms = now_ms;
    }
    else
    {
        slots = (uint32_t)(now_ms - s_history_ms) / APP_BARO_HISTORY_PERIOD_MS;
        s_history_ms += slots * APP_BARO_HISTORY_PERIOD_MS;
    }
    baro_lock();
    if (s_service_valid != 0 && s_baro_data.pressure_pa > 0 && isfinite(s_baro_data.alt_m) &&
        s_baro_data.alt_m >= -30000.0f && s_baro_data.alt_m <= 30000.0f)
    {
        altitude = (int16_t)roundf(s_baro_data.alt_m);
    }
    if (slots > APP_BARO_HISTORY_POINTS)
    {
        slots = APP_BARO_HISTORY_POINTS;
    }
    /* 运动模式暂停气压后，经过的十分钟位置留空，不能用恢复后的数值补造历史。 */
    for (i = 0; i < slots; ++i)
    {
        memmove(s_history.altitude_m, s_history.altitude_m + 1, (APP_BARO_HISTORY_POINTS - 1) * sizeof(int16_t));
        s_history.altitude_m[APP_BARO_HISTORY_POINTS - 1] = APP_BARO_HISTORY_NONE;
    }
    s_history.altitude_m[APP_BARO_HISTORY_POINTS - 1] = altitude;
    ++s_history.sequence;
    baro_unlock();
}

void app_baro_get_history(app_baro_history_struct *history)
{
    if (history == NULL)
    {
        return;
    }
    baro_lock();
    *history = s_history;
    baro_unlock();
}

void App_Baro_Service(void)
{
    int32_t praw = 0;
    int32_t traw = 0;
    int32_t ppa = 0;
    float alt;
    uint8_t pr, tr;
    uint16_t result;
    uint32_t read_started = app_diag_clock_count();
    s_service_valid = 0;

    result = spl006_read_sample(&praw, &traw);
    pr = (uint8_t)result;
    tr = (uint8_t)(result >> 8);

    if ((pr == 0) && (tr == 0) && (spl006_process_pressure(praw, traw, &ppa) == 0))
    {
        app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_OK, 0, read_started);
        /* 气压→高度: 除法前先判 ppa>0, 防异常帧导致 powf 域错误 */
        if (ppa > 0)
        {
            alt = APP_BARO_ALT_SCALE * (1.0f - powf((float)ppa / APP_BARO_SEA_LEVEL_PA, APP_BARO_ALT_POW));
        }
        else
        {
            alt = 0.0f;
        }
        s_err_streak = 0;
        s_sample_ms = HAL_GetTick();
        s_service_valid = 1;
        baro_lock();
        s_baro_data.pressure_pa = ppa;
        s_baro_data.alt_m = alt;
        s_baro_data.press_raw = praw;
        s_baro_data.temp_raw = traw;
        s_online = 1;
        baro_unlock();
    }
    else if ((pr == 2) || (tr == 2))
    {
        app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_ERROR, pr | ((uint32_t)tr << 8), read_started);
        /* 仅总线失败计入离线; 返回1=数据未就绪时保留快照，过期仍按3s判定。 */
        if (++s_err_streak >= APP_BARO_OFFLINE_STREAK)
        {
            baro_lock();
            s_online = 0;
            baro_unlock();
        }
    }
    else
    {
        if (pr == 1 || tr == 1)
        {
            app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_NOT_READY, pr | ((uint32_t)tr << 8), read_started);
        }
        else
        {
            app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_ERROR, 0x10000, read_started);
        }
        s_err_streak = 0;
    }
    if ((uint32_t)(HAL_GetTick() - s_sample_ms) >= SPL006_SAMPLE_MAX_AGE_MS)
    {
        baro_lock();
        s_online = 0;
        baro_unlock();
    }
}

uint8_t App_Baro_GetSnapshot(app_baro_struct *baro)
{
    uint8_t online;

    if (baro == NULL)
    {
        return 0;
    }

    baro_lock();
    *baro = s_baro_data;
    online = s_online;
    baro_unlock();

    return online;
}
