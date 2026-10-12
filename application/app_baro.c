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

#include "FreeRTOS.h"
#include "semphr.h"

#include "spl006.h"
#include <math.h>

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
static uint8_t s_err_streak;         /* 连续总线失败计数 (仅 Service 读写) */

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void baro_lock(void);
static void baro_unlock(void);

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 锁未创建成功时跳过锁操作；此时多字段快照的一致性不受保证。 */
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
    /* mutex 在任务上下文创建 (同 App_IMU_Init 的理由) */
    s_baro_mux = xSemaphoreCreateMutex();
}

void App_Baro_Service(void)
{
    int32_t praw = 0;
    int32_t traw = 0;
    int32_t ppa = 0;
    float alt;
    uint8_t pr, tr;

    pr = spl006_read_pressure(&praw);
    tr = spl006_read_temperature(&traw);

    if ((pr == 0) && (tr == 0) && (spl006_process_pressure(praw, traw, &ppa) == 0))
    {
        /* 气压→高度: 除法前先判 ppa>0, 防异常帧导致 powf 域错误 */
        alt = (ppa > 0) ? APP_BARO_ALT_SCALE * (1.0f - powf((float)ppa / APP_BARO_SEA_LEVEL_PA, APP_BARO_ALT_POW)) : 0.0f;
        s_err_streak = 0;
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
        /* 仅总线失败计入离线; 返回1=数据未就绪 —— 1Hz背景模式对250ms轮询
         * 两次有效采样之间可能多次返回未就绪，快照与在线状态保持不变 */
        if (++s_err_streak >= APP_BARO_OFFLINE_STREAK)
        {
            baro_lock();
            s_online = 0;
            baro_unlock();
        }
    }
    else
    {
        s_err_streak = 0;
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
