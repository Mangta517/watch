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

/* 标准模型保留原参数；有手机环境温度时，按0.0065K/m递减率求温压高度。
 * 手机温度近似所在地空气温度，芯片原始温度仍只用于SPL006压力补偿。 */
#define APP_BARO_ALT_SCALE    44330.0f
#define APP_BARO_ALT_POW      0.1902989f
#define APP_BARO_LAPSE_K_PER_M 0.0065f

/**********************
 *  STATIC VARIABLES
 **********************/
static SemaphoreHandle_t s_baro_mux; /* 保护快照 */
static app_baro_struct s_baro_data;  /* 最新数据 (锁内赋值) */
static uint32_t s_sea_level_pa = 101325; /* 手机未同步时使用标准参考，和快照共用互斥锁。 */
static int16_t s_temperature_x10;
static int16_t s_calibration_altitude_m;
static float s_pressure_correction_pa; /* 已知海拔校准得到的压力偏差，仅用于高度换算。 */
static uint8_t s_online;             /* 1=快照有效 */
static uint32_t s_sample_ms;
static uint8_t s_err_streak; /* 连续总线失败计数 (仅 Service 读写) */
static app_baro_history_struct s_history; /* 24柱高度历史，锁内推进与复制 */
static uint32_t s_history_ms;             /* 上一柱的基准时刻，用于补空柱 */
static uint8_t s_service_valid;           /* 本轮 Service 是否取到有效读数 (仅 sensor 任务读写) */

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
    /* 锁已创建才取；初始化之前访问不阻塞，读的是静态零值快照。 */
    if (s_baro_mux != NULL)
    {
        xSemaphoreTake(s_baro_mux, portMAX_DELAY);
    }
}

static void baro_unlock(void)
{
    /* 与 baro_lock 成对：只有确实取过锁才释放，避免未创建时误给。 */
    if (s_baro_mux != NULL)
    {
        xSemaphoreGive(s_baro_mux);
    }
}

/* 调用方持有快照锁，压力输入为SPL006温度补偿后的Pa。 */
static float baro_altitude(float pressure_pa)
{
    float temperature_k;
    /* 压力或校准后气压非正时无法求幂，返回0m保护。 */
    if (pressure_pa <= 0.0f || pressure_pa + s_pressure_correction_pa <= 0.0f)
    {
        return 0.0f;
    }
    /* 先叠加已知海拔校准偏差；若手机环境温度有效则按实测温压公式，否则用标准大气模型。 */
    pressure_pa += s_pressure_correction_pa;
    if (s_baro_data.temperature_corrected != 0)
    {
        temperature_k = (float)s_temperature_x10 / 10.0f + 273.15f;
        return temperature_k / APP_BARO_LAPSE_K_PER_M *
               (powf((float)s_sea_level_pa / pressure_pa, APP_BARO_ALT_POW) - 1.0f);
    }
    return APP_BARO_ALT_SCALE * (1.0f - powf(pressure_pa / (float)s_sea_level_pa, APP_BARO_ALT_POW));
}

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void app_baro_init_hw(void)
{
    app_diag_sensor_init(APP_DIAG_BARO, spl006_init());
}

uint8_t app_baro_set_sleep(uint8_t sleeping)
{
    return spl006_set_sleep(sleeping);
}

void app_baro_recover_step(uint32_t now_ms)
{
    app_diag_sensor_recovery(APP_DIAG_BARO, spl006_recover_step(now_ms));
}

/* 只做软件初始化：器件配置在sensor任务调用app_baro_init_hw时完成。 */
void App_Baro_Init(void)
{
    uint16_t i; /* 遍历 24 个历史槽的下标。 */
    /* mutex 在任务上下文创建 (同 App_IMU_Init 的理由) */
    s_baro_mux = xSemaphoreCreateMutex();
    if (s_baro_mux == NULL)
    {
        Error_Handler();
        return;
    }
    /* 以当前 tick 作为采样时间基准，使首次新鲜度判断从此刻起算。 */
    s_sample_ms = HAL_GetTick();
    /* 历史序列号清零，表示还没有落过任何一柱。 */
    s_history.sequence = 0;
    for (i = 0; i < APP_BARO_HISTORY_POINTS; ++i)
    {
        /* 逐槽写入缺测标记 INT16_MIN，开机画面先显示 24 个空柱。 */
        s_history.altitude_m[i] = APP_BARO_HISTORY_NONE;
    }
}

/* 仅sensor任务调用：把本轮已取得的有效高度落柱；每跨10分钟一柱，
 * 跨过多柱时补等量空柱。无有效读数或高度超出物理范围时该柱留空。 */
void app_baro_record_history(void)
{
    uint32_t now_ms = HAL_GetTick();          /* 本次落柱的当前时刻，毫秒。 */
    uint32_t slots = 1;                       /* 本次要推进的柱数，缺省一柱。 */
    uint32_t i;                               /* 补空柱循环下标。 */
    int16_t altitude = APP_BARO_HISTORY_NONE; /* 待写入新柱的高度，默认缺测。 */
    /* 未到下一柱直接返回；跨过多柱时先补等量的空柱，保持横轴每格10分钟。 */
    if (s_history.sequence != 0 && (uint32_t)(now_ms - s_history_ms) < APP_BARO_HISTORY_PERIOD_MS)
    {
        return;
    }
    if (s_history.sequence == 0)
    {
        /* 第一柱：把基准时刻定在此刻，避免用开机到现在的时长算出一堆空柱。 */
        s_history_ms = now_ms;
    }
    else
    {
        /* 已过至少一柱：按整柱数计算本次要推进几格，并把基准时刻前移相同跨度。 */
        slots = (uint32_t)(now_ms - s_history_ms) / APP_BARO_HISTORY_PERIOD_MS;
        s_history_ms += slots * APP_BARO_HISTORY_PERIOD_MS;
    }
    baro_lock();
    /* 只有本轮读数有效且高度在物理合理范围内才落柱，异常帧宁可留空位。 */
    if (s_service_valid != 0 && s_baro_data.pressure_pa > 0 && isfinite(s_baro_data.alt_m) &&
        s_baro_data.alt_m >= -30000.0f && s_baro_data.alt_m <= 30000.0f)
    {
        /* 四舍五入成整数米后存入 int16 柱；超出 ±30000m 的帧不参与。 */
        altitude = (int16_t)roundf(s_baro_data.alt_m);
    }
    if (slots > APP_BARO_HISTORY_POINTS)
    {
        /* 一次跨过太多柱时只补满整屏，历史长度固定为 24 格。 */
        slots = APP_BARO_HISTORY_POINTS;
    }
    /* 运动模式暂停气压后，经过的十分钟位置留空，不能用恢复后的数值补造历史。 */
    for (i = 0; i < slots; ++i)
    {
        /* 每补一柱：整段左移一格(最旧丢弃)，末端先填缺测标记。 */
        memmove(s_history.altitude_m, s_history.altitude_m + 1, (APP_BARO_HISTORY_POINTS - 1) * sizeof(int16_t));
        s_history.altitude_m[APP_BARO_HISTORY_POINTS - 1] = APP_BARO_HISTORY_NONE;
    }
    /* 补完空柱后，最右一格写入本次高度(可能是缺测)。 */
    s_history.altitude_m[APP_BARO_HISTORY_POINTS - 1] = altitude;
    /* 序列号自增，表示已落过柱，下一轮不再走首柱分支。 */
    ++s_history.sequence;
    baro_unlock();
}

/* 锁内复制24柱历史；NULL不处理，首帧前为初始化后的全空柱。 */
void app_baro_get_history(app_baro_history_struct *history)
{
    /* 输出指针为空时不处理，避免写空地址。 */
    if (history == NULL)
    {
        return;
    }
    /* 锁内整体复制 24 个高度槽与序列号，保证调用方拿到同一时刻的整屏历史。 */
    baro_lock();
    *history = s_history;
    baro_unlock();
}

/* 仅sensor调用：调用驱动读取一组压力与温度原始计数并做温度补偿，换算温度修正后的气压高度。
 * 仅总线失败累计离线连败；未就绪保留快照，超新鲜度门限统一判离线。 */
void App_Baro_Service(void)
{
    int32_t praw = 0;  /* 本次读到的原始压力计数，驱动填充。 */
    int32_t traw = 0;  /* 本次读到的原始温度计数，用于补偿。 */
    int32_t ppa = 0;   /* 温度补偿后的气压，单位 Pa。 */
    float reference_pressure_pa, temperature_k; /* 已知海拔处的模型气压及环境绝对温度。 */
    uint8_t pr, tr;    /* 从返回码拆出的压力字节、温度字节，0/1/2 为状态。 */
    uint16_t result;   /* spl006_read_sample 的 16 位返回码：低字节压力、高字节温度。 */
    uint32_t read_started = app_diag_clock_count(); /* 读开始时刻，仅用于诊断耗时。 */
    s_service_valid = 0; /* 先假定本轮没有有效读数，成功后再置 1。 */

    /* 调用驱动读取一组压力与温度原始计数(驱动先读状态，后读6字节，共两次总线事务)：
     * 低字节是压力结果、高字节是温度结果，返回码 0=成功 1=器件未转换好 2=总线失败 (见 spl006.h)。 */
    result = spl006_read_sample(&praw, &traw);
    /* 拆字节：低 8 位是压力状态，高 8 位是温度状态。 */
    pr = (uint8_t)result;
    tr = (uint8_t)(result >> 8);

    /* 本次结果与校准收尾一起发布，释放锁后到达的新请求留给下一次采样。 */
    baro_lock();

    if ((pr == 0) && (tr == 0) && (spl006_process_pressure(praw, traw, &ppa) == 0))
    {
        /* 两路都成功且补偿成功：记一条 OK 诊断，值传 0。 */
        app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_OK, 0, read_started);
        if (s_baro_data.calibration_state == APP_BARO_CAL_PENDING && ppa > 0)
        {
            /* 用当前参考反算已知海拔处的气压，偏差在后续采样及手机刷新后继续生效。 */
            if (s_baro_data.temperature_corrected != 0)
            {
                temperature_k = (float)s_temperature_x10 / 10.0f + 273.15f;
                reference_pressure_pa = (float)s_sea_level_pa /
                    powf(1.0f + APP_BARO_LAPSE_K_PER_M * (float)s_calibration_altitude_m / temperature_k, 1.0f / APP_BARO_ALT_POW);
            }
            else
            {
                reference_pressure_pa = (float)s_sea_level_pa *
                    powf(1.0f - (float)s_calibration_altitude_m / APP_BARO_ALT_SCALE, 1.0f / APP_BARO_ALT_POW);
            }
            s_pressure_correction_pa = reference_pressure_pa - (float)ppa;
            s_baro_data.calibration_state = APP_BARO_CAL_APPLIED;
        }
        /* 有效帧清零连续总线失败计数，并刷新最近成功采样时刻(ms)。 */
        s_err_streak = 0;
        s_sample_ms = HAL_GetTick();
        s_service_valid = 1;
        /* 发布本轮快照：补偿气压、高度、两路原始计数，同时把在线标志置1。 */
        s_baro_data.pressure_pa = ppa;
        s_baro_data.alt_m = baro_altitude((float)ppa);
        s_baro_data.press_raw = praw;
        s_baro_data.temp_raw = traw;
        s_online = 1;
    }
    else if ((pr == 2) || (tr == 2))
    {
        /* 任一路是总线失败：把两个状态字节拼回诊断值(高字节温度)后记录错误。 */
        app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_ERROR, pr | ((uint32_t)tr << 8), read_started);
        /* 仅总线失败计入离线; 返回1=数据未就绪时保留快照，过期仍按3s判定。 */
        if (++s_err_streak >= APP_BARO_OFFLINE_STREAK)
        {
            /* 连续 3 次总线失败：加锁把在线标志清 0，快照数值保留。 */
            s_online = 0;
        }
    }
    else
    {
        if (pr == 1 || tr == 1)
        {
            /* 器件尚未转换好：记未就绪诊断，不算总线故障。 */
            app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_NOT_READY, pr | ((uint32_t)tr << 8), read_started);
        }
        else
        {
            /* 读数状态不属于未就绪或总线失败，但补偿未成功，以0x10000记录错误。 */
            app_diag_sensor_record(APP_DIAG_BARO, APP_DIAG_ERROR, 0x10000, read_started);
        }
        s_err_streak = 0; /* 未就绪/补偿失败不是总线故障，不累计离线连败 */
    }
    if (s_baro_data.calibration_state == APP_BARO_CAL_PENDING)
    {
        /* 本次采样失败时结束请求，保留此前的校准偏差，等待用户重新提交。 */
        s_baro_data.calibration_state = APP_BARO_CAL_FAILED;
    }
    /* 1Hz 器件在暂停(运动/环境窗口)后仍可能留旧快照，用新鲜度门限统一判离线。 */
    if ((uint32_t)(HAL_GetTick() - s_sample_ms) >= SPL006_SAMPLE_MAX_AGE_MS)
    {
        /* 距最近成功采样已超过最大允许年龄：加锁清在线标志，快照数值不动。 */
        s_online = 0;
    }
    baro_unlock();
}

/* 参考气压和温度与高度同锁更新；若参数变化且已有有效气压则立即重算海拔，sensor下一次采样继续沿用。 */
void app_baro_set_reference(uint32_t pressure_pa, int16_t temperature_x10, uint8_t temperature_valid)
{
    uint8_t changed;
    baro_lock();
    /* 收到非零海平面气压时标记手机参考就绪；传0则沿用当前海平面气压。 */
    if (pressure_pa != 0)
    {
        s_baro_data.reference_ready = 1;
    }
    else
    {
        pressure_pa = s_sea_level_pa;
    }
    /* 天气温度无效或过期时清零温度缓存，退回标准模型。 */
    if (temperature_valid == 0)
    {
        temperature_x10 = 0;
    }
    changed = s_sea_level_pa != pressure_pa || s_temperature_x10 != temperature_x10 ||
              s_baro_data.temperature_corrected != temperature_valid;
    s_sea_level_pa = pressure_pa;
    s_temperature_x10 = temperature_x10;
    s_baro_data.temperature_corrected = temperature_valid;
    /* 参考气压或温补状态变化且已有气压读数时，立即重算当前海拔供UI即时刷新。 */
    if (changed != 0 && s_baro_data.pressure_pa > 0)
    {
        s_baro_data.alt_m = baro_altitude((float)s_baro_data.pressure_pa);
    }
    baro_unlock();
}

/* 提交已知海拔(-1000..9000m)校准请求，需先同步手机海平面气压；返回0=已受理，1=越界或未同步。 */
uint8_t app_baro_calibrate_altitude(int16_t altitude_m)
{
    /* 限制输入海拔在-1000..9000m合理范围内。 */
    if (altitude_m < -1000 || altitude_m > 9000)
    {
        return 1;
    }
    baro_lock();
    /* 必须先收到手机海平面气压参考才能校准。 */
    if (s_baro_data.reference_ready == 0)
    {
        baro_unlock();
        return 1;
    }
    /* 记录目标海拔并置等待态，由sensor任务下一轮新采样计算压力偏差。 */
    s_calibration_altitude_m = altitude_m;
    s_baro_data.calibration_state = APP_BARO_CAL_PENDING;
    baro_unlock();
    return 0;
}

/* 锁内查询当前是否处于待校准状态(APP_BARO_CAL_PENDING)。 */
uint8_t app_baro_calibration_pending(void)
{
    uint8_t pending;
    baro_lock();
    pending = s_baro_data.calibration_state == APP_BARO_CAL_PENDING;
    baro_unlock();
    return pending;
}

/* 清除本次开机的压力校准偏差与待处理请求，并按当前参考重新计算海拔。 */
void app_baro_reset_calibration(void)
{
    baro_lock();
    s_pressure_correction_pa = 0.0f;
    s_baro_data.calibration_state = APP_BARO_CAL_NONE;
    s_baro_data.alt_m = baro_altitude((float)s_baro_data.pressure_pa);
    baro_unlock();
}

/* 加锁复制快照并返回在线位；NULL返回0，不访问器件。 */
uint8_t App_Baro_GetSnapshot(app_baro_struct *baro)
{
    uint8_t online; /* 锁内取出的在线标志，函数返回它。 */

    /* 输出指针为空时直接返回 0，不读器件也不写内存。 */
    if (baro == NULL)
    {
        return 0;
    }

    /* 锁内整体复制快照结构体并读在线位，保证返回值和数据对应同一时刻。 */
    baro_lock();
    *baro = s_baro_data;
    online = s_online;
    baro_unlock();

    /* 1 表示快照有效(最近一次采样成功且未判离线)，0 表示离线或首帧未完成。 */
    return online;
}
