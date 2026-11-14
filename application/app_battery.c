/**
 * @file app_battery.c
 * @brief PA1/ADC1_IN1 单次转换：启动/亮屏1s，启动稳定后熄屏10s，不在UI中等待ADC。
 */
#include "app_battery.h"
#include "app_diag.h"
#include "battery_adc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

#define APP_BATTERY_ADC_TIMEOUT_MS          2 /* 轮询上限：转换等待使用有限超时，失败则发布本轮错误状态 */
#define APP_BATTERY_STARTUP_SAMPLES         8 /* 启动阶段采样次数 */
#define APP_BATTERY_STARTUP_AVERAGE_SAMPLES 4 /* 取最后4次平均作为低通初值，丢掉上电头几帧 */

static SemaphoreHandle_t s_battery_mux;
static app_battery_struct s_battery_data;
/* 以下节拍与滤波状态只有 sensor 任务读写，快照字段才进锁。 */
static uint32_t s_attempt_ms;      /* 上次发起转换的时刻，用于按周期判定到期 */
static uint32_t s_filter_ms;       /* 上次推进低通的采样时刻，按经过秒数补步长 */
static int32_t s_filtered_mv_q8;   /* 1/8指数低通的Q8定点状态，单位256分之一mV */
static uint32_t s_startup_sum_mv;  /* 启动末4次电压累加，用于低通初值 */
static uint8_t s_startup_samples;  /* 启动阶段已完成次数，等于8后滤波才生效 */
static uint8_t s_last_percent;     /* 本次开机已发布的电量下限(只降不升) */
static uint8_t s_sleep_mode;       /* sensor任务写入的熄屏标志，决定当轮周期档位 */

static uint8_t battery_percent(uint32_t voltage_mv)
{
    /* 2026-10-31按同电池未充电日志归一化：3.5V为0%，4.0V为100%，节点间线性插值。 */
    /* 仅定义显示电量下限，不触发关机；31点中位数及计算依据见docs/battery-log.md。 */
    static const uint16_t voltage_points_mv[] = {3500, 3550, 3600, 3650, 3700, 3750, 3800, 3850, 3900, 3950, 4000};
    static const uint8_t percent_points[] = {0, 14, 24, 36, 51, 62, 69, 79, 88, 94, 100};
    uint32_t i; /* 从第 1 个电压节点开始向后查找的数组下标。 */
    /* 低于最低节点 3.5V 时直接返回 0%，不再插值。 */
    if (voltage_mv <= voltage_points_mv[0])
    {
        return 0;
    }
    for (i = 1; i < sizeof(voltage_points_mv) / sizeof(voltage_points_mv[0]); ++i)
    {
        /* 找到第一个高于当前电压的节点，说明落在 [i-1, i) 这一段内。 */
        if (voltage_mv < voltage_points_mv[i])
        {
            /* 本段电压跨度(mV)与本段百分比跨度，用于线性插值。 */
            uint32_t span_mv = voltage_points_mv[i] - voltage_points_mv[i - 1];
            uint32_t rise = (voltage_mv - voltage_points_mv[i - 1]) * (percent_points[i] - percent_points[i - 1]);
            /* 左端点百分比加上按比例折算的增量，+span_mv/2 做整数四舍五入。 */
            return (uint8_t)(percent_points[i - 1] + (rise + span_mv / 2) / span_mv);
        }
    }
    /* 高过最高节点 4.0V：按满电 100% 返回。 */
    return 100;
}

/* ADC通道由MX_ADC1_Init配置；电池分压倍率在换算中使用。本层只管节拍、滤波和快照。 */
void app_battery_init(void)
{
    /* 先创建快照互斥锁；创建失败直接进错误处理，不再往下初始化。 */
    s_battery_mux = xSemaphoreCreateMutex();
    if (s_battery_mux == NULL)
    {
        Error_Handler();
        return;
    }
    /* 加锁把整份快照清零：电压、原始值、结果、valid、percent 全部为 0。 */
    xSemaphoreTake(s_battery_mux, portMAX_DELAY);
    memset(&s_battery_data, 0, sizeof(s_battery_data));
    xSemaphoreGive(s_battery_mux);
    s_attempt_ms = HAL_GetTick() - APP_BATTERY_PERIOD_MS; /* 起点前移一整周期，首轮立即尝试转换；完成8次有效采样后才发布有效电压 */
    /* 启动计数与累加清零，随后 8 次采样重新累计。 */
    s_startup_samples = 0;
    s_startup_sum_mv = 0;
    /* 电量下限从 0 起，熄屏标志默认亮屏档(0)。 */
    s_last_percent = 0;
    s_sleep_mode = 0;
    /* 诊断通道登记一次初始结果 0，表示尚无错误。 */
    app_diag_sensor_init(APP_DIAG_BATTERY, 0);
}

void app_battery_set_sleep_mode(uint8_t sleeping)
{
    /* 由 sensor 任务按屏幕状态切换周期档位；读写同任务，不需要加锁。 */
    /* 参数归一成 0/1 存进熄屏标志，service 每轮据此选 1s 或 10s 周期。 */
    s_sleep_mode = sleeping != 0;
}

/* 每轮至多一笔转换。快照整体替换：本轮失败时valid清0，不向UI沿用旧电压。 */
void app_battery_service(void)
{
    app_battery_struct sample = {0}; /* 本轮结果先攒在局部，末尾整体替换快照。 */
    uint32_t now = HAL_GetTick();    /* 本轮进入时刻(ms)，用于周期判断与低通补步。 */
    uint16_t raw = 0;                /* 本次 12 位 ADC 原始值，0~4095。 */
    uint32_t read_started;           /* 采样开始时刻，仅用于诊断耗时。 */
    uint32_t period_ms = APP_BATTERY_PERIOD_MS; /* 当轮周期，默认亮屏/启动档 1s。 */
    uint32_t filter_steps;           /* 本次需要推进的低通步数，按经过的秒数折算。 */
    /* 熄屏降频只在启动平均完成后生效，避免熄屏打断前8帧均值。 */
    if (s_sleep_mode != 0 && s_startup_samples == APP_BATTERY_STARTUP_SAMPLES)
    {
        /* 熄屏且启动 8 帧已完成：周期放宽到 10s 一采。 */
        period_ms = APP_BATTERY_SLEEP_PERIOD_MS;
    }
    /* 锁未建或距上次尝试不足一个周期时直接返回，本轮不做转换。 */
    if (s_battery_mux == NULL || now - s_attempt_ms < period_ms)
    {
        return;
    }
    /* 记录本次尝试时刻，下一轮从这里起算周期。 */
    s_attempt_ms = now;
    read_started = app_diag_clock_count();
    sample.result = battery_adc_read_raw(APP_BATTERY_ADC_TIMEOUT_MS, &raw);
    if (sample.result == HAL_OK)
    {
        /* 整条 ADC 流程成功：保存原始值供诊断显示。 */
        sample.adc_raw = (uint16_t)raw;
        /* 12位ADC满量程4095，按1:2分压还原电池端电压，+2047为四舍五入。 */
        sample.voltage_mv = (raw * APP_BATTERY_VREF_MV * 2 + 2047) / 4095;
        /* 启动前4次不入均值，后4次均值作为初值；沿用每秒调度，转换等待带有限超时。 */
        if (s_startup_samples < APP_BATTERY_STARTUP_SAMPLES)
        {
            /* 启动阶段：先累加本次完成次数，再决定这帧是否进入平均。 */
            ++s_startup_samples;
            if (s_startup_samples > APP_BATTERY_STARTUP_SAMPLES - APP_BATTERY_STARTUP_AVERAGE_SAMPLES)
            {
                /* 第 5~8 次(末 4 帧)才累加进 s_startup_sum_mv。 */
                s_startup_sum_mv += sample.voltage_mv;
            }
            if (s_startup_samples == APP_BATTERY_STARTUP_SAMPLES)
            {
                /* 第 8 帧到齐：4 帧均值转成 Q8(乘256)作为低通初值。 */
                s_filtered_mv_q8 = (int32_t)(s_startup_sum_mv * 256 / APP_BATTERY_STARTUP_AVERAGE_SAMPLES);
                /* 记录低通基准时刻，并从此刻起允许熄屏降频。 */
                s_filter_ms = now;
                /* 用初值算出首个电量下限，+128 是把 Q8 四舍五入回 mV。 */
                s_last_percent = battery_percent((uint32_t)(s_filtered_mv_q8 + 128) / 256);
            }
        }
        else
        {
            /* 按经过秒数推进1/8低通，熄屏10s采样不把响应拖慢十倍；长故障后最多补10步。 */
            filter_steps = (uint32_t)(now - s_filter_ms) / APP_BATTERY_PERIOD_MS;
            if (filter_steps > APP_BATTERY_SLEEP_PERIOD_MS / APP_BATTERY_PERIOD_MS)
            {
                /* 最多补 10 步(对应熄屏 10s)，防止长时间停顿后一次跑很多轮。 */
                filter_steps = APP_BATTERY_SLEEP_PERIOD_MS / APP_BATTERY_PERIOD_MS;
            }
            while (filter_steps != 0)
            {
                /* 每步把 Q8 状态向本次电压靠拢 1/8：差值除以 8 后累加。 */
                s_filtered_mv_q8 += ((int32_t)sample.voltage_mv * 256 - s_filtered_mv_q8) / 8;
                --filter_steps;
            }
            /* 低通已推进到当前时刻，刷新基准时间。 */
            s_filter_ms = now;
        }
        if (s_startup_samples == APP_BATTERY_STARTUP_SAMPLES)
        {
            /* 启动 8 帧完成：把 Q8 低通值四舍五入回 mV 作为对外电压。 */
            sample.voltage_mv = (uint32_t)(s_filtered_mv_q8 + 128) / 256;
            /* 用滤波后电压查表得到本轮百分比。 */
            sample.percent = battery_percent(sample.voltage_mv);
            /* 尚无充电状态输入：本次开机电量只降不升，电压回弹不代表补充了电荷。 */
            if (sample.percent < s_last_percent)
            {
                /* 新百分比更低才更新下限，否则保持旧值。 */
                s_last_percent = sample.percent;
            }
            /* 对外只发布只降不升的下限值。 */
            sample.percent = s_last_percent;
            /* 电压与百分比都可用，置有效位供 UI 展示。 */
            sample.valid = 1;
        }
        else
        {
            /* 启动 8 帧未满：本轮不发布电压，清 0 且 valid 保持 0。 */
            sample.voltage_mv = 0;
        }
    }
    if (sample.result == HAL_OK)
    {
        /* 整轮成功：记一条 OK 诊断，附本轮结果码与耗时。 */
        app_diag_sensor_record(APP_DIAG_BATTERY, APP_DIAG_OK, (uint32_t)sample.result, read_started);
    }
    else
    {
        /* 任一步失败：记错误诊断，sample 里 valid 仍为 0。 */
        app_diag_sensor_record(APP_DIAG_BATTERY, APP_DIAG_ERROR, (uint32_t)sample.result, read_started);
    }
    /* 加锁用本轮 sample 整体替换快照，失败帧也会覆盖掉旧电压。 */
    xSemaphoreTake(s_battery_mux, portMAX_DELAY);
    s_battery_data = sample;
    xSemaphoreGive(s_battery_mux);
}

/* 在快照锁内整体复制，避免UI读到半新半旧的字段撕裂。 */
void app_battery_get_snapshot(app_battery_struct *snapshot)
{
    /* 输出指针为空时不处理，避免写空地址。 */
    if (snapshot == NULL)
    {
        return;
    }
    /* 锁未建时无法复制，输出一份全零快照。 */
    if (s_battery_mux == NULL)
    {
        memset(snapshot, 0, sizeof(*snapshot));
        return;
    }
    /* 锁内整体复制五个字段，保证 UI 读到同一轮的结果。 */
    xSemaphoreTake(s_battery_mux, portMAX_DELAY);
    *snapshot = s_battery_data;
    xSemaphoreGive(s_battery_mux);
}
