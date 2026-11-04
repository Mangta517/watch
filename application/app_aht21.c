/**
 * @file app_aht21.c
 * @brief 复用现有 sensor 任务；触发转换后至少等 80ms 读回，忙状态最多等到 200ms。
 *        只有本层(单生产者)读写传感器与节拍状态，UI 通过 app_aht21_get_snapshot 取值。
 */
#include "app_aht21.h"
#include "app_diag.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

#define APP_AHT21_TIMEOUT_MS 200 /* 自触发算起允许推迟读回的最长毫秒数，超时即丢弃该次转换 */

static SemaphoreHandle_t s_aht21_mux;
static app_aht21_struct s_aht21_data; /* 仅锁内可跨任务读写的快照 */
/* 以下节拍状态只有 sensor 任务读写，不加锁：周期起点、转换起始、待到期、恢复流程。 */
static uint32_t s_period_start_ms, s_conversion_ms;
static uint8_t s_pending, s_sample_due;
static uint8_t s_recover_needed, s_recovering;
static uint32_t s_period_ms = APP_AHT21_PERIOD_MS;

/* sensor任务切换采样周期：缩短时置采样待办，转换或恢复在途则只更新周期起点、沿用本次事务；
 * 拉长时清掉待办与在途转换，在途恢复改为待恢复，并把measuring清零。 */
void app_aht21_set_period(uint32_t period_ms)
{
    /* 先比较新旧周期：新周期更短则为1，表示切换后要尽快补一次采样。 */
    uint8_t sample_due = period_ms < s_period_ms;
    /* 新周期立即生效，后续 service 的到期判断都按这个毫秒数计算。 */
    s_period_ms = period_ms;
    /* 把周期起点重设为当前 tick，切换周期后从这一刻重新计时。 */
    s_period_start_ms = HAL_GetTick();
    if (sample_due != 0 && (s_pending != 0 || s_recovering != 0))
    {
        return; /* 进入详情页沿用正在转换/恢复的本次采样，避免芯片忙时重复触发。 */
    }
    /* 周期拉长或本来就没有在途事务：清掉待读回标志，待办只由 sample_due 决定。 */
    s_pending = 0;
    s_sample_due = sample_due;
    if (s_recovering != 0)
    {
        /* 在途恢复不能中途丢弃：退成待恢复，由后续 service 重新发起恢复流程。 */
        s_recovering = 0;
        s_recover_needed = 1;
    }
    /* 加锁清掉快照里的 measuring 位，UI 不再显示正在测量。 */
    xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
    s_aht21_data.measuring = 0;
    xSemaphoreGive(s_aht21_mux);
}

/* sensor任务启动阶段调用：建快照锁并初始化驱动；初始化失败不阻塞启动，交给恢复流程。 */
void app_aht21_init(void)
{
    aht21_result result; /* 暂存驱动初始化的返回码，稍后同时写快照和诊断。 */
    /* 在 sensor 任务上下文创建快照锁；创建失败进入 Error_Handler，停止后续初始化。 */
    s_aht21_mux = xSemaphoreCreateMutex();
    if (s_aht21_mux == NULL)
    {
        Error_Handler();
        return;
    }
    /* 执行一次 AHT21 上电初始化，返回值决定是否需要走恢复流程。 */
    result = aht21_init();
    /* 把本次初始化结果登记到 AHT 诊断通道，供诊断页显示。 */
    app_diag_sensor_init(APP_DIAG_AHT, (uint32_t)result);
    /* 加锁把快照清零，只把 result 填成刚才的初始化结果，其余字段为 0。 */
    xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
    memset(&s_aht21_data, 0, sizeof(s_aht21_data));
    s_aht21_data.result = result;
    xSemaphoreGive(s_aht21_mux);
    /* 清空待读回与待采样两项待办，开机的第一帧由下面的周期起点前移来安排。 */
    s_pending = s_sample_due = 0;
    /* 初始化成功则无需恢复；失败则置待恢复，由 service 按周期尝试重新初始化。 */
    s_recover_needed = result != AHT21_OK;
    s_recovering = 0;
    s_period_ms = APP_AHT21_PERIOD_MS;
    s_period_start_ms = HAL_GetTick();
    if (s_recover_needed == 0)
    {
        /* 起点前移一整周期，首轮service立即触发首帧转换，至少80ms后才读回。 */
        s_period_start_ms -= s_period_ms;
    }
}

/* 每轮至多推进一笔事务：恢复中→到期触发→在途读回；等待转换期间直接返回，不占总线也不延时。 */
void app_aht21_service(void)
{
    aht21_struct sample;   /* 接收本次读回结果，成功后才整体拷进快照。 */
    aht21_result result;   /* 本笔事务(恢复/触发/读回)的驱动返回码。 */
    uint32_t now = HAL_GetTick();          /* 本轮进入时刻，用于恢复流程和采样时间戳。 */
    uint32_t elapsed = now - s_period_start_ms; /* 距周期起点已过的毫秒数，判断采样是否到期。 */
    uint32_t read_started; /* 读/触发开始时刻，仅用于诊断记录耗时。 */
    /* 锁还没建好(初始化未执行)时直接返回，不访问传感器。 */
    if (s_aht21_mux == NULL)
    {
        return;
    }
    if (s_recovering != 0)
    {
        /* 恢复=驱动层重新初始化，逐笔事务推进；BUSY 表示尚未结束，本轮不采样也不判失败。 */
        result = aht21_recover_step(now);
        if (result == AHT21_BUSY)
        {
            return;
        }
        /* 恢复已出结果：清恢复中标志，失败则继续挂待恢复，成功则挂一次待采样。 */
        s_recovering = 0;
        s_recover_needed = result != AHT21_OK;
        s_sample_due = result == AHT21_OK;
        /* 把恢复结果写入诊断，并把快照的 result 更新为该结果、valid 清0。 */
        app_diag_sensor_init(APP_DIAG_AHT, (uint32_t)result);
        xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
        s_aht21_data.result = result;
        s_aht21_data.valid = 0;
        xSemaphoreGive(s_aht21_mux);
        return;
    }
    if (s_recover_needed != 0)
    {
        /* 恢复按采样周期安排；恢复成功后置采样待办，由后续service触发。 */
        if (elapsed >= s_period_ms)
        {
            /* 起点按整周期前移，丢掉错过的周期，只补发起一次恢复。 */
            s_period_start_ms += elapsed / s_period_ms * s_period_ms;
            /* 初始化恢复状态并置恢复中标志，后续由recover_step等待并推进硬件事务。 */
            aht21_recover_begin(now);
            s_recovering = 1;
        }
        return;
    }
    if (s_pending != 0)
    {
        /* 有在途转换：改用转换起始时刻重新计算已过毫秒数，判断能否读回。 */
        elapsed = now - s_conversion_ms;
        if (elapsed < AHT21_CONVERSION_MS)
        {
            return;
        }
        read_started = app_diag_clock_count();
        if (elapsed > APP_AHT21_TIMEOUT_MS)
        {
            result = AHT21_TIMEOUT; /* 跨窗口的旧转换不能读回后再标成当前采样。 */
        }
        else
        {
            /* 转换已够时且未超时：从芯片读回本次温湿度并填入 sample。 */
            result = aht21_read(&sample);
        }
        if (result == AHT21_BUSY && elapsed < APP_AHT21_TIMEOUT_MS)
        {
            /* 芯片仍未转换完且未超时：记一次未就绪，保留 pending 下轮再读。 */
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_NOT_READY, (uint32_t)result, read_started);
            return;
        }
        if (result == AHT21_BUSY)
        {
            /* 达到200ms仍是BUSY，统一归为超时，结束这笔转换。 */
            result = AHT21_TIMEOUT;
        }
        if (result == AHT21_OK)
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_OK, (uint32_t)result, read_started);
        }
        else
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_ERROR, (uint32_t)result, read_started);
        }
        /* 无论成功失败，本次转换已收尾，清待读回标志。 */
        s_pending = 0;
        if (result == AHT21_CALIBRATION_ERROR)
        {
            /* 片上校准位丢失只能靠重新初始化解决，普通重试读回无意义。 */
            s_recover_needed = 1;
        }
        /* 加锁发布本轮结果：measuring 清0，valid 只有成功才置1。 */
        xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
        s_aht21_data.measuring = 0;
        s_aht21_data.result = result;
        s_aht21_data.valid = result == AHT21_OK;
        if (result == AHT21_OK)
        {
            /* 成功才拷贝采样值，并记录本次成功时刻(ms)与累计成功次数。 */
            s_aht21_data.sample = sample;
            s_aht21_data.sample_ms = HAL_GetTick();
            ++s_aht21_data.sample_count;
        }
        xSemaphoreGive(s_aht21_mux);
    }
    else if (s_sample_due != 0 || elapsed >= s_period_ms)
    {
        if (s_sample_due == 0)
        {
            /* 固定周期不跟随事务耗时后移；超期只测一次，不补跑旧采样。 */
            s_period_start_ms += elapsed / s_period_ms * s_period_ms;
        }
        /* 待办只消费一次，本轮发起触发后即清零。 */
        s_sample_due = 0;
        read_started = app_diag_clock_count();
        /* 向芯片下发一次转换命令，返回码决定是否进入待读回状态。 */
        result = aht21_trigger();
        if (result != AHT21_OK)
        {
            app_diag_sensor_record(APP_DIAG_AHT, APP_DIAG_ERROR, (uint32_t)result, read_started);
        }
        /* 记录触发时刻，后续按它计算 80ms 转换时间与 200ms 超时。 */
        s_conversion_ms = HAL_GetTick();
        /* 触发成功才置待读回，失败则本轮直接收尾。 */
        s_pending = result == AHT21_OK;
        /* 加锁把 measuring 与 result 发布给 UI；失败时同时清 valid，避免展示旧值。 */
        xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
        s_aht21_data.measuring = s_pending;
        s_aht21_data.result = result;
        if (result != AHT21_OK)
        {
            s_aht21_data.valid = 0;
        }
        xSemaphoreGive(s_aht21_mux);
    }
}

/* 在快照锁内整体复制，避免UI读到半新半旧的字段撕裂。 */
void app_aht21_get_snapshot(app_aht21_struct *snapshot)
{
    /* 输出指针为空时直接返回，不解引用。 */
    if (snapshot == NULL)
    {
        return;
    }
    /* 锁未建(初始化未跑)时无法复制，给调用方一份全零快照。 */
    if (s_aht21_mux == NULL)
    {
        memset(snapshot, 0, sizeof(*snapshot));
        return;
    }
    /* 锁内整体结构体赋值，保证 sample/valid/result 等字段来自同一时刻。 */
    xSemaphoreTake(s_aht21_mux, portMAX_DELAY);
    *snapshot = s_aht21_data;
    xSemaphoreGive(s_aht21_mux);
}

/* 供环境窗口判断能否提前收尾：在途转换、恢复流程或待办采样任一存在即返回1。 */
uint8_t app_aht21_is_busy(void)
{
    /* 只统计进行中的工作：在途转换、恢复流程、待读回；未到期(含未到期恢复)不算进行中。 */
    if (s_pending != 0 || s_recovering != 0 || s_sample_due != 0)
    {
        /* 三项任一非零即有工作未收尾，告知调用方还不能提前结束窗口。 */
        return 1;
    }
    /* 三项全为 0 表示当前没有待办事务。 */
    return 0;
}
