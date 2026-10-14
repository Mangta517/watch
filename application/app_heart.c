/**
 * @file app_heart.c
 * @brief 去背景、低通、多门限峰谷检测及RR方差选优；有效范围40～200BPM。
 */
#include "app_heart.h"
#include "app_diag.h"
#include "em7028.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <math.h>
#include <string.h>

#define APP_HEART_POLL_MS      30U /* 现有 sensor 任务为 10ms，覆盖器件的 25ms 档 */
#define APP_HEART_WARMUP_MS    1000U
#define APP_HEART_MAX_GAP_MS   120U /* 长时间缺采样后重新建立滤波与周期 */
#define APP_HEART_STALE_MS     250U
#define APP_HEART_MIN_BEAT_MS  300U  /* 200 BPM */
#define APP_HEART_MAX_BEAT_MS  1500U /* 40 BPM */
#define APP_HEART_BEAT_TIMEOUT 2500U
#define APP_HEART_MIN_SIGNAL   8.0f /* 峰谷反向变化的最小幅度；单位 ADC 计数 */
#define APP_HEART_STABLE_BEATS 3U
#define APP_HEART_RR_TOLERANCE 0.35f /* 单路原始峰间隔容差；最终平滑周期仍检查25%连续性 */
#define APP_HEART_MIN_BPM      40U
#define APP_HEART_MAX_BPM      200U
#define APP_HEART_GATE_COUNT   5U
#define APP_HEART_SAMPLE_COUNT 32U
#define APP_HEART_JUMP_COUNTS  1024U /* 突变门限下限；还须超过上一原始值25%，实板诊断参数 */

/* HeartPy fit_peaks 的多门限/RR方差选优思路，独立实现为固定内存的C实时版本。 */
static const float heart_gate_factors[APP_HEART_GATE_COUNT] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f};

typedef struct
{
    float cycle_peak;
    float trough;
    float swing;
    float interval_ms;
    float rr[APP_HEART_STABLE_BEATS];
    uint32_t beat_tick;
    uint32_t candidate_tick;
    uint32_t peak_tick;
    uint16_t last_interval_ms;
    uint8_t armed;
    uint8_t have_beat;
    uint8_t stable_beats;
    uint8_t rr_next;
} heart_peak_struct;

typedef struct
{
    float dc;
    float filtered;
    float amplitude;
    uint32_t warmup_tick;
    uint32_t output_beat_tick;
    float output_interval_ms;
    uint16_t previous_raw;
    uint8_t seeded;
    heart_peak_struct peaks[APP_HEART_GATE_COUNT];
} heart_detector_struct;

static SemaphoreHandle_t heart_mutex;
static app_heart_struct heart_data;
static heart_detector_struct detector;
static uint8_t requested;
static uint8_t running;
static uint32_t poll_tick;
static uint32_t good_tick;
static app_heart_sample_struct heart_samples[APP_HEART_SAMPLE_COUNT];
static uint16_t sample_head;
static uint16_t sample_count;
static uint16_t sample_sequence;
static uint32_t sample_start_tick;

static void heart_publish(const app_heart_struct *data)
{
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    heart_data = *data;
    xSemaphoreGive(heart_mutex);
}

static void heart_fail(app_heart_struct *data)
{
    em7028_get_fault(&data->fault); /* 先保留读/启动失败，不能被 stop 的结果覆盖 */
    (void)em7028_stop();            /* 即使总线失败也先关闭外部照明 */
    running = 0;
    data->bpm = 0;
    data->state = APP_HEART_ERROR;
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    requested = 0;
    sample_head = sample_count = 0;
    heart_data = *data;
    xSemaphoreGive(heart_mutex);
}

static void heart_reset_peaks(uint32_t now)
{
    unsigned i;
    memset(detector.peaks, 0, sizeof(detector.peaks));
    for (i = 0; i < APP_HEART_GATE_COUNT; ++i)
    {
        detector.peaks[i].trough = detector.peaks[i].cycle_peak = detector.filtered;
        detector.peaks[i].peak_tick = now;
    }
}

/* 每个门限独立记录峰谷和周期，避免一个门限的错峰污染其他候选。 */
static void heart_track_peak(heart_peak_struct *peak, float factor, uint32_t now)
{
    float threshold = peak->swing * factor;
    float swing;
    uint32_t interval;
    uint32_t candidate_interval;
    if (peak->have_beat != 0 && (uint32_t)(now - peak->beat_tick) > APP_HEART_BEAT_TIMEOUT)
    {
        memset(peak, 0, sizeof(*peak));
        peak->trough = peak->cycle_peak = detector.filtered;
        peak->peak_tick = now;
        return;
    }
    if (threshold < APP_HEART_MIN_SIGNAL)
    {
        threshold = APP_HEART_MIN_SIGNAL;
    }
    if (peak->armed == 0)
    {
        if (detector.filtered < peak->trough)
        {
            peak->trough = detector.filtered;
        }
        if (detector.filtered - peak->trough >= threshold)
        {
            peak->armed = 1;
            peak->cycle_peak = detector.filtered;
            peak->peak_tick = now;
        }
        return;
    }
    if (detector.filtered > peak->cycle_peak)
    {
        peak->cycle_peak = detector.filtered;
        peak->peak_tick = now;
    }
    if (peak->cycle_peak - detector.filtered < threshold)
    {
        return;
    }
    swing = peak->cycle_peak - peak->trough;
    peak->armed = 0;
    peak->trough = detector.filtered;
    if (swing < APP_HEART_MIN_SIGNAL * 2.0f)
    {
        peak->have_beat = 0;
        peak->stable_beats = 0;
        peak->swing = 0.0f;
        return;
    }
    if (peak->swing == 0.0f)
    {
        peak->swing = swing;
    }
    else
    {
        peak->swing = peak->swing * 0.75f + swing * 0.25f;
    }
    interval = (uint32_t)(peak->peak_tick - peak->beat_tick);
    candidate_interval = (uint32_t)(peak->peak_tick - peak->candidate_tick);
    peak->candidate_tick = peak->peak_tick;
    if (peak->have_beat == 0)
    {
        peak->beat_tick = peak->peak_tick;
        peak->have_beat = 1;
        return;
    }
    /* 短次峰保留有效心跳；检查相邻候选间隔，避免把持续高频峰隔拍拼成心率。 */
    if (candidate_interval + APP_HEART_POLL_MS < APP_HEART_MIN_BEAT_MS)
    {
        if (peak->stable_beats == 0)
        {
            peak->last_interval_ms = (uint16_t)candidate_interval;
        }
        return;
    }
    peak->beat_tick = peak->peak_tick;
    if (interval > UINT16_MAX)
    {
        peak->last_interval_ms = UINT16_MAX;
    }
    else
    {
        peak->last_interval_ms = (uint16_t)interval;
    }
    /* 允许一帧采样量化误差，最终输出仍必须在40～200BPM内。 */
    if (interval + APP_HEART_POLL_MS < APP_HEART_MIN_BEAT_MS || interval > APP_HEART_MAX_BEAT_MS + APP_HEART_POLL_MS)
    {
        peak->stable_beats = 0;
        return;
    }
    if (peak->stable_beats == 0 || fabsf((float)interval - peak->interval_ms) > peak->interval_ms * APP_HEART_RR_TOLERANCE)
    {
        peak->interval_ms = (float)interval;
        peak->rr[0] = (float)interval;
        peak->stable_beats = 1;
        peak->rr_next = 1;
        return;
    }
    peak->interval_ms = peak->interval_ms * 0.75f + (float)interval * 0.25f;
    peak->rr[peak->rr_next] = (float)interval;
    peak->rr_next = (uint8_t)((peak->rr_next + 1U) % APP_HEART_STABLE_BEATS);
    if (peak->stable_beats < APP_HEART_STABLE_BEATS)
    {
        ++peak->stable_beats;
    }
}

/* 在合法且与近期有效周期连续的候选中按RR方差选优。
 * 五组共享滤波，三个连续相近间隔后才输出；旧周期超过2.5秒未更新时允许重新选定。 */
static void heart_process(uint16_t raw, uint32_t now, uint32_t elapsed, app_heart_struct *data)
{
    heart_peak_struct *best = NULL;
    float best_variance = 0.0f;
    float mean, variance, difference;
    uint16_t jump_limit, raw_difference;
    uint16_t bpm;
    unsigned i, j;
    uint8_t active = 0;
    data->raw = raw;
    data->sample_ms = now;
    if (raw > detector.previous_raw)
    {
        raw_difference = raw - detector.previous_raw;
    }
    else
    {
        raw_difference = detector.previous_raw - raw;
    }
    jump_limit = detector.previous_raw / 4U;
    if (jump_limit < APP_HEART_JUMP_COUNTS)
    {
        jump_limit = APP_HEART_JUMP_COUNTS;
    }
    /* 极值/大幅阶跃重建基线；上一帧滤波偏移也覆盖多帧累计的接触变化。原始值仍照常导出。 */
    if (raw == 0 || raw == UINT16_MAX || (detector.seeded != 0 && (raw_difference > jump_limit ||
        (fabsf(detector.filtered) > APP_HEART_JUMP_COUNTS && fabsf(detector.filtered) > detector.dc * 0.25f))))
    {
        memset(&detector, 0, sizeof(detector));
    }
    if (detector.seeded == 0)
    {
        detector.dc = (float)raw;
        detector.warmup_tick = now;
        detector.seeded = 1;
    }
    detector.previous_raw = raw;
    if ((uint32_t)(now - detector.warmup_tick) < APP_HEART_WARMUP_MS)
    {
        /* 启动/重建期间跟随当前背景，避免首帧未稳定让1.5秒背景滤波长时间追赶。 */
        detector.dc = (float)raw;
        detector.filtered = detector.amplitude = 0.0f;
        heart_reset_peaks(now);
        data->signal_x10 = 0;
        data->interval_ms = 0;
        data->bpm = 0;
        if (raw == 0 || raw == UINT16_MAX)
        {
            data->state = APP_HEART_NO_SIGNAL;
        }
        else
        {
            data->state = APP_HEART_MEASURING;
        }
        return;
    }
    detector.dc += (float)elapsed / (1500.0f + (float)elapsed) * ((float)raw - detector.dc);
    detector.filtered += (float)elapsed / (70.0f + (float)elapsed) * ((float)raw - detector.dc - detector.filtered);
    detector.amplitude += (float)elapsed / (300.0f + (float)elapsed) * (fabsf(detector.filtered) - detector.amplitude);
    data->signal_x10 = (uint32_t)(detector.amplitude * 10.0f + 0.5f);
    for (i = 0; i < APP_HEART_GATE_COUNT; ++i)
    {
        heart_peak_struct *peak = &detector.peaks[i];
        heart_track_peak(peak, heart_gate_factors[i], now);
        active |= peak->have_beat;
        if (peak->stable_beats < APP_HEART_STABLE_BEATS)
        {
            continue;
        }
        bpm = (uint16_t)(60000.0f / peak->interval_ms + 0.5f);
        if (bpm < APP_HEART_MIN_BPM || bpm > APP_HEART_MAX_BPM)
        {
            continue;
        }
        /* 最近有效周期仍新鲜时，只在原25%周期容差内选优；失去连续性就重新测量。 */
        if (detector.output_interval_ms != 0.0f && (uint32_t)(now - detector.output_beat_tick) <= APP_HEART_BEAT_TIMEOUT &&
            fabsf(peak->interval_ms - detector.output_interval_ms) > detector.output_interval_ms * 0.25f)
        {
            continue;
        }
        mean = 0.0f;
        for (j = 0; j < APP_HEART_STABLE_BEATS; ++j)
        {
            mean += peak->rr[j] / (float)APP_HEART_STABLE_BEATS;
        }
        variance = 0.0f;
        for (j = 0; j < APP_HEART_STABLE_BEATS; ++j)
        {
            difference = peak->rr[j] - mean;
            variance += difference * difference / (float)APP_HEART_STABLE_BEATS;
        }
        /* 方差相等时选较高门限，减少重复识别小次峰。 */
        if (best == NULL || variance <= best_variance)
        {
            best = peak;
            best_variance = variance;
        }
    }
    if (best != NULL)
    {
        detector.output_interval_ms = best->interval_ms;
        detector.output_beat_tick = best->beat_tick;
        data->bpm = (uint16_t)(60000.0f / best->interval_ms + 0.5f);
        data->interval_ms = best->last_interval_ms;
        data->state = APP_HEART_VALID;
        return;
    }
    data->bpm = 0;
    data->interval_ms = detector.peaks[2].last_interval_ms; /* 未选优时保留原30%门限的周期诊断 */
    if (active != 0 || (uint32_t)(now - detector.warmup_tick) <= APP_HEART_BEAT_TIMEOUT)
    {
        data->state = APP_HEART_MEASURING;
    }
    else
    {
        data->state = APP_HEART_NO_SIGNAL;
    }
}

void app_heart_init(void)
{
    heart_mutex = xSemaphoreCreateMutex();
}

void app_heart_set_enabled(uint8_t enabled)
{
    if (heart_mutex == NULL)
    {
        return;
    }
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    requested = enabled != 0;
    sample_head = sample_count = 0;
    xSemaphoreGive(heart_mutex);
}

void app_heart_get_snapshot(app_heart_struct *heart)
{
    if (heart == NULL)
    {
        return;
    }
    if (heart_mutex == NULL)
    {
        memset(heart, 0, sizeof(*heart));
        heart->state = APP_HEART_ERROR;
        return;
    }
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    *heart = heart_data;
    heart->enabled = requested;
    xSemaphoreGive(heart_mutex);
    if (heart->enabled != 0 && (heart->state == APP_HEART_OFF || heart->state == APP_HEART_ERROR))
    {
        heart->state = APP_HEART_MEASURING; /* 启动请求可能正在等待 sensor 任务处理 */
    }
    if (heart->enabled == 0 && heart->state != APP_HEART_ERROR)
    {
        heart->bpm = 0;
        heart->state = APP_HEART_OFF;
    }
    if (heart->state == APP_HEART_VALID && (uint32_t)(HAL_GetTick() - heart->sample_ms) > APP_HEART_STALE_MS)
    {
        heart->bpm = 0;
        heart->state = APP_HEART_NO_SIGNAL;
    }
}

uint16_t app_heart_read_samples(app_heart_sample_struct *samples, uint16_t size)
{
    uint16_t count = 0;
    if (samples == NULL || heart_mutex == NULL)
    {
        return 0;
    }
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    while (count < size && sample_count != 0)
    {
        samples[count++] = heart_samples[sample_head];
        sample_head = (uint16_t)((sample_head + 1U) % APP_HEART_SAMPLE_COUNT);
        --sample_count;
    }
    xSemaphoreGive(heart_mutex);
    return count;
}

void app_heart_service(void)
{
    app_heart_struct data;
    app_heart_sample_struct sample;
    em7028_result result;
    uint16_t raw;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed;
    uint8_t enabled;
    uint32_t read_started;
    if (heart_mutex == NULL)
    {
        return;
    }
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    enabled = requested;
    data = heart_data;
    xSemaphoreGive(heart_mutex);
    if (enabled == 0)
    {
        if (running != 0)
        {
            result = em7028_stop();
            running = 0;
            data.bpm = 0;
            if (result == EM7028_OK)
            {
                data.state = APP_HEART_OFF;
            }
            else
            {
                data.state = APP_HEART_ERROR;
            }
            if (result != EM7028_OK)
            {
                em7028_get_fault(&data.fault);
            }
            heart_publish(&data);
        }
        return;
    }
    if (running == 0)
    {
        result = em7028_start();
        app_diag_sensor_init(APP_DIAG_HEART, (uint32_t)result);
        if (result != EM7028_OK)
        {
            heart_fail(&data);
            return;
        }
        memset(&detector, 0, sizeof(detector));
        memset(&data, 0, sizeof(data));
        data.state = APP_HEART_MEASURING;
        poll_tick = good_tick = now;
        sample_start_tick = now;
        sample_sequence = 0;
        running = 1;
        heart_publish(&data);
        return;
    }
    elapsed = (uint32_t)(now - poll_tick);
    if (elapsed < APP_HEART_POLL_MS)
    {
        return;
    }
    poll_tick = now;
    if (elapsed > APP_HEART_MAX_GAP_MS)
    {
        memset(&detector, 0, sizeof(detector));
        data.bpm = 0;
        data.state = APP_HEART_MEASURING;
    }
    read_started = app_diag_clock_count();
    result = em7028_read_raw(&raw);
    app_diag_sensor_record(APP_DIAG_HEART, (uint8_t)result, (uint32_t)result, read_started);
    /* 短暂 NACK/跨更新只丢弃本帧；连续 250ms 无有效读数才关闭测量。 */
    if (result != EM7028_OK && (uint32_t)(now - good_tick) > APP_HEART_STALE_MS)
    {
        heart_fail(&data);
        return;
    }
    if (result == EM7028_OK)
    {
        elapsed = (uint32_t)(now - good_tick);
        if (elapsed > APP_HEART_MAX_GAP_MS)
        {
            memset(&detector, 0, sizeof(detector));
            data.bpm = 0;
            data.state = APP_HEART_MEASURING;
        }
        good_tick = now;
        heart_process(raw, now, elapsed, &data);
        sample.elapsed_ms = (uint32_t)(now - sample_start_tick);
        sample.raw = raw;
        sample.filtered = detector.filtered;
        sample.dc = detector.dc;
        sample.bpm = data.bpm;
        sample.interval_ms = data.interval_ms;
        sample.state = data.state;
        sample.sequence = sample_sequence++;
        /* 仅成功读数入队；与UI共用快照锁，UART发送时不占此锁。 */
        xSemaphoreTake(heart_mutex, portMAX_DELAY);
        if (requested != 0)
        {
            if (sample_count == APP_HEART_SAMPLE_COUNT)
            {
                app_diag_heart_queue(1);
                sample_head = (uint16_t)((sample_head + 1U) % APP_HEART_SAMPLE_COUNT);
                --sample_count;
            }
            heart_samples[(sample_head + sample_count) % APP_HEART_SAMPLE_COUNT] = sample;
            ++sample_count;
        }
        xSemaphoreGive(heart_mutex);
    }
    heart_publish(&data);
}
