/* 真实心率服务，模拟 EM7028 波形、时钟和快照锁；不代表实板脉搏准确性。 */
#include "app_heart.h"
#include "em7028.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t tick;
static uint16_t raw = 2000;
static uint8_t locked;
static uint8_t running;
static em7028_result start_result = EM7028_OK;
static em7028_result read_result = EM7028_OK;
static em7028_result stop_result = EM7028_OK;
static unsigned reads;
static unsigned starts;
static uint8_t expect_rejected;
static em7028_fault read_fault = {EM7028_FAULT_READ, 0x29, 0};

void em7028_get_fault(em7028_fault *fault)
{
    assert(locked == 0 && fault != NULL);
    *fault = read_fault;
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return &locked;
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    assert(mutex == &locked && wait == portMAX_DELAY && locked == 0);
    locked = 1;
    return 1;
}

int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex == &locked && locked != 0);
    locked = 0;
    return 1;
}

em7028_result em7028_start(void)
{
    assert(locked == 0);
    ++starts;
    running = start_result == EM7028_OK;
    return start_result;
}

em7028_result em7028_stop(void)
{
    assert(locked == 0);
    running = 0;
    if (stop_result == EM7028_ERROR)
    {
        read_fault.kind = EM7028_FAULT_WRITE;
        read_fault.reg = 0x01;
    }
    return stop_result;
}

em7028_result em7028_read_raw(uint16_t *value)
{
    assert(locked == 0 && running != 0 && value != NULL);
    ++reads;
    if (read_result == EM7028_OK)
    {
        *value = raw;
        read_fault.kind = EM7028_FAULT_NONE;
    }
    else
    {
        read_fault.kind = read_result == EM7028_ERROR ? EM7028_FAULT_READ : EM7028_FAULT_DATA_CHANGED;
        read_fault.reg = 0x29;
    }
    return read_result;
}

static void begin(void)
{
    app_heart_set_enabled(0);
    app_heart_service();
    app_heart_set_enabled(1);
    app_heart_service();
}

static void wave(unsigned bpm, unsigned duration_ms, float amplitude)
{
    app_heart_struct heart;
    unsigned elapsed;
    for (elapsed = 0; elapsed < duration_ms; elapsed += 30)
    {
        tick += 30;
        raw = (uint16_t)(2000.0f + amplitude * sinf((float)elapsed * (float)bpm * 6.2831853f / 60000.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        assert(heart.state != APP_HEART_VALID || (heart.bpm >= 40 && heart.bpm <= 200));
        if (expect_rejected != 0)
        {
            assert(heart.state != APP_HEART_VALID && heart.bpm == 0);
        }
    }
}

/* 窄脉冲的平均 AC 在两次跳动之间下降，不能因此丢掉过零和周期历史。 */
static void pulse_wave(unsigned bpm, unsigned duration_ms, float amplitude, uint16_t baseline)
{
    unsigned elapsed;
    float phase;
    for (elapsed = 0; elapsed < duration_ms; elapsed += 30)
    {
        tick += 30;
        phase = fmodf((float)elapsed * (float)bpm / 60000.0f, 1.0f);
        raw = (uint16_t)((float)baseline + amplitude * expf(-powf((phase - 0.2f) / 0.065f, 2.0f)));
        app_heart_service();
    }
}

/* 接触压力/环境光缓慢漂移时，脉搏仍在，但去背景后的波形未必跨过零。 */
static void drift_wave(unsigned bpm, unsigned duration_ms, float amplitude, float slope)
{
    unsigned elapsed;
    for (elapsed = 0; elapsed < duration_ms; elapsed += 30)
    {
        tick += 30;
        raw =
            (uint16_t)(30000.0f + slope * (float)elapsed / 1000.0f + amplitude * sinf((float)elapsed * (float)bpm * 6.2831853f / 60000.0f));
        app_heart_service();
    }
}

/* 主峰后的次峰可使单一低门限重复计数；门限选优应保留主脉搏周期。 */
static void notched_wave(unsigned duration_ms)
{
    unsigned elapsed;
    float phase;
    for (elapsed = 0; elapsed < duration_ms; elapsed += 30)
    {
        tick += 30;
        phase = fmodf((float)elapsed / 1000.0f, 1.0f);
        raw =
            (uint16_t)(28000.0f + 120.0f * expf(-powf((phase - 0.22f) / 0.06f, 2.0f)) + 55.0f * expf(-powf((phase - 0.55f) / 0.08f, 2.0f)));
        app_heart_service();
    }
}

/* 每次主峰后180ms出现强次峰，不能反复清空真实的1000ms周期。 */
static void short_secondary_wave(unsigned duration_ms)
{
    app_heart_struct heart;
    unsigned elapsed;
    float phase;
    for (elapsed = 0; elapsed < duration_ms; elapsed += 30)
    {
        tick += 30;
        phase = fmodf((float)elapsed / 1000.0f, 1.0f);
        raw = (uint16_t)(40000.0f + 12000.0f * expf(-powf((phase - 0.18f) / 0.025f, 2.0f)) +
                         12000.0f * expf(-powf((phase - 0.36f) / 0.025f, 2.0f)));
        app_heart_service();
        if (elapsed >= 8000U)
        {
            app_heart_get_snapshot(&heart);
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
            assert(heart.interval_ms >= 970 && heart.interval_ms <= 1030);
        }
    }
}

/* 60BPM叠加90ms纹波；七点确认应排除短促局部假峰，跨tick回绕仍按真实时间计拍。 */
static void window_peak_tests(void)
{
    app_heart_struct heart;
    unsigned elapsed;
    uint32_t step;
    tick = UINT32_MAX - 3000U;
    begin();
    for (elapsed = 0; elapsed < 72000U; elapsed += 30U)
    {
        tick += 30U;
        raw = (uint16_t)(30000.0f + 100.0f * sinf((float)elapsed * 6.2831853f / 1000.0f) +
                         160.0f * sinf((float)elapsed * 6.2831853f / 90.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        if (elapsed >= 8000U)
        {
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 57U && heart.bpm <= 63U);
            assert(heart.interval_ms >= 930U && heart.interval_ms <= 1080U);
        }
    }
    /* 非固定采样间隔使用实际HAL tick，不能按样本下标估算心率。 */
    begin();
    elapsed = 0;
    while (elapsed < 18000U)
    {
        step = 30U;
        if (elapsed % 70U == 30U)
        {
            step = 40U;
        }
        elapsed += step;
        tick += step;
        raw = (uint16_t)(30000.0f + 100.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        if (elapsed >= 8000U)
        {
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 87U && heart.bpm <= 93U);
        }
    }
    puts("PASS: seven-point peaks retain 60BPM with 90ms ripple, 72s/tick wrap and timestamped uneven sampling.");
}

/* 峰间隔适度延长时保持连续识别；持续不相近的间隔仍须重新测量。 */
static void continuity_tests(void)
{
    static const unsigned periods[] = {810, 810, 810, 810, 810, 810, 1080, 1080, 1080, 1080, 810, 810, 810, 810, 810, 810};
    static const unsigned irregular_periods[] = {480, 1410};
    app_heart_struct heart;
    app_heart_sample_struct sample;
    unsigned elapsed, cycle_start, period_index, period;
    unsigned i;
    begin();
    cycle_start = period_index = 0;
    for (elapsed = 0; elapsed < 24000; elapsed += 30)
    {
        period = periods[period_index];
        if (elapsed - cycle_start >= period)
        {
            cycle_start += period;
            period_index = (period_index + 1U) % (sizeof(periods) / sizeof(periods[0]));
            period = periods[period_index];
        }
        tick += 30;
        raw = (uint16_t)(30000.0f + 300.0f * sinf((float)(elapsed - cycle_start) * 6.2831853f / (float)period));
        app_heart_service();
        if (elapsed >= 5000U)
        {
            app_heart_get_snapshot(&heart);
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 60 && heart.bpm <= 80);
        }
    }
    begin();
    cycle_start = period_index = 0;
    for (elapsed = 0; elapsed < 18000; elapsed += 30)
    {
        period = irregular_periods[period_index];
        if (elapsed - cycle_start >= period)
        {
            cycle_start += period;
            period_index = (period_index + 1U) % (sizeof(irregular_periods) / sizeof(irregular_periods[0]));
            period = irregular_periods[period_index];
        }
        tick += 30;
        raw = (uint16_t)(30000.0f + 300.0f * sinf((float)(elapsed - cycle_start) * 6.2831853f / (float)period));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        assert(heart.state != APP_HEART_VALID && heart.bpm == 0);
    }
    /* 每帧只变约2%～3%，但累计移开变化不能继续沿用旧周期。 */
    begin();
    pulse_wave(60, 12000, 800.0f, 50000);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    for (i = 1; i <= 20; ++i)
    {
        tick += 30;
        raw = (uint16_t)(50000U - i * 1000U);
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_MEASURING && heart.bpm == 0 && heart.interval_ms == 0);
    assert(app_heart_read_samples(&sample, 1) != 0);
    while (app_heart_read_samples(&sample, 1) != 0)
    {
    }
    assert(sample.raw == 30000U && sample.filtered == 0.0f);
    pulse_wave(60, 12000, 800.0f, 30000);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    puts("PASS: 810->1080ms intervals stay continuous, irregular intervals rejected, cumulative contact ramp resets/reacquires.");
}

/* 已建立周期后，较迟且较弱的一拍应更新真实峰间隔，不能只延长旧BPM的显示。 */
static void delayed_weak_beat_tests(void)
{
    app_heart_struct heart;
    unsigned amplitude, elapsed, mode;
    float strength, phase;
    for (amplitude = 80; amplitude <= 90; amplitude += 10)
    {
        begin();
        pulse_wave(60, 12000, 800.0f, 30000);
        for (elapsed = 0; elapsed < 7000; elapsed += 30)
        {
            strength = 800.0f;
            if (elapsed < 1000U)
            {
                strength = (float)amplitude;
            }
            /* 下一拍从200ms延后到400ms，只有这一拍的幅度下降到原来的约1/10。 */
            phase = fmodf((float)elapsed, 1000.0f);
            tick += 30;
            raw = (uint16_t)(30000.0f + strength * expf(-powf((phase - 400.0f) / 65.0f, 2.0f)));
            app_heart_service();
            app_heart_get_snapshot(&heart);
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 55 && heart.bpm <= 63);
        }
        assert(heart.interval_ms >= 970U && heart.interval_ms <= 1030U);
    }
    /* 门限降低后，平直信号和持续高频波也必须在旧波峰超时后撤销输出。 */
    for (mode = 0; mode < 2; ++mode)
    {
        begin();
        pulse_wave(60, 12000, 800.0f, 30000);
        for (elapsed = 0; elapsed < 8000; elapsed += 30)
        {
            tick += 30;
            raw = 30000;
            if (mode != 0)
            {
                raw = (uint16_t)(30000.0f + 80.0f * sinf((float)elapsed * 6.2831853f / 120.0f));
            }
            app_heart_service();
            app_heart_get_snapshot(&heart);
            if (elapsed >= 2700U)
            {
                assert(heart.state != APP_HEART_VALID && heart.bpm == 0);
            }
        }
    }
    puts("PASS: delayed weak beat updates RR without dropout; flat/high-frequency input expires after a valid train.");
}

static void recovery_tests(void)
{
    app_heart_struct heart;
    app_heart_sample_struct sample;
    unsigned i;
    uint32_t restart_tick;
    begin();
    tick += 30;
    raw = 17;
    app_heart_service();
    tick += 44;
    raw = 35674;
    app_heart_service();
    assert(app_heart_read_samples(&sample, 1) == 1 && sample.raw == 17);
    assert(app_heart_read_samples(&sample, 1) == 1 && sample.raw == 35674);
    assert(sample.filtered == 0.0f && sample.dc == 35674.0f && sample.bpm == 0 && sample.interval_ms == 0);
    for (i = 0; i < 30; ++i)
    {
        tick += 30;
        raw = (uint16_t)(35674U + i);
        app_heart_service();
        app_heart_get_snapshot(&heart);
        assert(heart.state == APP_HEART_MEASURING && heart.bpm == 0 && heart.interval_ms == 0);
        assert(app_heart_read_samples(&sample, 1) == 1 && sample.filtered == 0.0f && sample.dc == (float)raw);
    }
    begin();
    wave(60, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    /* 清掉旧诊断队列，只比较本次突变以及下一帧恢复；原始读数不被替换。 */
    while (app_heart_read_samples(&sample, 1) != 0)
    {
    }
    tick += 30;
    raw = 65531;
    app_heart_service();
    assert(app_heart_read_samples(&sample, 1) == 1 && sample.raw == 65531);
    assert(sample.state == APP_HEART_MEASURING && sample.bpm == 0 && sample.interval_ms == 0 && sample.filtered == 0.0f);
    tick += 30;
    raw = 2000;
    app_heart_service();
    assert(app_heart_read_samples(&sample, 1) == 1 && sample.raw == 2000 && sample.filtered == 0.0f);
    restart_tick = tick;
    wave(60, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    assert((uint32_t)(tick - restart_tick) == 12000U);
    /* 周期真的改变后允许重新确认，不能永久锁在上一次BPM。 */
    wave(120, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 117 && heart.bpm <= 123);
    wave(60, 14000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    raw = 0;
    tick += 30;
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0 && heart.interval_ms == 0 && heart.signal_x10 == 0);
    raw = UINT16_MAX;
    tick += 30;
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0 && heart.interval_ms == 0 && heart.signal_x10 == 0);
    wave(60, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    puts("PASS: startup baseline, 65531 jump reset/recovery, extreme-value filter reset and 60->120->60 BPM reacquisition.");
}

static void sport_tests(void)
{
    app_heart_struct heart;
    app_heart_sample_struct samples[2];
    unsigned elapsed, before, valid = 0;
    app_heart_set_enabled(0);
    app_heart_service();
    app_heart_set_sport_mode(1);
    app_heart_service();
    assert(running != 0);
    before = reads;
    for (elapsed = 10U; elapsed <= 60000U; elapsed += 10U)
    {
        tick += 10U;
        raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        assert(running != 0 && heart.enabled != 0);
        if (heart.state == APP_HEART_VALID)
        {
            ++valid;
        }
    }
    assert(reads - before == 2000U && valid > 1000U);
    assert(heart.bpm >= 86U && heart.bpm <= 94U);
    assert(app_heart_read_samples(samples, 2) == 0);
    app_heart_set_sport_mode(0);
    app_heart_service();
    assert(running == 0);
    app_heart_set_enabled(1);
    app_heart_service();
    app_heart_set_sport_mode(1);
    app_heart_set_sport_mode(0);
    app_heart_service();
    assert(running != 0); /* 退出运动模式恢复原手动请求。 */
    app_heart_set_enabled(0);
    app_heart_service();
    start_result = EM7028_ERROR;
    app_heart_set_sport_mode(1);
    app_heart_service();
    before = starts;
    start_result = EM7028_OK;
    for (elapsed = 0; elapsed < 20U; ++elapsed)
    {
        tick += 10U;
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(starts == before && heart.state == APP_HEART_ERROR && heart.enabled == 0);
    app_heart_set_sport_mode(0);
    app_heart_set_sport_mode(1);
    app_heart_service();
    assert(running != 0 && starts == before + 1U);
    app_heart_set_sport_mode(0);
    app_heart_service();
    puts("PASS: sport PPG continuously reads every 30ms for 60s, suppresses BLE queue, restores manual request and latches faults until retry.");
}

static void sleep_tests(void)
{
    app_heart_struct heart;
    app_heart_sample_struct samples[2];
    uint32_t elapsed, cycle_ms;
    unsigned before;
    stop_result = start_result = read_result = EM7028_OK;
    app_heart_set_enabled(1);
    app_heart_service();
    tick = UINT32_MAX - 1000U;
    app_heart_set_sleep_mode(1);
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    before = reads;
    tick += 599999U;
    app_heart_service();
    assert(running == 0 && reads == before);
    ++tick;
    assert(app_heart_sleep_pending() != 0);
    app_heart_service();
    assert(running != 0);
    cycle_ms = tick;
    raw = 2000;
    for (elapsed = 20U; elapsed <= 10000U; elapsed += 20U)
    {
        tick += 20U;
        app_heart_service();
        if (elapsed < 10000U)
        {
            assert(running != 0);
        }
    }
    app_heart_get_snapshot(&heart);
    assert(running == 0 && heart.enabled == 0 && heart.state == APP_HEART_NO_SIGNAL);
    assert(app_heart_sleep_pending() == 0 && app_heart_read_samples(samples, 2) == 0);
    tick = cycle_ms + 599999U;
    assert(app_heart_sleep_pending() == 0);
    ++tick;
    app_heart_service();
    assert(running != 0);
    for (elapsed = 20U; elapsed <= 10000U && running != 0; elapsed += 20U)
    {
        tick += 20U;
        raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(running == 0 && elapsed < 10000U && heart.state == APP_HEART_VALID);
    assert(heart.bpm >= 86U && heart.bpm <= 94U && heart.enabled == 0);
    assert(app_heart_read_samples(samples, 2) == 0);
    tick += 5000U;
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID); /* 自动结果保留到下一轮，不当作实时流。 */
    app_heart_set_sleep_mode(0);
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(running != 0 && heart.enabled != 0); /* 恢复进入熄屏前的手动请求。 */
    app_heart_set_sleep_mode(1);
    app_heart_service();
    start_result = EM7028_ERROR;
    tick += 600000U;
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    start_result = EM7028_OK;
    app_heart_set_sleep_mode(0);
    app_heart_set_enabled(0);
    app_heart_service();
    puts("PASS: sleep heart first/next 10min deadlines, 10s timeout, early valid stop, retained result, no raw queue, failed start, manual restore and tick wrap.");
}

int main(void)
{
    app_heart_struct heart;
    app_heart_sample_struct samples[32];
    unsigned i;
    static const unsigned rates[] = {40, 60, 90, 120, 180, 200};
    static const unsigned rejected_rates[] = {30, 35, 220, 250, 500};
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && heart.bpm == 0);
    app_heart_init();
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_OFF && running == 0);
    app_heart_set_enabled(1);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_MEASURING && running == 0);
    app_heart_set_enabled(0);

    /* 原始样本按成功读取顺序发送，溢出保留最新32帧，失败读数不得入队。 */
    assert(app_heart_read_samples(NULL, 1) == 0);
    begin();
    for (i = 0; i < 40; ++i)
    {
        tick += 30;
        raw = (uint16_t)(30000U + i);
        app_heart_service();
    }
    assert(app_heart_read_samples(samples, 4) == 4);
    for (i = 0; i < 4; ++i)
    {
        assert(samples[i].raw == 30008U + i);
        assert(samples[i].sequence == 8U + i && samples[i].elapsed_ms == (9U + i) * 30U);
        assert(samples[i].state == APP_HEART_MEASURING && samples[i].bpm == 0);
        assert(isfinite(samples[i].filtered) && isfinite(samples[i].dc) && samples[i].dc >= 30000.0f);
    }
    assert(app_heart_read_samples(samples, 32) == 28);
    for (i = 0; i < 28; ++i)
    {
        assert(samples[i].raw == 30012U + i && samples[i].sequence == 12U + i);
        assert(samples[i].elapsed_ms == (13U + i) * 30U);
    }
    assert(app_heart_read_samples(samples, 1) == 0);
    read_result = EM7028_NOT_READY;
    tick += 30;
    app_heart_service();
    assert(app_heart_read_samples(samples, 1) == 0);
    read_result = EM7028_OK;
    tick += 30;
    app_heart_service();
    assert(app_heart_read_samples(samples, 1) == 1);
    assert(samples[0].sequence == 40 && samples[0].elapsed_ms == 1260U);
    tick += 30;
    app_heart_service();
    app_heart_set_enabled(0);
    assert(app_heart_read_samples(samples, 1) == 0);
    puts("PASS: raw sample queue, ordering, overflow, unsuccessful read and STOP flush.");
    recovery_tests();
    continuity_tests();
    delayed_weak_beat_tests();
    window_peak_tests();

    for (i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i)
    {
        begin();
        app_heart_get_snapshot(&heart);
        assert(heart.state == APP_HEART_MEASURING && heart.bpm == 0);
        wave(rates[i], 12000, i == 1 ? -80.0f : 80.0f);
        app_heart_get_snapshot(&heart);
        printf("Synthetic %u BPM: state %u, period %u ms, output %u BPM\n", rates[i], (unsigned)heart.state, (unsigned)heart.interval_ms,
               (unsigned)heart.bpm);
        fflush(stdout);
        assert(heart.state == APP_HEART_VALID);
        assert(heart.signal_x10 >= 80U);
        assert(heart.bpm >= rates[i] - 3 && heart.bpm <= rates[i] + 3);
        assert(app_heart_read_samples(samples, 32) == 32);
        assert(samples[31].raw == heart.raw && samples[31].bpm == heart.bpm);
        assert(samples[31].state == heart.state && samples[31].interval_ms == heart.interval_ms);
        assert(samples[31].elapsed_ms == 12000U && samples[31].sequence == 399U);
        printf("PASS: synthetic %u BPM -> %u BPM\n", rates[i], (unsigned)heart.bpm);
    }
    expect_rejected = 1;
    for (i = 0; i < sizeof(rejected_rates) / sizeof(rejected_rates[0]); ++i)
    {
        begin();
        wave(rejected_rates[i], 18000, 80.0f);
        printf("PASS: %u BPM outside 40-200 rejected without clamping\n", rejected_rates[i]);
    }
    expect_rejected = 0;
    begin();
    short_secondary_wave(16000);
    app_heart_get_snapshot(&heart);
    printf("Short secondary peaks: state %u, AC %.1f, period %u ms, BPM %u\n", (unsigned)heart.state,
           (double)heart.signal_x10 / 10.0, (unsigned)heart.interval_ms, (unsigned)heart.bpm);
    fflush(stdout);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    assert(heart.interval_ms >= 970 && heart.interval_ms <= 1030);
    begin();
    notched_wave(16000);
    app_heart_get_snapshot(&heart);
    printf("Notched pulse: state %u, period %u ms, BPM %u\n", (unsigned)heart.state, (unsigned)heart.interval_ms, (unsigned)heart.bpm);
    fflush(stdout);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    for (i = 0; i < 2; ++i)
    {
        begin();
        drift_wave(60, 16000, 80.0f, i == 0 ? 150.0f : -150.0f);
        app_heart_get_snapshot(&heart);
        printf("Drifting background: direction %u, AC %.1f, period %u ms, BPM %u\n", i, (double)heart.signal_x10 / 10.0,
               (unsigned)heart.interval_ms, (unsigned)heart.bpm);
        fflush(stdout);
        assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    }
    for (i = 0; i < 2; ++i)
    {
        begin();
        drift_wave(60, 10000, 0.0f, i == 0 ? 150.0f : -150.0f);
        app_heart_get_snapshot(&heart);
        assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0 && heart.interval_ms == 0);
    }
    for (i = 0; i < 2; ++i)
    {
        begin();
        pulse_wave(60, 16000, i == 0 ? 80.0f : -80.0f, 2000);
        app_heart_get_snapshot(&heart);
        printf("Narrow pulse: polarity %u, state %u, raw %u, AC %.1f, BPM %u\n", i, (unsigned)heart.state, (unsigned)heart.raw,
               (double)heart.signal_x10 / 10.0, (unsigned)heart.bpm);
        fflush(stdout);
        assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
        assert(heart.interval_ms >= 970 && heart.interval_ms <= 1030);
        assert(heart.raw == raw && heart.sample_ms == tick);
    }
    tick += 300;
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0);
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_MEASURING && heart.bpm == 0);
    wave(60, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID);
    wave(60, 6000, 0.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0);
    begin();
    wave(60, 8000, 2.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0);
    assert(heart.raw >= 1998 && heart.raw <= 2002 && heart.signal_x10 < 80U);
    begin();
    wave(60, 8000, 6.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0);
    begin();
    for (i = 0; i < 400; ++i)
    {
        tick += 30;
        raw = i % 2 == 0 ? 2300 : 1700;
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state != APP_HEART_VALID && heart.bpm == 0);
    assert(heart.interval_ms == 0U); /* 60ms高频波不再通过七点波峰确认。 */
    begin();
    pulse_wave(90, 16000, 1200.0f, 30000);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 87 && heart.bpm <= 93);
    assert(heart.raw == raw && heart.raw >= 30000 && heart.raw <= 31200 && heart.sample_ms == tick);
    assert(heart.interval_ms >= 630 && heart.interval_ms <= 700);
    printf("PASS: 30000-count background, narrow pulses -> %u BPM, period %u ms, coherent raw/timestamp snapshot\n", (unsigned)heart.bpm,
           (unsigned)heart.interval_ms);
    begin();
    wave(60, 12000, 80.0f);
    read_result = EM7028_ERROR;
    tick += 30;
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.enabled != 0 && running != 0);
    read_result = EM7028_OK;
    wave(60, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    read_result = EM7028_ERROR;
    stop_result = EM7028_ERROR;
    for (i = 0; i < 10; ++i)
    {
        tick += 30;
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && heart.enabled == 0 && heart.bpm == 0 && running == 0);
    assert(heart.fault.kind == EM7028_FAULT_READ && heart.fault.reg == 0x29);
    stop_result = EM7028_OK;
    read_result = EM7028_OK;
    begin();
    read_result = EM7028_NOT_READY;
    for (i = 0; i < 10; ++i)
    {
        tick += 30;
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && running == 0);
    read_result = EM7028_OK;
    start_result = EM7028_NOT_READY;
    begin();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && heart.enabled == 0);
    start_result = EM7028_OK;
    tick = UINT32_MAX - 2000U;
    begin();
    wave(90, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 87 && heart.bpm <= 93);
    assert(app_heart_read_samples(samples, 32) == 32);
    assert(samples[31].elapsed_ms == 12000U && samples[31].sequence == 399U);
    for (i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i)
    {
        unsigned elapsed, initial_reads, valid = 0;
        begin();
        initial_reads = reads;
        for (elapsed = 10U; elapsed <= 24000U; elapsed += 10U)
        {
            tick += 10U;
            raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * (float)rates[i] * 6.2831853f / 60000.0f));
            app_heart_service();
            app_heart_get_snapshot(&heart);
            if (elapsed >= 8000U)
            {
                assert(heart.state == APP_HEART_VALID);
                assert(abs((int)heart.bpm - (int)rates[i]) <= 4);
                ++valid;
            }
        }
        assert(reads - initial_reads == 800U);
        app_heart_get_snapshot(&heart);
        printf("Low-power synthetic %u BPM: state %u, output %u BPM\n", rates[i], heart.state, heart.bpm);
        assert(valid > 100U);
        assert(heart.state == APP_HEART_VALID && abs((int)heart.bpm - (int)rates[i]) <= 4);
    }
    app_heart_set_enabled(0);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_OFF && heart.bpm == 0);
    stop_result = EM7028_ERROR;
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && running == 0);
    assert(reads > 0);
    puts("PASS: heart service start/stop, stale/gap/flat/noise rejection, I2C failure, snapshot locks and tick wrap.");
    sleep_tests();
    sport_tests();
    return 0;
}
