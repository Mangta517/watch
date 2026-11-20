/* 真实心率服务，模拟 EM7028 波形、时钟和快照锁；不代表实板脉搏准确性。 */
#include "app_heart.h"
#include "bl24c02.h"
#include "em7028.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t tick, start_delay_ms;
static uint16_t raw = 2000;
static uint8_t locked;
static uint8_t allocation_failed;
static unsigned fatal_errors;
static uint8_t interval_change_on_take;
static uint8_t running;
static uint8_t chip_sampling;
static em7028_result start_result = EM7028_OK;
static em7028_result read_result = EM7028_OK;
static em7028_result stop_result = EM7028_OK;
static unsigned reads;
static unsigned starts;
static unsigned stops;
static uint8_t expect_rejected;
static em7028_fault read_fault = {EM7028_FAULT_READ, 0x29, 0};
extern uint32_t host_heart_fault_diag;

static uint8_t eeprom[BL24C02_CAPACITY_BYTES];
static uint8_t eeprom_read_fail, eeprom_write_protected;
static uint8_t eeprom_write_limit = 8;
static unsigned eeprom_writes, eeprom_transactions;
static bl24c02_transfer_struct *eeprom_active;

uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length)
{
    uint16_t i;
    assert(locked == 0 && eeprom_active == NULL);
    assert(address == 0xF0 && length == 16);
    if (eeprom_read_fail != 0)
    {
        return 1;
    }
    for (i = 0; i < length; ++i)
    {
        data[i] = eeprom[address + i];
    }
    return 0;
}

void bl24c02_transfer_begin(bl24c02_transfer_struct *io, uint16_t address, uint8_t *data, uint16_t length, uint8_t write)
{
    assert(locked == 0 && eeprom_active == NULL);
    assert(address >= 0xF0 && address + length <= sizeof(eeprom));
    if (write != 0)
    {
        assert(length == 8 && address / BL24C02_PAGE_BYTES == (address + length - 1) / BL24C02_PAGE_BYTES);
    }
    eeprom_active = io;
    ++eeprom_transactions;
    io->address = address;
    io->data = data;
    io->remaining = length;
    io->write = write;
    io->phase = 0;
}

uint8_t bl24c02_transfer_step(bl24c02_transfer_struct *io)
{
    uint16_t i;
    assert(locked == 0 && eeprom_active == io);
    if (io->phase++ == 0)
    {
        return 2;
    }
    eeprom_active = NULL;
    if (io->write != 0)
    {
        ++eeprom_writes;
        if (eeprom_write_protected == 0)
        {
            for (i = 0; i < io->remaining && i < eeprom_write_limit; ++i)
            {
                eeprom[io->address + i] = io->data[i];
            }
        }
        if (eeprom_write_limit < io->remaining)
        {
            return 1;
        }
    }
    else
    {
        if (eeprom_read_fail != 0)
        {
            return 1;
        }
        for (i = 0; i < io->remaining; ++i)
        {
            io->data[i] = eeprom[io->address + i];
        }
    }
    return 0;
}

void em7028_get_fault(em7028_fault *fault)
{
    assert(locked == 0 && fault != NULL);
    *fault = read_fault;
}

em7028_result em7028_init(void)
{
    return EM7028_OK;
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    if (allocation_failed != 0)
    {
        return NULL;
    }
    return &locked;
}

void Error_Handler(void)
{
    ++fatal_errors;
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    assert(mutex == &locked && wait == portMAX_DELAY && locked == 0);
    if (interval_change_on_take != 0)
    {
        interval_change_on_take = 0;
        ++tick;
        app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    }
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
    tick += start_delay_ms;
    running = start_result == EM7028_OK;
    chip_sampling = running;
    return start_result;
}

em7028_result em7028_stop(void)
{
    assert(locked == 0);
    ++stops;
    running = 0;
    if (stop_result == EM7028_OK)
    {
        chip_sampling = 0;
    }
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
        if (read_result == EM7028_ERROR)
        {
            read_fault.kind = EM7028_FAULT_READ;
        }
        else
        {
            read_fault.kind = EM7028_FAULT_DATA_CHANGED;
        }
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
        if (elapsed >= 8000)
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
    tick = UINT32_MAX - 3000;
    begin();
    for (elapsed = 0; elapsed < 72000; elapsed += 30)
    {
        tick += 30;
        raw = (uint16_t)(30000.0f + 100.0f * sinf((float)elapsed * 6.2831853f / 1000.0f) +
                         160.0f * sinf((float)elapsed * 6.2831853f / 90.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        if (elapsed >= 8000)
        {
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
            assert(heart.interval_ms >= 930 && heart.interval_ms <= 1080);
        }
    }
    /* 非固定采样间隔使用实际HAL tick，不能按样本下标估算心率。 */
    begin();
    elapsed = 0;
    while (elapsed < 18000)
    {
        step = 30;
        if (elapsed % 70 == 30)
        {
            step = 40;
        }
        elapsed += step;
        tick += step;
        raw = (uint16_t)(30000.0f + 100.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        if (elapsed >= 8000)
        {
            assert(heart.state == APP_HEART_VALID && heart.bpm >= 87 && heart.bpm <= 93);
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
            period_index = (period_index + 1) % (sizeof(periods) / sizeof(periods[0]));
            period = periods[period_index];
        }
        tick += 30;
        raw = (uint16_t)(30000.0f + 300.0f * sinf((float)(elapsed - cycle_start) * 6.2831853f / (float)period));
        app_heart_service();
        if (elapsed >= 5000)
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
            period_index = (period_index + 1) % (sizeof(irregular_periods) / sizeof(irregular_periods[0]));
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
        raw = (uint16_t)(50000 - i * 1000);
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_MEASURING && heart.bpm == 0 && heart.interval_ms == 0);
    assert(app_heart_read_samples(&sample, 1) != 0);
    while (app_heart_read_samples(&sample, 1) != 0)
    {
    }
    assert(sample.raw == 30000 && sample.filtered == 0.0f);
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
            if (elapsed < 1000)
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
        assert(heart.interval_ms >= 970 && heart.interval_ms <= 1030);
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
            if (elapsed >= 2700)
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
        raw = (uint16_t)(35674 + i);
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
    assert((uint32_t)(tick - restart_tick) == 12000);
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
    for (elapsed = 10; elapsed <= 60000; elapsed += 10)
    {
        tick += 10;
        raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        assert(running != 0 && heart.enabled != 0);
        if (heart.state == APP_HEART_VALID)
        {
            ++valid;
        }
    }
    assert(reads - before == 2000 && valid > 1000);
    assert(heart.bpm >= 86 && heart.bpm <= 94);
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
    for (elapsed = 0; elapsed < 20; ++elapsed)
    {
        tick += 10;
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(starts == before && heart.state == APP_HEART_ERROR && heart.enabled == 0);
    app_heart_set_sport_mode(0);
    app_heart_set_sport_mode(1);
    app_heart_service();
    assert(running != 0 && starts == before + 1);
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
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_enabled(1);
    app_heart_service();
    tick = UINT32_MAX - 1000;
    app_heart_set_sleep_mode(1);
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    before = reads;
    tick += 599999;
    app_heart_service();
    assert(running == 0 && reads == before);
    ++tick;
    assert(app_heart_sleep_pending() != 0);
    app_heart_service();
    assert(running != 0);
    cycle_ms = tick;
    raw = 2000;
    for (elapsed = 20; elapsed <= 10000 && running != 0; elapsed += 20)
    {
        tick += 20;
        app_heart_service();
    }
    assert(elapsed >= 5500 && elapsed <= 5700);
    app_heart_get_snapshot(&heart);
    assert(running == 0 && heart.enabled == 0 && heart.state == APP_HEART_NO_SIGNAL);
    assert(app_heart_sleep_pending() == 0 && app_heart_read_samples(samples, 2) == 0);
    tick = cycle_ms + 599999;
    assert(app_heart_sleep_pending() == 0);
    ++tick;
    app_heart_service();
    assert(running != 0);
    for (elapsed = 20; elapsed <= 10000 && running != 0; elapsed += 20)
    {
        tick += 20;
        raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(running == 0 && elapsed < 10000 && heart.state == APP_HEART_VALID);
    assert(heart.bpm >= 86 && heart.bpm <= 94 && heart.enabled == 0);
    assert(app_heart_read_samples(samples, 2) == 0);
    tick += 5000;
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID); /* 自动结果保留到下一轮，不当作实时流。 */
    app_heart_set_sleep_mode(0);
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(running != 0 && heart.enabled != 0); /* 恢复进入熄屏前的手动请求。 */
    app_heart_set_sleep_mode(1);
    app_heart_service();
    start_result = EM7028_ERROR;
    tick += 600000;
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    start_result = EM7028_OK;
    app_heart_set_sleep_mode(0);
    app_heart_set_enabled(0);
    app_heart_service();
    puts("PASS: sleep heart first/next 10min deadlines, early flat-signal and valid stop, retained result, no raw queue, failed start, manual restore and tick wrap.");
}

/* 两种周期切换重新计时，重复点击保持截止时间；窗口、手动测量和SPORT沿用原流程。 */
static void auto_interval_tests(void)
{
    app_heart_struct heart;
    uint32_t elapsed;
    stop_result = start_result = read_result = EM7028_OK;
    app_heart_set_sleep_mode(0);
    app_heart_set_enabled(0);
    app_heart_service();
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    tick = UINT32_MAX - 30000;
    app_heart_set_sleep_mode(1);
    /* 模拟UI抢先更新周期起点，sensor使用取得锁后的tick避免无符号减法误判到期。 */
    interval_change_on_take = 1;
    app_heart_service();
    assert(running == 0 && app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
    tick += 30000;
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    app_heart_set_auto_interval((app_heart_auto_interval)-1);
    app_heart_set_auto_interval((app_heart_auto_interval)2);
    app_heart_set_auto_interval((app_heart_auto_interval)256);
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
    tick += 29999;
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    ++tick;
    assert(app_heart_sleep_pending() != 0);
    app_heart_service();
    assert(running != 0 && chip_sampling != 0);
    /* 在途窗口切换周期仍在原10秒截止停测，下一周期从切换时刻起算。 */
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    tick += 9999;
    raw = 2000;
    app_heart_service();
    assert(running != 0);
    ++tick;
    app_heart_service();
    assert(running == 0 && chip_sampling == 0);
    tick += 589999;
    assert(app_heart_sleep_pending() == 0);
    ++tick;
    app_heart_service();
    assert(running != 0);
    /* 有效波形提前停测，最近有效值跨周期切换保留。 */
    for (elapsed = 20; elapsed <= 10000 && running != 0; elapsed += 20)
    {
        tick += 20;
        raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(running == 0 && elapsed < 10000 && heart.last_bpm >= 86 && heart.last_bpm <= 94);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    tick += 59000;
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    tick += 599999;
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    ++tick;
    assert(app_heart_sleep_pending() != 0);
    app_heart_set_sleep_mode(0);
    app_heart_set_enabled(1);
    app_heart_service();
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(running != 0 && heart.enabled != 0);
    app_heart_set_enabled(0);
    app_heart_service();
    app_heart_set_sport_mode(1);
    app_heart_service();
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    tick += 10000;
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(running != 0 && heart.enabled != 0);
    app_heart_set_sport_mode(0);
    app_heart_service();
    assert(running == 0);
    puts("PASS: 1/10min deadlines, same/invalid selections, UI preemption, tick wrap, in-window changes, early valid stop, manual and SPORT.");
}

static void store_drain(void)
{
    uint8_t i;
    for (i = 0; i < 24; ++i)
    {
        app_heart_store_service(1);
    }
    assert(eeprom_active == NULL);
}

static void store_tests(void)
{
    uint16_t i;
    uint8_t prefix, byte;
    unsigned before;
    /* CRC夹具由Python zlib.crc32独立生成，覆盖序号255到0及相同序号。 */
    static const uint8_t old_record[8] = {'H', 'R', 1, 255, 0xDC, 0x8B, 0xC5, 0x24};
    static const uint8_t next_record[8] = {'H', 'R', 0, 0, 0x10, 0x55, 0xDC, 0x10};
    static const uint8_t invalid_interval[8] = {'H', 'R', 2, 1, 0x04, 0x07, 0xED, 0x55};
    for (i = 0; i < 0xF0; ++i)
    {
        eeprom[i] = (uint8_t)(i ^ 0x5A);
    }
    memset(eeprom + 0xF0, 0xFF, 16);
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    store_drain();
    assert(eeprom_writes == 0 && eeprom_transactions == 0);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN && eeprom_transactions == 0);
    assert(app_heart_store_service(0) == 0 && eeprom_transactions == 0);
    assert(app_heart_store_service(1) == 1);
    while (app_heart_store_service(0) != 0)
    {
        assert(eeprom_transactions <= 2);
    }
    assert(eeprom_writes == 1 && eeprom[0xF0] == 'H' && eeprom[0xF2] == 1 && eeprom[0xF3] == 1);
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
    store_drain();
    assert(eeprom_writes == 1);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    store_drain();
    assert(eeprom[0xFA] == 0 && eeprom[0xFB] == 2 && eeprom_writes == 2);
    for (byte = 0; byte < 8; ++byte)
    {
        eeprom[0xF8 + byte] ^= 0x80;
        app_heart_init();
        assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
        eeprom[0xF8 + byte] ^= 0x80;
    }
    /* 在途值先保存完成，后续请求仍须继续落盘。 */
    app_heart_init();
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    assert(app_heart_store_service(1) == 1);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    store_drain();
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN && eeprom_writes == 4);
    /* 写已完成而读回失败，随后改回旧值，必须先重读再决定是否还需要写。 */
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    assert(app_heart_store_service(1) == 1);
    assert(app_heart_store_service(0) == 1);
    assert(app_heart_store_service(0) == 1);
    assert(eeprom_active != NULL && eeprom_active->write == 0);
    eeprom_read_fail = 1;
    assert(app_heart_store_service(0) == 1);
    assert(app_heart_store_service(0) == 0);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    before = eeprom_transactions;
    assert(app_heart_store_service(1) == 0 && eeprom_transactions == before);
    eeprom_read_fail = 0;
    tick += 1000;
    store_drain();
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN && eeprom_writes == 6);
    /* 写保护触发读回不一致，事务释放后延迟重试。 */
    eeprom_write_protected = 1;
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    store_drain();
    assert(eeprom_writes == 7 && app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
    before = eeprom_transactions;
    tick += 999;
    assert(app_heart_store_service(1) == 0 && eeprom_transactions == before);
    eeprom_write_protected = 0;
    ++tick;
    store_drain();
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN && eeprom_writes == 8);
    /* 每个不完整前缀模拟目标槽掉电撕裂，重启恢复另一个有效槽。 */
    for (prefix = 0; prefix < 8; ++prefix)
    {
        memset(eeprom + 0xF8, 0xFF, 8);
        app_heart_init();
        eeprom_write_limit = prefix;
        app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
        assert(app_heart_store_service(1) == 1);
        assert(app_heart_store_service(1) == 1);
        assert(app_heart_store_service(1) == 0);
        app_heart_init();
        assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
    }
    eeprom_write_limit = 8;
    /* 启动读失败时暂用默认值，总线恢复后自动读回旧周期。 */
    eeprom_read_fail = 1;
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN);
    tick += 1000;
    store_drain();
    eeprom_read_fail = 0;
    tick += 1000;
    before = eeprom_writes;
    store_drain();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN && eeprom_writes == before);
    /* EEPROM尚未读通时明确选择屏幕上的默认值，也应保存。 */
    eeprom_read_fail = 1;
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    eeprom_read_fail = 0;
    tick += 1000;
    store_drain();
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN && eeprom_writes == before + 1);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    store_drain();
    /* 验证独立CRC夹具以及8位序号回绕后的最新选择。 */
    for (byte = 0; byte < 8; ++byte)
    {
        eeprom[0xF0 + byte] = old_record[byte];
        eeprom[0xF8 + byte] = invalid_interval[byte];
    }
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_1MIN);
    for (byte = 0; byte < 8; ++byte)
    {
        eeprom[0xF8 + byte] = next_record[byte];
    }
    app_heart_init();
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_1MIN);
    store_drain();
    assert(eeprom[0xF3] == 1);
    for (byte = 0; byte < 8; ++byte)
    {
        eeprom[0xF8 + byte] = eeprom[0xF0 + byte];
    }
    app_heart_init();
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    store_drain();
    assert(eeprom[0xFB] == 2); /* 相同序号选槽0，下一次应写槽1。 */
    for (i = 0; i < 260; ++i)
    {
        app_heart_set_auto_interval((app_heart_auto_interval)((i + 1) % 2));
        store_drain();
        app_heart_init();
        assert(app_heart_get_auto_interval() == (app_heart_auto_interval)((i + 1) % 2));
    }
    for (i = 0; i < 0xF0; ++i)
    {
        assert(eeprom[i] == (uint8_t)(i ^ 0x5A));
    }
    puts("PASS: HR CRC fixtures, defaults, async arbitration, reboot restore, corruption/torn writes, concurrent changes, read/write failures, retry and sequence wrap.");
}

int main(int argc, char **argv)
{
    app_heart_struct heart;
    uint16_t retained_bpm;
    app_heart_sample_struct samples[32];
    unsigned i;
    static const unsigned rates[] = {40, 60, 90, 120, 180, 200};
    static const unsigned rejected_rates[] = {30, 35, 220, 250, 500};
    if (argc > 1 && strcmp(argv[1], "allocation_failed") == 0)
    {
        allocation_failed = 1;
        app_heart_init();
        assert(fatal_errors == 1);
        app_heart_service();
        app_heart_get_snapshot(&heart);
        assert(heart.state == APP_HEART_ERROR && starts == 0 && reads == 0);
        puts("PASS: heart mutex allocation failure follows Error_Handler and leaves sampling stopped.");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "store") == 0)
    {
        store_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "telemetry_off") == 0)
    {
        app_heart_init();
        begin();
        wave(60, 12000, 80.0f);
        app_heart_get_snapshot(&heart);
        assert(heart.state == APP_HEART_VALID && heart.bpm == 60);
        assert(app_heart_read_samples(samples, 32) == 0);
        puts("PASS: default build measures BPM without filling the waveform queue.");
        return 0;
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && heart.bpm == 0);
    read_fault = (em7028_fault){EM7028_FAULT_CONFIG, 0x0D, 0xFF};
    app_heart_init();
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && running == 0);
    assert(heart.fault.kind == EM7028_FAULT_CONFIG && heart.fault.reg == 0x0D && heart.fault.value == 0xFF);
    assert(host_heart_fault_diag == 0xFF0D0402);
    puts("PASS: startup fault is available to the BLE snapshot and existing diagnostic result before manual START.");
    assert(app_heart_get_auto_interval() == APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_enabled(1);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_MEASURING && running == 0);
    app_heart_set_enabled(0);

    /* 默认周期在首次熄屏满10分钟起测，10秒超时后停止。 */
    app_heart_set_sleep_mode(1);
    tick += 599999;
    app_heart_service();
    assert(running == 0 && app_heart_sleep_pending() == 0);
    tick += 1;
    app_heart_service();
    assert(running != 0 && chip_sampling != 0);
    tick += 10000;
    app_heart_service();
    assert(running == 0 && chip_sampling == 0 && app_heart_sleep_pending() == 0);
    app_heart_set_sleep_mode(0);
    puts("PASS: default auto heart starts after 10min screen-off and stops after 10s.");

    /* 启动 I2C 跨过 6ms 时，轮询和样本时间仍以器件完成启动为起点。 */
    start_delay_ms = 6;
    begin();
    start_delay_ms = 0;
    tick += 24;
    assert(app_heart_service() == 0);
    tick += 6;
    assert(app_heart_service() != 0);
    assert(app_heart_read_samples(samples, 1) == 1 && samples[0].elapsed_ms == 30);
    app_heart_set_enabled(0);
    app_heart_service();

    /* 原始样本按成功读取顺序发送，溢出保留最新32帧，失败读数不得入队。 */
    assert(app_heart_read_samples(NULL, 1) == 0);
    begin();
    for (i = 0; i < 40; ++i)
    {
        tick += 30;
        raw = (uint16_t)(30000 + i);
        app_heart_service();
    }
    assert(app_heart_read_samples(samples, 4) == 4);
    for (i = 0; i < 4; ++i)
    {
        assert(samples[i].raw == 30008 + i);
        assert(samples[i].sequence == 8 + i && samples[i].elapsed_ms == (9 + i) * 30);
        assert(samples[i].state == APP_HEART_MEASURING && samples[i].bpm == 0);
        assert(isfinite(samples[i].filtered) && isfinite(samples[i].dc) && samples[i].dc >= 30000.0f);
    }
    assert(app_heart_read_samples(samples, 32) == 28);
    for (i = 0; i < 28; ++i)
    {
        assert(samples[i].raw == 30012 + i && samples[i].sequence == 12 + i);
        assert(samples[i].elapsed_ms == (13 + i) * 30);
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
    assert(samples[0].sequence == 40 && samples[0].elapsed_ms == 1260);
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
        if (i == 1)
        {
            wave(rates[i], 12000, -80.0f);
        }
        else
        {
            wave(rates[i], 12000, 80.0f);
        }
        app_heart_get_snapshot(&heart);
        printf("Synthetic %u BPM: state %u, period %u ms, output %u BPM\n", rates[i], (unsigned)heart.state, (unsigned)heart.interval_ms,
               (unsigned)heart.bpm);
        fflush(stdout);
        assert(heart.state == APP_HEART_VALID);
        assert(heart.signal_x10 >= 80);
        assert(heart.bpm >= rates[i] - 3 && heart.bpm <= rates[i] + 3);
        assert(app_heart_read_samples(samples, 32) == 32);
        assert(samples[31].raw == heart.raw && samples[31].bpm == heart.bpm);
        assert(samples[31].state == heart.state && samples[31].interval_ms == heart.interval_ms);
        assert(samples[31].elapsed_ms == 12000 && samples[31].sequence == 399);
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
        if (i == 0)
        {
            drift_wave(60, 16000, 80.0f, 150.0f);
        }
        else
        {
            drift_wave(60, 16000, 80.0f, -150.0f);
        }
        app_heart_get_snapshot(&heart);
        printf("Drifting background: direction %u, AC %.1f, period %u ms, BPM %u\n", i, (double)heart.signal_x10 / 10.0,
               (unsigned)heart.interval_ms, (unsigned)heart.bpm);
        fflush(stdout);
        assert(heart.state == APP_HEART_VALID && heart.bpm >= 57 && heart.bpm <= 63);
    }
    for (i = 0; i < 2; ++i)
    {
        begin();
        if (i == 0)
        {
            drift_wave(60, 10000, 0.0f, 150.0f);
        }
        else
        {
            drift_wave(60, 10000, 0.0f, -150.0f);
        }
        app_heart_get_snapshot(&heart);
        assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0 && heart.interval_ms == 0);
    }
    for (i = 0; i < 2; ++i)
    {
        begin();
        if (i == 0)
        {
            pulse_wave(60, 16000, 80.0f, 2000);
        }
        else
        {
            pulse_wave(60, 16000, -80.0f, 2000);
        }
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
    assert(heart.raw >= 1998 && heart.raw <= 2002 && heart.signal_x10 < 80);
    begin();
    wave(60, 8000, 6.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_NO_SIGNAL && heart.bpm == 0);
    begin();
    for (i = 0; i < 400; ++i)
    {
        tick += 30;
        if (i % 2 == 0)
        {
            raw = 2300;
        }
        else
        {
            raw = 1700;
        }
        app_heart_service();
    }
    app_heart_get_snapshot(&heart);
    assert(heart.state != APP_HEART_VALID && heart.bpm == 0);
    assert(heart.interval_ms == 0); /* 60ms高频波不再通过七点波峰确认。 */
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
    assert(chip_sampling != 0 && app_heart_sleep_pending() == 0);
    assert((host_heart_fault_diag & 0x00FFFFFF) == 0x00290102); /* stop失败不能覆盖原始READ。 */
    stop_result = EM7028_OK;
    read_result = EM7028_OK;
    tick += 1000;
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
    tick = UINT32_MAX - 2000;
    begin();
    wave(90, 12000, 80.0f);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_VALID && heart.bpm >= 87 && heart.bpm <= 93);
    assert(app_heart_read_samples(samples, 32) == 32);
    assert(samples[31].elapsed_ms == 12000 && samples[31].sequence == 399);
    for (i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i)
    {
        unsigned elapsed, initial_reads, valid = 0;
        begin();
        initial_reads = reads;
        for (elapsed = 10; elapsed <= 24000; elapsed += 10)
        {
            tick += 10;
            raw = (uint16_t)(2000.0f + 80.0f * sinf((float)elapsed * (float)rates[i] * 6.2831853f / 60000.0f));
            app_heart_service();
            app_heart_get_snapshot(&heart);
            if (elapsed >= 8000)
            {
                assert(heart.state == APP_HEART_VALID);
                assert(abs((int)heart.bpm - (int)rates[i]) <= 4);
                ++valid;
            }
        }
        assert(reads - initial_reads == 800);
        app_heart_get_snapshot(&heart);
        printf("Low-power synthetic %u BPM: state %u, output %u BPM\n", rates[i], heart.state, heart.bpm);
        assert(valid > 100);
        assert(heart.state == APP_HEART_VALID && abs((int)heart.bpm - (int)rates[i]) <= 4);
    }
    retained_bpm = heart.bpm;
    assert(retained_bpm != 0 && heart.last_bpm == retained_bpm);
    app_heart_set_enabled(0);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_OFF && heart.bpm == 0 && heart.last_bpm == retained_bpm);
    stop_result = EM7028_ERROR;
    app_heart_service();
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && running == 0);
    assert(heart.last_bpm == retained_bpm);
    start_result = EM7028_ERROR;
    app_heart_set_enabled(1);
    assert(app_heart_service() == 0);
    app_heart_get_snapshot(&heart);
    assert(heart.state == APP_HEART_ERROR && heart.last_bpm == retained_bpm);
    tick += 1000;
    assert(app_heart_sleep_pending() != 0);
    assert(app_heart_service() == 1);
    stop_result = EM7028_OK;
    tick += 1000;
    assert(app_heart_service() == 1);
    assert(app_heart_service() == 1); /* 清完待机请求后再处理失败的START。 */
    assert(app_heart_service() == 0);
    start_result = EM7028_OK;
    puts("PASS: last valid BPM survives STOP, stop/start failure and idle service without presenting a current valid measurement.");
    assert(reads > 0);
    puts("PASS: heart service start/stop, stale/gap/flat/noise rejection, I2C failure, snapshot locks and tick wrap.");
    /* 连续100轮停测NACK只占到期的总线轮次，其他轮次留给指南针。 */
    begin();
    app_heart_set_enabled(0);
    stop_result = EM7028_ERROR;
    tick = UINT32_MAX - 500;
    {
        unsigned before = stops;
        unsigned compass_slots = 0;
        assert(app_heart_service() == 1 && stops == before + 1);
        for (i = 0; i < 100; ++i)
        {
            tick += 25;
            if (app_heart_service() == 0)
            {
                ++compass_slots;
            }
        }
        assert(compass_slots == 98 && stops == before + 3);
    }
    assert(chip_sampling != 0 && app_heart_sleep_pending() == 0);
    stop_result = EM7028_OK;
    tick += 499;
    assert(app_heart_sleep_pending() == 0 && app_heart_service() == 0);
    ++tick;
    assert(app_heart_sleep_pending() != 0);
    assert(app_heart_service() == 1 && app_heart_sleep_pending() == 0);
    assert(chip_sampling == 0);
    assert(app_heart_service() == 0);
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_sleep_mode(1);
    tick += 600000;
    app_heart_service();
    stop_result = EM7028_ERROR;
    tick += 10000;
    app_heart_service();
    assert(app_heart_sleep_pending() == 0 && app_heart_service() == 0);
    tick += 1000;
    assert(app_heart_sleep_pending() != 0);
    stop_result = EM7028_OK;
    app_heart_service();
    assert(app_heart_sleep_pending() == 0);
    app_heart_set_sleep_mode(0);
    app_heart_set_sleep_mode(1);
    tick += 600000;
    app_heart_service();
    stop_result = EM7028_ERROR;
    for (i = 0; i < 10000 && app_heart_sleep_pending() != 0; i += 30)
    {
        app_heart_struct latest;
        tick += 30;
        raw = (uint16_t)(2000.0f + 80.0f * sinf((float)i * 90.0f * 6.2831853f / 60000.0f));
        app_heart_service();
        app_heart_get_snapshot(&latest);
        if (latest.enabled == 0)
        {
            assert(latest.last_bpm >= 86 && latest.last_bpm <= 94);
            break;
        }
    }
    assert(i < 10000 && chip_sampling != 0 && app_heart_sleep_pending() == 0);
    {
        unsigned before = starts;
        app_heart_set_sleep_mode(0);
        app_heart_set_enabled(1);
        assert(app_heart_service() == 0 && starts == before);
        stop_result = EM7028_OK;
        tick += 1000;
        app_heart_service();
        assert(starts == before && chip_sampling == 0);
        app_heart_service();
        assert(starts == before + 1 && chip_sampling != 0);
        app_heart_set_enabled(0);
        app_heart_service();
    }
    puts("PASS: manual/automatic/fault stops retain a pending close through NACKs and stop retrying after success.");
    sleep_tests();
    sport_tests();
    auto_interval_tests();
    /* 后台窗口启动后转入运动模式：旧窗口的10秒截止不得关闭连续采样。 */
    app_heart_set_sport_mode(0);
    app_heart_set_sleep_mode(0);
    app_heart_set_enabled(0);
    app_heart_service();
    app_heart_set_auto_interval(APP_HEART_AUTO_INTERVAL_10MIN);
    app_heart_set_sleep_mode(1);
    tick += 600000;
    app_heart_service();
    assert(chip_sampling != 0);
    app_heart_set_sport_mode(1);
    tick += 10000;
    {
        unsigned stops_before = stops;
        app_heart_service();
        assert(stops == stops_before && chip_sampling != 0);
    }
    puts("PASS: entering sport mode during an automatic window preserves continuous sampling past its old deadline.");
    return 0;
}
