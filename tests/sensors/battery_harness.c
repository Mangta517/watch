/* 编译真实电池服务，检查ADC生命周期、分压换算、故障重试与快照同步。 */
#include "app_battery.h"
#include "adc.h"
#include "battery_adc.h"
#include "semphr.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
Host_ADC_TypeDef host_adc1;
uint8_t host_adc_clock_enabled;
static uint32_t primask;
GPIO_TypeDef host_gpio_a;
static jmp_buf shutdown_wait;
static uint8_t shutdown_stage, shutdown_expected;

uint32_t __get_PRIMASK(void)
{
    return primask;
}

void __disable_irq(void)
{
    primask = 1;
}

void __set_PRIMASK(uint32_t value)
{
    primask = value;
}

void HAL_Delay(uint32_t ms)
{
    assert(shutdown_expected != 0 && shutdown_stage == 0 && ms >= 3 && primask == 0 && host_adc_clock_enabled == 0);
    shutdown_stage = 1;
}

void tps3823_disable(void)
{
    assert(shutdown_stage == 1 && primask != 0);
    shutdown_stage = 2;
}

void HAL_SuspendTick(void)
{
    assert(shutdown_stage == 2 && primask != 0);
    shutdown_stage = 3;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == POWER_EN_GPIO_Port && pin == POWER_EN_Pin && state == GPIO_PIN_RESET);
    assert(shutdown_stage == 3 && primask != 0);
    shutdown_stage = 4;
}

void HAL_PWR_EnterSLEEPMode(uint32_t regulator, uint8_t entry)
{
    assert(regulator == PWR_MAINREGULATOR_ON && entry == PWR_SLEEPENTRY_WFI);
    assert(shutdown_stage == 4 && primask != 0);
    longjmp(shutdown_wait, 1);
}

void host_adc_clock_set(uint8_t enabled)
{
    assert(primask == 1 && enabled != host_adc_clock_enabled);
    host_adc_clock_enabled = enabled;
}
static uint32_t now_ms;
static uint32_t adc_value;
uint32_t SystemCoreClock = 84000000;
uint16_t host_vrefint_cal = 1500;
static uint16_t reference_value = 1500;
static uint32_t selected_channel;
static HAL_StatusTypeDef config_result = HAL_OK;
static uint8_t fail_battery_channel;
static uint32_t starts, polls, reads, stops;
static uint8_t locked, allocation_failed;
static unsigned fatal_errors;
static HAL_StatusTypeDef start_result = HAL_OK;
static HAL_StatusTypeDef poll_result = HAL_OK;
static HAL_StatusTypeDef stop_result = HAL_OK;

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

void Error_Handler(void)
{
    ++fatal_errors;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    if (allocation_failed)
    {
        return NULL;
    }
    else
    {
        return &locked;
    }
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    assert(mutex == &locked && !locked && wait == portMAX_DELAY);
    locked = 1;
    return 1;
}

int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex == &locked && locked);
    locked = 0;
    return 1;
}

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc, ADC_ChannelConfTypeDef *config)
{
    assert(host_adc_clock_enabled != 0 && primask == 0);
    assert(adc == &hadc1 && config->Rank == 1 && config->SamplingTime == ADC_SAMPLETIME_480CYCLES);
    selected_channel = config->Channel;
    if (fail_battery_channel != 0 && selected_channel == ADC_CHANNEL_1)
    {
        return HAL_ERROR;
    }
    if (selected_channel == ADC_CHANNEL_VREFINT)
    {
        ADC->CCR |= ADC_CCR_TSVREFE;
    }
    return config_result;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *adc)
{
    assert(host_adc_clock_enabled != 0 && primask == 0 && adc == &hadc1 && !locked);
    ++starts;
    return start_result;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout_ms)
{
    assert(host_adc_clock_enabled != 0 && primask == 0 && adc == &hadc1 && !locked && timeout_ms > 0 && timeout_ms <= 2U);
    ++polls;
    return poll_result;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc)
{
    assert(host_adc_clock_enabled != 0 && primask == 0 && adc == &hadc1 && !locked && poll_result == HAL_OK);
    ++reads;
    if (selected_channel == ADC_CHANNEL_VREFINT)
    {
        return reference_value;
    }
    return adc_value;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *adc)
{
    assert(host_adc_clock_enabled != 0 && primask == 0 && adc == &hadc1 && !locked);
    ++stops;
    return stop_result;
}

static void next_sample(void)
{
    now_ms += APP_BATTERY_PERIOD_MS;
    app_battery_service();
}

static void check_shutdown(uint8_t sleep)
{
    unsigned i;

    reference_value = 1548; /* VDDA=3198mV，ADC 2081正好折算为3250mV。 */
    now_ms = UINT32_MAX - 12000;
    app_battery_init();
    if (sleep != 0)
    {
        adc_value = 2497;
        for (i = 0; i < 8; ++i)
        {
            next_sample();
        }
        app_battery_set_sleep_mode(1);
    }
    adc_value = 2081;
    now_ms += APP_BATTERY_SLEEP_PERIOD_MS;
    app_battery_service();
    next_sample();
    adc_value = 2082; /* 3252mV，清除已有两次的低压计数。 */
    next_sample();

    adc_value = 2081;
    now_ms += APP_BATTERY_SLEEP_PERIOD_MS;
    app_battery_service();
    next_sample();
    poll_result = HAL_TIMEOUT;
    next_sample(); /* ADC失败中断连续有效采样。 */
    poll_result = HAL_OK;

    now_ms += APP_BATTERY_SLEEP_PERIOD_MS;
    app_battery_service();
    next_sample();
    assert(shutdown_stage == 0);
    shutdown_expected = 1;
    if (setjmp(shutdown_wait) == 0)
    {
        next_sample();
        assert(0); /* 第三次有效低压采样后不会返回。 */
    }
    assert(shutdown_stage == 4 && locked == 0);
    puts("PASS: 3250mV requires three consecutive valid readings, including across tick wrap, then releases PA3.");
}

static void check_failure(HAL_StatusTypeDef result, uint8_t valid)
{
    app_battery_struct snapshot;
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid == valid && snapshot.result == result && !locked);
    if (valid == 0)
    {
        assert(snapshot.voltage_mv == 0 && snapshot.adc_raw == 0 && snapshot.percent == 0);
    }
}

static void check_percent(void)
{
    app_battery_struct snapshot;
    uint8_t previous_percent;
    unsigned i;
    app_battery_init();
    /* 复现开机3.8V升到3.9V：前7次不显示过渡值，第8次直接显示后4次均值。 */
    adc_value = (3800 * 4095 + 3300) / 6600;
    app_battery_service();
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    adc_value = (3900 * 4095 + 3300) / 6600;
    for (i = 1; i < 7; ++i)
    {
        next_sample();
        app_battery_get_snapshot(&snapshot);
        assert(!snapshot.valid && snapshot.result == HAL_OK);
    }
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.voltage_mv == 3900 && snapshot.percent == 73);
    /* 复现日志里的短时压降：电压快速跟随，电量慢滤波减小短时压降造成的掉量。 */
    adc_value = (3710 * 4095 + 3300) / 6600;
    for (i = 0; i < 60; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv < 3720 && snapshot.percent == 66);
    previous_percent = snapshot.percent;
    adc_value = (3900 * 4095 + 3300) / 6600;
    for (i = 0; i < 2048; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv == 3900 && snapshot.percent == previous_percent);
    /* 放电时电量下降；之后即使电压回到3.9V，也不能回涨。 */
    adc_value = (3750 * 4095 + 3300) / 6600;
    for (i = 0; i < 2048; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv == 3750 && snapshot.percent == 45);
    adc_value = (3900 * 4095 + 3300) / 6600;
    for (i = 0; i < 120; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv == 3900 && snapshot.percent == 45);
    /* ADC故障不丢失电量历史，恢复后也不能回涨。 */
    poll_result = HAL_TIMEOUT;
    next_sample();
    check_failure(HAL_TIMEOUT, 1);
    poll_result = HAL_OK;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.percent == 45);
    previous_percent = snapshot.percent;
    /* 持续降至3.50V后慢滤波到达显示下限，随后电压回弹也保持0%。 */
    adc_value = (3500 * 4095 + 3300) / 6600;
    for (i = 0; i < 2048; ++i)
    {
        next_sample();
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.percent <= previous_percent);
        previous_percent = snapshot.percent;
    }
    assert(snapshot.voltage_mv == 3501 && snapshot.percent == 0);
    adc_value = (4200 * 4095 + 3300) / 6600;
    for (i = 0; i < 120; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.voltage_mv >= 4199 && snapshot.voltage_mv <= 4201 && snapshot.percent == 0);
    puts("PASS: startup settling, transient dip attenuation, sustained discharge, rebound and failure recovery.");
}

static void check_full_voltage(void)
{
    /* 通过真实ADC换算与启动均值核对所有节点、各段中点、限幅和取整边界。 */
    static const app_battery_curve_point_struct expected[] =
    {
        {3400, 0}, {3499, 0}, {3500, 0}, {3502, 0}, {3503, 1},
        {3525, 5}, {3550, 9}, {3575, 14}, {3600, 18}, {3613, 20}, {3614, 21}, {3625, 23},
        {3650, 27}, {3675, 32}, {3700, 36}, {3725, 41}, {3750, 45}, {3775, 50}, {3800, 55},
        {3825, 60}, {3850, 64}, {3875, 69}, {3900, 73}, {3925, 78}, {3950, 82},
        {3975, 87}, {4000, 91}, {4025, 96}, {4047, 99}, {4048, 100}, {4050, 100}, {4052, 100}, {4300, 100}
    };
    app_battery_struct snapshot;
    uint8_t count;
    const app_battery_curve_point_struct *points = app_battery_get_curve(&count);
    unsigned i, sample;
    const uint32_t supply_mv = 3198; /* VREFINT读数1548反算所得，可精确量化测试表中的每个电压。 */
    assert(count == 12 && points[0].voltage_mv == 3500 && points[count - 1].voltage_mv == 4050);
    assert(app_battery_voltage_to_percent(0) == 0);
    for (i = 0; i < count; ++i)
    {
        assert(points[i].voltage_mv == 3500 + i * 50);
        assert(app_battery_voltage_to_percent(points[i].voltage_mv) == points[i].percent);
    }
    reference_value = 1548;
    for (i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i)
    {
        assert(app_battery_voltage_to_percent(expected[i].voltage_mv) == expected[i].percent);
        app_battery_init();
        adc_value = ((uint32_t)expected[i].voltage_mv * 4095 + supply_mv) / (supply_mv * 2);
        for (sample = 0; sample < 8; ++sample)
        {
            next_sample();
        }
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.result == HAL_OK);
        assert(snapshot.voltage_mv == expected[i].voltage_mv && snapshot.percent == expected[i].percent);
    }
    reference_value = host_vrefint_cal;
    puts("PASS: 12 curve nodes, segment midpoints and exact 0/1%, 20/21%, 99/100% transitions through the ADC conversion.");
}

static void check_sleep_sampling(void)
{
    app_battery_struct snapshot;
    uint32_t initial_starts, awake_voltage_mv = 0;
    uint8_t awake_percent = 0;
    unsigned wrap, sample, profile;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        now_ms = 0;
        if (wrap != 0)
        {
            now_ms = UINT32_MAX - 12000;
        }
        app_battery_init();
        app_battery_set_sleep_mode(1);
        adc_value = (3900 * 4095 + 3300) / 6600;
        initial_starts = starts;
        for (sample = 0; sample < 8; ++sample)
        {
            next_sample();
        }
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && starts == initial_starts + 16);
        now_ms += APP_BATTERY_SLEEP_PERIOD_MS - 1;
        app_battery_service();
        assert(starts == initial_starts + 16);
        ++now_ms;
        app_battery_service();
        assert(starts == initial_starts + 18 && stops == polls);
        app_battery_set_sleep_mode(0);
        next_sample();
        assert(starts == initial_starts + 20);
    }
    /* 相同的10秒电压变化，亮屏逐秒更新与熄屏一次更新保持相同滤波响应。 */
    for (profile = 0; profile < 2; ++profile)
    {
        now_ms = 0;
        app_battery_init();
        adc_value = (3900 * 4095 + 3300) / 6600;
        for (sample = 0; sample < 8; ++sample)
        {
            next_sample();
        }
        adc_value = (3750 * 4095 + 3300) / 6600;
        if (profile == 0)
        {
            for (sample = 0; sample < 10; ++sample)
            {
                next_sample();
            }
        }
        else
        {
            app_battery_set_sleep_mode(1);
            now_ms += APP_BATTERY_SLEEP_PERIOD_MS;
            app_battery_service();
        }
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.percent == 72);
        if (profile == 0)
        {
            awake_voltage_mv = snapshot.voltage_mv;
            awake_percent = snapshot.percent;
        }
        else
        {
            assert(snapshot.voltage_mv == awake_voltage_mv);
            assert(snapshot.percent == awake_percent);
        }
    }
    puts("PASS: screen-off keeps eight 1s startup samples, switches to 10s across tick wrap, resumes 1s on wake and preserves filter response.");
}

static void check_reference_calibration(void)
{
    static const uint32_t supplies_mv[] = {3000, 3278, 3300, 3600};
    app_battery_struct snapshot;
    uint16_t raw;
    uint32_t vdda_mv, previous_starts;
    unsigned supply, i;

    for (supply = 0; supply < sizeof(supplies_mv) / sizeof(supplies_mv[0]); ++supply)
    {
        reference_value = (3300 * host_vrefint_cal + supplies_mv[supply] / 2) / supplies_mv[supply];
        adc_value = (3850 * 4095 + supplies_mv[supply]) / (supplies_mv[supply] * 2);
        app_battery_init();
        for (i = 0; i < 8; ++i)
        {
            next_sample();
        }
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.voltage_mv >= 3847 && snapshot.voltage_mv <= 3853);
        assert((ADC->CCR & ADC_CCR_TSVREFE) == 0 && selected_channel == ADC_CHANNEL_1);
    }
    raw = 123;
    vdda_mv = 456;
    fail_battery_channel = 1;
    assert(battery_adc_read_raw(2, &raw, &vdda_mv) == HAL_ERROR);
    assert(raw == 123 && vdda_mv == 456 && (ADC->CCR & ADC_CCR_TSVREFE) == 0);
    fail_battery_channel = 0;
    reference_value = 0;
    assert(battery_adc_read_raw(2, &raw, &vdda_mv) == HAL_ERROR);
    assert(raw == 123 && vdda_mv == 456);
    reference_value = 1500;
    config_result = HAL_ERROR;
    previous_starts = starts;
    assert(battery_adc_read_raw(2, &raw, &vdda_mv) == HAL_ERROR && starts == previous_starts);
    config_result = HAL_OK;
    host_vrefint_cal = 0xFFFF;
    assert(battery_adc_read_raw(2, &raw, &vdda_mv) == HAL_ERROR && starts == previous_starts);
    host_vrefint_cal = 0;
    assert(battery_adc_read_raw(2, &raw, &vdda_mv) == HAL_ERROR && starts == previous_starts);
    host_vrefint_cal = 1500;
    ADC->CCR |= ADC_CCR_TSVREFE;
    assert(battery_adc_read_raw(2, &raw, &vdda_mv) == HAL_OK && vdda_mv == 3300);
    assert(host_adc_clock_enabled == 0 && primask == 0);
    assert((ADC->CCR & ADC_CCR_TSVREFE) != 0 && polls == stops);
    puts("PASS: VREFINT keeps 3.85V battery readings stable at VDDA 3.000/3.278/3.300/3.600V; invalid calibration/reference and second-channel failures publish nothing and restore reference power.");
}

int main(int argc, char **argv)
{
    app_battery_struct snapshot;
    unsigned i;
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    (void)argv;
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    app_battery_service();
    assert(starts == 0);
    if (argc > 1 && strcmp(argv[1], "percentage") == 0)
    {
        check_full_voltage();
        check_percent();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "calibration") == 0)
    {
        check_reference_calibration();
        return 0;
    }
    if (argc > 1 && (strcmp(argv[1], "shutdown") == 0 || strcmp(argv[1], "shutdown_sleep") == 0))
    {
        check_shutdown(strcmp(argv[1], "shutdown_sleep") == 0);
        return 0;
    }
    if (argc > 1)
    {
        allocation_failed = 1;
        app_battery_init();
        assert(fatal_errors == 1);
        app_battery_service();
        assert(starts == 0);
        puts("PASS: battery mutex allocation failure follows Error_Handler.");
        return 0;
    }
    host_adc1.CR2 = 0x100;
    assert(battery_adc_is_stop_idle() != 0);
    host_adc1.CR2 |= ADC_CR2_ADON;
    assert(battery_adc_is_stop_idle() != 0); /* 时钟关闭时不读ADC寄存器。 */
    host_adc_clock_enabled = 1;
    assert(battery_adc_is_stop_idle() == 0 && host_adc1.CR2 == 0x101);
    host_adc1.CR2 &= ~ADC_CR2_ADON;
    assert(battery_adc_is_stop_idle() != 0);
    host_adc_clock_enabled = 0;
    assert(battery_adc_read_raw(2, NULL, NULL) == HAL_ERROR);
    assert(starts == 0 && polls == 0 && reads == 0 && stops == 0);
    puts("PASS: NULL ADC output returns HAL_ERROR without starting a conversion.");
    app_battery_init();
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    adc_value = 2389;
    app_battery_service();
    app_battery_get_snapshot(&snapshot);
    assert(!snapshot.valid && snapshot.result == HAL_OK);
    for (i = 1; i < 8; ++i)
    {
        next_sample();
    }
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.adc_raw == 2389 && snapshot.voltage_mv == 3850);
    assert(snapshot.percent == 64);
    assert(starts == 16 && polls == 16 && reads == 16 && stops == 16);
    now_ms += APP_BATTERY_PERIOD_MS - 1;
    app_battery_service();
    assert(starts == 16);
    ++now_ms;
    app_battery_service();
    assert(starts == 18);
    adc_value = 0;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.adc_raw == 0 && snapshot.voltage_mv == 3369);
    adc_value = 4095;
    next_sample();
    check_failure(HAL_ERROR, 1); /* 6.6V不进入滤波。 */
    adc_value = (4200 * 4095 + 3300) / 6600;
    next_sample();
    app_battery_get_snapshot(&snapshot);
    assert(snapshot.valid && snapshot.result == HAL_OK && snapshot.voltage_mv > 3369);
    for (i = 0; i < sizeof(failures) / sizeof(failures[0]); ++i)
    {
        uint32_t previous_polls = polls;
        uint32_t previous_reads = reads;
        uint32_t previous_stops = stops;
        uint32_t previous_voltage_mv = snapshot.voltage_mv;
        start_result = failures[i];
        next_sample();
        check_failure(failures[i], 1);
        assert(polls == previous_polls && reads == previous_reads && stops == previous_stops);
        start_result = HAL_OK;
        poll_result = failures[i];
        next_sample();
        check_failure(failures[i], 1);
        assert(polls == previous_polls + 1 && reads == previous_reads && stops == previous_stops + 1);
        poll_result = HAL_OK;
        stop_result = failures[i];
        next_sample();
        check_failure(failures[i], 0);
        stop_result = HAL_OK;
        adc_value = 2048;
        next_sample();
        app_battery_get_snapshot(&snapshot);
        assert(snapshot.valid && snapshot.result == HAL_OK && snapshot.adc_raw == 2048);
        assert(snapshot.voltage_mv > 3301 && snapshot.voltage_mv < previous_voltage_mv);
    }
    /* 即使HAL毫秒计数回绕，仍按一秒间隔采样。 */
    now_ms = UINT32_MAX - 500;
    app_battery_service();
    {
        uint32_t previous_starts = starts;
        now_ms += 999;
        app_battery_service();
        assert(starts == previous_starts);
        ++now_ms;
        app_battery_service();
        assert(starts == previous_starts + 2);
    }
    app_battery_get_snapshot(NULL);
    assert(!locked && fatal_errors == 0);
    check_sleep_sampling();
    puts("PASS: battery startup sampling, filtered voltage, ADC cleanup, failures/recovery and tick wrap.");
    return 0;
}
