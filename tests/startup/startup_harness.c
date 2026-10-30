/* 执行真实任务入口，模拟等待期间sensor获得调度，验证首帧和采集不等RTC/蓝牙。 */
#include <stdint.h>
#define app_diag_boot_set startup_boot_set
#define app_diag_take_imu_calibration startup_take_imu_calibration
#include "../../application/app_task.c"
#include "bl24c02.h"
#include "app_weather.h"

static unsigned battery_log_calls;
void app_battery_log_init(void)
{
    battery_log_calls = 0;
}
void app_battery_log_service(uint32_t seconds)
{
    (void)seconds;
    ++battery_log_calls;
}
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static uint32_t reset_feed_events;
void app_reset_watchdog_feed(uint32_t now_ms)
{
    assert(now_ms == HAL_GetTick());
    ++reset_feed_events;
}

void app_reset_capture_stop(uint32_t kind, uint32_t pc, uint32_t task_tag)
{
    (void)kind;
    (void)pc;
    (void)task_tag;
    assert(0 && "normal startup must not enter a fault recorder");
}

static TaskFunction_t entries[3];
static jmp_buf touch_stop;
static unsigned int touch_inits, touch_samples;
static uint32_t touch_work_ms;
static uint8_t touch_sleep_test;
static unsigned int touch_notifications, touch_blocks;
static uint8_t lvgl_wait_test, wake_key_active;
static unsigned lvgl_notifications, lvgl_waits, wake_polls, irq_notifications, irq_yields;
static uint8_t background_test, background_sleeping;
static uint32_t mock_imu_period = 10;
static unsigned background_samples, motion_samples, background_wakes, background_sleeps;
static uint32_t background_wake_ms, background_last_sample_ms, background_start_ms;
static uint32_t background_baro_ms, background_battery_ms;
static unsigned background_recovery_steps, mag_recovery_steps, baro_recovery_steps;
static unsigned int task_count, fail_task;
static uint8_t fail_devices, displayed, frame, rtc_done, ble_done, sensor_ran;
static unsigned int hardware_count, service_count, battery_samples;
static unsigned int cal_begins, restores, saves, sensor_rounds, recovery_steps;
static unsigned int baro_calls;
static uint32_t sensor_elapsed_ms;
static uint8_t low_rate_mask;
static uint8_t baro_cadence;
static uint8_t power_profile, heart_enabled;
static uint8_t battery_sleep_mode;
static uint32_t aht_period_ms;
static uint8_t environment_test, environment_stage, environment_bus_parked;
static uint32_t environment_mode_ms, environment_last_aht_ms = UINT32_MAX;
static uint32_t environment_aht_window_ms, environment_baro_window_ms;
static unsigned environment_reads, environment_history, environment_aht_windows, environment_mode_windows;
static uint8_t environment_sleep_retry_test;
static unsigned environment_standby_writes, environment_retry_writes;
static uint8_t reference_recovery_test, reference_enabled;
static unsigned reference_samples, reference_full_samples, reference_recovery_steps;
static uint32_t reference_ready_ms, reference_first_full_ms;
static struct
{
    uint32_t start_ms, loop_ms, ppg_ms, battery_ms, baro_ms;
    unsigned ppg_reads, short_waits, normal_waits;
    uint8_t mode, started;
} sensor_timing;
static uint8_t bluetooth_resume_test, bluetooth_resume_phase;
static uint8_t sport_test, sport_active, sport_baro_sleeping;
static unsigned sport_samples, sport_save_calls, sport_save_steps;
static uint32_t sport_first_ms = UINT32_MAX;
static uint8_t sleep_test, sleep_bus_parked, sleep_imu, sleep_baro, sleep_heart_mode;
static uint32_t sleep_cycle_ms, sleep_window_ms, sleep_environment_ms, sleep_first_ms;
static uint32_t sleep_imu_sample_ms, sleep_heart_advance_ms;
static uint32_t sleep_heart_duration_ms = 10000U;
static unsigned int sleep_parks, sleep_resumes, sleep_heart_starts, sleep_history;
static unsigned sleep_idle_waits;
static uint8_t sleep_first_seen;
static uint8_t sleep_pose_seen;
static unsigned int sleep_pose_samples;
static uint32_t sleep_pose_ms;
static uint8_t sleep_recovery_test, sleep_recovery_sample;
static uint32_t sleep_recovery_last_ms;
static unsigned sleep_recovery_steps;
static uint8_t restored_mode, load_error, save_error, manual_request, complete_cal, fail_cal;
static mpu6050_cal_state mock_cal_state;
static const mpu6050_offset_struct stored_offset = {{20, -10, 30}, {66, -33, 99}};
static uint8_t uptime_eeprom[256], uptime_io_active, uptime_eeprom_ready;
static unsigned uptime_writes, weather_writes;
static uint8_t weather_task_test, weather_task_updated;
static uint16_t storage_address;
static unsigned critical_depth;

void vPortEnterCritical(void)
{
    ++critical_depth;
}

void vPortExitCritical(void)
{
    assert(critical_depth != 0);
    --critical_depth;
}

uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length)
{
    assert(critical_depth == 0 && uptime_io_active == 0);
    assert(address >= 0xA0U && address + length <= sizeof(uptime_eeprom));
    if (uptime_eeprom_ready == 0)
    {
        memset(uptime_eeprom, 0xFF, sizeof(uptime_eeprom));
        uptime_eeprom_ready = 1;
    }
    memcpy(data, uptime_eeprom + address, length);
    return 0;
}

void bl24c02_transfer_begin(bl24c02_transfer_struct *io, uint16_t address, uint8_t *data, uint16_t length, uint8_t write)
{
    assert(critical_depth == 0);
    assert(uptime_io_active == 0 && address >= 0xA0U && address + length <= sizeof(uptime_eeprom));
    uptime_io_active = 1;
    io->address = address;
    storage_address = address;
    io->data = data;
    io->remaining = length;
    io->write = write;
    io->phase = 0;
}

uint8_t bl24c02_transfer_step(bl24c02_transfer_struct *io)
{
    assert(critical_depth == 0);
    assert(uptime_io_active != 0);
    if (io->phase++ == 0)
    {
        return 2;
    }
    if (io->write != 0)
    {
        memcpy(uptime_eeprom + io->address, io->data, io->remaining);
        if (io->address < 0xC0)
        {
            ++uptime_writes;
        }
        else
        {
            ++weather_writes;
        }
    }
    else
    {
        memcpy(io->data, uptime_eeprom + io->address, io->remaining);
    }
    uptime_io_active = 0;
    return 0;
}

uint8_t startup_take_imu_calibration(void)
{
    uint8_t request = manual_request;
    manual_request = 0;
    return request;
}
app_imu_store_result app_imu_store_load(mpu6050_offset_struct *offset)
{
    if (load_error != 0)
    {
        return APP_IMU_STORE_ERROR;
    }
    if (restored_mode != 0)
    {
        *offset = stored_offset;
        return APP_IMU_STORE_OK;
    }
    return APP_IMU_STORE_EMPTY;
}
app_imu_store_result app_imu_store_save_begin(const mpu6050_offset_struct *offset)
{
    assert(uptime_io_active == 0);
    assert(sport_active == 0);
    assert(memcmp(offset, &stored_offset, sizeof(*offset)) == 0);
    ++saves;
    return APP_IMU_STORE_BUSY;
}
app_imu_store_result app_imu_store_save_step(void)
{
    if (sport_test == 2U && ++sport_save_steps < 4U)
    {
        return APP_IMU_STORE_BUSY;
    }
    if (save_error != 0)
    {
        return APP_IMU_STORE_ERROR;
    }
    return APP_IMU_STORE_OK;
}
uint8_t App_IMU_SaveService(void)
{
    assert(uptime_io_active == 0);
    if (sport_test != 0)
    {
        assert(sport_active == 0);
        ++sport_save_calls;
    }
    return 0;
}
void my_mpu6050_restore_offset(const mpu6050_offset_struct *offset)
{
    assert(memcmp(offset, &stored_offset, sizeof(*offset)) == 0);
    ++restores;
    mock_cal_state = MPU6050_CAL_RESTORED;
}
void my_mpu6050_get_offset(mpu6050_offset_struct *offset)
{
    *offset = stored_offset;
}
static uint32_t milestones[APP_BOOT_FIELD_COUNT];
static TickType_t tick;
static TickType_t rtc_begin_tick, ble_begin_tick;
static jmp_buf stop, sensor_stop;
static unsigned int watchdog_inits, watchdog_enables, watchdog_feeds;
static uint32_t watchdog_enable_delay;
static uint8_t first_frame_stall;
static uint8_t first_frame_slow, first_frame_deadline;
static uint32_t first_frame_chunk_ms = 150U;
static unsigned int first_frame_chunks = 10U;
static void (*display_progress_callback)(void);
static uint32_t watchdog_last_falling_ms;
static uint8_t runtime_render, runtime_sensor_stalled;
static uint32_t runtime_chunk_ms = 150U;
static unsigned int runtime_chunks = 10U, render_yields;

void tps3823_init(void)
{
    ++watchdog_inits;
}
void tps3823_enable(void)
{
    assert(displayed && !frame && app_sensor_watchdog_progress == 0);
    ++watchdog_enables;
    tick += watchdog_enable_delay;
    watchdog_last_falling_ms = tick;
}
void tps3823_feed(void)
{
    assert(watchdog_enables == 1);
    if (runtime_render != 0)
    {
        /* 在喂狗前检查，不能让一个迟到的脉冲掩盖已经跨过的硬件超时。 */
        assert((uint32_t)(tick - watchdog_last_falling_ms) < 900U);
    }
    ++watchdog_feeds;
    watchdog_last_falling_ms = tick;
}

static void test_watchdog_progress(uint8_t wrap)
{
    displayed = 1;
    watchdog_enable_delay = 302;
    tick = 1000;
    if (wrap != 0)
    {
        tick = UINT32_MAX - 350U;
    }
    app_watchdog_begin();
    assert(watchdog_enables == 1 && watchdog_feeds == 0);
    assert(reset_feed_events == 1U);
    tick += 100;
    app_watchdog_service();
    assert(watchdog_feeds == 1);
    ++app_sensor_watchdog_progress;
    app_watchdog_service();
    assert(watchdog_feeds == 1);
    tick += 99;
    ++app_sensor_watchdog_progress;
    app_watchdog_service();
    assert(watchdog_feeds == 1);
    ++tick;
    app_watchdog_service();
    assert(watchdog_feeds == 2);
    /* sensor 卡住时，即使 LVGL 继续执行，也不再喂狗。 */
    tick += 2000;
    app_watchdog_service();
    assert(watchdog_feeds == 2);
    ++app_sensor_watchdog_progress;
    app_watchdog_service();
    assert(watchdog_feeds == 3);
    /* LVGL 卡住时，sensor 的计数更新本身不会触发喂狗。 */
    tick += 2000;
    ++app_sensor_watchdog_progress;
    assert(watchdog_feeds == 3);
    app_sensor_watchdog_progress = UINT32_MAX;
    app_watchdog_service();
    ++app_sensor_watchdog_progress;
    tick += 100;
    app_watchdog_service();
    assert(watchdog_feeds == 5 && app_watchdog.sensor_started != 0);
    tick += 100;
    app_watchdog_service();
    assert(watchdog_feeds == 5);
    assert(reset_feed_events == watchdog_feeds + 1U);
    puts("Watchdog: protection before first frame, 100ms cadence, task stalls and tick/progress wrap: PASS");
}

static void test_watchdog_startup(uint8_t wrap)
{
    unsigned int i;
    displayed = 1;
    tick = 1000;
    if (wrap != 0)
    {
        tick = UINT32_MAX - 350U;
    }
    app_watchdog_begin();
    for (i = 0; i < 49U; ++i)
    {
        tick += 100U;
        app_watchdog_service();
    }
    assert(watchdog_enables == 1 && watchdog_feeds == 49U);
    for (i = 0; i < 51U; ++i)
    {
        tick += 100U;
        app_watchdog_service();
    }
    assert(watchdog_feeds == 49U && app_watchdog.sensor_started == 0);
    puts("Watchdog: sensor startup may progress for 5s, then missing first round stops feeding, including tick wrap: PASS");
}

void startup_boot_set(uint8_t field, uint32_t value)
{
    milestones[field] = value;
}
uint32_t HAL_GetTick(void)
{
    return tick;
}

void __DMB(void)
{
}
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack, void *arg, UBaseType_t priority, TaskHandle_t *handle)
{
    assert(arg == NULL && task_count < 3);
    if (task_count == 0)
    {
        assert(handle == &s_lvgl_task_handle);
    }
    else if (task_count == 1)
    {
        assert(handle == NULL);
    }
    else
    {
        assert(handle == &s_touch_task_handle);
    }
    if (task_count == 0)
    {
        assert(strcmp(name, "lvgl") == 0 && stack == 1024 && priority == 3);
        assert(!displayed && hardware_count == 0);
    }
    else if (task_count == 1)
    {
        assert(strcmp(name, "sensor") == 0 && stack == 512 && priority == 2);
        assert(frame && !rtc_done && !ble_done && hardware_count == 0);
    }
    else
    {
        assert(strcmp(name, "touch") == 0 && stack == 256 && priority == 1);
        assert(frame && !rtc_done && !ble_done && hardware_count == 0);
    }
    if (fail_task == task_count + 1)
    {
        return 0;
    }
    entries[task_count++] = entry;
    if (handle != NULL)
    {
        if (task_count == 1)
        {
            *handle = &lvgl_notifications;
        }
        else
        {
            *handle = &touch_inits;
        }
    }
    return pdPASS;
}

BaseType_t xTaskNotifyGive(TaskHandle_t task)
{
    if (task == &lvgl_notifications)
    {
        assert(app_task_is_screen_off() != 0);
        ++lvgl_notifications;
        return pdPASS;
    }
    assert(task == &touch_inits);
    ++touch_notifications;
    return pdPASS;
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    assert(task == &lvgl_notifications && woken != NULL && *woken == pdFALSE);
    ++irq_notifications;
    ++lvgl_notifications;
    *woken = pdTRUE;
}

void startup_yield_from_isr(BaseType_t woken)
{
    assert(woken == pdTRUE);
    ++irq_yields;
}

uint8_t App_UI_IsWakeKeyActive(void)
{
    return wake_key_active;
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait)
{
    if (lvgl_wait_test != 0)
    {
        uint32_t before = tick;
        assert(clear == pdTRUE && wait == 200 && rtc_done != 0 && ble_done != 0 && wake_key_active == 0);
        ++lvgl_waits;
        if (lvgl_waits == 1)
        {
            assert(lvgl_notifications == 1);
            lvgl_notifications = 0;
            assert(tick == before);
            return 1;
        }
        assert(lvgl_notifications == 0);
        if (lvgl_waits == 2)
        {
            tick += 37;
            app_task_wake_from_isr();
            assert(lvgl_notifications == 1);
            lvgl_notifications = 0;
            ++app_sensor_watchdog_progress;
            return 1;
        }
        assert(lvgl_waits == 3 && wake_polls == 3);
        tick += wait;
        ++app_sensor_watchdog_progress;
        app_watchdog_service();
        assert((uint32_t)(tick - watchdog_last_falling_ms) < 900);
        longjmp(stop, 5);
    }
    assert(touch_sleep_test != 0 && clear == pdTRUE && wait == portMAX_DELAY);
    assert(touch_samples == 1U && touch_inits == 1U);
    ++touch_blocks;
    if (touch_notifications == 0)
    {
        tick += 5000U; /* 整段熄屏不运行触摸轮询，直到PA4通知。 */
        app_task_set_power_state(0, 0, 0, 0);
    }
    assert(touch_notifications == 1U);
    touch_notifications = 0;
    return 1;
}
void Error_Handler(void)
{
    assert(fail_task != 0);
    longjmp(stop, 2);
}
void __disable_irq(void)
{
    assert(0);
}
void vTaskStartScheduler(void)
{
    assert(task_count == 1);
    entries[0](NULL);
    assert(0);
}
TickType_t xTaskGetTickCount(void)
{
    return tick;
}
static void sensor_timing_wait(TickType_t wait)
{
    uint32_t work = tick - sensor_timing.loop_ms;
    assert(wait >= 1 && wait <= 10);
    assert(++sensor_rounds == app_sensor_watchdog_progress);
    if (work >= 10)
    {
        assert(wait == 1); /* 超期后必须让出CPU，但不能额外等待完整10ms。 */
        ++sensor_timing.short_waits;
    }
    else
    {
        assert(wait == 10 - work);
        if (sensor_timing.mode == 2 || sensor_timing.mode == 4)
        {
            assert((uint32_t)(tick - sensor_timing.start_ms) < 100); /* 加速度参考初次启用先完成100ms稳定等待。 */
        }
        ++sensor_timing.normal_waits;
    }
    tick += wait;
    sensor_elapsed_ms = tick - sensor_timing.start_ms;
    low_rate_mask = 0;
    app_watchdog_service();
    if (sensor_elapsed_ms >= 3000)
    {
        assert(watchdog_feeds > 20 && battery_samples >= 3);
        if (sensor_timing.mode == 1 || sensor_timing.mode == 3)
        {
            assert(sensor_rounds == 300 && sensor_timing.short_waits == 0);
            assert(sensor_timing.normal_waits != 0 && sensor_timing.ppg_reads >= 90);
            printf("PASS: 5ms MPU + alternate 3ms MAG/PPG model: %u loops / %lums, %u PPG reads; no normal read overlap.\n",
                   sensor_rounds, (unsigned long)sensor_elapsed_ms, sensor_timing.ppg_reads);
        }
        else
        {
            assert(sensor_timing.short_waits + sensor_timing.normal_waits == sensor_rounds);
            assert(sensor_timing.normal_waits <= 10);
            puts("PASS: sustained/exact-budget work and a long stall retain a one-tick yield without catch-up bursts.");
        }
        longjmp(sensor_stop, 1);
    }
}
void vTaskDelay(TickType_t ticks)
{
    unsigned int feeds = watchdog_feeds;
    if (sensor_timing.mode != 0)
    {
        assert(ticks == 1);
        sensor_timing_wait(ticks);
        return;
    }
    assert((ticks == 5 || ticks == 1 || ((power_profile != 0 || lvgl_wait_test != 0) && (s_power_state & APP_POWER_IDLE) != 0 && ticks == 20U)) && task_count == 3);
    /* Tickless由独立硬件/内核模型验证；任务阻塞不能替代进度喂狗。 */
    assert(watchdog_feeds == feeds);
    tick += ticks;
    if (lvgl_wait_test != 0 && sensor_ran != 0)
    {
        ++app_sensor_watchdog_progress;
    }
    if (ticks == 1)
    {
        ++render_yields;
    }
    if (sensor_ran == 0)
    {
        assert(!rtc_done && !ble_done && service_count == 1);
        sensor_ran = 1;
        if (setjmp(sensor_stop) == 0)
        {
            entries[1](NULL);
            assert(0);
        }
        if (baro_cadence != 0)
        {
            longjmp(stop, 5);
        }
        if (setjmp(touch_stop) == 0)
        {
            entries[2](NULL);
            assert(0);
        }
    }
    else if (runtime_render != 0 && runtime_sensor_stalled == 0)
    {
        ++app_sensor_watchdog_progress;
    }
    if (rtc_done && ble_done)
    {
        if (lvgl_wait_test != 0)
        {
            assert(wake_key_active != 0 && ticks == 20);
            if (++wake_polls == 3)
            {
                wake_key_active = 0;
            }
            ++app_sensor_watchdog_progress;
            return;
        }
        if (bluetooth_resume_test != 0 && bluetooth_resume_phase == 0)
        {
            bluetooth_resume_phase = 1;
            ble_done = 0;
            kt6368a_init_begin(); /* 首次启动已完成后，模拟PA4亮屏重新启动。 */
            return;
        }
        longjmp(stop, 1);
    }
    if (bluetooth_resume_phase != 0)
    {
        assert((uint32_t)(tick - ble_begin_tick) < 2000U);
    }
}
void vTaskDelayUntil(TickType_t *last, TickType_t ticks)
{
    if (reference_recovery_test != 0)
    {
        assert(ticks == 10 || ticks == 20);
        assert(++sensor_rounds == app_sensor_watchdog_progress);
        low_rate_mask = 0;
        app_watchdog_service();
        if (sensor_elapsed_ms >= 300 && reference_recovery_test == 1)
        {
            assert(reference_samples >= 10 && reference_enabled != 0);
            reference_recovery_test = 2;
            heart_enabled = 0;
            app_task_set_power_state(0, 1, 0, 0);
        }
        if (sensor_elapsed_ms >= 900)
        {
            assert(reference_enabled == 0 && reference_recovery_steps == 5 && reference_ready_ms != 0);
            assert(reference_full_samples >= 10 && reference_first_full_ms - reference_ready_ms >= 100);
            assert(reference_first_full_ms - reference_ready_ms < 120);
            longjmp(sensor_stop, 1);
        }
        sensor_elapsed_ms += ticks;
        tick += ticks;
        *last = tick;
        return;
    }
    if (environment_test != 0)
    {
        assert(++sensor_rounds == app_sensor_watchdog_progress);
        if (environment_sleep_retry_test != 0 && environment_retry_writes != 0 && environment_retry_writes < 3)
        {
            assert(environment_bus_parked == 0 && sport_baro_sleeping == 0);
        }
        if (app_task_is_screen_off() != 0)
        {
            assert((ticks == 100 || ticks == 200) && aht_period_ms == 60000 && battery_sleep_mode != 0);
        }
        else
        {
            assert(ticks == 40 && aht_period_ms == 1000 && battery_sleep_mode == 0);
        }
        low_rate_mask = 0;
        app_watchdog_service();
        if (sensor_elapsed_ms >= 10000 && environment_stage == 0)
        {
            environment_stage = 1;
            app_task_set_power_state(1, 0, 1, 0);
        }
        else if (sensor_elapsed_ms >= 125000 && environment_stage == 1)
        {
            assert(environment_aht_windows == 1);
            environment_stage = 2;
            app_task_set_power_state(0, 0, 0, 0);
        }
        else if (sensor_elapsed_ms >= 135000 && environment_stage == 2)
        {
            environment_stage = 3;
            app_task_set_power_state(1, 0, 1, 0);
        }
        if (sensor_elapsed_ms >= 610000)
        {
            assert(environment_reads == 3 && environment_history == 3 && environment_aht_windows == 8);
            if (environment_sleep_retry_test != 0)
            {
                assert(environment_retry_writes == 3 && environment_standby_writes == 5);
            }
            longjmp(sensor_stop, 1);
        }
        sensor_elapsed_ms += ticks;
        tick += ticks;
        *last = tick;
        return;
    }
    if (sensor_timing.mode != 0)
    {
        TickType_t wait = (TickType_t)(*last + ticks - tick);
        assert(ticks == 10);
        *last += ticks;
        sensor_timing_wait(wait);
        return;
    }
    if (background_test != 0)
    {
        assert(ticks == 20 || ticks == 40);
        assert(++sensor_rounds == app_sensor_watchdog_progress);
        app_watchdog_service();
        if (sensor_elapsed_ms >= 6000 && sensor_elapsed_ms < 8000)
        {
            if (background_test == 1)
            {
                app_task_set_power_state(0, 1, 0, 0);
            }
            else if (cal_begins == 0)
            {
                manual_request = 1;
            }
        }
        if (sensor_elapsed_ms >= 8000)
        {
            app_task_set_power_state(0, 0, 0, 0);
        }
        if (sensor_elapsed_ms >= 14500)
        {
            if (background_test == 1)
            {
                assert(background_samples >= 58 && background_samples <= 64);
                assert(motion_samples >= 90);
            }
            else
            {
                assert(background_samples >= 68 && background_samples <= 73);
                assert(motion_samples == 1);
                assert(cal_begins == 1 && saves == 1 && mock_cal_state == MPU6050_CAL_DONE);
            }
            assert(background_wakes == 1 && background_sleeps == 2 && background_sleeping != 0);
            assert((uint32_t)(tick - background_start_ms) == sensor_elapsed_ms);
            assert(watchdog_feeds > 100 && battery_samples >= 14);
            longjmp(sensor_stop, 1);
        }
        low_rate_mask = 0;
        sensor_elapsed_ms += ticks;
        tick += ticks;
        *last = tick;
        return;
    }
    if (touch_sleep_test != 0)
    {
        assert(touch_blocks == 1U && touch_samples == 2U && ticks == APP_TOUCH_PERIOD_MS);
        assert(*last == tick);
        longjmp(touch_stop, 1);
    }
    if (sport_test != 0)
    {
        uint32_t finish_ms = 2000U;
        if (weather_task_test != 0 && weather_task_updated == 0 && uptime_io_active != 0 && storage_address < 0xC0)
        {
            static const char update[] = "WEATHER,61,-53,81\n";
            app_weather_receive((const uint8_t *)update, sizeof(update) - 1, tick);
            app_weather_store_sync();
            weather_task_updated = 1;
        }
        if (sport_test == 3U)
        {
            finish_ms = 1202000U;
        }
        assert(ticks == 10U || ticks == 20U || ticks == 40U);
        assert(++sensor_rounds == app_sensor_watchdog_progress);
        if (sport_active != 0)
        {
            assert(ticks == 10U && sport_baro_sleeping != 0 && baro_calls == 0U);
            if (sport_first_ms == UINT32_MAX)
            {
                sport_first_ms = sensor_elapsed_ms;
            }
        }
        if (sensor_elapsed_ms >= 600U && sensor_elapsed_ms < finish_ms)
        {
            app_task_set_power_state(1, 0, 1, 1); /* 熄屏仍保持运动频率。 */
        }
        if (sensor_elapsed_ms >= finish_ms)
        {
            app_task_set_power_state(0, 0, 0, 0);
        }
        if (sensor_elapsed_ms >= finish_ms + 6000)
        {
            uint32_t seconds;
            assert(sport_active == 0 && sport_baro_sleeping != 0);
            if (sport_test == 3)
            {
                assert(baro_calls != 0);
            }
            else
            {
                assert(baro_calls == 0);
            }
            assert(sport_samples >= 180U && sport_save_calls != 0U);
            if (sport_test == 3U)
            {
                assert(uptime_writes == 2U && app_uptime_get_saved(&seconds) && seconds == 1200U);
                if (weather_task_test != 0)
                {
                    assert(weather_task_updated != 0 && weather_writes == 2 && critical_depth == 0);
                    assert(memcmp(uptime_eeprom + 0xC0, "WTH1", 4) == 0);
                    assert(memcmp(uptime_eeprom + 0xD0, "WTH1", 4) == 0);
                    assert(saves == 1);
                }
            }
            if (sport_test == 2U)
            {
                assert(saves == 1U && sport_save_steps == 4U && sport_first_ms >= 80U);
            }
            longjmp(sensor_stop, 1);
        }
        app_watchdog_service();
        sensor_elapsed_ms += ticks;
        tick += ticks;
        *last = tick;
        return;
    }
    if (sleep_test != 0)
    {
        assert(ticks == 10U || ticks == 20U || ticks == 40U || ticks == 100U || ticks == 200);
        if (ticks == 200)
        {
            ++sleep_idle_waits;
            assert(lvgl_notifications == sleep_idle_waits);
        }
        if (heart_enabled != 0 && (uint32_t)(tick - sleep_window_ms) >= 100U)
        {
            assert(ticks == 10U);
        }
        assert(++sensor_rounds == app_sensor_watchdog_progress);
        assert((uint32_t)(tick - watchdog_last_falling_ms) < 900);
        app_watchdog_service();
        if (sleep_recovery_test != 0 && sensor_elapsed_ms >= 14500U)
        {
            assert(sleep_recovery_steps == 16U && sleep_recovery_sample != 0);
            assert(sleep_imu != 0 && sleep_bus_parked != 0 && sleep_pose_samples >= 50);
            assert(watchdog_feeds > 60 && battery_samples >= 14);
            longjmp(sensor_stop, 1);
        }
        if (sensor_elapsed_ms >= 1212000U)
        {
            app_task_set_power_state(0, 0, 0, 0);
        }
        if (sensor_elapsed_ms >= 1214000U)
        {
            uint32_t seconds;
            assert(uptime_writes >= 2U && app_uptime_get_saved(&seconds) && seconds >= 1200U);
            assert(sleep_heart_starts == 2 && sleep_history == 5);
            assert(sleep_parks >= 3U && sleep_resumes >= 2U);
            assert(sleep_pose_samples >= 5900 && sleep_pose_samples <= 6100);
            assert(sleep_parks > 5700 && sleep_resumes > 5700);
            assert(sleep_imu != 0 && sleep_baro != 0 && sleep_bus_parked == 0);
            assert(watchdog_feeds > 5900 && battery_samples > 1200);
            assert(sleep_idle_waits > 5700 && sleep_idle_waits < 6000);
            assert(battery_sleep_mode == 0);
            longjmp(sensor_stop, 1);
        }
        sensor_elapsed_ms += ticks;
        tick += ticks;
        *last = tick;
        return;
    }
    if (touch_inits != 0)
    {
        uint32_t expected = APP_TOUCH_PERIOD_MS;
        if ((s_power_state & APP_POWER_IDLE) != 0)
        {
            expected = APP_IDLE_POLL_MS;
        }
        assert(ticks == expected);
        assert(last != NULL && touch_inits == 1 && touch_samples == 1);
        if (touch_work_ms >= APP_TOUCH_PERIOD_MS)
        {
            assert(*last == tick); /* 读取超期后不补跑。 */
        }
        else
        {
            assert(*last == tick - touch_work_ms);
        }
        longjmp(touch_stop, 1);
    }
    assert(last != NULL && (ticks == 10U || ticks == APP_SENSOR_TASK_PERIOD || ticks == APP_SENSOR_IDLE_PERIOD_MS));
    if (heart_enabled != 0)
    {
        assert(ticks == 10U);
    }
    assert(hardware_count == 5 && battery_samples <= 1 + sensor_elapsed_ms / 1000);
    assert(battery_samples >= sensor_elapsed_ms / 1000);
    assert(low_rate_mask == 1U || low_rate_mask == 2U || low_rate_mask == 4U);
    low_rate_mask = 0;
    assert(!rtc_done && !ble_done);
    ++sensor_rounds;
    sensor_elapsed_ms += ticks;
    assert(recovery_steps == baro_recovery_steps + background_recovery_steps + mag_recovery_steps);
    assert(app_sensor_watchdog_progress == sensor_rounds);
    if (power_profile != 0 && heart_enabled != 0 && sensor_rounds == 1U)
    {
        heart_enabled = 0; /* 下一轮必须恢复原姿态/校准采样档位。 */
    }
    if (sensor_rounds < 2 || (baro_cadence != 0 && sensor_rounds < 150))
    {
        tick += ticks;
        *last = tick;
        return;
    }
    longjmp(sensor_stop, 1);
}
void App_Display_Init(void)
{
    assert(task_count == 1 && hardware_count == 0);
    tick = 650;
    displayed = 1;
}
void lv_refr_now(void *display)
{
    unsigned int i, feeds;
    assert(display == NULL && displayed && !rtc_done && !ble_done);
    assert(watchdog_enables == 1 && app_sensor_watchdog_progress == 0);
    assert(display_progress_callback != NULL);
    if (first_frame_stall != 0)
    {
        tick += 5000U;
        assert(watchdog_feeds == 0 && task_count == 1);
        longjmp(stop, 3);
    }
    if (first_frame_slow != 0 || first_frame_deadline != 0)
    {
        for (i = 0; i < first_frame_chunks; ++i)
        {
            tick += first_frame_chunk_ms; /* 模拟长首帧，按最短900ms检查有效下降沿间隔。 */
            assert((uint32_t)(tick - watchdog_last_falling_ms) < 900U);
            display_progress_callback();
        }
        if (first_frame_deadline != 0)
        {
            tick += 3500U;
            feeds = watchdog_feeds;
            display_progress_callback();
            tick += 100U;
            display_progress_callback();
            assert(watchdog_feeds == feeds && !frame);
            longjmp(stop, 4);
        }
    }
    frame = 1;
}
void lv_port_disp_set_progress_callback(void (*callback)(void))
{
    display_progress_callback = callback;
}
void App_Display_Service(void)
{
    unsigned int i, feeds, yields_before;
    TickType_t callback_tick;
    assert(frame);
    ++service_count;
    if (lvgl_wait_test != 0 && rtc_done != 0 && ble_done != 0)
    {
        if (lvgl_waits == 0)
        {
            app_task_wake_from_isr();
        }
        else if (lvgl_waits == 1 && wake_polls == 0)
        {
            wake_key_active = 1;
        }
    }
    if (runtime_render != 0)
    {
        if (service_count == 2U && runtime_render == 2U)
        {
            runtime_sensor_stalled = 1;
        }
        if (service_count == 2U && runtime_render == 3U)
        {
            runtime_chunks = 40U;
        }
        feeds = watchdog_feeds;
        for (i = 0; i < runtime_chunks; ++i)
        {
            tick += runtime_chunk_ms; /* DMA已完成，模拟CPU持续渲染，不走DMA等待。 */
            yields_before = render_yields;
            callback_tick = tick;
            if (display_progress_callback != NULL)
            {
                display_progress_callback();
            }
            /* 有效分块必须让出一次CPU；超过5s预算后不延时也不再喂狗。 */
            if ((uint32_t)(callback_tick - app_watchdog.render_started_ms) < APP_WATCHDOG_RENDER_TIMEOUT_MS)
            {
                assert(render_yields == yields_before + 1U);
            }
            else
            {
                assert(render_yields == yields_before);
            }
            if (service_count == 1U || runtime_render == 1U)
            {
                assert((uint32_t)(tick - watchdog_last_falling_ms) < 900U);
            }
        }
        if (service_count == 2U)
        {
            assert(sensor_ran && render_yields != 0 && app_watchdog.sensor_started != 0);
            if (runtime_render == 1U)
            {
                assert(watchdog_feeds > feeds);
            }
            else
            {
                assert((uint32_t)(tick - watchdog_last_falling_ms) >= 900U);
            }
            longjmp(stop, 5);
        }
    }
}
void watch_rtc_init_begin(void)
{
    assert(frame && task_count == 3);
    rtc_begin_tick = tick;
}
uint8_t watch_rtc_init_is_pending(void)
{
    return rtc_done == 0;
}
HAL_StatusTypeDef watch_rtc_init_service(void)
{
    if ((uint32_t)(tick - rtc_begin_tick) < 5000U)
    {
        return HAL_BUSY;
    }
    rtc_done = 1;
    if (fail_devices)
    {
        return HAL_TIMEOUT;
    }
    return HAL_OK;
}
void kt6368a_init_begin(void)
{
    assert(frame && task_count == 3);
    ble_begin_tick = tick;
}
void kt6368a_set_rx_callback(void (*callback)(void))
{
    assert(callback != NULL);
}
uint8_t kt6368a_rx_pending(void)
{
    return 0;
}
uint8_t kt6368a_init_is_pending(void)
{
    return ble_done == 0;
}
HAL_StatusTypeDef kt6368a_init_service(void)
{
    if ((uint32_t)(tick - ble_begin_tick) < 1700U)
    {
        return HAL_BUSY;
    }
    ble_done = 1;
    if (fail_devices)
    {
        return HAL_TIMEOUT;
    }
    return HAL_OK;
}
void my_i2c_init(my_i2c_bus_t bus)
{
    if (environment_test != 0 && hardware_count == 5)
    {
        assert(bus == MY_I2C_BUS_SENSOR && environment_bus_parked != 0);
        environment_bus_parked = 0;
        return;
    }
    if (sleep_test != 0 && hardware_count == 5U)
    {
        assert(bus == MY_I2C_BUS_SENSOR && sleep_bus_parked != 0);
        sleep_bus_parked = 0;
        ++sleep_resumes;
        return;
    }
    assert(bus == MY_I2C_BUS_SENSOR && frame && !ble_done && !rtc_done);
    assert(hardware_count++ == 0);
}
uint8_t my_mpu6050_init(void)
{
    assert(hardware_count++ == 1);
    return fail_devices;
}
uint8_t spl006_init(void)
{
    assert(hardware_count++ == 2);
    return fail_devices;
}
uint8_t lsm303dlhc_mag_init(void)
{
    assert(hardware_count++ == 3);
    return fail_devices;
}
em7028_result em7028_init(void)
{
    assert(hardware_count++ == 4);
    if (fail_devices)
    {
        return EM7028_ERROR;
    }
    return EM7028_OK;
}
void App_IMU_Init(void)
{
    assert(hardware_count == 5);
}
void App_Baro_Init(void)
{
    assert(hardware_count == 5);
}
void app_heart_init(void)
{
    assert(hardware_count == 5);
}
void app_heart_set_sleep_mode(uint8_t sleeping)
{
    assert(sleeping <= 1U);
    if (sleep_test != 0)
    {
        sleep_heart_mode = sleeping;
        heart_enabled = 0;
        sleep_cycle_ms = tick - sleep_heart_advance_ms;
    }
}
void app_heart_set_sport_mode(uint8_t enabled)
{
    sport_active = enabled;
    heart_enabled = enabled;
}
uint8_t app_heart_sleep_pending(void)
{
    if (sleep_test != 0 && sleep_heart_mode != 0)
    {
        return heart_enabled != 0 || (uint32_t)(tick - sleep_cycle_ms) >= 600000U;
    }
    return 0;
}
uint8_t app_imu_is_calibrating(void)
{
    return 0;
}
uint8_t spl006_set_sleep(uint8_t sleeping)
{
    assert(sleeping <= 1U);
    if (environment_test != 0)
    {
        assert(environment_bus_parked == 0);
        if (sleeping != 0)
        {
            ++environment_standby_writes;
            if (environment_sleep_retry_test != 0 && environment_reads == 2)
            {
                ++environment_retry_writes;
                assert(environment_retry_writes <= 3);
                if (environment_retry_writes < 3)
                {
                    return 1; /* 第二个气压窗口结束时模拟两次NACK，器件仍保持转换状态。 */
                }
            }
        }
    }
    sport_baro_sleeping = sleeping;
    if (environment_test != 0 && sleeping == 0)
    {
        assert(environment_bus_parked == 0);
        environment_baro_window_ms = sensor_elapsed_ms;
    }
    if (sleep_test != 0)
    {
        assert(sleep_bus_parked == 0);
        sleep_baro = sleeping;
        if (sleeping == 0 && sleep_heart_mode != 0)
        {
            sleep_environment_ms = sensor_elapsed_ms;
        }
    }
    return 0;
}
void my_i2c_suspend(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR);
    if (environment_test != 0)
    {
        assert(environment_bus_parked == 0 && sport_baro_sleeping != 0);
        environment_bus_parked = 1;
    }
    if (sleep_test != 0)
    {
        assert(sleep_imu != 0 && sleep_baro != 0 && heart_enabled == 0 && sleep_bus_parked == 0);
        sleep_bus_parked = 1;
        ++sleep_parks;
    }
}
void app_aht21_set_period(uint32_t period_ms)
{
    assert(period_ms == 1000 || period_ms == 60000);
    aht_period_ms = period_ms;
    if (environment_test != 0)
    {
        environment_mode_ms = sensor_elapsed_ms;
        environment_mode_windows = 0;
        environment_last_aht_ms = UINT32_MAX;
    }
}
void app_baro_record_history(void)
{
    if (environment_test != 0)
    {
        ++environment_history;
    }
    if (sleep_test != 0 && sleep_heart_mode != 0)
    {
        assert(sleep_bus_parked == 0 && sleep_baro == 0);
        ++sleep_history;
    }
}
void app_battery_init(void)
{
    assert(hardware_count == 5);
}
void app_battery_set_sleep_mode(uint8_t sleeping)
{
    assert(sleeping <= 1);
    battery_sleep_mode = sleeping;
}
void app_battery_service(void)
{
    if (environment_test != 0)
    {
        ++battery_samples;
        return;
    }
    if (sensor_timing.mode != 0)
    {
        if (battery_samples != 0)
        {
            uint32_t now = tick - sensor_timing.start_ms;
            assert(now - sensor_timing.battery_ms >= 1000 && now - sensor_timing.battery_ms <= 1200);
            sensor_timing.battery_ms = now;
        }
        ++battery_samples;
        return;
    }
    if (background_test != 0)
    {
        if (battery_samples != 0)
        {
            assert(sensor_elapsed_ms - background_battery_ms >= 1000);
            assert(sensor_elapsed_ms - background_battery_ms <= 1040);
            background_battery_ms = sensor_elapsed_ms;
        }
        ++battery_samples;
        return;
    }
    if (sleep_test != 0 || sport_test != 0)
    {
        ++battery_samples;
        return;
    }
    if (battery_samples != 0)
    {
        assert(low_rate_mask == 0);
        low_rate_mask |= 1U;
        assert(sensor_elapsed_ms >= battery_samples * 1000);
        assert(sensor_elapsed_ms < battery_samples * 1000 + 40);
    }
    ++battery_samples;
}
void my_mpu6050_zero_offset_begin(uint32_t now)
{
    assert(now == tick);
    if (background_test != 0)
    {
        assert(background_sleeping != 0 && battery_samples >= 6);
    }
    else
    {
        assert(battery_samples == 1);
    }
    ++cal_begins;
    mock_cal_state = MPU6050_CAL_WAIT_STILL;
}
mpu6050_cal_state my_mpu6050_zero_offset_state(void)
{
    return mock_cal_state;
}
void my_mpu6050_zero_offset_progress(uint16_t *stable, uint16_t *average)
{
    *stable = 0;
    *average = 0;
}
void app_aht21_init(void)
{
    assert(battery_samples == 1);
    aht_period_ms = APP_AHT21_PERIOD_MS;
}
void App_IMU_Service(uint8_t read_mag)
{
    assert(hardware_count == 5 && !rtc_done && !ble_done);
    (void)read_mag;
    if (reference_recovery_test != 0)
    {
        if (reference_enabled != 0)
        {
            assert(mock_imu_period == 10);
            ++reference_samples;
        }
        else
        {
            assert(mock_imu_period == 20 && reference_ready_ms != 0);
            assert((uint32_t)(tick - reference_ready_ms) >= 100);
            if (reference_full_samples == 0)
            {
                reference_first_full_ms = tick;
            }
            ++reference_full_samples;
        }
    }
    if (sensor_timing.mode != 0)
    {
        if (sensor_timing.started == 0)
        {
            sensor_timing.start_ms = sensor_timing.ppg_ms = tick;
            sensor_timing.started = 1;
        }
        if (sensor_timing.mode == 2)
        {
            tick += 15;
            if (sensor_rounds == 30)
            {
                tick += 120; /* 模拟一次长显示抢占。 */
            }
        }
        else if (sensor_timing.mode == 4)
        {
            tick += 10; /* 恰好到期也必须让出至少一拍。 */
        }
        else
        {
            tick += 5;
            if (read_mag != 0)
            {
                tick += 3;
            }
        }
    }
    if (background_test != 0)
    {
        assert(background_sleeping == (mock_imu_period == 200));
        if (mock_imu_period != 200)
        {
            assert((uint32_t)(tick - background_wake_ms) >= 100);
            if (motion_samples != 0)
            {
                assert((uint32_t)(tick - background_last_sample_ms) == 20);
            }
            ++motion_samples;
        }
        else
        {
            assert(mock_imu_period == 200);
            if (background_samples != 0 && (uint32_t)(tick - background_last_sample_ms) < 600)
            {
                assert((uint32_t)(tick - background_last_sample_ms) >= 200);
            }
            ++background_samples;
        }
        background_last_sample_ms = tick;
    }
    if (sport_active != 0)
    {
        ++sport_samples;
    }
    if (sleep_test != 0)
    {
        assert(sleep_bus_parked == 0 && sleep_imu == (mock_imu_period == 200));
        sleep_imu_sample_ms = tick;
        if (sleep_recovery_steps != 0 && sleep_recovery_sample == 0)
        {
            assert(sleep_recovery_steps == 16U && (uint32_t)(tick - sleep_recovery_last_ms) >= 200U);
            sleep_recovery_sample = 1;
        }
        if (sleep_heart_mode != 0 && mock_imu_period == 200)
        {
            if (sleep_pose_seen != 0 && sleep_recovery_test == 0)
            {
                assert(sensor_elapsed_ms - sleep_pose_ms >= 200);
                assert(sensor_elapsed_ms - sleep_pose_ms <= 300);
            }
            sleep_pose_ms = sensor_elapsed_ms;
            sleep_pose_seen = 1;
            ++sleep_pose_samples;
        }
        else
        {
            sleep_pose_seen = 0;
        }
    }
    if (complete_cal != 0 && mock_cal_state == MPU6050_CAL_WAIT_STILL)
    {
        mock_cal_state = MPU6050_CAL_DONE;
    }
    else if (fail_cal != 0)
    {
        mock_cal_state = MPU6050_CAL_FAILED;
    }
}
uint32_t app_imu_set_sample_period(uint32_t period_ms)
{
    assert(period_ms == 10U || period_ms == 20U || period_ms == 40U || period_ms == 200);
    if (mock_imu_period != period_ms)
    {
        if (background_test != 0 && (mock_imu_period == 200 || period_ms == 200))
        {
            background_sleeping = period_ms == 200;
            if (background_sleeping != 0)
            {
                ++background_sleeps;
            }
            else
            {
                ++background_wakes;
                background_wake_ms = tick;
            }
        }
        if (sleep_test != 0)
        {
            assert(sleep_bus_parked == 0);
            sleep_imu = period_ms == 200;
            if (sleep_imu != 0 && sleep_first_seen == 0)
            {
                sleep_first_seen = 1;
                sleep_first_ms = sensor_elapsed_ms;
            }
        }
        mock_imu_period = period_ms;
    }
    if (power_profile != 0)
    {
        uint32_t expected = 200;
        if (restored_mode == 0 || (s_power_state & APP_POWER_MOTION) != 0)
        {
            expected = 20U;
        }
        if (heart_enabled != 0 && restored_mode != 0)
        {
            expected = 10U;
        }
        assert(period_ms == expected);
    }
    return period_ms;
}
void app_imu_set_accel_reference(uint8_t enabled)
{
    assert(enabled <= 1);
    reference_enabled = enabled;
}
void app_heart_get_snapshot(app_heart_struct *heart)
{
    memset(heart, 0, sizeof(*heart));
    heart->enabled = heart_enabled;
}
void App_Baro_Service(void)
{
    assert(hardware_count == 5);
    if (environment_test != 0)
    {
        assert(environment_bus_parked == 0);
        assert(sensor_elapsed_ms - environment_baro_window_ms >= 1600);
        assert(sensor_elapsed_ms - environment_baro_window_ms < 1700);
        assert(sensor_elapsed_ms >= environment_reads * 300000 + 1600);
        assert(sensor_elapsed_ms < environment_reads * 300000 + 1900);
        ++environment_reads;
        return;
    }
    if (sensor_timing.mode != 0)
    {
        uint32_t now = tick - sensor_timing.start_ms;
        uint32_t minimum = 300000;
        if (baro_calls == 0)
        {
            minimum = 1600;
        }
        assert(now - sensor_timing.baro_ms >= minimum && now - sensor_timing.baro_ms <= minimum + 200);
        sensor_timing.baro_ms = now;
        ++baro_calls;
        return;
    }
    if (background_test != 0)
    {
        if (baro_calls == 0)
        {
            assert(sensor_elapsed_ms >= 1600 && sensor_elapsed_ms < 1640);
        }
        else
        {
            assert(sensor_elapsed_ms - background_baro_ms >= 300000);
            assert(sensor_elapsed_ms - background_baro_ms <= 300040);
        }
        background_baro_ms = sensor_elapsed_ms;
        ++baro_calls;
        return;
    }
    if (sport_test != 0)
    {
        assert(sport_active == 0 && sport_baro_sleeping == 0);
        ++baro_calls;
        return;
    }
    if (sleep_test != 0)
    {
        assert(sleep_bus_parked == 0 && sleep_baro == 0);
        if (sleep_heart_mode != 0)
        {
            assert(sensor_elapsed_ms - sleep_environment_ms >= 1600U);
            assert(sensor_elapsed_ms - sleep_environment_ms < 1700);
            assert(sensor_elapsed_ms >= baro_calls * 300000 + 1600);
            assert(sensor_elapsed_ms < baro_calls * 300000 + 1900);
        }
        ++baro_calls;
        return;
    }
    assert(low_rate_mask == 0);
    low_rate_mask |= 2U;
    ++baro_calls;
    assert(sensor_elapsed_ms >= 1600 + (baro_calls - 1) * 300000);
    assert(sensor_elapsed_ms < 1640 + (baro_calls - 1) * 300000);
}
uint8_t app_heart_service(void)
{
    uint8_t bus_used = 0;
    if (sensor_timing.mode != 0)
    {
        sensor_timing.loop_ms = tick;
        if (sensor_timing.started == 0)
        {
            sensor_timing.start_ms = sensor_timing.ppg_ms = tick;
            sensor_timing.started = 1;
        }
    }
    assert(hardware_count == 5 && !rtc_done && !ble_done);
    if (sensor_timing.mode != 0 && (uint32_t)(tick - sensor_timing.ppg_ms) >= 30)
    {
        sensor_timing.ppg_ms = tick;
        tick += 3;
        bus_used = 1;
        ++sensor_timing.ppg_reads;
    }
    if (sleep_test != 0)
    {
        assert(sleep_bus_parked == 0);
        if (heart_enabled != 0 && (uint32_t)(tick - sleep_window_ms) > 100U)
        {
            /* 100ms预热边界先读PPG，紧接着产生首个MPU参考；后续各拍保持新鲜。 */
            assert(sleep_imu == 0 && (uint32_t)(tick - sleep_imu_sample_ms) <= 50U);
        }
        if (sleep_heart_mode != 0 && heart_enabled == 0 && (uint32_t)(tick - sleep_cycle_ms) >= 600000U)
        {
            sleep_cycle_ms = sleep_window_ms = tick;
            heart_enabled = 1;
            ++sleep_heart_starts;
        }
        if (heart_enabled != 0 && (uint32_t)(tick - sleep_window_ms) >= sleep_heart_duration_ms)
        {
            heart_enabled = 0;
        }
    }
    return bus_used;
}
void app_aht21_service(void)
{
    assert(hardware_count == 5 && !rtc_done && !ble_done);
    if (environment_test != 0)
    {
        assert(environment_bus_parked == 0);
        if (app_task_is_screen_off() != 0)
        {
            assert(aht_period_ms == 60000);
            if (environment_last_aht_ms == UINT32_MAX || sensor_elapsed_ms - environment_last_aht_ms > 1000)
            {
                ++environment_mode_windows;
                ++environment_aht_windows;
                assert(sensor_elapsed_ms - environment_mode_ms >= environment_mode_windows * 60000);
                assert(sensor_elapsed_ms - environment_mode_ms < environment_mode_windows * 60000 + 200);
                environment_aht_window_ms = sensor_elapsed_ms;
            }
            else
            {
                assert(sensor_elapsed_ms - environment_last_aht_ms <= 200);
            }
            assert(sensor_elapsed_ms - environment_aht_window_ms <= 1600);
        }
        else
        {
            assert(aht_period_ms == 1000);
        }
        environment_last_aht_ms = sensor_elapsed_ms;
        return;
    }
    if (sport_test != 0)
    {
        return;
    }
    if (sleep_test != 0)
    {
        assert(sleep_bus_parked == 0);
        return;
    }
    assert(low_rate_mask == 0);
    low_rate_mask |= 4U;
}
void app_touch_init(void)
{
    assert(frame && sensor_ran && hardware_count == 5);
    ++touch_inits;
}
void app_touch_service(void)
{
    assert(touch_inits == 1);
    ++touch_samples;
    tick += touch_work_ms;
}
uint8_t app_touch_is_sleeping(void)
{
    uint8_t sleeping = touch_sleep_test != 0 && (s_power_state & APP_POWER_SCREEN_OFF) != 0;
    if (sleeping != 0 && touch_sleep_test == 2U)
    {
        /* PA4通知先于ulTaskNotifyTake到达，也不能丢失。 */
        app_task_set_power_state(0, 0, 0, 0);
    }
    return sleeping;
}
int main(int argc, char **argv)
{
    int result;
    if (argc > 1 && strcmp(argv[1], "reference_recovery") == 0)
    {
        reference_recovery_test = restored_mode = heart_enabled = displayed = 1;
        tick = 1000;
        app_watchdog_begin();
        frame = 1;
        if (setjmp(sensor_stop) == 0)
        {
            app_sensor_task(NULL);
        }
        puts("PASS: heart acceleration reference switches to full attitude only after MPU recovery is ready and the next 100ms settle completes.");
        return 0;
    }
    if (argc > 1 && strncmp(argv[1], "environment_policy", 18) == 0)
    {
        environment_test = restored_mode = displayed = 1;
        tick = 1000;
        if (strcmp(argv[1], "environment_policy_wrap") == 0)
        {
            tick = UINT32_MAX - 10000;
        }
        environment_sleep_retry_test = strcmp(argv[1], "environment_policy_sleep_retry") == 0;
        app_watchdog_begin();
        frame = 1;
        if (setjmp(sensor_stop) == 0)
        {
            app_sensor_task(NULL);
        }
        puts("PASS: independent 5min barometer and screen-off 1min/screen-on 1s AHT periods survive two screen transitions and tick wrap.");
        if (environment_sleep_retry_test != 0)
        {
            puts("PASS: two barometer standby NACKs retain the bus, retry only standby, stop on ACK and preserve reads/history/5min deadlines.");
        }
        return 0;
    }
    if (argc > 1 && strncmp(argv[1], "sensor_timing", 13) == 0)
    {
        sensor_timing.mode = 1;
        if (strcmp(argv[1], "sensor_timing_overload") == 0)
        {
            sensor_timing.mode = 2;
        }
        else if (strcmp(argv[1], "sensor_timing_wrap") == 0)
        {
            sensor_timing.mode = 3;
            tick = UINT32_MAX - 1000;
        }
        else if (strcmp(argv[1], "sensor_timing_exact") == 0)
        {
            sensor_timing.mode = 4;
        }
        restored_mode = heart_enabled = displayed = 1;
        app_task_set_power_state(0, 0, 0, 0);
        app_watchdog_begin();
        frame = 1;
        if (setjmp(sensor_stop) == 0)
        {
            app_sensor_task(NULL);
        }
        return 0;
    }
    if (argc > 1 && strncmp(argv[1], "background_", 11) == 0)
    {
        background_test = 1;
        restored_mode = 1;
        tick = 1000;
        if (strcmp(argv[1], "background_wrap") == 0)
        {
            tick = UINT32_MAX - 6000;
        }
        if (strcmp(argv[1], "background_calibration") == 0)
        {
            background_test = 2;
            complete_cal = 1;
        }
        background_start_ms = tick;
        displayed = 1;
        app_watchdog_begin();
        frame = 1;
        app_task_set_power_state(0, 0, 0, 0);
        if (setjmp(sensor_stop) == 0)
        {
            app_sensor_task(NULL);
        }
        puts("PASS: background accelerometer runs at 5Hz, Motion/calibration restores 50Hz after 100ms, and both transitions retain watchdog progress across wrap.");
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "lvgl_wake") == 0)
    {
        app_task_wake_from_isr();
        assert(irq_notifications == 0 && irq_yields == 0);
        lvgl_wait_test = 1;
        app_task_set_power_state(1, 0, 1, 0);
        if (setjmp(stop) == 0)
        {
            App_Task_Create();
            App_Task_Start();
            assert(0);
        }
        assert(lvgl_waits == 3 && irq_notifications == 2 && irq_yields == 2 && lvgl_notifications == 0);
        assert(wake_polls == 3 && wake_key_active == 0 && watchdog_feeds > 30);
        puts("PASS: PA4 IRQ ignores a missing task, preserves an early notification, interrupts a 200ms sleep, and keeps 20ms debounce polls and watchdog progress.");
        return 0;
    }
    if (argc > 1 && strncmp(argv[1], "touch_sleep", 11) == 0)
    {
        touch_sleep_test = 1;
        if (strcmp(argv[1], "touch_sleep_early") == 0)
        {
            touch_sleep_test = 2;
        }
        frame = sensor_ran = 1;
        hardware_count = 5;
        s_touch_task_handle = &touch_inits;
        app_task_set_power_state(1, 0, 1, 0);
        if (setjmp(touch_stop) == 0)
        {
            app_touch_task(NULL);
        }
        assert(touch_blocks == 1U && touch_samples == 2U && touch_notifications == 0);
        puts("PASS: touch task blocks indefinitely after chip sleep; PA4 or a pre-arriving notification resumes normal polling without a catch-up loop.");
        return 0;
    }
    if (argc > 1 && strncmp(argv[1], "sport_", 6) == 0)
    {
        sport_test = 1;
        restored_mode = 1;
        if (strcmp(argv[1], "sport_save") == 0)
        {
            sport_test = 2;
            restored_mode = 0;
            complete_cal = 1;
        }
        if (strcmp(argv[1], "sport_uptime") == 0)
        {
            sport_test = 3;
        }
        if (strcmp(argv[1], "sport_weather") == 0)
        {
            static const char initial[] = "WEATHER,3,263,80\n";
            sport_test = 3;
            weather_task_test = 1;
            restored_mode = 0;
            complete_cal = 1;
            app_weather_receive((const uint8_t *)initial, sizeof(initial) - 1, tick);
            app_weather_store_sync();
        }
        displayed = 1;
        app_watchdog_begin();
        frame = 1;
        app_task_set_power_state(0, 1, 0, 1);
        if (setjmp(sensor_stop) == 0)
        {
            app_sensor_task(NULL);
        }
        assert(battery_log_calls > 100U);
        puts("PASS: sport 100Hz continues through screen-off, barometer/calibration storage stay idle, 10min uptime saves serialize and normal sampling resumes.");
        return 0;
    }
    if (argc > 1 && strncmp(argv[1], "sleep_", 6) == 0)
    {
        sleep_test = 1;
        s_lvgl_task_handle = &lvgl_notifications;
        restored_mode = 1;
        tick = 1000U;
        if (strncmp(argv[1], "sleep_recovery", 14) == 0)
        {
            sleep_recovery_test = 1;
            if (strcmp(argv[1], "sleep_recovery_wrap") == 0)
            {
                tick = UINT32_MAX - 5500U;
            }
        }
        if (strcmp(argv[1], "sleep_wrap") == 0)
        {
            tick = UINT32_MAX - 1000U;
        }
        if (strcmp(argv[1], "sleep_calibration") == 0)
        {
            restored_mode = 0;
            complete_cal = 1;
        }
        if (strcmp(argv[1], "sleep_heart_early") == 0)
        {
            sleep_heart_duration_ms = 600U;
        }
        if (strcmp(argv[1], "sleep_heart_offset") == 0)
        {
            sleep_heart_advance_ms = 2000U;
        }
        displayed = 1;
        app_watchdog_begin();
        frame = 1;
        app_task_set_power_state(1, 0, 1, 0);
        if (setjmp(sensor_stop) == 0)
        {
            app_sensor_task(NULL);
        }
        assert(sleep_first_seen != 0);
        assert(battery_log_calls == sensor_rounds && battery_log_calls > 60);
        if (complete_cal != 0)
        {
            assert(cal_begins == 1U && saves == 1U && sleep_first_ms >= 20U);
        }
        if (sleep_recovery_test != 0)
        {
            puts("PASS: sleep defers IMU/bus parking through recovery and warmup; sampling, battery and watchdog continue across tick wrap.");
        }
        else
        {
            puts("PASS: 20-minute sleep timeline, 5Hz accelerometer, 5min barometer, 10min heart/uptime, bus parking, battery/watchdog, calibration and wake restore.");
        }
        return 0;
    }
    if (argc > 1 && (strcmp(argv[1], "watchdog_progress") == 0 || strcmp(argv[1], "watchdog_wrap") == 0))
    {
        test_watchdog_progress((uint8_t)(strcmp(argv[1], "watchdog_wrap") == 0));
        return 0;
    }
    if (argc > 1 && (strcmp(argv[1], "watchdog_startup") == 0 || strcmp(argv[1], "watchdog_startup_wrap") == 0))
    {
        test_watchdog_startup((uint8_t)(strcmp(argv[1], "watchdog_startup_wrap") == 0));
        return 0;
    }
    if (argc > 1)
    {
        bluetooth_resume_test = strcmp(argv[1], "bluetooth_resume") == 0;
        baro_cadence = strcmp(argv[1], "baro_cadence") == 0;
        if (baro_cadence != 0)
        {
            app_task_set_power_state(0, 0, 0, 0);
        }
        if (strcmp(argv[1], "touch_overrun") == 0)
        {
            touch_work_ms = 12U;
        }
        if (strcmp(argv[1], "runtime_slow") == 0 || strcmp(argv[1], "runtime_sparse") == 0)
        {
            runtime_render = 1;
        }
        else if (strcmp(argv[1], "runtime_sensor_stalled") == 0)
        {
            runtime_render = 2;
        }
        else if (strcmp(argv[1], "runtime_deadline") == 0)
        {
            runtime_render = 3;
        }
        if (strcmp(argv[1], "runtime_sparse") == 0)
        {
            runtime_chunk_ms = 500U;
            runtime_chunks = 6U;
        }
        first_frame_stall = strcmp(argv[1], "first_frame_stall") == 0;
        first_frame_slow = strcmp(argv[1], "first_frame_slow") == 0 || strcmp(argv[1], "first_frame_sparse") == 0;
        if (strcmp(argv[1], "first_frame_sparse") == 0)
        {
            first_frame_chunk_ms = 500U;
            first_frame_chunks = 6U;
        }
        first_frame_deadline = strcmp(argv[1], "first_frame_deadline") == 0;
        fail_devices = strcmp(argv[1], "devices_failed") == 0;
        power_profile = strncmp(argv[1], "power_", 6) == 0;
        heart_enabled = strcmp(argv[1], "power_heart") == 0 || strcmp(argv[1], "power_heart_calibration") == 0;
        restored_mode = strcmp(argv[1], "restored") == 0 || strcmp(argv[1], "manual_cal") == 0 ||
                        (power_profile != 0 && strcmp(argv[1], "power_calibration") != 0 && strcmp(argv[1], "power_heart_calibration") != 0);
        if (power_profile != 0)
        {
            app_task_set_power_state(strcmp(argv[1], "power_idle") == 0, strcmp(argv[1], "power_motion") == 0, 0, 0);
        }
        load_error = strcmp(argv[1], "load_error") == 0;
        save_error = strcmp(argv[1], "save_error") == 0;
        manual_request = strcmp(argv[1], "manual_cal") == 0;
        complete_cal = strcmp(argv[1], "cal_success") == 0 || save_error != 0 || manual_request != 0;
        fail_cal = strcmp(argv[1], "cal_failed") == 0;
        if (strcmp(argv[1], "display_alloc_failed") == 0)
        {
            fail_task = 1;
        }
        else if (strcmp(argv[1], "sensor_alloc_failed") == 0)
        {
            fail_task = 2;
        }
        else if (strcmp(argv[1], "touch_alloc_failed") == 0)
        {
            fail_task = 3;
        }
    }
    result = setjmp(stop);
    if (result == 0)
    {
        App_Task_Create();
        App_Task_Start();
        assert(0);
    }
    if (baro_cadence != 0)
    {
        assert(result == 5 && sensor_rounds == 150 && baro_calls == 1);
        assert(recovery_steps == 381 && baro_recovery_steps == 81 && app_sensor_watchdog_progress == 150);
        puts("Sensor cadence: initial 1.6s barometer window reads once, then sleeps without recovery polling while sensor/watchdog continue: PASS");
        return 0;
    }
    if (bluetooth_resume_test != 0)
    {
        assert(result == 1 && bluetooth_resume_phase == 1 && ble_done != 0);
        assert(milestones[APP_BOOT_BT_END_MS] == milestones[APP_BOOT_FIRST_FRAME_MS] + 1700U);
        puts("PASS: LVGL continues a second Bluetooth startup after the initial boot completed, without rewriting boot milestones.");
        return 0;
    }
    if (fail_task != 0)
    {
        assert(watchdog_inits == 1 && watchdog_enables == (fail_task >= 2) && watchdog_feeds == 0);
        assert(result == 2);
        assert(frame == (fail_task >= 2));
        puts("Startup allocation failure: PASS");
        return 0;
    }
    if (touch_work_ms != 0)
    {
        assert(result == 1 && touch_inits == 1 && touch_samples == 1);
        puts("Touch task: priority 1, 1KB stack, post-first-frame startup, 5ms cadence and no catch-up after a slow read: PASS");
        return 0;
    }
    if (runtime_render != 0)
    {
        assert(result == 5 && frame && watchdog_enables == 1);
        puts("Watchdog: render chunks yield once for lower-priority sensor progress; feeds stay below 900ms; stalls and render deadline stop feeding: PASS");
        return 0;
    }
    if (first_frame_stall != 0)
    {
        assert(result == 3 && watchdog_enables == 1 && watchdog_feeds == 0 && !frame);
        puts("Startup: stalled first-frame refresh leaves watchdog enabled and never feeds: PASS");
        return 0;
    }
    if (first_frame_deadline != 0)
    {
        assert(result == 4 && watchdog_enables == 1 && !frame);
        puts("Startup: first-frame chunk progress cannot extend the 5s deadline indefinitely: PASS");
        return 0;
    }
    if (power_profile != 0)
    {
        assert(result == 1 && frame && sensor_rounds == 2U && touch_samples == 1U);
        assert(service_count > 200U && watchdog_enables == 1U);
        assert(milestones[APP_BOOT_RTC_END_MS] >= milestones[APP_BOOT_FIRST_FRAME_MS] + 5000U);
        assert(milestones[APP_BOOT_RTC_END_MS] <= milestones[APP_BOOT_FIRST_FRAME_MS] + 5020U);
        printf("Power policy %s: dynamic sensor period, idle touch/display period and continuing RTC startup: PASS\n", argv[1]);
        return 0;
    }
    assert(result == 1 && frame && service_count > 900);
    assert(watchdog_inits == 1 && watchdog_enables == 1);
    assert(hardware_count == 5 && battery_samples == 1 && sensor_ran);
    assert(touch_inits == 1 && touch_samples == 1 && task_count == 3);
    if (first_frame_slow != 0)
    {
        assert(watchdog_feeds == first_frame_chunks + 1U);
        assert(milestones[APP_BOOT_FIRST_FRAME_MS] == 650U + first_frame_chunks * first_frame_chunk_ms);
        assert(milestones[APP_BOOT_SAMPLING_BEGIN_MS] == milestones[APP_BOOT_FIRST_FRAME_MS] + 5U);
        assert(app_watchdog.started_ms == milestones[APP_BOOT_FIRST_FRAME_MS] && display_progress_callback != NULL);
        puts("Startup: 1.5s/3s first frames maintain falling-edge intervals below 900ms; startup phases have separate deadlines: PASS");
    }
    else
    {
        assert(watchdog_feeds == 1);
        assert(milestones[APP_BOOT_FIRST_FRAME_MS] == 650);
        assert(milestones[APP_BOOT_SAMPLING_BEGIN_MS] == 655);
    }
    assert(milestones[APP_BOOT_RTC_END_MS] == milestones[APP_BOOT_FIRST_FRAME_MS] + 5000U);
    assert(milestones[APP_BOOT_BT_END_MS] == milestones[APP_BOOT_FIRST_FRAME_MS] + 1700U);
    if (complete_cal != 0)
    {
        assert(milestones[APP_BOOT_CAL_STATE] == MPU6050_CAL_DONE && saves == 1 && cal_begins == 1);
        if (save_error != 0)
        {
            assert(milestones[APP_BOOT_CAL_RESULT] == 2);
        }
        else
        {
            assert(milestones[APP_BOOT_CAL_RESULT] == 0);
        }
    }
    else if (restored_mode != 0)
    {
        assert(milestones[APP_BOOT_CAL_STATE] == MPU6050_CAL_RESTORED && restores == 1 && cal_begins == 0 && saves == 0);
    }
    else if (fail_cal != 0)
    {
        assert(milestones[APP_BOOT_CAL_STATE] == MPU6050_CAL_FAILED && saves == 0 && milestones[APP_BOOT_CAL_RESULT] == 1);
    }
    else
    {
        assert(milestones[APP_BOOT_CAL_STATE] == MPU6050_CAL_WAIT_STILL && cal_begins == 1 && saves == 0);
    }
    if (fail_devices)
    {
        assert(milestones[APP_BOOT_RTC_RESULT] == HAL_TIMEOUT && milestones[APP_BOOT_BT_RESULT] == HAL_TIMEOUT);
    }
    puts("Startup: first frame, sensor sampling and UI service proceed during RTC/Bluetooth waits: PASS");
    return 0;
}

uint8_t my_mpu6050_recover_step(uint32_t now_ms)
{
    assert(now_ms == tick);
    ++recovery_steps;
    ++background_recovery_steps;
    if (reference_recovery_test == 2)
    {
        if (reference_recovery_steps < 5)
        {
            ++reference_recovery_steps;
            ++tick;
            return 2;
        }
        if (reference_ready_ms == 0)
        {
            reference_ready_ms = now_ms;
        }
    }
    if (sleep_recovery_test != 0 && sensor_elapsed_ms >= 5000U && sleep_recovery_steps < 16U)
    {
        assert(sleep_bus_parked == 0);
        ++sleep_recovery_steps;
        sleep_recovery_last_ms = now_ms;
        ++tick; /* I2C事务可能跨过tick，任务不能用较早的now_ms减去事务后的时间戳。 */
        return 2;
    }
    if (fail_devices != 0)
    {
        return 2; /* Recovering devices must not stop sensor progress. */
    }
    return 0;
}
uint8_t spl006_recover_step(uint32_t now_ms)
{
    assert(sport_active == 0);
    if (sleep_test != 0)
    {
        assert(sleep_bus_parked == 0 && sleep_baro == 0);
    }
    assert(now_ms == tick);
    ++baro_recovery_steps;
    ++recovery_steps;
    if (fail_devices != 0)
    {
        return 2; /* Recovering devices must not stop sensor progress. */
    }
    return 0;
}
uint8_t lsm303dlhc_mag_recover_step(uint32_t now_ms)
{
    ++mag_recovery_steps;
    assert(now_ms == tick);
    ++recovery_steps;
    if (fail_devices != 0)
    {
        return 2; /* Recovering devices must not stop sensor progress. */
    }
    return 0;
}
