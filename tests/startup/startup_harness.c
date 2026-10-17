/* 执行真实任务入口，模拟等待期间sensor获得调度，验证首帧和采集不等RTC/蓝牙。 */
#define app_diag_boot_set startup_boot_set
#define app_diag_take_imu_calibration startup_take_imu_calibration
#include "../../application/app_task.c"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static TaskFunction_t entries[2];
static unsigned int task_count, fail_task;
static uint8_t fail_devices, displayed, frame, rtc_done, ble_done, sensor_ran;
static unsigned int hardware_count, service_count, battery_samples;
static unsigned int cal_begins, restores, saves, sensor_rounds, recovery_steps;
static uint8_t restored_mode, load_error, save_error, manual_request, complete_cal, fail_cal;
static mpu6050_cal_state mock_cal_state;
static const mpu6050_offset_struct stored_offset = {{20, -10, 30}, {66, -33, 99}};

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
app_imu_store_result app_imu_store_save(const mpu6050_offset_struct *offset)
{
    assert(memcmp(offset, &stored_offset, sizeof(*offset)) == 0);
    ++saves;
    if (save_error != 0)
    {
        return APP_IMU_STORE_ERROR;
    }
    return APP_IMU_STORE_OK;
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
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack, void *arg, UBaseType_t priority, TaskHandle_t *handle)
{
    assert(arg == NULL && handle == NULL && task_count < 2);
    if (task_count == 0)
    {
        assert(strcmp(name, "lvgl") == 0 && stack == 1024 && priority == 3);
        assert(!displayed && hardware_count == 0);
    }
    else
    {
        assert(strcmp(name, "sensor") == 0 && stack == 512 && priority == 2);
        assert(frame && !rtc_done && !ble_done && hardware_count == 0);
    }
    if (fail_task == task_count + 1)
    {
        return 0;
    }
    entries[task_count++] = entry;
    return pdPASS;
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
void vTaskDelay(TickType_t ticks)
{
    assert((ticks == 5 || ticks == 1) && task_count == 2);
    tick += ticks;
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
    }
    else if (runtime_render != 0 && runtime_sensor_stalled == 0)
    {
        ++app_sensor_watchdog_progress;
    }
    if (rtc_done && ble_done)
    {
        longjmp(stop, 1);
    }
}
void vTaskDelayUntil(TickType_t *last, TickType_t ticks)
{
    assert(last != NULL && ticks == MPU6050_SAMPLE_PERIOD_MS);
    assert(hardware_count == 5 && battery_samples == sensor_rounds + 2);
    assert(!rtc_done && !ble_done);
    ++sensor_rounds;
    assert(recovery_steps == sensor_rounds * 3U);
    assert(app_sensor_watchdog_progress == sensor_rounds);
    if (sensor_rounds < 2)
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
    unsigned int i, feeds;
    assert(frame);
    ++service_count;
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
            if (display_progress_callback != NULL)
            {
                display_progress_callback();
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
    assert(frame && task_count == 2);
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
    assert(frame && task_count == 2);
    ble_begin_tick = tick;
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
void app_battery_init(void)
{
    assert(hardware_count == 5);
}
void app_battery_service(void)
{
    ++battery_samples;
}
void my_mpu6050_zero_offset_begin(uint32_t now)
{
    assert(now == tick && battery_samples == 1);
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
}
void App_IMU_Service(void)
{
    assert(hardware_count == 5 && !rtc_done && !ble_done);
    if (complete_cal != 0 && mock_cal_state == MPU6050_CAL_WAIT_STILL)
    {
        mock_cal_state = MPU6050_CAL_DONE;
    }
    else if (fail_cal != 0)
    {
        mock_cal_state = MPU6050_CAL_FAILED;
    }
}
void App_Baro_Service(void)
{
    assert(hardware_count == 5);
}
void app_heart_service(void)
{
    assert(hardware_count == 5 && !rtc_done && !ble_done);
}
void app_aht21_service(void)
{
    assert(hardware_count == 5 && !rtc_done && !ble_done);
}
int main(int argc, char **argv)
{
    int result;
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
        restored_mode = strcmp(argv[1], "restored") == 0 || strcmp(argv[1], "manual_cal") == 0;
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
    }
    result = setjmp(stop);
    if (result == 0)
    {
        App_Task_Create();
        App_Task_Start();
        assert(0);
    }
    if (fail_task != 0)
    {
        assert(watchdog_inits == 1 && watchdog_enables == (fail_task == 2) && watchdog_feeds == 0);
        assert(result == 2);
        assert(frame == (fail_task == 2));
        puts("Startup allocation failure: PASS");
        return 0;
    }
    if (runtime_render != 0)
    {
        assert(result == 5 && frame && watchdog_enables == 1);
        puts("Watchdog: post-first-frame rendering yields to sensor and feeds below 900ms; sensor stalls and endless rendering stop feeding: PASS");
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
    assert(result == 1 && frame && service_count > 900);
    assert(watchdog_inits == 1 && watchdog_enables == 1);
    assert(hardware_count == 5 && battery_samples == 3 && sensor_ran);
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
    if (fail_devices != 0)
    {
        return 2; /* Recovering devices must not stop sensor progress. */
    }
    return 0;
}
uint8_t spl006_recover_step(uint32_t now_ms)
{
    assert(now_ms == tick);
    ++recovery_steps;
    if (fail_devices != 0)
    {
        return 2; /* Recovering devices must not stop sensor progress. */
    }
    return 0;
}
uint8_t lsm303dlhc_mag_recover_step(uint32_t now_ms)
{
    assert(now_ms == tick);
    ++recovery_steps;
    if (fail_devices != 0)
    {
        return 2; /* Recovering devices must not stop sensor progress. */
    }
    return 0;
}
