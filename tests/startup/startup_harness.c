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
static unsigned int cal_begins, restores, saves, sensor_rounds;
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
static jmp_buf stop, sensor_stop;
static unsigned int watchdog_inits, watchdog_enables, watchdog_feeds;
static uint32_t watchdog_enable_delay;

void tps3823_init(void)
{
    ++watchdog_inits;
}
void tps3823_enable(void)
{
    assert(frame && app_sensor_watchdog_progress != 0);
    ++watchdog_enables;
    tick += watchdog_enable_delay;
}
void tps3823_feed(void)
{
    assert(watchdog_enables == 1);
    ++watchdog_feeds;
}

static void test_watchdog_progress(uint8_t wrap)
{
    frame = 1;
    watchdog_enable_delay = 302;
    tick = 1000;
    if (wrap != 0)
    {
        tick = UINT32_MAX - 350U;
    }
    app_watchdog_service();
    assert(watchdog_enables == 0 && watchdog_feeds == 0);
    ++app_sensor_watchdog_progress;
    app_watchdog_service();
    assert(watchdog_enables == 1 && watchdog_feeds == 0);
    tick += 99;
    ++app_sensor_watchdog_progress;
    app_watchdog_service();
    assert(watchdog_feeds == 0);
    ++tick;
    app_watchdog_service();
    assert(watchdog_feeds == 1);
    /* sensor 卡住时，即使 LVGL 继续执行，也不再喂狗。 */
    tick += 2000;
    app_watchdog_service();
    assert(watchdog_feeds == 1);
    ++app_sensor_watchdog_progress;
    app_watchdog_service();
    assert(watchdog_feeds == 2);
    /* LVGL 卡住时，sensor 的计数更新本身不会触发喂狗。 */
    tick += 2000;
    ++app_sensor_watchdog_progress;
    assert(watchdog_feeds == 2);
    puts("Watchdog: first-round gate, 100ms cadence, task stalls and tick wrap: PASS");
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
    assert(ticks == 5 && task_count == 2);
    tick += ticks;
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
    assert(display == NULL && displayed && !rtc_done && !ble_done);
    frame = 1;
}
void App_Display_Service(void)
{
    assert(frame);
    ++service_count;
}
void watch_rtc_init_begin(void)
{
    assert(frame && task_count == 2);
}
uint8_t watch_rtc_init_is_pending(void)
{
    return rtc_done == 0;
}
HAL_StatusTypeDef watch_rtc_init_service(void)
{
    if (tick < 5650)
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
}
uint8_t kt6368a_init_is_pending(void)
{
    return ble_done == 0;
}
HAL_StatusTypeDef kt6368a_init_service(void)
{
    if (tick < 2350)
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
    if (argc > 1)
    {
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
        assert(watchdog_inits == 1 && watchdog_enables == 0 && watchdog_feeds == 0);
        assert(result == 2);
        assert(frame == (fail_task == 2));
        puts("Startup allocation failure: PASS");
        return 0;
    }
    assert(result == 1 && frame && service_count > 900);
    assert(watchdog_inits == 1 && watchdog_enables == 1 && watchdog_feeds == 0);
    assert(hardware_count == 5 && battery_samples == 3 && sensor_ran);
    assert(milestones[APP_BOOT_FIRST_FRAME_MS] == 650);
    assert(milestones[APP_BOOT_SAMPLING_BEGIN_MS] == 655);
    assert(milestones[APP_BOOT_RTC_END_MS] == 5650);
    assert(milestones[APP_BOOT_BT_END_MS] == 2350);
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
