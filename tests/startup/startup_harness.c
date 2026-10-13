/* 执行真实任务入口；验证慢初始化/器件失败前已提交首帧，且UI与AT不交叉收发。 */
#include "../../application/app_task.c"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static TaskFunction_t entries[2];
static unsigned int task_count, fail_task;
static uint8_t fail_devices, displayed, frame, rtc_done, ble_done;
static unsigned int hardware_count, service_count, battery_samples;
static TickType_t tick;
static jmp_buf stop;

BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack, void *arg, UBaseType_t priority, TaskHandle_t *handle)
{
    assert(arg == NULL && handle == NULL && task_count < 2);
    if (task_count == 0)
    {
        assert(strcmp(name, "lvgl") == 0 && stack == 1024 && priority == 3);
        assert(!displayed && !rtc_done && !ble_done && hardware_count == 0);
    }
    else
    {
        assert(strcmp(name, "sensor") == 0 && stack == 512 && priority == 2);
        assert(frame && rtc_done && ble_done && hardware_count == 0);
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
    assert(ticks == 5 && service_count == 1 && task_count == 2);
    longjmp(stop, 1);
}

void vTaskDelayUntil(TickType_t *last, TickType_t ticks)
{
    assert(last != NULL && ticks == MPU6050_SAMPLE_PERIOD_MS);
    assert(hardware_count == 5 && battery_samples == 2);
    longjmp(stop, 1);
}

void App_Display_Init(void)
{
    assert(task_count == 1 && !rtc_done && !ble_done && hardware_count == 0);
    displayed = 1;
}

void lv_refr_now(void *display)
{
    assert(display == NULL && displayed && !rtc_done && !ble_done);
    frame = 1;
}

void App_Display_Service(void)
{
    assert(frame && rtc_done && ble_done);
    ++service_count;
}

HAL_StatusTypeDef watch_rtc_init(void)
{
    assert(frame && !ble_done && service_count == 0);
    tick += 5000; /* LSE启动失败的等待不能推迟首帧。 */
    rtc_done = 1;
    return fail_devices ? HAL_TIMEOUT : HAL_OK;
}

HAL_StatusTypeDef kt6368a_init(void)
{
    assert(frame && rtc_done && service_count == 0);
    tick += 1700;
    ble_done = 1;
    return fail_devices ? HAL_TIMEOUT : HAL_OK;
}

void my_i2c_init(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_SENSOR && frame && ble_done && task_count == 2);
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
    return fail_devices ? EM7028_ERROR : EM7028_OK;
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

uint8_t my_mpu6050_zero_offset(void)
{
    assert(battery_samples == 1);
    return fail_devices;
}

void app_aht21_init(void)
{
    assert(battery_samples == 1);
}

void App_IMU_Service(void)
{
    assert(hardware_count == 5);
}

void App_Baro_Service(void)
{
    assert(hardware_count == 5);
}

void app_heart_service(void)
{
    assert(hardware_count == 5);
}

void app_aht21_service(void)
{
    assert(hardware_count == 5);
}

int main(int argc, char **argv)
{
    int result;
    if (argc > 1)
    {
        fail_devices = strcmp(argv[1], "devices_failed") == 0;
        fail_task = strcmp(argv[1], "display_alloc_failed") == 0 ? 1 : (strcmp(argv[1], "sensor_alloc_failed") == 0 ? 2 : 0);
    }
    result = setjmp(stop);
    if (result == 0)
    {
        App_Task_Create();
        assert(task_count == 1 && !displayed && !rtc_done && !ble_done);
        App_Task_Start();
        assert(0);
    }
    if (fail_task != 0)
    {
        assert(result == 2);
        assert(frame == (fail_task == 2));
        puts("Startup allocation failure: PASS");
        return 0;
    }
    assert(result == 1 && frame && service_count == 1 && hardware_count == 0);
    if (setjmp(stop) == 0)
    {
        entries[1](NULL);
        assert(0);
    }
    assert(hardware_count == 5 && battery_samples == 2);
    puts("Display before slow/failed initialization, sensor sampling preserved: PASS");
    return 0;
}
