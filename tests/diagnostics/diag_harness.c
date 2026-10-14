#include "../../application/app_diag.c"
#include <assert.h>
#include <stdio.h>

diag_timer_struct diag_timer;
diag_rcc_struct diag_rcc;
static uint32_t tick;
static uint8_t locked;
static FILE *fixture;
static uint32_t sent;
static uint8_t tx_busy, tx_ready, force_busy;
static uint32_t memory_free = 5000;
static const uint8_t *pending_frame;
static uint8_t frame_copy[78];
static uint16_t pending_size;

void diag_enter(void)
{
    assert(locked == 0);
    locked = 1;
}

void diag_exit(void)
{
    assert(locked == 1);
    locked = 0;
}

uint32_t HAL_RCC_GetPCLK1Freq(void)
{
    return 50000000U;
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

size_t xPortGetFreeHeapSize(void)
{
    return 8000;
}

size_t xPortGetMinimumEverFreeHeapSize(void)
{
    return 7000;
}

void lv_mem_monitor(lv_mem_monitor_t *memory)
{
    memory->total_size = 44000;
    memory->free_size = memory_free;
    memory->max_used = 0; /* 真实LVGL实现不维护这个字段。 */
}

uint8_t kt6368a_init_is_pending(void)
{
    return 0;
}
uint32_t kt6368a_get_rx_dropped(void)
{
    return 3;
}

uint32_t kt6368a_get_rx_errors(void)
{
    return 4;
}

UBaseType_t uxTaskGetSystemState(TaskStatus_t *tasks, UBaseType_t count, uint32_t *total)
{
    assert(count >= 3 && locked == 0);
    tasks[0] = (TaskStatus_t){"sensor", 2, 2, 2000, 300};
    tasks[1] = (TaskStatus_t){"IDLE", 0, 1, 5000, 100};
    tasks[2] = (TaskStatus_t){"lvgl", 3, 0, 3000, 700};
    *total = 10000;
    return 3;
}

uint8_t kt6368a_tx_is_busy(void)
{
    return tx_busy;
}

uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    if (tx_ready == 0)
    {
        return 0;
    }
    assert(memcmp(pending_frame, frame_copy, pending_size) == 0);
    tx_ready = tx_busy = 0;
    if (sent == 2)
    {
        *result = HAL_TIMEOUT;
    }
    else
    {
        *result = HAL_OK;
    }
    return 1;
}

HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    assert(locked == 0 && size <= 78 && timeout_ms == 10);
    assert(memcmp(data, "WDG1", 4) == 0);
    assert(diag_crc16(data, size - 2) == ((uint16_t)data[size - 2] | ((uint16_t)data[size - 1] << 8)));
    if (force_busy != 0)
    {
        return HAL_BUSY;
    }
    assert(tx_busy == 0);
    pending_frame = data;
    pending_size = size;
    memcpy(frame_copy, data, size);
    tx_busy = 1;
    assert(fwrite(data, 1, size, fixture) == size);
    ++sent;
    return HAL_OK;
}

int main(int argc, char **argv)
{
    uint8_t i;
    assert(argc == 2);
    fixture = fopen(argv[1], "wb");
    assert(fixture != NULL);
    diag_rcc.CFGR = RCC_CFGR_PPRE1;
    app_diag_clock_init();
    assert(diag_timer.PSC == 9999 && diag_timer.CR1 == TIM_CR1_CEN);
    assert(s_sensors[APP_DIAG_MPU].init_result == UINT32_MAX);
    if (APP_DEBUG_TELEMETRY == 0)
    {
        tick = 1000;
        app_diag_receive((const uint8_t *)"DIAG,1\n", 7);
        for (i = 0; i < 20; ++i)
        {
            app_diag_service();
            tick += 100;
        }
        assert(sent == 0 && s_enabled == 0 && s_tx_pending == 0);
        fclose(fixture);
        puts("PASS: performance build emits no diagnostics, including after DIAG,1.");
        return 0;
    }

    diag_timer.CNT = UINT32_MAX - 4U;
    tick = UINT32_MAX - 5U;
    app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_OK, 0, UINT32_MAX - 9U);
    diag_timer.CNT = 5;
    tick = 4;
    app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_OK, 0, UINT32_MAX - 4U);
    app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_NOT_READY, 1, 3);
    app_diag_sensor_record(APP_DIAG_MPU, APP_DIAG_ERROR, 2, 2);
    assert(s_sensors[APP_DIAG_MPU].gap_last_ms == 10);
    assert(s_sensors[APP_DIAG_MPU].ok == 2 && s_sensors[APP_DIAG_MPU].not_ready == 1 && s_sensors[APP_DIAG_MPU].errors == 1);
    assert(s_sensors[APP_DIAG_MPU].work_max_us == 1000);
    app_diag_sensor_init(APP_DIAG_MPU, 0);
    app_diag_task_begin(APP_DIAG_SENSOR_TASK);
    diag_timer.CNT += 120;
    app_diag_task_end(APP_DIAG_SENSOR_TASK, 10);
    app_diag_task_begin(APP_DIAG_SENSOR_TASK);
    diag_timer.CNT += 10;
    app_diag_task_end(APP_DIAG_SENSOR_TASK, 10);
    assert(s_tasks[1].overruns == 1 && s_tasks[1].work_max == 12000 && s_tasks[1].period_last == 12000);
    app_diag_heart_queue(1);
    app_diag_heart_tx(2, 1);
    app_diag_heart_tx(1, 0);
    assert(s_boot_values[APP_BOOT_CAL_END_MS] == UINT32_MAX);
    app_diag_boot_set(APP_BOOT_FIRST_FRAME_MS, 50);
    app_diag_boot_set(APP_BOOT_RTC_BEGIN_MS, 60);
    app_diag_boot_set(APP_BOOT_RTC_END_MS, 80);
    app_diag_boot_set(APP_BOOT_RTC_RESULT, HAL_OK);
    app_diag_boot_set(APP_BOOT_BT_BEGIN_MS, 60);
    app_diag_boot_set(APP_BOOT_BT_END_MS, 950);
    app_diag_boot_set(APP_BOOT_BT_RESULT, HAL_OK);
    app_diag_boot_set(APP_BOOT_SENSOR_BEGIN_MS, 65);
    app_diag_boot_set(APP_BOOT_SENSOR_READY_MS, 200);
    app_diag_boot_set(APP_BOOT_SAMPLING_BEGIN_MS, 200);
    app_diag_boot_set(APP_BOOT_CAL_BEGIN_MS, 200);
    app_diag_boot_set(APP_BOOT_CAL_END_MS, 900);
    app_diag_boot_set(APP_BOOT_CAL_RESULT, 0);
    app_diag_boot_set(APP_BOOT_CAL_STATE, 3);
    app_diag_boot_set(APP_BOOT_CAL_FRAMES, 100);
    app_diag_boot_set(APP_BOOT_CAL_STABLE_FRAMES, 100);
    app_diag_imu_save_record(4, 2, 0x70, 12);
    {
        const uint32_t cal_values[16] = {(uint32_t)(int32_t)-500, 25, 16384, 200, 0, 10, 1228, 149, 128, 161,
                                         20, 180, 12, 11, 200, 4};
        app_diag_mpu_cal_record(cal_values);
    }
    tick = 1000;
    diag_timer.CNT = 10000;
    tx_busy = tx_ready = 1; /* 模拟心率帧结果待其原发起者收取。 */
    app_diag_service();
    assert(tx_busy && tx_ready && sent == 0 && s_tx_ok == 0);
    tx_busy = tx_ready = 0;
    force_busy = 1;
    app_diag_service();
    assert(sent == 0 && s_sequence == 0 && s_row == 0 && s_tx_ok == 0 && s_tx_failed == 0);
    force_busy = 0;
    for (i = 0; i < 13; ++i)
    {
        app_diag_service();
        app_diag_service();
        assert(sent == i + 1U);
        tx_ready = 1;
        app_diag_service();
        tick += 100;
    }
    assert(s_tx_ok == 12 && s_tx_failed == 1);
    assert((int32_t)s_mpu_cal_words[0] == -500 && s_mpu_cal_words[6] == 1228 && s_mpu_cal_words[9] == 161);
    assert(s_imu_save_words[0] == 4 && s_imu_save_words[1] == 2 && s_imu_save_words[2] == 0x70 && s_imu_save_words[3] == 12);
    assert(s_task_words[0][2] == 2048 && s_task_words[0][3] == 1200 && s_task_words[0][4] == 200);
    assert(s_task_words[2][2] == 4096 && s_task_words[2][3] == 2800 && s_task_words[2][4] == 300);
    assert(s_sensor_words[1][4] == UINT32_MAX);
    assert(s_system_words[3] == 39000);
    memory_free = 3000;
    diag_snapshot();
    assert(s_system_words[3] == 41000);
    memory_free = 6000;
    diag_snapshot();
    assert(s_system_words[3] == 41000);
    app_diag_receive((const uint8_t *)"DI", 2);
    app_diag_receive((const uint8_t *)"AG,0\n", 5);
    app_diag_service();
    assert(s_enabled == 0 && sent == 13);
    app_diag_receive((const uint8_t *)"xxxxxxxxxxxxxxxxDIAG,1\n", 23);
    assert(s_enabled == 0);
    app_diag_receive((const uint8_t *)"DIAG,1\n", 7);
    assert(s_enabled == 1 && s_row == 0);
    assert(app_diag_take_imu_calibration() == 0);
    app_diag_service();
    assert(tx_busy && sent == 14 && s_tx_ok == 12);
    app_diag_receive((const uint8_t *)"DIAG,0\n", 7);
    tx_ready = 1;
    app_diag_service();
    assert(!tx_busy && s_enabled == 0 && s_tx_ok == 13 && s_tx_failed == 1);
    app_diag_receive((const uint8_t *)"IMU,", 4);
    assert(app_diag_take_imu_calibration() == 0);
    app_diag_receive((const uint8_t *)"CAL\r\n", 5);
    assert(app_diag_take_imu_calibration() == 1 && app_diag_take_imu_calibration() == 0);
    app_diag_receive((const uint8_t *)"IMU,CALx\n", 9);
    assert(app_diag_take_imu_calibration() == 0);
    fclose(fixture);
    puts("Diagnostics: stack bytes, CPU, deadlines, read outcomes, wrap, pacing, commands and CRC: PASS");
    return 0;
}
