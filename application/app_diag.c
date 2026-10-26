#include "app_diag.h"
#include "app_touch.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "kt6368a.h"
#include "lvgl.h"
#include <string.h>

#define APP_DIAG_MAX_TASKS 8
#define APP_DIAG_FRAME_MS  100

typedef struct
{
    uint32_t started;
    uint32_t loops;
    uint32_t period_last;
    uint32_t period_min;
    uint32_t period_max;
    uint32_t work_last;
    uint32_t work_max;
    uint32_t overruns;
    uint32_t target_ms;
    uint8_t have_start;
} app_diag_task_struct;

typedef struct
{
    uint32_t ok;
    uint32_t not_ready;
    uint32_t errors;
    uint32_t result;
    uint32_t good_ms;
    uint32_t gap_last_ms;
    uint32_t gap_max_ms;
    uint32_t work_last_us;
    uint32_t work_max_us;
    uint32_t init_result;
    uint32_t recovery_attempts;
    uint8_t recovery_state;
} app_diag_sensor_struct;

static app_diag_task_struct s_tasks[APP_DIAG_TASK_COUNT];
static app_diag_sensor_struct s_sensors[APP_DIAG_SENSOR_COUNT];
static TaskStatus_t s_task_status[APP_DIAG_MAX_TASKS];
static uint32_t s_task_words[APP_DIAG_MAX_TASKS][14];
static uint8_t s_task_ids[APP_DIAG_MAX_TASKS];
static uint32_t s_sensor_words[APP_DIAG_SENSOR_COUNT][10];
static uint32_t s_system_words[16];
static uint32_t s_boot_values[APP_BOOT_FIELD_COUNT];
static uint32_t s_boot_words[APP_BOOT_FIELD_COUNT];
static uint32_t s_imu_save_values[4] = {0, 0, UINT32_MAX, UINT32_MAX};
static uint32_t s_imu_save_words[4];
static uint32_t s_mpu_cal_values[16];
static uint32_t s_mpu_cal_words[16];
static uint32_t s_previous_runtime[APP_DIAG_MAX_TASKS];
static uint32_t s_previous_total;
static uint32_t s_snapshot_ms;
static uint32_t s_last_send_ms;
static uint32_t s_queue_dropped, s_heart_tx_ok, s_heart_tx_failed;
static uint32_t s_tx_ok, s_tx_failed;
static uint32_t s_lvgl_peak_used;
static uint8_t s_tx_frame[78]; /* USART1中断发送期间保持有效。 */
static uint16_t s_sequence;
static uint8_t s_task_count, s_row, s_enabled = APP_DEBUG_TELEMETRY;
static uint8_t s_snapshot_ready;
static uint8_t s_tx_pending;

void app_diag_clock_init(void)
{
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
    uint8_t i;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
    {
        timer_hz *= 2;
    }
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;
    TIM2->PSC = timer_hz / 10000 - 1;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;
    for (i = 0; i < APP_DIAG_SENSOR_COUNT; ++i)
    {
        s_sensors[i].init_result = UINT32_MAX;
        s_sensors[i].result = UINT32_MAX;
    }
    for (i = 0; i < APP_BOOT_FIELD_COUNT; ++i)
    {
        s_boot_values[i] = UINT32_MAX;
    }
    s_boot_values[APP_BOOT_CAL_STATE] = 0;
    s_boot_values[APP_BOOT_CAL_FRAMES] = 0;
    s_boot_values[APP_BOOT_CAL_STABLE_FRAMES] = 0;
}

void app_diag_boot_set(uint8_t field, uint32_t value)
{
    taskENTER_CRITICAL();
    s_boot_values[field] = value;
    taskEXIT_CRITICAL();
}

uint32_t app_diag_clock_count(void)
{
    return TIM2->CNT;
}

void app_diag_imu_save_record(uint32_t stage, uint32_t error, uint32_t address, uint32_t elapsed_ms)
{
    taskENTER_CRITICAL();
    s_imu_save_values[0] = stage;
    s_imu_save_values[1] = error;
    s_imu_save_values[2] = address;
    s_imu_save_values[3] = elapsed_ms;
    taskEXIT_CRITICAL();
}

void app_diag_mpu_cal_record(const uint32_t *values)
{
    taskENTER_CRITICAL();
    memcpy(s_mpu_cal_values, values, sizeof(s_mpu_cal_values));
    taskEXIT_CRITICAL();
}

void app_diag_task_begin(uint8_t id)
{
    app_diag_task_struct *stats = &s_tasks[id];
    uint32_t now = app_diag_clock_count();
    uint32_t period;
    taskENTER_CRITICAL();
    if (stats->have_start != 0)
    {
        period = (now - stats->started) * 100;
        stats->period_last = period;
        if (stats->period_min == 0 || period < stats->period_min)
        {
            stats->period_min = period;
        }
        if (period > stats->period_max)
        {
            stats->period_max = period;
        }
    }
    stats->started = now;
    stats->have_start = 1;
    taskEXIT_CRITICAL();
}

void app_diag_task_end(uint8_t id, uint32_t target_ms)
{
    app_diag_task_struct *stats = &s_tasks[id];
    uint32_t work = (app_diag_clock_count() - stats->started) * 100;
    taskENTER_CRITICAL();
    stats->work_last = work;
    stats->target_ms = target_ms;
    if (work > stats->work_max)
    {
        stats->work_max = work;
    }
    if (work > target_ms * 1000)
    {
        ++stats->overruns;
    }
    ++stats->loops;
    taskEXIT_CRITICAL();
}

void app_diag_sensor_init(uint8_t id, uint32_t result)
{
    taskENTER_CRITICAL();
    s_sensors[id].init_result = result;
    taskEXIT_CRITICAL();
}

void app_diag_sensor_recovery(uint8_t id, uint8_t state)
{
    app_diag_sensor_struct *stats = &s_sensors[id];
    taskENTER_CRITICAL();
    if (state == 2 && stats->recovery_state != 2)
    {
        ++stats->recovery_attempts;
    }
    if (stats->recovery_state == 2 && state != 2)
    {
        stats->init_result = state;
    }
    stats->recovery_state = state;
    taskEXIT_CRITICAL();
}

void app_diag_sensor_record(uint8_t id, uint8_t outcome, uint32_t result, uint32_t started)
{
    app_diag_sensor_struct *stats = &s_sensors[id];
    uint32_t now_ms = HAL_GetTick();
    uint32_t work_us = (app_diag_clock_count() - started) * 100;
    taskENTER_CRITICAL();
    stats->result = result;
    stats->work_last_us = work_us;
    if (work_us > stats->work_max_us)
    {
        stats->work_max_us = work_us;
    }
    if (outcome == APP_DIAG_OK)
    {
        if (stats->ok != 0)
        {
            stats->gap_last_ms = now_ms - stats->good_ms;
            if (stats->gap_last_ms > stats->gap_max_ms)
            {
                stats->gap_max_ms = stats->gap_last_ms;
            }
        }
        stats->good_ms = now_ms;
        ++stats->ok;
    }
    else if (outcome == APP_DIAG_NOT_READY)
    {
        ++stats->not_ready;
    }
    else
    {
        ++stats->errors;
    }
    taskEXIT_CRITICAL();
}

void app_diag_heart_queue(uint8_t dropped)
{
    taskENTER_CRITICAL();
    s_queue_dropped += dropped;
    taskEXIT_CRITICAL();
}

void app_diag_heart_tx(uint16_t count, uint8_t success)
{
    if (success != 0)
    {
        s_heart_tx_ok += count;
    }
    else
    {
        s_heart_tx_failed += count;
    }
}

static uint8_t s_imu_cal_requested;
static char s_rx_command[16];
static uint8_t s_rx_length, s_rx_overflow;

void app_diag_reset_receive(void)
{
    s_rx_length = 0;
    s_rx_overflow = 0;
}

uint8_t app_diag_take_imu_calibration(void)
{
    uint8_t requested;
    taskENTER_CRITICAL();
    requested = s_imu_cal_requested;
    s_imu_cal_requested = 0;
    taskEXIT_CRITICAL();
    return requested;
}

void app_diag_receive(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    for (i = 0; i < size; ++i)
    {
        if (data[i] == '\n' || data[i] == '\r')
        {
            s_rx_command[s_rx_length] = '\0';
            if (s_rx_overflow == 0 && strcmp(s_rx_command, "DIAG,0") == 0)
            {
                s_enabled = 0;
            }
            else if (s_rx_overflow == 0 && strcmp(s_rx_command, "DIAG,1") == 0)
            {
                s_enabled = APP_DEBUG_TELEMETRY;
                s_row = 0;
                s_snapshot_ready = 0;
            }
            else if (s_rx_overflow == 0 && strcmp(s_rx_command, "IMU,CAL") == 0)
            {
                taskENTER_CRITICAL();
                s_imu_cal_requested = 1;
                taskEXIT_CRITICAL();
            }
            app_diag_reset_receive();
        }
        else if (s_rx_length < sizeof(s_rx_command) - 1)
        {
            s_rx_command[s_rx_length++] = (char)data[i];
        }
        else
        {
            s_rx_overflow = 1;
        }
    }
}

static uint8_t diag_task_id(const char *name)
{
    if (strcmp(name, "lvgl") == 0)
    {
        return 0;
    }
    if (strcmp(name, "sensor") == 0)
    {
        return 1;
    }
    if (strcmp(name, "IDLE") == 0)
    {
        return 2;
    }
    return 3;
}

static void diag_snapshot(void)
{
    uint32_t total, delta, runtime, *words;
    uint8_t i, id;
    app_diag_task_struct task_stats;
    app_diag_sensor_struct sensor_stats;
    lv_mem_monitor_t memory;
    s_task_count = (uint8_t)uxTaskGetSystemState(s_task_status, APP_DIAG_MAX_TASKS, &total);
    delta = total - s_previous_total;
    s_previous_total = total;
    s_snapshot_ms = HAL_GetTick();
    for (i = 0; i < s_task_count; ++i)
    {
        id = diag_task_id(s_task_status[i].pcTaskName);
        s_task_ids[i] = id;
        words = s_task_words[i];
        memset(words, 0, sizeof(s_task_words[i]));
        words[0] = s_task_status[i].uxCurrentPriority;
        words[1] = s_task_status[i].eCurrentState;
        if (id == 0)
        {
            words[2] = 1024 * sizeof(StackType_t);
        }
        else if (id == 1)
        {
            words[2] = 512 * sizeof(StackType_t);
        }
        else if (id == 2)
        {
            words[2] = configMINIMAL_STACK_SIZE * sizeof(StackType_t);
        }
        else if (strcmp(s_task_status[i].pcTaskName, "touch") == 0)
        {
            words[2] = APP_TOUCH_STACK_WORDS * sizeof(StackType_t);
        }
        words[3] = s_task_status[i].usStackHighWaterMark * sizeof(StackType_t);
        runtime = s_task_status[i].ulRunTimeCounter - s_previous_runtime[id];
        s_previous_runtime[id] = s_task_status[i].ulRunTimeCounter;
        if (delta != 0)
        {
            words[4] = (uint32_t)((uint64_t)runtime * 1000 / delta);
        }
        if (id < APP_DIAG_TASK_COUNT)
        {
            taskENTER_CRITICAL();
            task_stats = s_tasks[id];
            taskEXIT_CRITICAL();
            words[5] = task_stats.loops;
            words[6] = task_stats.period_last;
            words[7] = task_stats.period_min;
            words[8] = task_stats.period_max;
            words[9] = task_stats.work_last;
            words[10] = task_stats.work_max;
            words[11] = task_stats.overruns;
            words[12] = task_stats.target_ms;
            words[13] = task_stats.have_start;
        }
    }
    for (i = 0; i < APP_DIAG_SENSOR_COUNT; ++i)
    {
        taskENTER_CRITICAL();
        sensor_stats = s_sensors[i];
        taskEXIT_CRITICAL();
        words = s_sensor_words[i];
        words[0] = sensor_stats.ok;
        words[1] = sensor_stats.not_ready;
        words[2] = sensor_stats.errors;
        words[3] = sensor_stats.result;
        words[4] = UINT32_MAX;
        if (sensor_stats.ok != 0)
        {
            words[4] = s_snapshot_ms - sensor_stats.good_ms;
        }
        words[5] = sensor_stats.gap_last_ms;
        words[6] = sensor_stats.gap_max_ms;
        words[7] = sensor_stats.work_last_us;
        words[8] = sensor_stats.work_max_us;
        words[9] = sensor_stats.init_result;
    }
    lv_mem_monitor(&memory);
    if (memory.total_size - memory.free_size > s_lvgl_peak_used)
    {
        s_lvgl_peak_used = memory.total_size - memory.free_size;
    }
    s_system_words[0] = xPortGetFreeHeapSize();
    s_system_words[1] = xPortGetMinimumEverFreeHeapSize();
    s_system_words[2] = memory.free_size;
    s_system_words[3] = s_lvgl_peak_used;
    s_system_words[4] = kt6368a_get_rx_dropped();
    s_system_words[5] = kt6368a_get_rx_errors();
    taskENTER_CRITICAL();
    s_system_words[6] = s_queue_dropped;
    taskEXIT_CRITICAL();
    s_system_words[7] = s_heart_tx_ok;
    s_system_words[8] = s_heart_tx_failed;
    s_system_words[9] = s_tx_ok;
    s_system_words[10] = s_tx_failed;
    s_system_words[11] = s_task_count;
    s_system_words[12] = 100;
    s_system_words[13] = APP_DIAG_FRAME_MS;
    s_system_words[14] = s_enabled;
    s_system_words[15] = total;
    taskENTER_CRITICAL();
    memcpy(s_boot_words, s_boot_values, sizeof(s_boot_words));
    memcpy(s_imu_save_words, s_imu_save_values, sizeof(s_imu_save_words));
    memcpy(s_mpu_cal_words, s_mpu_cal_values, sizeof(s_mpu_cal_words));
    taskEXIT_CRITICAL();
}

static uint16_t diag_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < size; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000) != 0)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void diag_put_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

void app_diag_service(void)
{
    uint8_t *frame = s_tx_frame;
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef status;
    const uint32_t *words;
    uint16_t crc, size;
    uint8_t count, i, id;
    /* 关闭发送后仍收取在途结果，成功计数只在UART完成后更新。 */
    if (s_tx_pending != 0 && kt6368a_take_tx_result(&status) != 0)
    {
        s_tx_pending = 0;
        if (status == HAL_OK)
        {
            ++s_tx_ok;
        }
        else
        {
            ++s_tx_failed;
        }
    }
    if (s_enabled == 0 || kt6368a_init_is_pending() != 0 || kt6368a_tx_is_busy() != 0 ||
        now - s_last_send_ms < APP_DIAG_FRAME_MS)
    {
        return;
    }
    if (s_row == 0 && s_snapshot_ready == 0)
    {
        diag_snapshot();
        s_snapshot_ready = 1;
    }
    memcpy(frame, "WDG1", 4);
    if (s_row < s_task_count)
    {
        frame[4] = 1;
        id = s_task_ids[s_row];
        words = s_task_words[s_row];
        count = 14;
    }
    else if (s_row < s_task_count + APP_DIAG_SENSOR_COUNT)
    {
        frame[4] = 2;
        id = s_row - s_task_count;
        words = s_sensor_words[id];
        count = 10;
    }
    else if (s_row == s_task_count + APP_DIAG_SENSOR_COUNT)
    {
        frame[4] = 3;
        id = 0;
        words = s_system_words;
        count = 16;
    }
    else if (s_row == s_task_count + APP_DIAG_SENSOR_COUNT + 1)
    {
        frame[4] = 4;
        id = 0;
        words = s_boot_words;
        count = APP_BOOT_FIELD_COUNT;
    }
    else if (s_row == s_task_count + APP_DIAG_SENSOR_COUNT + 2)
    {
        frame[4] = 5;
        id = 0;
        words = s_imu_save_words;
        count = 4;
    }
    else
    {
        frame[4] = 6;
        id = 0;
        words = s_mpu_cal_words;
        count = 16;
    }
    frame[5] = id;
    frame[6] = (uint8_t)s_sequence;
    frame[7] = (uint8_t)(s_sequence >> 8);
    diag_put_u32(frame + 8, s_snapshot_ms);
    for (i = 0; i < count; ++i)
    {
        diag_put_u32(frame + 12 + i * 4, words[i]);
    }
    size = 12 + count * 4;
    crc = diag_crc16(frame, size);
    frame[size++] = (uint8_t)crc;
    frame[size++] = (uint8_t)(crc >> 8);
    status = kt6368a_write_async(frame, size, 10);
    if (status == HAL_BUSY)
    {
        return; /* 保留本行和序号，下轮重试。 */
    }
    if (status != HAL_OK)
    {
        ++s_tx_failed;
    }
    else
    {
        s_tx_pending = 1;
    }
    s_last_send_ms = now;
    ++s_sequence;
    ++s_row;
    if (s_row >= s_task_count + APP_DIAG_SENSOR_COUNT + 4)
    {
        s_row = 0;
        s_snapshot_ready = 0;
    }
}
