#ifndef APP_DIAG_H
#define APP_DIAG_H

#include <stdint.h>

/* 性能观察版关闭诊断和心率波形发送；调试构建可通过编译选项置1。 */
#ifndef APP_DEBUG_TELEMETRY
#define APP_DEBUG_TELEMETRY 0
#endif

enum
{
    APP_DIAG_LVGL,
    APP_DIAG_SENSOR_TASK,
    APP_DIAG_TASK_COUNT
};

enum
{
    APP_DIAG_MPU,
    APP_DIAG_MAG,
    APP_DIAG_HEART,
    APP_DIAG_BARO,
    APP_DIAG_AHT,
    APP_DIAG_BATTERY,
    APP_DIAG_SENSOR_COUNT
};

enum
{
    APP_DIAG_OK,
    APP_DIAG_NOT_READY,
    APP_DIAG_ERROR
};

enum
{
    APP_BOOT_FIRST_FRAME_MS,
    APP_BOOT_RTC_BEGIN_MS,
    APP_BOOT_RTC_END_MS,
    APP_BOOT_RTC_RESULT,
    APP_BOOT_BT_BEGIN_MS,
    APP_BOOT_BT_END_MS,
    APP_BOOT_BT_RESULT,
    APP_BOOT_SENSOR_BEGIN_MS,
    APP_BOOT_SENSOR_READY_MS,
    APP_BOOT_SAMPLING_BEGIN_MS,
    APP_BOOT_CAL_BEGIN_MS,
    APP_BOOT_CAL_END_MS,
    APP_BOOT_CAL_RESULT,
    APP_BOOT_CAL_STATE,
    APP_BOOT_CAL_FRAMES,
    APP_BOOT_CAL_STABLE_FRAMES,
    APP_BOOT_FIELD_COUNT
};

void app_diag_boot_set(uint8_t field, uint32_t value);
void app_diag_imu_save_record(uint32_t stage, uint32_t error, uint32_t address, uint32_t elapsed_ms);
void app_diag_mpu_cal_record(const uint32_t *values);

/* TIM2专用于诊断，100us一计数；FreeRTOS在调度器启动时调用初始化。 */
void app_diag_clock_init(void);
uint32_t app_diag_clock_count(void);
void app_diag_task_begin(uint8_t id);
void app_diag_task_end(uint8_t id, uint32_t target_ms);
void app_diag_sensor_init(uint8_t id, uint32_t result);
void app_diag_sensor_record(uint8_t id, uint8_t outcome, uint32_t result, uint32_t started);
void app_diag_heart_queue(uint8_t dropped);
void app_diag_heart_tx(uint16_t count, uint8_t success);
void app_diag_receive(const uint8_t *data, uint16_t size);
/* IMU,CAL请求仅传递标志；sensor任务消费后执行校准/EEPROM保存。 */
uint8_t app_diag_take_imu_calibration(void);
/* 仅LVGL任务发送，每100ms最多一帧；不增加任务，不在传感器/ISR里发串口。 */
void app_diag_service(void);

#endif
