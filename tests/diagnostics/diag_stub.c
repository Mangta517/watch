/* Existing functional harnesses do not simulate the independent diagnostic clock. */
#include "../../application/app_diag.h"
void app_diag_clock_init(void)
{
}
uint32_t app_diag_clock_count(void)
{
    return 0;
}
void app_diag_task_begin(uint8_t id)
{
    (void)id;
}
void app_diag_task_end(uint8_t id, uint32_t target_ms)
{
    (void)id;
    (void)target_ms;
}
void app_diag_sensor_init(uint8_t id, uint32_t result)
{
    (void)id;
    (void)result;
}
void app_diag_sensor_record(uint8_t id, uint8_t outcome, uint32_t result, uint32_t started)
{
    (void)id;
    (void)outcome;
    (void)result;
    (void)started;
}
void app_diag_heart_queue(uint8_t dropped)
{
    (void)dropped;
}
void app_diag_heart_tx(uint16_t count, uint8_t success)
{
    (void)count;
    (void)success;
}
uint8_t app_diag_take_imu_calibration(void)
{
    return 0;
}
void app_diag_receive(const uint8_t *data, uint16_t size)
{
    (void)data;
    (void)size;
}
void app_diag_boot_set(uint8_t field, uint32_t value)
{
    (void)field;
    (void)value;
}
void app_diag_service(void)
{
}

uint32_t host_imu_save_diag[4];
uint32_t host_mpu_cal_diag[16];
void app_diag_mpu_cal_record(const uint32_t *values)
{
    uint8_t i;
    for (i = 0; i < 16U; ++i)
    {
        host_mpu_cal_diag[i] = values[i];
    }
}
void app_diag_imu_save_record(uint32_t stage, uint32_t error, uint32_t address, uint32_t elapsed_ms)
{
    host_imu_save_diag[0] = stage;
    host_imu_save_diag[1] = error;
    host_imu_save_diag[2] = address;
    host_imu_save_diag[3] = elapsed_ms;
}
