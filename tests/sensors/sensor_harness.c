/* Real driver/service/math, fake bus. Inject EXTI while an I2C transaction is active. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_imu.h"
#include "app_heart.h"
#include "common_imu.h"
#include "common_activity.h"
#include "my_i2c.h"
#include "spl006.h"
#include "app_baro.h"
#include "semphr.h"
#include "bl24c02.h"
#include "app_mag_store.h"
#include "app_imu_store.h"
#include "app_uptime.h"
#include "task.h"

#ifdef _WIN32
/* Exit on failed checks instead of opening a Windows CRT assertion dialog. */
#undef assert
#define assert(condition)                                                                                                                  \
    do                                                                                                                                     \
    {                                                                                                                                      \
        if (!(condition))                                                                                                                  \
        {                                                                                                                                  \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition);                                                                   \
            exit(1);                                                                                                                       \
        }                                                                                                                                  \
    } while (0)
#endif

void HAL_GPIO_EXTI_Callback(uint16_t pin);
static uint8_t regs[128][256];
static TickType_t tick;
static uint8_t mpu_auto_ready = 1;
static uint32_t heart_motion_ms;
static int scheduler_state = taskSCHEDULER_RUNNING;
static unsigned hal_delay_calls, task_delay_calls;
static uint32_t primask;
static uint8_t bus_active, mutex_taken;
static int fail_dev = -1, fail_reg = -1;
static unsigned reads, mag_frames, mag_config_writes, bus_transactions;
static uint32_t transfer_ms, spl_reset_ended, spl_status_ended;
static uint8_t check_recovery_waits, spl_status_polls;
static int spl_fail_read_at = -1, spl_fail_write_at = -1;
static unsigned spl_reads, spl_writes;
static unsigned mpu_reads, mpu_writes;
static uint16_t mpu_last_frame_size;
static int mpu_fail_read_at = -1, mpu_fail_write_at = -1;
static uint8_t spl_reset_status = SPL06_SENSOR_RDY | SPL06_COEF_RDY;
static TickType_t motion_at_tick;
static uint8_t eeprom[BL24C02_CAPACITY_BYTES];
static uint8_t eeprom_write_protected;
static TickType_t eeprom_busy_until;
static int eeprom_fail_write_after = -1;
static unsigned eeprom_writes;
static unsigned eeprom_max_transfer;
static int eeprom_read_fail_after = -1;
static TickType_t eeprom_busy_ms = 3;
extern uint32_t host_imu_save_diag[4];
static void word(uint8_t dev, uint8_t reg, int16_t value);
static uint32_t heart_motion_frames;
static int16_t heart_motion_accel[3];

void app_heart_motion_update(int16_t accel_x, int16_t accel_y, int16_t accel_z, uint32_t sample_ms)
{
    heart_motion_accel[0] = accel_x;
    heart_motion_accel[1] = accel_y;
    heart_motion_accel[2] = accel_z;
    assert((uint32_t)(HAL_GetTick() - sample_ms) < 1000);
    heart_motion_ms = sample_ms;
    ++heart_motion_frames;
}

uint32_t __get_PRIMASK(void)
{
    return primask;
}
void __disable_irq(void)
{
    primask = 1;
}
void __set_PRIMASK(uint32_t mask)
{
    primask = mask;
}
void HAL_Delay(uint32_t ms)
{
    assert(scheduler_state != taskSCHEDULER_RUNNING && !bus_active && !mutex_taken && !primask);
    ++hal_delay_calls;
    tick += ms;
}
int xTaskGetSchedulerState(void)
{
    return scheduler_state;
}
void Error_Handler(void)
{
    abort();
}
uint32_t HAL_GetTick(void)
{
    return tick;
}

void __DMB(void)
{
}
TickType_t xTaskGetTickCount(void)
{
    return tick;
}
void vTaskDelay(TickType_t ticks)
{
    assert(scheduler_state == taskSCHEDULER_RUNNING && !bus_active && !mutex_taken && !primask && ticks > 0);
    ++task_delay_calls;
    tick += ticks;
    if (motion_at_tick && tick >= motion_at_tick)
    {
        word(MPU6050_ADDR, 0x3B, 1000);
        motion_at_tick = 0;
    }
}
SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return &mutex_taken;
}
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait)
{
    assert(mutex && !mutex_taken);
    (void)wait;
    mutex_taken = 1;
    return 1;
}
int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    assert(mutex && mutex_taken);
    mutex_taken = 0;
    return 1;
}

static uint8_t eeprom_parked;
static unsigned eeprom_parks, eeprom_resumes;

static void bus_enter(my_i2c_bus_t bus)
{
    assert((bus == MY_I2C_BUS_SENSOR || bus == MY_I2C_BUS_EEPROM) && !bus_active);
    if (bus == MY_I2C_BUS_EEPROM)
    {
        assert(eeprom_parked == 0);
    }
    assert(!primask && !mutex_taken); /* No whole-transfer IRQ mask or snapshot mutex. */
    tick += transfer_ms;
    ++bus_transactions;
    bus_active = 1;
    HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
    assert(bus_active && !primask); /* ISR cannot nest an I2C transaction. */
}
void my_i2c_init(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_EEPROM);
    eeprom_parked = 0;
    ++eeprom_resumes;
}
void my_i2c_suspend(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_EEPROM && !bus_active && !primask);
    eeprom_parked = 1;
    ++eeprom_parks;
}
uint8_t my_i2c_probe(my_i2c_bus_t bus, uint8_t dev)
{
    bus_enter(bus);
    assert(bus == MY_I2C_BUS_EEPROM && dev == BL24C02_ADDR);
    bus_active = 0;
    return fail_dev == dev || (int32_t)(tick - eeprom_busy_until) < 0;
}
uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t error;
    if (check_recovery_waits != 0 && dev == SPL006_ADDR && reg == SPL06_MEAS_CFG)
    {
        if (spl_status_polls == 0)
        {
            assert((uint32_t)(tick - spl_reset_ended) >= 20);
        }
        else if (spl_status_polls == 1)
        {
            assert((uint32_t)(tick - spl_status_ended) >= 50);
        }
        ++spl_status_polls;
    }
    bus_enter(bus);
    if (check_recovery_waits != 0 && dev == SPL006_ADDR && reg == SPL06_MEAS_CFG)
    {
        spl_status_ended = tick;
    }
    ++reads;
    error = (fail_dev == dev && (fail_reg < 0 || fail_reg == reg));
    if (dev == MPU6050_ADDR)
    {
        error |= (int)mpu_reads++ == mpu_fail_read_at;
    }
    if (dev == SPL006_ADDR)
    {
        error |= (int)spl_reads++ == spl_fail_read_at;
    }
    if (bus == MY_I2C_BUS_EEPROM)
    {
        assert(dev == BL24C02_ADDR && (unsigned)reg + len <= sizeof(eeprom));
        assert((int32_t)(tick - eeprom_busy_until) >= 0);
        if (len > eeprom_max_transfer)
        {
            eeprom_max_transfer = len;
        }
        if (eeprom_read_fail_after == 0)
        {
            error = 1;
        }
        else if (eeprom_read_fail_after > 0)
        {
            --eeprom_read_fail_after;
        }
        if (!error)
        {
            memcpy(data, eeprom + reg, len);
        }
        bus_active = 0;
        return error;
    }
    if (!error)
    {
        memcpy(data, &regs[dev][reg], len);
        if (dev == LSM303DLHC_MAG_ADDR && reg == 3)
        {
            assert(len == 6);
            regs[dev][9] = 0;
            ++mag_frames;
        }
        if (dev == MPU6050_ADDR && reg == 0x3A)
        {
            assert(len == 1 || len == 7 || len == 15);
            mpu_last_frame_size = len;
            if (mpu_auto_ready != 0)
            {
                data[0] |= 1U; /* 默认模拟持续100Hz输出；新鲜度用例可暂停转换。 */
            }
            regs[dev][0x3A] = 0;
        }
    }
    else if (dev == MPU6050_ADDR && reg == MPU6050_INT_STATUS && len == 15)
    {
        memset(data, 0xA5, 5); /* 模拟读到部分字节后NACK，驱动不得发布半帧。 */
    }
    bus_active = 0;
    return error;
}
uint8_t my_i2c_write_bytes(my_i2c_bus_t bus, uint8_t dev, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t error;
    bus_enter(bus);
    error = (fail_dev == dev && (fail_reg < 0 || fail_reg == reg));
    if (dev == MPU6050_ADDR)
    {
        error |= (int)mpu_writes++ == mpu_fail_write_at;
    }
    if (dev == SPL006_ADDR)
    {
        error |= (int)spl_writes++ == spl_fail_write_at;
    }
    if (bus == MY_I2C_BUS_EEPROM)
    {
        assert(dev == BL24C02_ADDR && len > 0 && len <= 16 - (reg % 16));
        assert((unsigned)reg + len <= sizeof(eeprom) && (int32_t)(tick - eeprom_busy_until) >= 0);
        if (len > eeprom_max_transfer)
        {
            eeprom_max_transfer = len;
        }
        if (eeprom_fail_write_after == 0)
        {
            memcpy(eeprom + reg, data, len / 2); /* Partial target page before power/transport failure. */
            error = 1;
        }
        else if (!error)
        {
            if (eeprom_fail_write_after > 0)
            {
                --eeprom_fail_write_after;
            }
            if (!eeprom_write_protected)
            {
                memcpy(eeprom + reg, data, len);
            }
            ++eeprom_writes;
            eeprom_busy_until = tick + eeprom_busy_ms;
        }
        bus_active = 0;
        return error;
    }
    if (!error)
    {
        uint8_t spl_ready = regs[SPL006_ADDR][SPL06_MEAS_CFG] & (SPL06_SENSOR_RDY | SPL06_COEF_RDY);
        memcpy(&regs[dev][reg], data, len);
        if (dev == SPL006_ADDR && reg == SPL06_MEAS_CFG)
        {
            regs[dev][reg] |= spl_ready; /* SENSOR_RDY/COEF_RDY为只读位，写standby不会清除。 */
        }
        if (dev == MPU6050_ADDR && reg == MPU6050_PWR_MGMT_1 && data[0] == 0x80)
        {
            regs[dev][reg] = 0x40; /* Device reset completes into SLEEP. */
        }
        if (dev == SPL006_ADDR && reg == SPL06_RESET)
        {
            regs[dev][SPL06_MEAS_CFG] = spl_reset_status;
            spl_reset_ended = tick;
            spl_status_polls = 0;
        }
        if (dev == LSM303DLHC_MAG_ADDR)
        {
            ++mag_config_writes;
        }
    }
    bus_active = 0;
    return error;
}
static void word(uint8_t dev, uint8_t reg, int16_t value)
{
    regs[dev][reg] = (uint16_t)value >> 8;
    regs[dev][reg + 1] = (uint8_t)value;
}
static void magnetic(float x, float y, float z)
{
    word(LSM303DLHC_MAG_ADDR, 3, (int16_t)lroundf(x * 11.0f));
    word(LSM303DLHC_MAG_ADDR, 5, (int16_t)lroundf(z * 9.8f));
    word(LSM303DLHC_MAG_ADDR, 7, (int16_t)lroundf(y * 11.0f));
    regs[LSM303DLHC_MAG_ADDR][9] = 1;
}
static float angle_error(float a, float b)
{
    return fabsf(remainderf(a - b, 360.0f));
}

static void driver_tests(void)
{
    lsm303dlhc_mag_struct mag, previous;
    Accel_Gyro_struct raw;
    unsigned before;
    uint16_t count;
    int32_t pressure;

    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x68;
    assert(my_mpu6050_init() == 0);
    assert(regs[MPU6050_ADDR][MPU6050_GYRO_CONFIG] == 0x10);
    assert(regs[MPU6050_ADDR][MPU6050_ACCEL_CONFIG] == 0x08);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 9);
    assert(regs[MPU6050_ADDR][MPU6050_CONFIG] == 3);
    assert(lsm303dlhc_mag_init() == 1); /* Wrong ID cannot enter measurement mode. */
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    fail_dev = LSM303DLHC_MAG_ADDR;
    fail_reg = 1;
    assert(lsm303dlhc_mag_init() == 1);
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_ERROR);
    fail_dev = fail_reg = -1;
    assert(lsm303dlhc_mag_init() == 0);
    assert(regs[0x1E][0] == 0x18 && regs[0x1E][1] == 0x20 && regs[0x1E][2] == 0);
    assert(mag_config_writes >= 3);
    magnetic(50, -25, 30);
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
    assert(mag.raw[0] == 550 && mag.raw[1] == -275 && mag.raw[2] == 294);
    assert(fabsf(mag.field_uT[0] - 50) < .01f && fabsf(mag.field_uT[2] - 30) < .01f);
    previous = mag;
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
    assert(memcmp(&previous, &mag, sizeof(mag)) == 0);
    regs[0x1E][9] = 2;
    before = mag_frames;
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
    assert(mag_frames == before + 1 && regs[0x1E][9] == 0); /* Drain LOCK. */
    regs[0x1E][9] = 3; /* 部分读失败可能同时留下DRDY和LOCK。 */
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
    assert(memcmp(&previous, &mag, sizeof(mag)) == 0 && regs[0x1E][9] == 0);
    magnetic(50, -25, 30); /* 数值相同仍可能是一次真实新转换，不能按值去重。 */
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
    magnetic(0, 0, 0);
    word(0x1E, 3, -4096);
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_ERROR);
    assert(memcmp(&previous, &mag, sizeof(mag)) == 0);
    magnetic(50, -25, 30);
    fail_dev = 0x1E;
    fail_reg = 3;
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_ERROR);
    assert(memcmp(&previous, &mag, sizeof(mag)) == 0);
    fail_dev = fail_reg = -1;

    before = reads;
    count = my_mpu6050_int_count();
    HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
    assert(reads == before && my_mpu6050_int_count() == (uint16_t)(count + 1));
    assert(my_mpu6050_int_check_clear() == 1 && my_mpu6050_int_check_clear() == 0);
    regs[MPU6050_ADDR][0x3A] = 1;
    word(MPU6050_ADDR, 0x3B, 123);
    word(MPU6050_ADDR, 0x3D, -456);
    word(MPU6050_ADDR, 0x3F, MPU6050_ACCEL_LSB_PER_G);
    word(MPU6050_ADDR, 0x43, 17);
    assert(my_mpu6050_get_data(&raw) == 0);
    assert(raw.Accel.Accel_x == 123 && raw.Accel.Accel_y == -456 && raw.Gyro.Gyro_x == 17);
    assert(reads == before + 1 && regs[MPU6050_ADDR][0x3A] == 0 && my_mpu6050_get_int_status() == 1);
    regs[SPL006_ADDR][8] = 0x10;
    assert(spl006_read_pressure(&pressure) == 0); /* Third device under injected EXTI. */
    word(MPU6050_ADDR, 0x3B, 0);
    word(MPU6050_ADDR, 0x3D, 0);
    word(MPU6050_ADDR, 0x43, 0);
    puts("PASS: ID/config, XZY/endian/units, NACK/saturation/LOCK, deferred MPU IRQ, shared bus");
}

static void missing_frame_tests(void)
{
    Accel_Gyro_struct raw, previous;
    mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};
    uint16_t stable, average;
    unsigned i, before;
    assert(my_mpu6050_init() == 0);
    memset(&raw, 0x35, sizeof(raw));
    previous = raw;
    mpu_auto_ready = 0;
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 0;
    before = reads;
    assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY);
    assert(reads == before + 1 && memcmp(&raw, &previous, sizeof(raw)) == 0);
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    assert(my_mpu6050_get_data(&raw) == 0);
    previous = raw;
    assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY);
    assert(memcmp(&raw, &previous, sizeof(raw)) == 0);
    mpu_auto_ready = 1;
    my_mpu6050_restore_offset(&zero);
    my_mpu6050_zero_offset_begin(0);
    for (i = 0; i <= 202; ++i)
    {
        raw = (Accel_Gyro_struct){.Gyro = {66, -33, 99}, .Accel = {20, -10, 8222}};
        my_mpu6050_zero_offset_step(&raw, i != 50U && i != 150U, i * 10U);
    }
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100 && average == 100);
    my_mpu6050_zero_offset_begin(UINT32_MAX - 1000);
    assert(my_mpu6050_zero_offset_step(NULL, 0, UINT32_MAX - 999) == MPU6050_CAL_WAIT_STILL);
    assert(my_mpu6050_zero_offset_step(NULL, 0, 1999) == MPU6050_CAL_FAILED);
    my_mpu6050_restore_offset(&zero);
    puts("PASS: DATA_RDY gates publication; missing calibration frames keep the wall-clock timeout and valid sample counts.");
}

static void calibration_tests(void)
{
    TickType_t start;
    Accel_Gyro_struct raw;
    mpu6050_offset_struct offset;
    const mpu6050_offset_struct zero = {0};
    word(0x68, 0x3B, 20);
    word(0x68, 0x3D, -10);
    word(0x68, 0x3F, 8192 + 30);
    word(0x68, 0x43, 66);
    word(0x68, 0x45, -33);
    word(0x68, 0x47, 99);
    start = tick;
    assert(my_mpu6050_zero_offset() == 0 && tick - start == 2000);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.accel[1] == -10 && offset.accel[2] == 30);
    assert(offset.gyro[0] == 66 && offset.gyro[1] == -33 && offset.gyro[2] == 99);
    assert(my_mpu6050_get_data(&raw) == 0);
    assert(raw.Accel.Accel_z == 8192 && raw.Gyro.Gyro_z == 0);
    assert(my_mpu6050_zero_offset() == 0);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.accel[1] == -10 && offset.accel[2] == 30);
    assert(offset.gyro[0] == 66 && offset.gyro[1] == -33 && offset.gyro[2] == 99);
    word(0x68, 0x3B, 2000);
    start = tick;
    assert(my_mpu6050_zero_offset() == 1 && tick - start == 3000);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.gyro[2] == 99);
    word(0x68, 0x3B, 20);
    motion_at_tick = tick + 1500; /* Start moving halfway through the averaging window. */
    assert(my_mpu6050_zero_offset() == 1);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.gyro[2] == 99);
    word(0x68, 0x3B, 20);
    fail_dev = 0x68;
    fail_reg = 0x3A; /* Fail after settling, before publishing averages. */
    assert(my_mpu6050_zero_offset() == 1);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.gyro[2] == 99);
    fail_dev = fail_reg = -1;
    my_mpu6050_restore_offset(&zero);
    word(0x68, 0x3B, 0);
    word(0x68, 0x3D, 0);
    word(0x68, 0x3F, 8192);
    word(0x68, 0x43, 0);
    word(0x68, 0x45, 0);
    word(0x68, 0x47, 0);
    puts("PASS: +/-4g calibration keeps 1g, 100Hz averaging, motion/NACK preserve old offsets");
}

static void incremental_calibration_tests(void)
{
    Accel_Gyro_struct raw;
    uint16_t stable, average;
    uint32_t now = UINT32_MAX - 500;
    TickType_t start = tick;
    unsigned bus_reads = reads;
    mpu6050_offset_struct offset;
    const mpu6050_offset_struct zero = {0};
    my_mpu6050_zero_offset_begin(now);
    for (unsigned i = 0; i <= 200; ++i)
    {
        raw = (Accel_Gyro_struct){.Gyro = {66, -33, 99}, .Accel = {20, -10, 8222}};
        my_mpu6050_zero_offset_step(&raw, 1, now);
        now += 10;
    }
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100 && average == 100);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.accel[1] == -10 && offset.accel[2] == 30);
    assert(offset.gyro[0] == 66 && offset.gyro[1] == -33 && offset.gyro[2] == 99);
    assert(raw.Accel.Accel_z == 8192 && raw.Gyro.Gyro_z == 0);
    assert(tick == start && reads == bus_reads); /* 使用现有采样，不额外读I2C或等待。 */
    {
        const uint32_t *diag = my_mpu6050_zero_offset_diag();
        assert(MPU_CAL_DIAG_FIELD_COUNT == 16 && diag[MPU_CAL_DIAG_STATE] == MPU6050_CAL_DONE);
        assert((int32_t)diag[MPU_CAL_DIAG_AY] == -10 && diag[MPU_CAL_DIAG_AZ] == 8222);
        assert(diag[MPU_CAL_DIAG_CHECKED] == 200 && diag[MPU_CAL_DIAG_ACCEPTED] == 200);
        assert(diag[MPU_CAL_DIAG_REJECT_SEEN] == 0 && diag[MPU_CAL_DIAG_STABLE_PEAK] == 100);
        assert(diag[MPU_CAL_DIAG_INTERVAL_MS] == 10 && diag[MPU_CAL_DIAG_LEVEL_LIMIT] == 1228);
    }
    my_mpu6050_zero_offset_begin(0);
    raw = (Accel_Gyro_struct){.Gyro = {0, 0, 0}, .Accel = {0, 0, 8192}};
    for (unsigned i = 0; i <= 100; ++i)
    {
        my_mpu6050_zero_offset_step(&raw, 1, i * 5U);
    }
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(stable == 50 && average == 0); /* 不把5ms轮询计作100Hz的新帧。 */
    assert(my_mpu6050_zero_offset_step(&raw, 1, 3000) == MPU6050_CAL_FAILED);
    my_mpu6050_zero_offset_begin(0);
    for (unsigned i = 0; i <= 100; ++i)
    {
        my_mpu6050_zero_offset_step(&raw, 1, i * 10U);
    }
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_AVERAGING);
    assert(my_mpu6050_zero_offset_step(&raw, 1, 3000) == MPU6050_CAL_FAILED);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.gyro[2] == 99);
    assert(my_mpu6050_zero_offset_diag()[MPU_CAL_DIAG_REJECT] == MPU_CAL_REJECT_AVERAGE_TIMEOUT);
    my_mpu6050_zero_offset_begin(now);
    assert(my_mpu6050_zero_offset_step(NULL, 0, now) == MPU6050_CAL_WAIT_STILL);
    assert(my_mpu6050_zero_offset_step(NULL, 0, now + 3000) == MPU6050_CAL_FAILED);
    my_mpu6050_get_offset(&offset);
    assert(offset.accel[0] == 20 && offset.gyro[2] == 99);
    my_mpu6050_restore_offset(&zero);
    puts("PASS: incremental calibration, wrap, frame cadence, atomic offsets, timeout and no extra I2C/waits");
}

static void calibration_diag_tests(void)
{
    const Accel_struct inputs[] = {{149, 0, 8192}, {0, -149, 8192}, {0, 0, 8341},
                                   {1228, 0, 8192}, {0, -1228, 8192}, {0, 0, 9420}};
    const uint32_t reasons[] = {MPU_CAL_REJECT_DX, MPU_CAL_REJECT_DY, MPU_CAL_REJECT_DZ,
                                MPU_CAL_REJECT_AX, MPU_CAL_REJECT_AY, MPU_CAL_REJECT_AZ};
    const mpu6050_offset_struct zero = {0};
    const uint32_t *diag;
    Accel_Gyro_struct raw;
    TickType_t started = tick;
    unsigned bus_reads = reads;
    unsigned i, sample;
    my_mpu6050_restore_offset(&zero);
    for (i = 0; i < sizeof(inputs) / sizeof(inputs[0]); ++i)
    {
        my_mpu6050_zero_offset_begin(0);
        raw = (Accel_Gyro_struct){.Gyro = {0, 0, 0}, .Accel = {0, 0, 8192}};
        if (i >= 3)
        {
            raw.Accel = inputs[i]; /* 固定水平偏差不混入帧间变化原因。 */
        }
        my_mpu6050_zero_offset_step(&raw, 1, 0);
        raw.Accel = inputs[i];
        my_mpu6050_zero_offset_step(&raw, 1, 10);
        diag = my_mpu6050_zero_offset_diag();
        assert(diag[MPU_CAL_DIAG_REJECT] == reasons[i] && diag[MPU_CAL_DIAG_REJECT_SEEN] == reasons[i]);
        assert(diag[MPU_CAL_DIAG_CHECKED] == 1 && diag[MPU_CAL_DIAG_REJECTED] == 1 && diag[MPU_CAL_DIAG_ACCEPTED] == 0);
        assert((int32_t)diag[MPU_CAL_DIAG_AY] == inputs[i].Accel_y);
        assert(diag[MPU_CAL_DIAG_LEVEL_LIMIT] == 1228 && diag[MPU_CAL_DIAG_STILL_LIMIT] == 149);
        assert(my_mpu6050_zero_offset_step(&raw, 1, 3000) == MPU6050_CAL_FAILED);
        diag = my_mpu6050_zero_offset_diag();
        assert(diag[MPU_CAL_DIAG_REJECT] == MPU_CAL_REJECT_WAIT_TIMEOUT);
        assert(diag[MPU_CAL_DIAG_REJECT_SEEN] == (reasons[i] | MPU_CAL_REJECT_WAIT_TIMEOUT));
    }
    my_mpu6050_zero_offset_begin(0);
    for (sample = 0; sample <= 50; ++sample)
    {
        raw = (Accel_Gyro_struct){.Gyro = {0, 0, 0}, .Accel = {0, 0, 8192}};
        my_mpu6050_zero_offset_step(&raw, 1, sample * 10U);
    }
    raw.Accel.Accel_x = 149;
    my_mpu6050_zero_offset_step(&raw, 1, 510);
    diag = my_mpu6050_zero_offset_diag();
    assert(diag[MPU_CAL_DIAG_STABLE_PEAK] == 50 && diag[MPU_CAL_DIAG_REJECT_SEEN] == MPU_CAL_REJECT_DX);
    assert(diag[MPU_CAL_DIAG_ACCEPTED] == 50 && diag[MPU_CAL_DIAG_REJECTED] == 1);
    my_mpu6050_zero_offset_begin(1000);
    diag = my_mpu6050_zero_offset_diag();
    assert(diag[MPU_CAL_DIAG_STABLE_PEAK] == 0 && diag[MPU_CAL_DIAG_REJECT_SEEN] == 0 && diag[MPU_CAL_DIAG_CHECKED] == 0);
    assert(my_mpu6050_zero_offset_step(NULL, 0, 1000) == MPU6050_CAL_WAIT_STILL);
    assert(my_mpu6050_zero_offset_diag()[MPU_CAL_DIAG_REJECT] == MPU_CAL_REJECT_READ);
    assert(tick == started && reads == bus_reads); /* 诊断自身不增加等待或总线读取。 */
    my_mpu6050_restore_offset(&zero);
    puts("PASS: actual calibration limits, signed inputs, per-axis rejection/timeout reasons, retained stability peak and new-attempt reset");
}

static void calibration_level_tests(void)
{
    const Accel_struct accepted[] = {{400, -300, 8842}, {-400, 300, 7542}, {500, 0, 8177}};
    const Accel_struct rejected[] = {{1638, 0, 8027}, {0, -1638, 8027}, {0, 0, 0}, {0, 0, 9500}};
    const mpu6050_offset_struct zero = {0};
    mpu6050_offset_struct before, after;
    Accel_Gyro_struct raw;
    uint16_t stable, average;
    unsigned sample, pose;

    /* 原99LSB水平门限会拒绝这些固定偏差；放宽后仍完整采满100+100帧。 */
    for (pose = 0; pose < sizeof(accepted) / sizeof(accepted[0]); ++pose)
    {
        my_mpu6050_restore_offset(&zero);
        my_mpu6050_zero_offset_begin(0);
        for (sample = 0; sample <= 200; ++sample)
        {
            raw.Accel = accepted[pose];
            raw.Gyro = (Gyro_struct){66, -33, 99};
            my_mpu6050_zero_offset_step(&raw, 1, sample * 10U);
        }
        my_mpu6050_zero_offset_progress(&stable, &average);
        my_mpu6050_get_offset(&after);
        assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100 && average == 100);
        assert(after.accel[0] == accepted[pose].Accel_x && after.accel[1] == accepted[pose].Accel_y);
        assert(after.accel[2] == accepted[pose].Accel_z - 8192);
        assert(after.gyro[0] == 66 && after.gyro[1] == -33 && after.gyro[2] == 99);
        assert(raw.Accel.Accel_x == 0 && raw.Accel.Accel_y == 0 && raw.Accel.Accel_z == 8192);
    }
    my_mpu6050_get_offset(&before);
    /* 明显倾斜、无重力或Z轴明显偏离1g仍然超时，不提交偏移。 */
    for (pose = 0; pose < sizeof(rejected) / sizeof(rejected[0]); ++pose)
    {
        my_mpu6050_zero_offset_begin(0);
        for (sample = 0; sample <= 300; ++sample)
        {
            raw.Accel = rejected[pose];
            raw.Gyro = (Gyro_struct){66, -33, 99};
            my_mpu6050_zero_offset_step(&raw, 1, sample * 10U);
        }
        my_mpu6050_zero_offset_progress(&stable, &average);
        my_mpu6050_get_offset(&after);
        assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_FAILED && stable == 0 && average == 0);
        assert(memcmp(&before, &after, sizeof(before)) == 0);
    }
    /* 每帧变化0.024g，绝对值仍在新水平容差内；原静止变化门限必须继续拒绝。 */
    my_mpu6050_zero_offset_begin(0);
    for (sample = 0; sample <= 300; ++sample)
    {
        raw = (Accel_Gyro_struct){.Gyro = {66, -33, 99}, .Accel = {0, 0, 8192}};
        if ((sample & 1U) != 0)
        {
            raw.Accel.Accel_x = 200;
        }
        my_mpu6050_zero_offset_step(&raw, 1, sample * 10U);
    }
    my_mpu6050_get_offset(&after);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_FAILED && memcmp(&before, &after, sizeof(before)) == 0);
    my_mpu6050_restore_offset(&zero);
    puts("PASS: relaxed level tolerance accepts static bias/small tilt, preserves 1g/100+100 frames and rejects motion/large tilt");
}

static void calibration_negative_gravity_tests(void)
{
    /* 成品板导出中的各次姿态与抖动范围，仍使用完整100+100帧。 */
    const Accel_struct poses[] = {{399, -782, -8726}, {420, -798, -8712}, {109, -14, -8767}, {1000, -1000, -9000}};
    const mpu6050_offset_struct zero = {0};
    Accel_Gyro_struct raw;
    mpu6050_offset_struct saved, loaded, restored;
    uint16_t stable, average;
    unsigned pose, sample;
    TickType_t started;

    for (pose = 0; pose < sizeof(poses) / sizeof(poses[0]); ++pose)
    {
        my_mpu6050_restore_offset(&zero);
        my_mpu6050_zero_offset_begin(0);
        for (sample = 0; sample <= 200; ++sample)
        {
            raw.Accel = poses[pose];
            raw.Gyro = (Gyro_struct){66, -33, 99};
            /* 平均阶段交替小幅噪声，100帧平均值仍为原姿态。 */
            if (sample > 100U && (sample & 1U) != 0)
            {
                raw.Accel.Accel_x += 20;
                raw.Accel.Accel_y -= 10;
                raw.Accel.Accel_z += 30;
            }
            else if (sample > 100)
            {
                raw.Accel.Accel_x -= 20;
                raw.Accel.Accel_y += 10;
                raw.Accel.Accel_z -= 30;
            }
            my_mpu6050_zero_offset_step(&raw, 1, sample * 10U);
        }
        my_mpu6050_zero_offset_progress(&stable, &average);
        my_mpu6050_get_offset(&saved);
        assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100 && average == 100);
        assert(saved.accel[0] == poses[pose].Accel_x && saved.accel[1] == poses[pose].Accel_y);
        assert(saved.accel[2] == poses[pose].Accel_z + 8192); /* 不把负向重力写成约-2g零偏。 */
        assert(raw.Accel.Accel_z == -8192 - 30 && raw.Gyro.Gyro_z == 0);
        memset(eeprom + 0x60, 0xFF, 64);
        assert(app_imu_store_save(&saved) == APP_IMU_STORE_OK);
        assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&saved, &loaded, sizeof(saved)) == 0);
        my_mpu6050_restore_offset(&loaded);
        word(MPU6050_ADDR, 0x3B, poses[pose].Accel_x);
        word(MPU6050_ADDR, 0x3D, poses[pose].Accel_y);
        word(MPU6050_ADDR, 0x3F, poses[pose].Accel_z);
        word(MPU6050_ADDR, 0x43, 66);
        word(MPU6050_ADDR, 0x45, -33);
        word(MPU6050_ADDR, 0x47, 99);
        assert(my_mpu6050_get_data(&raw) == 0);
        assert(raw.Accel.Accel_x == 0 && raw.Accel.Accel_y == 0 && raw.Accel.Accel_z == -8192);
        my_mpu6050_get_offset(&restored);
        started = tick;
        assert(my_mpu6050_zero_offset() == 0 && tick - started == 2000);
        my_mpu6050_get_offset(&loaded);
        assert(memcmp(&restored, &loaded, sizeof(loaded)) == 0); /* 重复校准不累加重力。 */
    }
    my_mpu6050_get_offset(&saved);
    my_mpu6050_zero_offset_begin(0);
    raw = (Accel_Gyro_struct){.Gyro = {0, 0, 0}, .Accel = {0, 0, -8192}};
    my_mpu6050_zero_offset_step(&raw, 1, 0);
    raw.Accel.Accel_z = 8192;
    my_mpu6050_zero_offset_step(&raw, 1, 10);
    my_mpu6050_zero_offset_step(&raw, 1, 20);
    assert(my_mpu6050_zero_offset_diag()[MPU_CAL_DIAG_REJECT] == MPU_CAL_REJECT_AZ);
    assert(my_mpu6050_zero_offset_step(&raw, 1, 3000) == MPU6050_CAL_FAILED);
    my_mpu6050_get_offset(&loaded);
    assert(memcmp(&saved, &loaded, sizeof(saved)) == 0);
    my_mpu6050_restore_offset(&zero);
    word(MPU6050_ADDR, 0x3B, 0);
    word(MPU6050_ADDR, 0x3D, 0);
    word(MPU6050_ADDR, 0x3F, 8192);
    word(MPU6050_ADDR, 0x43, 0);
    word(MPU6050_ADDR, 0x45, 0);
    word(MPU6050_ADDR, 0x47, 0);
    memset(eeprom, 0xFF, sizeof(eeprom));
    eeprom_writes = 0;
    puts("PASS: measured negative-Z poses/noise calibrate, retain -1g through EEPROM restore/recalibration, reject mid-calibration flips");
}

/* R=Rz(yaw)*Ry(pitch)*Rx(roll), truth fields expressed in the sensor body frame. */
static void truth(float roll_deg, float pitch_deg, float yaw_deg, Accel_Gyro_struct *raw, float mag[3], float gravity[3])
{
    float r = roll_deg / RtA, p = pitch_deg / RtA, y = yaw_deg / RtA;
    float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
    memset(raw, 0, sizeof(*raw));
    gravity[0] = -sp;
    gravity[1] = cp * sr;
    gravity[2] = cp * cr;
    raw->Accel.Accel_x = (int16_t)lroundf(gravity[0] * MPU6050_ACCEL_LSB_PER_G);
    raw->Accel.Accel_y = (int16_t)lroundf(gravity[1] * MPU6050_ACCEL_LSB_PER_G);
    raw->Accel.Accel_z = (int16_t)lroundf(gravity[2] * MPU6050_ACCEL_LSB_PER_G);
    mag[0] = 30 * cy * cp - 40 * sp;
    mag[1] = 30 * (cy * sp * sr - sy * cr) + 40 * cp * sr;
    mag[2] = 30 * (cy * sp * cr + sy * sr) + 40 * cp * cr;
}
static void fusion_startup_tests(void)
{
    static const float poses[][2] = {{180.0f, 0.0f}, {-179.0f, 5.0f}, {35.0f, 25.0f}, {0.0f, 90.0f}, {0.0f, -90.0f}};
    Accel_Gyro_struct raw;
    Euler_struct euler;
    float mag[3], gravity[3], predicted[3];
    uint32_t pose, frame, axis;

    for (pose = 0; pose < sizeof(poses) / sizeof(poses[0]); ++pose)
    {
        Common_IMU_Reset();
        memset(&raw, 0, sizeof(raw));
        Common_IMU_GetEulerAngle(&raw, &euler, 0.01f);
        raw.Accel.Accel_x = 24576; /* 3g冲击不能决定初始重力方向。 */
        Common_IMU_GetEulerAngle(&raw, &euler, 0.01f);
        assert(fabsf(euler.pitch) < 0.01f && fabsf(euler.roll) < 0.01f);
        truth(poses[pose][0], poses[pose][1], 0.0f, &raw, mag, gravity);
        for (frame = 0; frame < 1000; ++frame)
        {
            Common_IMU_GetEulerAngle(&raw, &euler, 0.01f);
            predicted[0] = -sinf(euler.pitch / RtA);
            predicted[1] = cosf(euler.pitch / RtA) * sinf(euler.roll / RtA);
            predicted[2] = cosf(euler.pitch / RtA) * cosf(euler.roll / RtA);
            for (axis = 0; axis < 3; ++axis)
            {
                assert(fabsf(predicted[axis] - gravity[axis]) < 0.001f);
            }
        }
        if (pose == 0)
        {
            assert(angle_error(euler.roll, 180.0f) < 0.01f);
            Common_IMU_Reset(); /* 校准结束后同样需要重新建立倒置姿态。 */
            Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, 0.01f);
            assert(angle_error(euler.roll, 180.0f) < 0.01f);
        }
    }
    puts("PASS: first valid gravity seeds inverted/tilted/vertical poses after reset; zero/3g samples defer seeding.");
}

static void fusion_tests(void)
{
    const float cases[][3] = {{0, 0, 90}, {35, 25, 90}, {-40, 30, -135}, {20, -55, 179}, {0, 0, -179}, {45, 45, 0}};
    Accel_Gyro_struct raw;
    Euler_struct euler = {0};
    float mag[3], gravity[3], without_mag, with_mag, before;
    unsigned c, n;
    fusion_startup_tests();
    for (c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        Common_IMU_Reset();
        truth(cases[c][0], cases[c][1], cases[c][2], &raw, mag, gravity);
        for (n = 0; n < 12000; ++n)
        {
            assert(Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f) == 1);
        }
        assert(angle_error(euler.roll, cases[c][0]) < .3f);
        assert(angle_error(euler.pitch, cases[c][1]) < .3f);
        assert(angle_error(euler.yaw, cases[c][2]) < .3f);
    }
    Common_IMU_Reset();
    truth(0, 0, 0, &raw, mag, gravity);
    raw.Gyro.Gyro_z = (int16_t)lroundf(1.2f * MPU6050_GYRO_LSB_PER_DPS);
    for (n = 0; n < 3000; ++n)
    {
        Common_IMU_GetEulerAngle(&raw, &euler, .01f);
    }
    without_mag = euler.yaw;
    Common_IMU_Reset();
    for (n = 0; n < 3000; ++n)
    {
        Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f);
    }
    with_mag = euler.yaw;
    assert(fabsf(without_mag) > 30 && fabsf(with_mag) < 1);
    before = euler.yaw;
    for (n = 0; n < 100; ++n)
    {
        Common_IMU_GetEulerAngle(&raw, &euler, .01f);
    }
    assert(angle_error(euler.yaw, before + raw.Gyro.Gyro_z * Gyro_G) < .05f); /* Dropout keeps orientation. */
    printf("PASS: tilted poses, bias drift %.2f -> %.2f deg, continuous 6-axis fallback\n", without_mag, with_mag);

    Common_IMU_Reset();
    truth(0, 0, 179, &raw, mag, gravity);
    for (n = 0; n < 1500; ++n)
    {
        Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f);
    }
    before = euler.yaw;
    truth(0, 0, -179, &raw, mag, gravity);
    Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f);
    assert(angle_error(euler.yaw, before) < .1f);
    for (n = 0; n < 1000; ++n)
    {
        Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f);
    }
    assert(angle_error(euler.yaw, -179) < .1f);
    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 50;
    assert(Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f) == 0);
    mag[0] = NAN;
    assert(Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f) == 0);
    memset(&raw, 0, sizeof(raw));
    Common_IMU_GetEulerAngle(&raw, &euler, .01f);
    assert(isfinite(euler.pitch) && isfinite(euler.roll) && isfinite(euler.yaw));
    puts("PASS: +/-180 shortest-path, unobservable/NaN magnetics, zero acceleration");

    /* World-Z turn at a fixed tilt: gyro has X/Y/Z components, not only Gyro_z. */
    Common_IMU_Reset();
    truth(35, 25, 0, &raw, mag, gravity);
    for (n = 0; n < 12000; ++n)
    {
        Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f);
    }
    for (n = 1; n <= 300; ++n)
    {
        truth(35, 25, n * .3f, &raw, mag, gravity);
        raw.Gyro.Gyro_x = (int16_t)lroundf(30 * gravity[0] / Gyro_G);
        raw.Gyro.Gyro_y = (int16_t)lroundf(30 * gravity[1] / Gyro_G);
        raw.Gyro.Gyro_z = (int16_t)lroundf(30 * gravity[2] / Gyro_G);
        Common_IMU_GetEulerAngle(&raw, &euler, .01f);
    }
    assert(angle_error(euler.yaw, 90) < .2f);
    puts("PASS: gyro yaw at tilt uses the full quaternion");

    /* A fast wrist turn: 600deg/s for 150ms -> 90deg, below +/-1000deg/s. */
    Common_IMU_Reset();
    truth(0, 0, 0, &raw, mag, gravity);
    raw.Gyro.Gyro_z = 19680; /* Datasheet 32.8LSB/(deg/s), independent physical expectation. */
    for (n = 0; n < 15; ++n)
    {
        Common_IMU_GetEulerAngle(&raw, &euler, .01f);
    }
    assert(angle_error(euler.yaw, 90) < .2f);
    /* Running impact: 3g along a tilted vector must not be treated as gravity. */
    Common_IMU_Reset();
    memset(&raw, 0, sizeof(raw));
    raw.Accel.Accel_x = 14746;
    raw.Accel.Accel_z = 19661; /* (1.8g,2.4g), magnitude3g. */
    for (n = 0; n < 100; ++n)
    {
        Common_IMU_GetEulerAngle(&raw, &euler, .01f);
    }
    assert(fabsf(euler.pitch) < .01f && fabsf(euler.roll) < .01f);
    puts("PASS: 600deg/s wrist turn and 3g running impact rejection");

    /* User-reported field (30,42,0)uT must be accepted while level. */
    Common_IMU_Reset();
    truth(0, 0, 0, &raw, mag, gravity);
    mag[0] = 30;
    mag[1] = 42;
    mag[2] = 0;
    for (n = 0; n < 1000; ++n)
    {
        assert(Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f) == 1);
    }
    assert(angle_error(euler.yaw, -atan2f(42, 30) * RtA) < .1f);
    printf("PASS: reported field30/42/0uT fuses, yaw converges to %.2fdeg while level\n", euler.yaw);
}

static void service_step(void)
{
    tick += 10;
    App_IMU_Service(1);
    (void)App_IMU_SaveService();
}
static void service_missing_frame_tests(void)
{
    app_imu_struct before, after;
    uint32_t motion_frames;
    mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};
    my_mpu6050_restore_offset(&zero);
    App_IMU_Init();
    fail_dev = LSM303DLHC_MAG_ADDR;
    word(MPU6050_ADDR, 0x3F, 8192);
    word(MPU6050_ADDR, 0x47, 6560); /* 200deg/s；20ms短间隔按实际读帧时间积分，即4度。 */
    service_step();
    assert(App_IMU_GetSnapshot(&before));
    assert(heart_motion_accel[0] == before.raw.Accel.Accel_x && heart_motion_accel[1] == before.raw.Accel.Accel_y &&
           heart_motion_accel[2] == before.raw.Accel.Accel_z);
    motion_frames = heart_motion_frames;
    mpu_auto_ready = 0;
    service_step();
    assert(App_IMU_GetSnapshot(&after));
    assert(after.yaw_deg == before.yaw_deg && memcmp(&after.raw, &before.raw, sizeof(after.raw)) == 0);
    assert(heart_motion_frames == motion_frames);
    mpu_auto_ready = 1;
    service_step();
    assert(App_IMU_GetSnapshot(&after));
    assert(heart_motion_frames == motion_frames + 1);
    assert(angle_error(after.yaw_deg, before.yaw_deg + 4.0f) < 0.1f);
    mpu_auto_ready = 0;
    for (unsigned i = 0; i < 5; ++i)
    {
        service_step();
    }
    assert(!App_IMU_GetSnapshot(&after));
    mpu_auto_ready = 1;
    service_step();
    assert(App_IMU_GetSnapshot(&after));
    fail_dev = -1;
    word(MPU6050_ADDR, 0x47, 0);
    puts("PASS: no DATA_RDY neither publishes nor integrates; fresh frames use read intervals; stale MPU goes offline and recovers.");
}

static void service_tests(uint8_t fail_save)
{
    app_imu_struct imu;
    unsigned n;
    App_IMU_Init();
    magnetic(30, 0, 40);
    service_step();
    service_step();
    service_step();
    assert(App_IMU_GetSnapshot(&imu) && imu.mag_online && imu.mag_fused && !imu.mag_calibrated);
    /* Flat rotation fails Z coverage, but failure alone must not disable raw magnetic fusion. */
    App_IMU_MagCalibrate(1);
    service_step();
    for (n = 0; n < 120; ++n)
    {
        magnetic(30 + 20 * cosf(n * .2f), 42 + 20 * sinf(n * .2f), 0);
        service_step();
    }
    App_IMU_MagCalibrate(0);
    magnetic(30, 42, 0);
    service_step();
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_cal_state == APP_MAG_CAL_FAILED && !imu.mag_calibrated);
    assert(imu.mag_persist_state == APP_MAG_PERSIST_NONE && eeprom_writes == 0);
    assert(imu.mag_cal_span_uT[2] < .1f && imu.mag_online && imu.mag_fused);
    assert(imu.mag_fusion_state == APP_MAG_FUSION_ACTIVE);
    for (n = 0; n < 6; ++n)
    {
        service_step(); /* No new DRDY: expire after50ms. */
    }
    App_IMU_GetSnapshot(&imu);
    assert(!imu.mag_online && !imu.mag_fused);
    assert(imu.mag_fusion_state == APP_MAG_FUSION_NO_DATA);
    magnetic(30, 0, 40);
    for (n = 0; n < 3; ++n)
    {
        service_step();
    }
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_fused);
    fail_dev = 0x1E;
    fail_reg = -1;
    service_step();
    assert(App_IMU_GetSnapshot(&imu) && imu.mag_online); /* One NACK keeps the last frame. */
    service_step();
    service_step();
    assert(App_IMU_GetSnapshot(&imu) && !imu.mag_online && !imu.mag_fused); /* Third NACK in a row stops mag fusion. */
    fail_dev = fail_reg = -1;
    magnetic(30, 0, 40);
    App_IMU_MagCalibrate(1);
    service_step();
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_cal_state == APP_MAG_CAL_RUNNING && !imu.mag_fused);
    assert(imu.mag_fusion_state == APP_MAG_FUSION_CALIBRATING);
    /* Synthetic hard-iron offset (10,-5,7), axis radii (40,50,60). */
    for (n = 0; n < 120; ++n)
    {
        unsigned axis = (n / 2) % 3;
        float sign;
        if ((n & 1) != 0)
        {
            sign = 1.0f;
        }
        else
        {
            sign = -1.0f;
        }
        if (axis == 0)
        {
            magnetic(10 + sign * 40, -5, 7);
        }
        else if (axis == 1)
        {
            magnetic(10, -5 + sign * 50, 7);
        }
        else
        {
            magnetic(10, -5, 7 + sign * 60);
        }
        service_step();
        service_step();
        service_step();
    }
    eeprom_write_protected = fail_save;
    App_IMU_MagCalibrate(0);
    service_step();
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_cal_state == APP_MAG_CAL_DONE && imu.mag_calibrated);
    assert(imu.mag_persist_state == APP_MAG_PERSIST_SAVING);
    for (n = 0; n < 40 && imu.mag_persist_state == APP_MAG_PERSIST_SAVING; ++n)
    {
        service_step();
        App_IMU_GetSnapshot(&imu);
    }
    if (fail_save)
    {
        assert(imu.mag_persist_state == APP_MAG_PERSIST_SAVE_FAILED);
    }
    else
    {
        assert(imu.mag_persist_state == APP_MAG_PERSIST_SAVED);
    }
    assert(imu.mag_cal_samples >= 120);
    assert(imu.mag_cal_span_uT[0] > 79 && imu.mag_cal_span_uT[1] > 99 && imu.mag_cal_span_uT[2] > 119);
    magnetic(50, -5, 7);
    for (n = 0; n < 3; ++n)
    {
        service_step();
    }
    App_IMU_GetSnapshot(&imu);
    assert(fabsf(imu.mag_body_uT[0] - 50) < .1f && fabsf(imu.mag_body_uT[1]) < .1f && fabsf(imu.mag_body_uT[2]) < .1f);
    assert(imu.mag_fusion_state == APP_MAG_FUSION_ACTIVE);
    magnetic(10, -5, 67); /* After installation correction, field is vertical (0,0,50). */
    service_step();
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_online && !imu.mag_fused && imu.mag_fusion_state == APP_MAG_FUSION_HORIZONTAL_WEAK);
    magnetic(50, -5, 7);
    service_step();
    App_IMU_MagCalibrate(1);
    service_step();
    App_IMU_MagCalibrate(0);
    service_step();
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_cal_state == APP_MAG_CAL_FAILED && imu.mag_calibrated);
    if (fail_save)
    {
        assert(imu.mag_persist_state == APP_MAG_PERSIST_SAVE_FAILED);
    }
    else
    {
        assert(imu.mag_persist_state == APP_MAG_PERSIST_SAVED);
    }
    assert(eeprom_writes == 3);
    assert(fabsf(imu.mag_body_uT[0] - 50) < .1f); /* Failed redo keeps previous coefficients. */
    magnetic(150, -5, 7);
    for (n = 0; n < 3; ++n)
    {
        service_step();
    }
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_online && !imu.mag_fused); /* Strong field excluded. */
    assert(imu.mag_fusion_state == APP_MAG_FUSION_FIELD_RANGE);
    fail_dev = MPU6050_ADDR;
    fail_reg = -1;
    for (n = 0; n < 300; ++n)
    {
        service_step();
    }
    assert(!App_IMU_GetSnapshot(&imu)); /* Failure counter cannot wrap into ONLINE. */
    assert(imu.mag_fusion_state == APP_MAG_FUSION_IMU_NO_DATA);
    fail_dev = fail_reg = -1;
    magnetic(50, -5, 7);
    service_step();
    assert(App_IMU_GetSnapshot(&imu));
    puts("PASS: service freshness/NACK, failed flat calibration still fuses raw field, field/horizontal rejection, MPU recovery");
}

static void eeprom_tests(void)
{
    uint8_t data[20], readback[20], previous[96];
    app_mag_calibration_struct a = {{10, -5, 7}, {1.25f, 1.0f, .8333333f}};
    app_mag_calibration_struct b = {{12, -3, 9}, {1.1f, .9f, 1.05f}}, loaded;
    unsigned i, before;
    TickType_t start;
    memset(eeprom, 0xFF, sizeof(eeprom));
    bl24c02_init();
    assert(eeprom_parked != 0 && eeprom_parks == eeprom_resumes);
    for (i = 0; i < sizeof(data); ++i)
    {
        data[i] = (uint8_t)(i + 1);
    }
    before = eeprom_writes;
    assert(bl24c02_write(14, data, sizeof(data)) == 0 && eeprom_writes == before + 3);
    assert(eeprom_parked != 0 && eeprom_parks == eeprom_resumes);
    assert(bl24c02_read(14, readback, sizeof(readback)) == 0 && memcmp(data, readback, sizeof(data)) == 0);
    assert(eeprom_parked != 0 && eeprom_parks == eeprom_resumes);
    assert(bl24c02_write(250, data, 20) == 1 && bl24c02_read(255, readback, 2) == 1);
    assert(bl24c02_write(256, NULL, 0) == 0 && bl24c02_read(0, NULL, 1) == 1);
    fail_dev = BL24C02_ADDR;
    start = tick;
    assert(bl24c02_read(0, readback, 1) == 1 && tick - start == 10);
    assert(eeprom_parked != 0 && eeprom_parks == eeprom_resumes);
    fail_dev = -1;
    memset(eeprom, 0xFF, sizeof(eeprom));
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_EMPTY);
    assert(app_mag_store_save(&a) == APP_MAG_STORE_OK);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    memcpy(previous, eeprom, sizeof(previous));
    eeprom_fail_write_after = 1;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    eeprom_fail_write_after = -1;
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    assert(memcmp(previous, eeprom, 36) == 0); /* Active copy survives a torn target write. */
    assert(app_mag_store_save(&b) == APP_MAG_STORE_OK);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && memcmp(&loaded, &b, sizeof(b)) == 0);
    eeprom[48 + 8] ^= 1; /* Reject latest bad CRC, recover older valid copy. */
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    eeprom_write_protected = 1;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR); /* ACK is not proof of persisted data. */
    eeprom_write_protected = 0;
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    fail_dev = BL24C02_ADDR;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    fail_dev = -1;
    b.scale[0] = NAN;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    for (i = 96; i < sizeof(eeprom); ++i)
    {
        assert(eeprom[i] == 0xFF); /* Other settings region is untouched. */
    }
    memset(eeprom, 0xFF, sizeof(eeprom));
    eeprom_writes = 0;
    puts("PASS: EEPROM page splits/ACK timeout, dual-copy restore, torn write/CRC/WP failures, reserved range");
}

/* 独立按逐位多项式重算测试记录，构造CRC正确但量程/序号变化的数据。 */
static void imu_test_crc(uint8_t *record)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 24; ++i)
    {
        crc ^= record[i];
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            uint32_t low = crc & 1U;
            crc >>= 1;
            if (low != 0)
            {
                crc ^= 0xEDB88320U;
            }
        }
    }
    crc ^= UINT32_MAX;
    for (unsigned i = 0; i < 4; ++i)
    {
        record[24 + i] = (uint8_t)(crc >> (8 * i));
    }
}

static void imu_eeprom_tests(void)
{
    mpu6050_offset_struct a = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct b = {{-21, 11, -31}, {-67, 34, -100}}, loaded, runtime;
    app_mag_calibration_struct mag = {{10, -5, 7}, {1.25f, 1.0f, .8333333f}}, mag_loaded;
    uint8_t preserved_mag[96], preserved_imu[64];
    unsigned writes_before;
    uint16_t stable, average;
    memset(eeprom, 0xFF, sizeof(eeprom));
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_EMPTY);
    assert(app_mag_store_save(&mag) == APP_MAG_STORE_OK);
    memcpy(preserved_mag, eeprom, sizeof(preserved_mag));
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&a, &loaded, sizeof(a)) == 0);
    assert(memcmp(preserved_mag, eeprom, sizeof(preserved_mag)) == 0);
    writes_before = eeprom_writes;
    my_mpu6050_restore_offset(&loaded);
    my_mpu6050_get_offset(&runtime);
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(memcmp(&runtime, &a, sizeof(a)) == 0 && stable == 0 && average == 0);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_RESTORED);
    assert(my_mpu6050_zero_offset_step(NULL, 0, tick) == MPU6050_CAL_RESTORED);
    assert(eeprom_writes == writes_before); /* 恢复/正常采集不重新写EEPROM。 */
    eeprom_fail_write_after = 1;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    eeprom_fail_write_after = -1;
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    assert(app_imu_store_save(&b) == APP_IMU_STORE_OK);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &b, sizeof(b)) == 0);
    eeprom[0x80 + 12] ^= 1;
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    eeprom_write_protected = 1;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    eeprom_write_protected = 0;
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    fail_dev = BL24C02_ADDR;
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_ERROR);
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    fail_dev = -1;
    memcpy(preserved_imu, eeprom + 0x60, sizeof(preserved_imu));
    assert(app_mag_store_save(&mag) == APP_MAG_STORE_OK);
    assert(app_mag_store_load(&mag_loaded) == APP_MAG_STORE_OK && memcmp(&mag_loaded, &mag, sizeof(mag)) == 0);
    assert(memcmp(preserved_imu, eeprom + 0x60, sizeof(preserved_imu)) == 0);
    for (unsigned i = 0xA0; i < sizeof(eeprom); ++i)
    {
        assert(eeprom[i] == 0xFF);
    }
    a = (mpu6050_offset_struct){{INT16_MIN, INT16_MAX, 0}, {-1, 1, -1234}};
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    eeprom[0x60 + 8] ^= 1;
    eeprom[0x80 + 8] ^= 1;
    imu_test_crc(eeprom + 0x60);
    imu_test_crc(eeprom + 0x80);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_EMPTY); /* CRC正确仍拒绝旧量程。 */
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    memset(eeprom + 0x60 + 4, 0xFF, 4); /* 活跃记录序号变为UINT32_MAX。 */
    imu_test_crc(eeprom + 0x60);
    assert(app_imu_store_save(&b) == APP_IMU_STORE_OK);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &b, sizeof(b)) == 0);
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    memset(eeprom + 0x60, 0xFF, 64);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_EMPTY);
    assert(app_imu_store_load(NULL) == APP_IMU_STORE_ERROR && app_imu_store_save(NULL) == APP_IMU_STORE_ERROR);
    memset(&runtime, 0, sizeof(runtime));
    my_mpu6050_restore_offset(&runtime);
    memset(eeprom, 0xFF, sizeof(eeprom));
    eeprom_writes = 0;
    puts("PASS: IMU EEPROM signed offsets, restore without recalibration/write, torn writes, CRC/WP/I2C failure, MAG isolation");
}

static void imu_save_diagnostic_tests(void)
{
    mpu6050_offset_struct a = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct b = {{-21, 11, -31}, {-67, 34, -100}}, loaded;
    const uint32_t stages[] = {APP_IMU_SAVE_READ_SLOT0, APP_IMU_SAVE_READ_SLOT1, APP_IMU_SAVE_VERIFY_READ};
    const uint32_t addresses[] = {0x60, 0x80, 0x80};
    unsigned i;
    memset(eeprom, 0xFF, sizeof(eeprom));
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_DONE && host_imu_save_diag[1] == BL24C02_ERROR_NONE);
    assert(host_imu_save_diag[2] == 0x60 && host_imu_save_diag[3] >= 6);
    for (i = 0; i < 3; ++i)
    {
        eeprom_read_fail_after = (int)i * 2; /* 每份28字节现在分两拍读取。 */
        assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
        assert(host_imu_save_diag[0] == stages[i] && host_imu_save_diag[1] == BL24C02_ERROR_READ_NACK);
        assert(host_imu_save_diag[2] == addresses[i]);
        eeprom_read_fail_after = -1;
        /* 清掉测试中已写成功的目标副本，让下一例仍以a为旧记录。 */
        memset(eeprom + 0x80, 0xFF, 32);
        assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    }
    eeprom_fail_write_after = 1;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_WRITE && host_imu_save_diag[1] == BL24C02_ERROR_WRITE_NACK);
    assert(host_imu_save_diag[2] == 0x90);
    eeprom_fail_write_after = -1;
    eeprom_busy_ms = 20;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_WRITE && host_imu_save_diag[1] == BL24C02_ERROR_READY_TIMEOUT);
    assert(host_imu_save_diag[2] == 0x80 && host_imu_save_diag[3] >= 10);
    tick = eeprom_busy_until;
    eeprom_busy_ms = 3;
    memset(eeprom + 0x80, 0xFF, 32);
    eeprom_write_protected = 1;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_VERIFY_DATA && host_imu_save_diag[1] == BL24C02_ERROR_NONE);
    assert(host_imu_save_diag[2] == 0x80);
    eeprom_write_protected = 0;
    fail_dev = BL24C02_ADDR;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_READ_SLOT0 && host_imu_save_diag[1] == BL24C02_ERROR_READY_TIMEOUT);
    fail_dev = -1;
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK && memcmp(&loaded, &a, sizeof(a)) == 0);
    assert(app_imu_store_save(NULL) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_ARGUMENT && host_imu_save_diag[1] == BL24C02_ERROR_ARGUMENT);
    assert(host_imu_save_diag[2] == UINT32_MAX);
    assert(app_imu_store_save(&b) == APP_IMU_STORE_OK);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_DONE && host_imu_save_diag[1] == BL24C02_ERROR_NONE);
    memset(eeprom, 0xFF, sizeof(eeprom));
    eeprom_writes = 0;
    puts("PASS: EEPROM pre-read/page-write/ready-timeout/readback/WP diagnostics preserve previous offsets and report recovery");
}

static uint8_t checked_recovery_step(uint8_t (*step)(uint32_t));

static void restore_tests(uint8_t fail_load)
{
    app_mag_calibration_struct calibration = {{10, -5, 7}, {1.25f, 1.0f, .8333333f}};
    app_imu_struct imu;
    memset(eeprom, 0xFF, sizeof(eeprom));
    assert(app_mag_store_save(&calibration) == APP_MAG_STORE_OK);
    if (fail_load)
    {
        fail_dev = BL24C02_ADDR;
    }
    App_IMU_Init(); /* Application globals are fresh in this separate process. */
    driver_tests();
    magnetic(50, -5, 7);
    service_step();
    assert(App_IMU_GetSnapshot(&imu));
    if (fail_load)
    {
        assert(!imu.mag_calibrated && imu.mag_persist_state == APP_MAG_PERSIST_LOAD_FAILED);
    }
    else
    {
        assert(imu.mag_calibrated && imu.mag_persist_state == APP_MAG_PERSIST_RESTORED);
        assert(fabsf(imu.mag_body_uT[0] - 50) < .01f && fabsf(imu.mag_body_uT[1]) < .01f);
        assert(imu.mag_fused && imu.mag_cal_samples == 0);
        {
            uint32_t last_sample = tick;
            unsigned saved_writes = eeprom_writes, i;
            for (i = 0; i < 6; ++i)
            {
                service_step(); /* 器件继续ACK，但不再产生DRDY。 */
            }
            App_IMU_GetSnapshot(&imu);
            assert(!imu.mag_online && imu.mag_persist_state == APP_MAG_PERSIST_RESTORED);
            tick = last_sample + 999;
            assert(lsm303dlhc_mag_recover_step(tick) == 0);
            ++tick;
            assert(lsm303dlhc_mag_recover_step(tick) == 2);
            for (i = 0; i < 10 && lsm303dlhc_mag_recover_step(tick) == 2; ++i)
            {
                tick += 10;
            }
            assert(i < 10);
            magnetic(50, -5, 7);
            service_step();
            App_IMU_GetSnapshot(&imu);
            assert(imu.mag_online && imu.mag_fused && imu.mag_persist_state == APP_MAG_PERSIST_RESTORED);
            assert(fabsf(imu.mag_body_uT[0] - 50.0f) < 0.01f && fabsf(imu.mag_body_uT[1]) < 0.01f);
            assert(eeprom_writes == saved_writes);
            /* 新一轮无数据跨tick回绕仍能触发恢复，恢复中每步至多一笔事务。 */
            tick = UINT32_MAX - 100;
            magnetic(50, -5, 7);
            service_step();
            service_step();
            tick += 1000;
            assert(checked_recovery_step(lsm303dlhc_mag_recover_step) == 2);
            for (i = 0; i < 10 && checked_recovery_step(lsm303dlhc_mag_recover_step) == 2; ++i)
            {
                tick += 10;
            }
            assert(i < 10 && eeprom_writes == saved_writes);
            puts("PASS: ACK-without-DRDY triggers bounded MAG recovery, retains restored EEPROM calibration and handles tick wrap.");
        }
    }
    if (fail_load)
    {
        puts("PASS: EEPROM load failure does not claim restored calibration");
    }
    else
    {
        puts("PASS: fresh-boot sensor service restores EEPROM coefficients");
    }
}

static void spl006_init_tests(void)
{
    int failure;
    int32_t pressure = -1;
    unsigned successful_reads, successful_writes;

    assert(spl006_process_pressure(0, 0, &pressure) == 1 && pressure == -1);
    memset(regs[SPL006_ADDR], 0, sizeof(regs[SPL006_ADDR]));
    regs[SPL006_ADDR][SPL06_C00_BYTE2] = 0x18;
    regs[SPL006_ADDR][SPL06_C00_BYTE1] = 0x6A;
    regs[SPL006_ADDR][SPL06_C00_BYTE0] = 0x0F; /* c00=100000, c10=-1. */
    regs[SPL006_ADDR][SPL06_C10_BYTE1] = 0xFF;
    regs[SPL006_ADDR][SPL06_C10_BYTE0] = 0xFF;
    word(SPL006_ADDR, SPL06_C01_BYTE1, -2);

    spl_reads = spl_writes = 0;
    assert(spl006_init() == 0);
    successful_reads = spl_reads;
    successful_writes = spl_writes;
    assert(successful_reads == 19 && successful_writes == 5);
    assert(spl006_process_pressure(0, 0, &pressure) == 0 && pressure == 100000);
    assert(spl006_process_pressure(253952, 253952, &pressure) == 0 && pressure == 99997);

    /* Every read, including both reads of 0x15, must invalidate a failed reinit. */
    for (failure = 0; failure < (int)successful_reads; ++failure)
    {
        spl_fail_read_at = failure;
        spl_reads = spl_writes = 0;
        if (failure == 1 || failure == 2)
        {
            assert(spl006_init() == 0); /* 就绪轮询中的单次NACK被容忍。 */
        }
        else
        {
            assert(spl006_init() == 1);
            pressure = -1;
            assert(spl006_process_pressure(0, 0, &pressure) == 1 && pressure == -1);
        }
        spl_fail_read_at = -1;
        assert(spl006_init() == 0);
    }
    for (failure = 0; failure < (int)successful_writes; ++failure)
    {
        spl_fail_write_at = failure;
        spl_reads = spl_writes = 0;
        assert(spl006_init() == 1);
        pressure = -1;
        assert(spl006_process_pressure(0, 0, &pressure) == 1 && pressure == -1);
        spl_fail_write_at = -1;
        assert(spl006_init() == 0);
    }
    for (failure = 0; failure < 2; ++failure)
    {
        if (failure)
        {
            spl_reset_status = SPL06_SENSOR_RDY;
        }
        else
        {
            spl_reset_status = SPL06_COEF_RDY;
        }
        assert(spl006_init() == 1);
        pressure = -1;
        assert(spl006_process_pressure(0, 0, &pressure) == 1 && pressure == -1);
        spl_reset_status = SPL06_SENSOR_RDY | SPL06_COEF_RDY;
        assert(spl006_init() == 0);
    }
    puts("PASS: SPL006 init read/write/not-ready failures reject compensation, successful retry and signed coefficients");
}

static void spl006_sample_tests(void)
{
    int32_t pressure = 123, temperature = 456;
    unsigned before;
    uint8_t ready;
    const uint16_t expected[] = {0x0101, 0x0100, 0x0001, 0};
    for (ready = 0; ready < 4; ++ready)
    {
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = ready << 4;
        before = spl_reads;
        assert(spl006_read_sample(&pressure, &temperature) == expected[ready]);
        if (ready < 3)
        {
            assert(spl_reads == before + 1 && pressure == 123 && temperature == 456);
        }
        else
        {
            assert(spl_reads == before + 2); /* 原整对读取需要四次事务。 */
        }
    }
    regs[SPL006_ADDR][0] = 0x80;
    regs[SPL006_ADDR][1] = 0;
    regs[SPL006_ADDR][2] = 0;
    regs[SPL006_ADDR][3] = 0x7F;
    regs[SPL006_ADDR][4] = 0xFF;
    regs[SPL006_ADDR][5] = 0xFF;
    assert(spl006_read_sample(&pressure, &temperature) == 0);
    assert(pressure == -8388608 && temperature == 8388607);
    for (ready = 0; ready < 2; ++ready)
    {
        spl_fail_read_at = (int)spl_reads + ready;
        pressure = 123;
        temperature = 456;
        assert(spl006_read_sample(&pressure, &temperature) == 0x0202U);
        assert(pressure == 123 && temperature == 456);
        spl_fail_read_at = -1;
    }
    puts("PASS: SPL006 paired read halves full-sample transactions, handles partial readiness, signed data and failures atomically.");
}

static void baro_service_tests(void)
{
    app_baro_struct sample;
    app_baro_history_struct history;
    App_Baro_Init();
    memset(regs[SPL006_ADDR], 0, 6);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 100000 && isfinite(sample.alt_m));
    app_baro_record_history();
    app_baro_get_history(&history);
    assert(history.sequence == 1 && history.altitude_m[23] == (int16_t)roundf(sample.alt_m));
    assert(history.altitude_m[22] == APP_BARO_HISTORY_NONE);
    app_baro_record_history();
    app_baro_get_history(&history);
    assert(history.sequence == 1);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 100000);
    fail_dev = SPL006_ADDR;
    App_Baro_Service();
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample));
    App_Baro_Service();
    assert(!App_Baro_GetSnapshot(&sample));
    fail_dev = -1;
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 100000);
    {
        uint32_t last = tick;
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = 0;
        tick += SPL006_SAMPLE_MAX_AGE_MS - 1;
        App_Baro_Service();
        assert(App_Baro_GetSnapshot(&sample));
        ++tick;
        App_Baro_Service();
        assert(!App_Baro_GetSnapshot(&sample));
        assert(spl006_recover_step(tick) == 2);
        for (unsigned i = 0; i < 100; ++i)
        {
            tick += 10;
            if (spl006_recover_step(tick) == 0)
            {
                break;
            }
            assert(i < 99);
        }
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
        App_Baro_Service();
        assert(App_Baro_GetSnapshot(&sample));
        tick = UINT32_MAX - 1000;
        App_Baro_Service();
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = 0;
        tick += SPL006_SAMPLE_MAX_AGE_MS;
        App_Baro_Service();
        assert(!App_Baro_GetSnapshot(&sample));
        tick = last;
        assert(spl006_init() == 0);
    }
    puts("PASS: real barometer service preserves compensation, partial-readiness snapshots, three-error offline threshold and recovery.");
    App_Baro_Init();
    tick = UINT32_MAX - 1000;
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    app_baro_record_history();
    tick += APP_BARO_HISTORY_PERIOD_MS - 1;
    app_baro_record_history();
    app_baro_get_history(&history);
    assert(history.sequence == 1);
    ++tick;
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = 0;
    App_Baro_Service();
    app_baro_record_history();
    app_baro_get_history(&history);
    assert(history.sequence == 2 && history.altitude_m[23] == APP_BARO_HISTORY_NONE);
    assert(history.altitude_m[22] != APP_BARO_HISTORY_NONE);
    puts("PASS: background 10min barometer history records once per slot, retains missing slots and survives tick wrap.");
    tick += APP_BARO_HISTORY_PERIOD_MS * 3;
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    app_baro_record_history();
    app_baro_get_history(&history);
    assert(history.altitude_m[23] != APP_BARO_HISTORY_NONE);
    assert(history.altitude_m[22] == APP_BARO_HISTORY_NONE && history.altitude_m[21] == APP_BARO_HISTORY_NONE);
    assert(history.altitude_m[19] != APP_BARO_HISTORY_NONE);
    tick += APP_BARO_HISTORY_PERIOD_MS * 25;
    App_Baro_Service();
    app_baro_record_history();
    app_baro_get_history(&history);
    for (unsigned i = 0; i < 23; ++i)
    {
        assert(history.altitude_m[i] == APP_BARO_HISTORY_NONE);
    }
    assert(history.altitude_m[23] != APP_BARO_HISTORY_NONE);
    puts("PASS: sport barometer pause leaves elapsed slots empty, including pauses longer than the history window.");
}

static void spl006_sleep_recovery_tests(void)
{
    int32_t pressure, temperature;
    uint32_t wrap, cycle, step, transactions;
    uint8_t result;

    for (wrap = 0; wrap < 2; ++wrap)
    {
        tick = 1000;
        if (wrap != 0)
        {
            tick = UINT32_MAX - 1000;
        }
        assert(spl006_init() == 0);
        for (cycle = 0; cycle < 3; ++cycle)
        {
            regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
            assert(spl006_read_sample(&pressure, &temperature) == 0);
            tick += 100;
            assert(spl006_set_sleep(1) == 0);
            tick += 300000;
            assert(spl006_set_sleep(1) == 0); /* 重复standby保留原始睡眠起点。 */
            transactions = bus_transactions;
            assert(checked_recovery_step(spl006_recover_step) == 2 && bus_transactions == transactions);
            tick += 300000;
            assert(spl006_set_sleep(0) == 0);
            assert(checked_recovery_step(spl006_recover_step) == 0);
            tick += 100;
            assert(spl006_set_sleep(0) == 0);
            assert(checked_recovery_step(spl006_recover_step) == 0);
        }

        regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
        assert(spl006_read_sample(&pressure, &temperature) == 0);
        assert(spl006_set_sleep(1) == 0);
        for (cycle = 0; cycle < 2; ++cycle)
        {
            tick += 600000;
            assert(spl006_set_sleep(0) == 0);
            for (step = 0; step < 16; ++step)
            {
                tick += 100;
                result = checked_recovery_step(spl006_recover_step);
                if (cycle == 0 || step < 13)
                {
                    assert(result == 0);
                }
                else
                {
                    assert(result == 2); /* 活动时间累计达到3000ms，不能被每轮唤醒清零。 */
                }
            }
            assert(spl006_read_sample(&pressure, &temperature) != 0);
            assert(spl006_set_sleep(1) == 0);
        }
        /* 恢复跨窗口保留系数读取进度；standby与唤醒本身都不能提前开测。 */
        result = 2;
        for (cycle = 0; cycle < 3 && result != 0; ++cycle)
        {
            transactions = bus_transactions;
            tick += 600000;
            assert(checked_recovery_step(spl006_recover_step) == 2 && bus_transactions == transactions);
            assert((regs[SPL006_ADDR][SPL06_MEAS_CFG] & 7) == 0);
            assert(spl006_set_sleep(0) == 0 && bus_transactions == transactions);
            for (step = 0; step < 16 && result != 0; ++step)
            {
                tick += 100;
                result = checked_recovery_step(spl006_recover_step);
                assert(result != 1);
            }
            if (result != 0)
            {
                assert(spl006_set_sleep(1) == 0);
            }
        }
        assert(result == 0 && (regs[SPL006_ADDR][SPL06_MEAS_CFG] & 7) == SPL06_MEAS_BG_PRSTMP);
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
        assert(spl006_read_sample(&pressure, &temperature) == 0);

        for (cycle = 0; cycle < 3; ++cycle)
        {
            assert(spl006_set_sleep(1) == 0);
            tick += 600000;
            assert(spl006_set_sleep(0) == 0);
            spl_fail_read_at = (int)spl_reads;
            assert(spl006_read_sample(&pressure, &temperature) == 0x0202);
            spl_fail_read_at = -1;
            tick += 100;
            assert(checked_recovery_step(spl006_recover_step) == 0);
        }
        assert(spl006_set_sleep(1) == 0);
        tick += 600000;
        assert(spl006_set_sleep(0) == 0);
        tick += 899;
        assert(checked_recovery_step(spl006_recover_step) == 0);
        ++tick;
        assert(checked_recovery_step(spl006_recover_step) == 2); /* 第三次NACK后的活动时间满1s。 */
        for (step = 0; step < 40; ++step)
        {
            tick += 100;
            result = checked_recovery_step(spl006_recover_step);
            if (result == 0)
            {
                break;
            }
            assert(result == 2);
        }
        assert(result == 0);
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
        assert(spl006_read_sample(&pressure, &temperature) == 0);
        tick += 1000;
        assert(checked_recovery_step(spl006_recover_step) == 0);
    }
    puts("PASS: SPL006 preserves no-data/NACK across 1.6s windows, excludes repeated 10min standby/tick wrap and pauses/resumes recovery without early conversions.");
}

static void mpu_init_error_tests(void)
{
    Accel_Gyro_struct raw, before;
    unsigned writes, read_count;
    int failure;
    mpu_reads = mpu_writes = 0;
    assert(my_mpu6050_init() == 0);
    writes = mpu_writes;
    read_count = mpu_reads;
    assert(writes == 12 && read_count == 3);
    memset(&raw, 0x5A, sizeof(raw));
    before = raw;
    for (failure = 0; failure < (int)writes; ++failure)
    {
        mpu_writes = 0;
        mpu_fail_write_at = failure;
        assert(my_mpu6050_init() == 1);
        mpu_fail_write_at = -1;
        assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &before, sizeof(raw)) == 0);
        assert(my_mpu6050_init() == 0);
    }
    for (failure = 1; failure < (int)read_count; ++failure)
    {
        mpu_reads = 0;
        mpu_fail_read_at = failure;
        if (failure == 1)
        {
            assert(my_mpu6050_init() == 0); /* 复位窗NACK后下一拍继续读。 */
        }
        else
        {
            assert(my_mpu6050_init() == 1);
            assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &before, sizeof(raw)) == 0);
        }
        mpu_fail_read_at = -1;
        assert(my_mpu6050_init() == 0);
    }
    puts("PASS: every MPU config write and final status read reports failure; incomplete reinit cannot publish an ACK-only frame.");
}

static void init_delay_tests(void)
{
    const int states[] = {taskSCHEDULER_NOT_STARTED, taskSCHEDULER_RUNNING, taskSCHEDULER_SUSPENDED};
    unsigned i;
    TickType_t started;

    for (i = 0; i < sizeof(states) / sizeof(states[0]); ++i)
    {
        scheduler_state = states[i];
        hal_delay_calls = task_delay_calls = 0;
        started = tick;
        assert(spl006_init() == 0);
        assert(tick - started >= 70);
        fail_dev = MPU6050_ADDR;
        fail_reg = MPU6050_WHO_AM_I;
        started = tick;
        assert(my_mpu6050_init() == 1);
        assert(tick - started >= 950 && tick - started <= 969);
        fail_dev = fail_reg = -1;
        if (scheduler_state == taskSCHEDULER_RUNNING)
        {
            assert(task_delay_calls == 21 && hal_delay_calls == 0);
        }
        else
        {
            assert(hal_delay_calls == 21 && task_delay_calls == 0);
        }
        assert(my_mpu6050_init() == 0);
    }
    scheduler_state = taskSCHEDULER_RUNNING;
    puts("PASS: SPL006/MPU initialization yields only with a running scheduler, preserves minimum waits and bounded retries");
}

static uint8_t checked_recovery_step(uint8_t (*step)(uint32_t))
{
    unsigned transactions = bus_transactions;
    unsigned task_waits = task_delay_calls, hal_waits = hal_delay_calls;
    TickType_t started = tick;
    uint8_t result = step(HAL_GetTick());
    assert(bus_transactions - transactions <= 1);
    assert(task_delay_calls == task_waits && hal_delay_calls == hal_waits);
    assert((uint32_t)(tick - started) == (bus_transactions - transactions) * transfer_ms);
    return result;
}

static void recovery_tests(void)
{
    uint8_t (*const steps[])(uint32_t) = {my_mpu6050_recover_step, spl006_recover_step, lsm303dlhc_mag_recover_step};
    uint8_t (*const inits[])(void) = {my_mpu6050_init, spl006_init, lsm303dlhc_mag_init};
    const int devices[] = {MPU6050_ADDR, SPL006_ADDR, LSM303DLHC_MAG_ADDR};
    const mpu6050_offset_struct saved = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct after;
    Accel_Gyro_struct raw, sentinel;
    lsm303dlhc_mag_struct mag, previous;
    uint8_t results[3] = {2, 2, 2};
    unsigned lap, device;
    int32_t pressure;
    tick = UINT32_MAX - 100;
    transfer_ms = 2;
    check_recovery_waits = 1;
    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x68;
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    regs[SPL006_ADDR][SPL06_C00_BYTE2] = 0x18;
    regs[SPL006_ADDR][SPL06_C00_BYTE1] = 0x6A;
    regs[SPL006_ADDR][SPL06_C00_BYTE0] = 0;
    my_mpu6050_restore_offset(&saved);
    my_mpu6050_recover_begin();
    spl006_recover_begin();
    lsm303dlhc_mag_recover_begin();
    memset(&raw, 0x5A, sizeof(raw));
    sentinel = raw;
    memset(&mag, 0x5A, sizeof(mag));
    previous = mag;
    for (lap = 0; lap < 80; ++lap)
    {
        if (lap == 2)
        {
            mpu_fail_read_at = (int)mpu_reads; /* RESET轮询恰好一次NACK。 */
        }
        for (device = 0; device < 3; ++device)
        {
            results[device] = checked_recovery_step(steps[device]);
            assert(results[device] != 1);
        }
        mpu_fail_read_at = -1;
        if (results[0] == 2)
        {
            assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &sentinel, sizeof(raw)) == 0);
        }
        if (results[1] == 2)
        {
            pressure = -123;
            assert(spl006_process_pressure(0, 0, &pressure) == 1 && pressure == -123);
        }
        if (results[2] == 2)
        {
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_ERROR && memcmp(&mag, &previous, sizeof(mag)) == 0);
        }
        tick += 10; /* 三状态机共同推进；真实任务随后还会走完其他服务和progress。 */
    }
    assert(results[0] == 0 && results[1] == 0 && results[2] == 0);
    assert(spl006_process_pressure(0, 0, &pressure) == 0 && pressure == 100000);
    /* 每个器件启动失败后，1s冷却结束才重试；无需重启MCU或调用阻塞init。 */
    for (device = 0; device < 3; ++device)
    {
        fail_dev = devices[device];
        assert(inits[device]() == 1);
        fail_dev = -1;
        tick += 999;
        assert(checked_recovery_step(steps[device]) == 1);
        ++tick;
        assert(checked_recovery_step(steps[device]) == 2);
        for (lap = 0; lap < 100 && checked_recovery_step(steps[device]) == 2; ++lap)
        {
            tick += 10;
        }
        assert(lap < 100 && checked_recovery_step(steps[device]) == 0);
    }
    /* 长期运行I2C故障触发MPU重配置；短暂失败仍可沿原采样路径自行恢复。 */
    fail_dev = MPU6050_ADDR;
    for (lap = 0; lap < 3; ++lap)
    {
        assert(my_mpu6050_get_data(&raw) == 1);
    }
    tick += 999;
    assert(checked_recovery_step(my_mpu6050_recover_step) == 0);
    ++tick;
    assert(checked_recovery_step(my_mpu6050_recover_step) == 2);
    for (lap = 0; lap < 130; ++lap)
    {
        checked_recovery_step(my_mpu6050_recover_step);
        tick += 10;
    }
    fail_dev = -1;
    for (lap = 0; lap < 150; ++lap)
    {
        checked_recovery_step(my_mpu6050_recover_step);
        tick += 10;
    }
    assert(checked_recovery_step(my_mpu6050_recover_step) == 0);
    my_mpu6050_get_offset(&after);
    assert(memcmp(&saved, &after, sizeof(saved)) == 0);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_RESTORED);
    puts("PASS: real recovery steps use at most one transaction/no waits, tolerate reset NACK, retry failed boot/runtime faults, preserve offsets and survive tick wrap.");
}

static void mpu_stale_recovery_tests(void)
{
    Accel_Gyro_struct raw;
    mpu6050_offset_struct saved = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct after;
    uint32_t start, writes_before;
    uint32_t i;
    for (i = 0; i < 2; ++i)
    {
        assert(my_mpu6050_init() == 0);
        my_mpu6050_restore_offset(&saved);
        mpu_auto_ready = 1;
        assert(my_mpu6050_get_data(&raw) == 0);
        if (i != 0)
        {
            tick = UINT32_MAX - 500;
            assert(my_mpu6050_get_data(&raw) == 0);
        }
        start = tick;
        mpu_auto_ready = 0;
        regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] = 0x40;
        writes_before = mpu_writes;
        tick = start + 999;
        assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY);
        assert(checked_recovery_step(my_mpu6050_recover_step) == 0);
        ++tick;
        assert(checked_recovery_step(my_mpu6050_recover_step) == 2);
        while (checked_recovery_step(my_mpu6050_recover_step) == 2)
        {
            tick += 10;
            assert((uint32_t)(tick - start) < 2000);
        }
        assert(mpu_writes > writes_before);
        mpu_auto_ready = 1;
        assert(my_mpu6050_get_data(&raw) == 0);
        tick += 999;
        assert(checked_recovery_step(my_mpu6050_recover_step) == 0);
        assert(my_mpu6050_get_data(&raw) == 0);
        tick += 999;
        assert(checked_recovery_step(my_mpu6050_recover_step) == 0);
        my_mpu6050_get_offset(&after);
        assert(memcmp(&saved, &after, sizeof(saved)) == 0);
    }
    puts("PASS: ACK without DATA_RDY recovers at 1s, preserves offsets, renews freshness and handles tick wrap.");
}

static void mpu_sleep_recovery_tests(void)
{
    Accel_Gyro_struct raw;
    const mpu6050_offset_struct saved = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct after;
    uint32_t wrap, fault, cycle, step, started;
    uint8_t result;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        for (fault = 0; fault < 2; ++fault)
        {
            fail_reg = -1;
            assert(my_mpu6050_init() == 0);
            my_mpu6050_restore_offset(&saved);
            tick = 1000;
            if (wrap != 0)
            {
                tick = UINT32_MAX - 7500;
            }
            mpu_auto_ready = 1;
            assert(my_mpu6050_get_data(&raw) == 0);
            for (cycle = 0; cycle < 5; ++cycle)
            {
                assert(my_mpu6050_set_sleep(1) == 0);
                tick += 800;
                assert(my_mpu6050_set_sleep(0) == 0);
                tick += 200;
                assert(checked_recovery_step(my_mpu6050_recover_step) == 0);
                assert(my_mpu6050_get_data(&raw) == 0);
            }
            mpu_auto_ready = 0;
            regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 0;
            if (fault != 0)
            {
                fail_reg = MPU6050_INT_STATUS;
            }
            result = 0;
            for (cycle = 0; cycle < 12 && result == 0; ++cycle)
            {
                assert(my_mpu6050_set_sleep(1) == 0);
                tick += 800;
                assert(my_mpu6050_set_sleep(0) == 0);
                for (step = 0; step < 3; ++step)
                {
                    result = checked_recovery_step(my_mpu6050_recover_step);
                    if (result != 0)
                    {
                        break;
                    }
                    if (step < 2)
                    {
                        tick += 100;
                    }
                }
                if (result == 0)
                {
                    assert(my_mpu6050_get_data(&raw) != 0);
                }
            }
            assert(result == 2 && cycle >= 5 && cycle <= 9);
            /* 恢复期间任务保持唤醒，器件重新应答后应完整走完复位和配置。 */
            fail_reg = -1;
            started = tick;
            do
            {
                tick += 100;
                result = checked_recovery_step(my_mpu6050_recover_step);
                assert((uint32_t)(tick - started) < 2500);
            } while (result == 2);
            assert(result == 0);
            tick += 200;
            mpu_auto_ready = 1;
            assert(my_mpu6050_get_data(&raw) == 0);
            my_mpu6050_get_offset(&after);
            assert(memcmp(&saved, &after, sizeof(saved)) == 0);
        }
    }
    puts("PASS: sleep excludes normal idle time but retains no-data/read failures; recovery restores frames and offsets across tick wrap.");
}

static void mag_sleep_recovery_tests(void)
{
    lsm303dlhc_mag_struct mag;
    uint32_t wrap, fault, cycle, step, writes_before;
    uint8_t result;

    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        for (fault = 0; fault < 2; ++fault)
        {
            fail_dev = fail_reg = -1;
            tick = 1000;
            if (wrap != 0)
            {
                tick = UINT32_MAX - 12500;
            }
            assert(lsm303dlhc_mag_init() == 0);
            magnetic(30.0f, 5.0f, 20.0f);
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
            for (cycle = 0; cycle < 5; ++cycle)
            {
                assert(lsm303dlhc_mag_set_sleep(1) == 0);
                tick += 800;
                assert(lsm303dlhc_mag_set_sleep(0) == 0);
                assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
                tick += 200;
                assert(checked_recovery_step(lsm303dlhc_mag_recover_step) == 0);
                magnetic(30.0f, 5.0f, 20.0f);
                assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
            }
            /* 等待首帧时长睡眠也不计入故障时间；正常新帧重新开始计时。 */
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
            assert(lsm303dlhc_mag_set_sleep(1) == 0);
            tick += 5000;
            assert(lsm303dlhc_mag_set_sleep(0) == 0);
            assert(checked_recovery_step(lsm303dlhc_mag_recover_step) == 0);
            magnetic(30.0f, 5.0f, 20.0f);
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);

            if (fault != 0)
            {
                fail_dev = LSM303DLHC_MAG_ADDR;
                fail_reg = 9; /* 仅状态读取NACK，休眠/唤醒写入仍应成功。 */
                for (step = 0; step < 3; ++step)
                {
                    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_ERROR);
                }
            }
            else
            {
                assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
            }
            for (cycle = 0; cycle < 5; ++cycle)
            {
                assert(lsm303dlhc_mag_set_sleep(1) == 0);
                tick += 400;
                assert(lsm303dlhc_mag_set_sleep(1) == 0); /* 重复请求不重置睡眠起点。 */
                tick += 400;
                assert(lsm303dlhc_mag_set_sleep(0) == 0);
                assert(checked_recovery_step(lsm303dlhc_mag_recover_step) == 0);
                tick += 100;
                assert(lsm303dlhc_mag_set_sleep(0) == 0); /* 重复唤醒不得再次补偿时间。 */
                tick += 99;
                assert(lsm303dlhc_mag_read(&mag) != LSM303DLHC_MAG_OK);
                assert(checked_recovery_step(lsm303dlhc_mag_recover_step) == 0);
                ++tick;
                result = checked_recovery_step(lsm303dlhc_mag_recover_step);
                if (cycle < 4)
                {
                    assert(result == 0);
                }
                else
                {
                    assert(result == 2); /* 累计活动999ms不恢复，1000ms必须开始恢复。 */
                }
            }
            fail_dev = fail_reg = -1;
            writes_before = mag_config_writes;
            for (step = 0; step < 4; ++step)
            {
                result = checked_recovery_step(lsm303dlhc_mag_recover_step);
            }
            assert(result == 0 && mag_config_writes == writes_before + 3);
            magnetic(30.0f, 5.0f, 20.0f);
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
            fail_dev = LSM303DLHC_MAG_ADDR;
            fail_reg = 9;
            for (step = 0; step < 3; ++step)
            {
                assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_ERROR);
            }
            fail_dev = fail_reg = -1;
            magnetic(30.0f, 5.0f, 20.0f);
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
            tick += 1500;
            assert(checked_recovery_step(lsm303dlhc_mag_recover_step) == 0);
        }
    }
    puts("PASS: MAG sleep pauses fault time, retains no-DRDY/NACK across cycles, recovers at 1000ms active time, handles repeated requests/tick wrap and clears faults on a fresh frame.");
}

static void polling_tests(void)
{
    Accel_Gyro_struct raw, before;
    const mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};
    const mpu6050_offset_struct offset = {{20, -10, 30}, {17, 33, -99}};
    app_imu_struct snapshot, previous;
    uint32_t started, motion_before, mag_reads;
    unsigned i, transactions;

    tick = 1000;
    memset(eeprom, 0xFF, sizeof(eeprom));
    assert(my_mpu6050_init() == 0);
    assert(regs[MPU6050_ADDR][MPU6050_INT_ENABLE] == 0x01);
    assert(regs[MPU6050_ADDR][MPU6050_FIFO_EN] == 0);
    assert(regs[MPU6050_ADDR][MPU6050_USER_CTRL] == 0);
    my_mpu6050_restore_offset(&zero);
    mpu_auto_ready = 0;
    memset(&raw, 0x5A, sizeof(raw));
    before = raw;
    transactions = bus_transactions;
    assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY);
    assert(bus_transactions == transactions + 1U && memcmp(&raw, &before, sizeof(raw)) == 0);

    /* 多次更新寄存器后只读取最后一帧，不消费任何历史队列。 */
    for (i = 1; i <= 15; ++i)
    {
        word(MPU6050_ADDR, 0x3B, (int16_t)i);
        regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    }
    word(MPU6050_ADDR, 0x3D, -456);
    word(MPU6050_ADDR, 0x3F, 8192);
    word(MPU6050_ADDR, 0x41, 12345); /* 温度字节不应混入角速度。 */
    word(MPU6050_ADDR, 0x43, -17);
    word(MPU6050_ADDR, 0x45, 33);
    word(MPU6050_ADDR, 0x47, -3280);
    transactions = bus_transactions;
    started = tick;
    transfer_ms = 3;
    assert(my_mpu6050_get_data(&raw) == 0);
    assert(bus_transactions == transactions + 1 && tick == started + 3);
    assert(my_mpu6050_sample_ms() == started);
    assert(raw.Accel.Accel_x == 15 && raw.Accel.Accel_y == -456 && raw.Accel.Accel_z == 8192);
    assert(raw.Gyro.Gyro_x == -17 && raw.Gyro.Gyro_y == 33 && raw.Gyro.Gyro_z == -3280);
    before = raw;
    assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY);
    assert(memcmp(&raw, &before, sizeof(raw)) == 0 && my_mpu6050_sample_ms() == started);
    fail_dev = MPU6050_ADDR;
    fail_reg = MPU6050_INT_STATUS;
    assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &before, sizeof(raw)) == 0);
    assert(my_mpu6050_sample_ms() == started);
    fail_dev = fail_reg = -1;
    transfer_ms = 0;
    my_mpu6050_restore_offset(&offset);
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    assert(my_mpu6050_get_data(&raw) == 0);
    assert(raw.Accel.Accel_x == -5 && raw.Accel.Accel_y == -446 && raw.Accel.Accel_z == 8162);
    assert(raw.Gyro.Gyro_x == -34 && raw.Gyro.Gyro_y == 0 && raw.Gyro.Gyro_z == -3181);

    my_mpu6050_restore_offset(&zero);
    App_IMU_Init();
    fail_dev = LSM303DLHC_MAG_ADDR;
    tick = 5000;
    word(MPU6050_ADDR, 0x3B, 0);
    word(MPU6050_ADDR, 0x3D, 0);
    word(MPU6050_ADDR, 0x43, 0);
    word(MPU6050_ADDR, 0x45, 0);
    word(MPU6050_ADDR, 0x47, 3280); /* 恒定100deg/s，用于验证短间隔的时间积分。 */
    motion_before = heart_motion_frames;
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&previous) && heart_motion_frames == motion_before + 1U);
    assert(angle_error(previous.yaw_deg, 1.0f) < 0.05f && heart_motion_ms == tick);
    tick += 20;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) && heart_motion_frames == motion_before + 1U);
    assert(snapshot.yaw_deg == previous.yaw_deg);
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) && heart_motion_frames == motion_before + 2U);
    assert(angle_error(snapshot.yaw_deg, previous.yaw_deg + 2.0f) < 0.05f);
    previous = snapshot;
    tick += 150;
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) && heart_motion_frames == motion_before + 3U);
    assert(angle_error(snapshot.yaw_deg, previous.yaw_deg + 1.0f) < 0.05f); /* 长断采不外推150ms。 */
    tick = UINT32_MAX - 4;
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&previous));
    tick += 20;
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 1;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot));
    assert(angle_error(snapshot.yaw_deg, previous.yaw_deg + 2.0f) < 0.05f);

    fail_dev = -1;
    mpu_auto_ready = 1;
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    assert(lsm303dlhc_mag_init() == 0);
    magnetic(30, 0, 40);
    tick += 10;
    App_IMU_Service(1);
    mag_reads = mag_frames;
    magnetic(30, 0, 40);
    tick += 5;
    App_IMU_Service(1);
    assert(mag_frames == mag_reads);
    tick += 4;
    App_IMU_Service(1);
    assert(mag_frames == mag_reads);
    ++tick;
    App_IMU_Service(1);
    assert(mag_frames == mag_reads + 1);
    puts("PASS: polling uses one transaction/latest frame, DRDY, signed axes/temperature skip, offsets, partial-read rejection, variable dt, long-gap restart, MAG cadence and tick wrap.");
}

static void async_save_tests(void)
{
    const mpu6050_offset_struct first = {{20, -10, 30}, {66, -33, 99}};
    const mpu6050_offset_struct second = {{-21, 11, -31}, {-67, 34, -100}};
    mpu6050_offset_struct input, loaded, zero = {{0, 0, 0}, {0, 0, 0}};
    app_imu_store_result result;
    app_imu_struct sample;
    unsigned int lap, before, waits, failure;
    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x68;
    assert(my_mpu6050_init() == 0);
    my_mpu6050_restore_offset(&zero);
    word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 8192);
    App_IMU_Init();
    assert(app_imu_store_save(&first) == APP_IMU_STORE_OK);
    for (failure = 0; failure < 3; ++failure)
    {
        input = second;
        eeprom_fail_write_after = -1;
        if (failure < 2)
        {
            eeprom_fail_write_after = (int)failure;
        }
        tick = UINT32_MAX - 55; /* 保存跨HAL tick回绕。 */
        eeprom_busy_until = tick;
        before = bus_transactions;
        assert(app_imu_store_save_begin(&input) == APP_IMU_STORE_BUSY);
        assert(bus_transactions == before);
        memset(&input, 0, sizeof(input)); /* begin之后调用方可重用输入缓冲。 */
        waits = task_delay_calls;
        eeprom_max_transfer = 0;
        result = APP_IMU_STORE_BUSY;
        for (lap = 0; lap < 80 && result == APP_IMU_STORE_BUSY; ++lap)
        {
            tick += 10;
            App_IMU_Service(1);
            assert(App_IMU_GetSnapshot(&sample));
            before = bus_transactions;
            result = app_imu_store_save_step();
            assert(bus_transactions - before <= 1 && task_delay_calls == waits);
        }
        assert(lap < 80 && eeprom_max_transfer <= 16);
        eeprom_fail_write_after = -1;
        assert(app_imu_store_load(&loaded) == APP_IMU_STORE_OK);
        if (failure < 2)
        {
            assert(result == APP_IMU_STORE_ERROR && memcmp(&loaded, &first, sizeof(first)) == 0);
            memset(eeprom + 0x80, 0xFF, 32);
        }
        else
        {
            assert(result == APP_IMU_STORE_OK && memcmp(&loaded, &second, sizeof(second)) == 0);
        }
    }
    puts("PASS: async save uses at most one 16-byte transaction/no waits per step; sampling continues, torn pages preserve the old copy, begin copies input and time wraps.");
}

static void euler_decimation_tests(void)
{
    Accel_Gyro_struct raw = {{0, 0, 6560}, {0, 0, 8192}};
    Euler_struct full, reduced;
    unsigned int frame;
    Common_IMU_Reset();
    for (frame = 0; frame < 100; ++frame)
    {
        Common_IMU_GetEulerAngleWithMag(&raw, NULL, &full, 0.01f);
    }
    Common_IMU_Reset();
    for (frame = 0; frame < 100; ++frame)
    {
        Euler_struct *output = NULL;
        if (frame % 2 != 0)
        {
            output = &reduced;
        }
        Common_IMU_GetEulerAngleWithMag(&raw, NULL, output, 0.01f);
    }
    assert(angle_error(full.yaw, reduced.yaw) < 0.001f);
    assert(fabsf(full.pitch - reduced.pitch) < 0.001f && fabsf(full.roll - reduced.roll) < 0.001f);
    puts("PASS: 50Hz Euler output preserves every 100Hz quaternion integration step.");
}

static void mag_schedule_tests(uint8_t defer_ppg, uint8_t wrap)
{
    app_imu_struct snapshot;
    const mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};
    uint32_t started, generated = 0, next_generated;
    unsigned i, before, accepted = 0;
    uint8_t read_mag;

    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x68;
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    assert(my_mpu6050_init() == 0 && lsm303dlhc_mag_init() == 0);
    my_mpu6050_restore_offset(&zero);
    App_IMU_Init();
    assert(app_imu_set_sample_period(10) == 10 && regs[0x1E][0] == 0x18);
    word(MPU6050_ADDR, 0x3F, 8192);
    tick = 1000;
    if (wrap != 0)
    {
        tick = UINT32_MAX - 1000;
    }
    started = tick;
    for (i = 1; i <= 300; ++i)
    {
        tick = started + i * 10;
        next_generated = i * 10 * 750 / 10000;
        if (next_generated != generated)
        {
            /* 自主75Hz转换，静止时每次新转换的三轴值可以完全相同。 */
            magnetic(30.0f, -10.0f, 40.0f);
            generated = next_generated;
        }
        read_mag = defer_ppg == 0 || i % 3 != 0;
        before = mag_frames;
        App_IMU_Service(read_mag);
        accepted += mag_frames - before;
        assert(App_IMU_GetSnapshot(&snapshot) != 0);
        if (read_mag == 0)
        {
            assert(mag_frames == before);
        }
        if (i > 2)
        {
            assert(snapshot.mag_online != 0 && snapshot.mag_fused != 0);
        }
    }
    assert(generated == 225);
    if (defer_ppg != 0)
    {
        assert(accepted == 174); /* 特定相位下跳过被覆盖的磁帧，不伪造75Hz采集。 */
    }
    else
    {
        assert(accepted == 225); /* 300次轮询不能重复计成300个新磁样本。 */
    }
    App_IMU_MagCalibrate(1);
    tick += 10;
    magnetic(30.0f, -10.0f, 40.0f);
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) != 0 && snapshot.mag_cal_samples == 1);
    tick += 10;
    App_IMU_Service(1); /* 没有新转换，DRDY清除后不能增加校准计数。 */
    assert(App_IMU_GetSnapshot(&snapshot) != 0 && snapshot.mag_cal_samples == 1);
    regs[0x1E][9] = 3;
    tick += 10;
    App_IMU_Service(1); /* LOCK+DRDY只排空，不能增加计数或刷新年龄。 */
    assert(App_IMU_GetSnapshot(&snapshot) != 0 && snapshot.mag_cal_samples == 1);
    for (i = 0; i < 6; ++i)
    {
        tick += 10;
        magnetic(30.0f, -10.0f, 40.0f);
        before = bus_transactions;
        App_IMU_Service(0);
        assert(bus_transactions == before + 1); /* 只保留MPU事务。 */
    }
    assert(App_IMU_GetSnapshot(&snapshot) != 0 && snapshot.mag_online == 0 && snapshot.mag_fused == 0);
    assert(snapshot.mag_cal_samples == 1);
    tick += 10;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) != 0 && snapshot.mag_online != 0 && snapshot.mag_cal_samples == 2);
    printf("PASS: 75Hz conversion model: %u accepted / 225 generated, PPG deferral=%u; no duplicate/LOCK calibration, age expires and wrap=%u.\n",
           accepted, defer_ppg, wrap);
}

static void power_profile_tests(void)
{
    Accel_Gyro_struct raw;
    app_imu_struct snapshot;
    uint16_t stable, average;
    unsigned i, writes;
    const mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};

    tick = 1000;
    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    assert(my_mpu6050_init() == 0 && lsm303dlhc_mag_init() == 0);
    App_IMU_Init();
    assert(app_imu_set_sample_period(20) == 20);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 19 && regs[MPU6050_ADDR][MPU6050_CONFIG] == 4);
    assert(regs[0x1E][0x00] == 0x14);
    writes = mpu_writes;
    assert(app_imu_set_sample_period(20) == 20 && mpu_writes == writes);
    my_mpu6050_restore_offset(&zero);
    my_mpu6050_zero_offset_begin(tick);
    for (i = 0; i <= 200; ++i)
    {
        raw = (Accel_Gyro_struct){.Gyro = {66, -33, 99}, .Accel = {20, -10, 8222}};
        my_mpu6050_zero_offset_step(&raw, 1, tick);
        tick += 20;
    }
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100 && average == 100);
    my_mpu6050_restore_offset(&zero);
    word(MPU6050_ADDR, 0x3F, 8192);
    assert(app_imu_set_sample_period(40) == 40);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 39 && regs[MPU6050_ADDR][MPU6050_CONFIG] == 5);
    assert(regs[0x1E][0x00] == 0x0C);
    for (i = 0; i < 10; ++i)
    {
        magnetic(30.0f, 42.0f, 0.0f);
        tick += 40;
        App_IMU_Service(1);
        assert(App_IMU_GetSnapshot(&snapshot) != 0);
        assert(heart_motion_ms == tick);
    }
    assert(spl006_init() == 0);
    assert(app_imu_set_sample_period(10) == 10);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 9 && regs[MPU6050_ADDR][MPU6050_CONFIG] == MPU6050_DLPF_CFG);
    assert(regs[0x1E][0x00] == 0x18);
    writes = mpu_writes;
    assert(app_imu_set_sample_period(10) == 10 && mpu_writes == writes);
    assert(app_imu_set_sample_period(40) == 40);
    app_imu_set_sleep(1);
    assert(spl006_set_sleep(1) == 0);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x41);
    assert(regs[0x1E][0x02] == 0x03 && (regs[SPL006_ADDR][SPL06_MEAS_CFG] & 7) == 0);
    tick += 300000;
    app_imu_set_sleep(0);
    assert(spl006_set_sleep(0) == 0);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x01);
    assert(regs[0x1E][0x02] == 0 && (regs[SPL006_ADDR][SPL06_MEAS_CFG] & 7) == 7);
    assert(my_mpu6050_recover_step(tick) == 0 && lsm303dlhc_mag_recover_step(tick) == 0 && spl006_recover_step(tick) == 0);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 39 && regs[0x1E][0x00] == 0x0C);
    for (i = 0; i < 500; ++i)
    {
        magnetic(30.0f, 42.0f, 0.0f);
        tick += 40;
        App_IMU_Service(1);
    }
    assert(App_IMU_GetSnapshot(&snapshot) != 0);
    assert(angle_error(snapshot.yaw_deg, -54.4623f) < 0.5f);
    for (i = 0; i < 5; ++i)
    {
        app_imu_set_sleep(1);
        tick += 800;
        app_imu_set_sleep(0);
        tick += 200;
        magnetic(30.0f, 42.0f, 0.0f);
        App_IMU_Service(1);
        assert(App_IMU_GetSnapshot(&snapshot) != 0);
        assert(angle_error(snapshot.yaw_deg, -54.4623f) < 0.5f);
    }
    puts("PASS: five 1Hz sleep/wake snapshots preserve settled magnetic heading instead of restarting at zero.");
    assert(app_imu_is_calibrating() == 0);
    App_IMU_MagCalibrate(1);
    assert(app_imu_is_calibrating() != 0);
    assert(app_imu_set_sample_period(40) == 20);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 19 && regs[0x1E][0x00] == 0x14);
    puts("PASS: 50/25Hz rate registers, no repeated reconfiguration, 50Hz 100+100-frame calibration, 25Hz fusion and pending magnetic calibration.");
}

static void uptime_finish(void)
{
    unsigned steps = 0, transactions;
    uint8_t busy;
    do
    {
        tick += 20;
        transactions = bus_transactions;
        busy = app_uptime_service(0); /* 校准请求到来后，已开始的事务仍必须收尾。 */
        assert(eeprom_parked != 0 && bus_transactions - transactions <= 1);
        assert(++steps < 20);
    } while (busy != 0);
}

static void uptime_tests(void)
{
    uint32_t saved, expected;
    uint8_t calibration[0xA0];
    unsigned writes;
    memset(eeprom, 0xFF, sizeof(eeprom));
    memset(eeprom, 0x5A, sizeof(calibration));
    memcpy(calibration, eeprom, sizeof(calibration));
    tick = 1234; /* 包括任务初始化前的开机时间。 */
    app_uptime_init();
    assert(!app_uptime_get_saved(&saved));
    assert(!app_uptime_get_previous(&saved));
    tick = 599999;
    assert(!app_uptime_service(1) && eeprom_writes == 0);
    tick = 600000;
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == 600U && eeprom_writes == 1U);
    assert(memcmp(eeprom + 0xA0, "UPT1", 4) == 0 && eeprom[0xB0] == 0xFF);
    tick = 1200000;
    assert(!app_uptime_service(0) && eeprom_writes == 1);
    tick += 2000;
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == 1202U && eeprom_writes == 2U);
    assert(!app_uptime_get_previous(&saved));
    assert(memcmp(calibration, eeprom, sizeof(calibration)) == 0);

    tick = eeprom_busy_until = 0;
    app_uptime_init();
    assert(app_uptime_get_saved(&saved) && saved == 1202U);
    assert(app_uptime_get_previous(&saved) && saved == 1202U);
    tick = 600000;
    eeprom_fail_write_after = 0;
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == 1202U);
    writes = eeprom_writes;
    tick += 1000;
    assert(!app_uptime_service(1) && eeprom_writes == writes);
    eeprom_fail_write_after = -1;
    tick = 1200000;
    eeprom_write_protected = 1;
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == 1202U);
    eeprom_write_protected = 0;
    tick = 1800000;
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == 1800U); /* 不叠加上次开机的1202秒。 */
    assert(app_uptime_get_previous(&saved) && saved == 1202U);
    eeprom[0xA8] ^= 1U; /* 模拟断电/坏记录，重启仍可回退到另一份。 */
    tick = eeprom_busy_until = 0;
    app_uptime_init();
    assert(app_uptime_get_saved(&saved) && saved == 1202U);

    eeprom_read_fail_after = 0;
    app_uptime_init();
    assert(!app_uptime_get_previous(&saved));
    eeprom_read_fail_after = -1;
    tick = 600000;
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == 600U);
    assert(app_uptime_get_previous(&saved) && saved == 1202U);

    tick = UINT32_MAX - 100;
    eeprom_busy_until = tick;
    app_uptime_init();
    assert(app_uptime_service(1));
    uptime_finish();
    tick += 600000; /* HAL tick越界；时间仍按差值递增。 */
    expected = (uint32_t)(((uint64_t)UINT32_MAX - 100 + (uint32_t)(tick - (UINT32_MAX - 100))) / 1000);
    assert(app_uptime_service(1));
    uptime_finish();
    assert(app_uptime_get_saved(&saved) && saved == expected);
    assert(memcmp(calibration, eeprom, sizeof(calibration)) == 0);
    assert(!app_uptime_get_saved(NULL) && eeprom_parked != 0);
    puts("PASS: 10min uptime records, calibration deferral/isolation, reboot reset, CRC fallback, write/WP/read failure, GPIO parking and tick wrap.");
}

static void activity_sample(common_activity_struct *state, float ax, float ay, float az, uint32_t *now)
{
    common_activity_update(state, ax, ay, az, *now);
    *now += 200;
}

static void activity_tests(void)
{
    common_activity_struct state = {0}, wrapped = {0};
    app_imu_struct snapshot;
    Accel_Gyro_struct raw, previous;
    static const float walking[] = {1.25f, 1.0f, 0.75f, 1.0f};
    uint32_t now = 0, wrap = UINT32_MAX - 1000;
    unsigned i, sample, before, heart_before, writes;
    float yaw;
    for (i = 0; i < 100; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.steps == 0 && state.wrist_sequence == 0);
    activity_sample(&state, 0.0f, 0.0f, -1.25f, &now);
    activity_sample(&state, 0.0f, 0.0f, -0.75f, &now);
    for (i = 0; i < 15; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.steps == 0);
    memset(&state, 0, sizeof(state));
    activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    activity_sample(&wrapped, 0.0f, 0.0f, -1.0f, &wrap);
    for (i = 0; i < 20; ++i)
    {
        for (sample = 0; sample < 4; ++sample)
        {
            activity_sample(&state, 0.0f, -walking[sample], 0.0f, &now);
            activity_sample(&wrapped, 0.0f, -walking[sample], 0.0f, &wrap);
        }
    }
    assert(state.steps == 20 && wrapped.steps == state.steps);
    before = state.steps;
    for (i = 0; i < 100; ++i)
    {
        common_activity_update(&state, 0.0f, 0.0f, -1.5f, state.sample_ms);
    }
    assert(state.steps == before);
    for (i = 0; i < 3; ++i)
    {
        activity_sample(&state, 0.0f, -1.0f, 0.0f, &now);
    }
    for (i = 0; i < 12; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.wrist_sequence == 1 && state.steps == before);
    for (i = 0; i < 4; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, 1.0f, &now);
    }
    assert(state.wrist_sequence == 1);
    for (i = 0; i < 3; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.wrist_sequence == 2);
    common_activity_update(&state, 0.0f, 0.0f, -1.0f, now + 10000);
    assert(state.steps == before && state.wrist_sequence == 2 && state.wrist_armed == 0);
    puts("PASS: stationary and isolated impacts do not count; 20 walking cycles count 20 across tick wrap; fresh down-to-view poses emit one raise and rearm.");

    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    assert(my_mpu6050_init() == 0 && lsm303dlhc_mag_init() == 0);
    App_IMU_Init();
    word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 0);
    word(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, 0);
    word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, -8192);
    word(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 2000);
    assert(app_imu_set_sample_period(20) == 20);
    App_IMU_Service(1);
    App_IMU_GetSnapshot(&snapshot);
    yaw = snapshot.yaw_deg;
    heart_before = heart_motion_frames;
    assert(app_imu_set_sample_period(200) == 200);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x28);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 0x47);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 3);
    writes = bus_transactions;
    assert(app_imu_set_sample_period(200) == 200 && bus_transactions == writes);
    tick += 200;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) == 0);
    assert(snapshot.raw.Gyro.Gyro_x == 0 && snapshot.yaw_deg == yaw && snapshot.mag_online == 0);
    assert(heart_motion_frames == heart_before);
    for (i = 0; i < 20; ++i)
    {
        for (sample = 0; sample < 4; ++sample)
        {
            word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, (int16_t)(-8192.0f * walking[sample]));
            tick += 200;
            App_IMU_Service(1);
        }
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.steps == 20);
    mpu_auto_ready = 0;
    memset(&raw, 0x35, sizeof(raw));
    previous = raw;
    assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY);
    assert(memcmp(&raw, &previous, sizeof(raw)) == 0);
    mpu_auto_ready = 1;
    assert(app_imu_set_sample_period(20) == 20);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 1 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 0);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 19);
    tick += 100;
    App_IMU_Service(1);
    assert(App_IMU_GetSnapshot(&snapshot) != 0 && snapshot.steps == 20);
    assert(heart_motion_frames == heart_before + 1);
    for (i = 0; i < 3; ++i)
    {
        assert(my_mpu6050_init() == 0);
        mpu_fail_write_at = (int)mpu_writes + (int)i;
        assert(my_mpu6050_set_sample_period(200) != 0);
        previous = raw;
        assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &previous, sizeof(raw)) == 0);
        mpu_fail_write_at = -1;
    }
    puts("PASS: real 5Hz cycle registers, accelerometer-only burst, magnetic sleep, no fusion/heart reference in low power, full-mode restore, retained steps and partial-mode failures.");
}

static void accel_reference_tests(void)
{
    const mpu6050_offset_struct offset = {{11, -22, 33}, {4, -5, 6}};
    mpu6050_offset_struct restored;
    app_imu_struct snapshot, previous;
    Accel_Gyro_struct raw, untouched;
    unsigned i, transactions, frames;
    uint32_t started;
    uint8_t recovery;

    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    assert(my_mpu6050_init() == 0 && lsm303dlhc_mag_init() == 0);
    App_IMU_Init();
    my_mpu6050_restore_offset(&offset);
    word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 111);
    word(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, -222);
    word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, -8192);
    word(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 3280);
    assert(app_imu_set_sample_period(20) == 20);
    App_IMU_Service(0);
    assert(App_IMU_GetSnapshot(&previous) != 0);

    app_imu_set_accel_reference(1);
    assert(app_imu_set_sample_period(10) == 10);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x08);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 0x07);
    assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 9);
    assert(regs[MPU6050_ADDR][MPU6050_CONFIG] == MPU6050_DLPF_CFG);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 3);
    frames = heart_motion_frames;
    for (i = 0; i < 600; ++i)
    {
        transactions = bus_transactions;
        assert(app_imu_set_sample_period(10) == 10 && bus_transactions == transactions);
        tick += 10;
        App_IMU_Service(1);
        assert(bus_transactions == transactions + 1 && mpu_last_frame_size == 7);
        assert(heart_motion_ms == tick && heart_motion_frames == frames + i + 1);
        assert(heart_motion_accel[0] == 100 && heart_motion_accel[1] == -200 && heart_motion_accel[2] == -8225);
    }
    assert(App_IMU_GetSnapshot(&snapshot) == 0);
    assert(snapshot.raw.Gyro.Gyro_x == 0 && snapshot.raw.Gyro.Gyro_y == 0 && snapshot.raw.Gyro.Gyro_z == 0);
    assert(snapshot.pitch_deg == previous.pitch_deg && snapshot.roll_deg == previous.roll_deg && snapshot.yaw_deg == previous.yaw_deg);
    assert(snapshot.mag_online == 0 && snapshot.mag_fused == 0);
    my_mpu6050_get_offset(&restored);
    assert(memcmp(&offset, &restored, sizeof(offset)) == 0);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_RESTORED);

    mpu_auto_ready = 0;
    frames = heart_motion_frames;
    tick += 10;
    App_IMU_Service(1);
    assert(heart_motion_frames == frames);
    memset(&raw, 0x35, sizeof(raw));
    untouched = raw;
    assert(my_mpu6050_get_data(&raw) == MPU6050_DATA_NOT_READY && memcmp(&raw, &untouched, sizeof(raw)) == 0);
    mpu_auto_ready = 1;
    assert(my_mpu6050_set_sleep(1) == 0 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x48);
    tick += 2000;
    assert(my_mpu6050_set_sleep(0) == 0 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x08);

    /* 同一10ms周期恢复六轴时也必须重新启用陀螺仪和磁力计。 */
    app_imu_set_accel_reference(0);
    assert(app_imu_set_sample_period(10) == 10);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 1 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 0);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    tick += 100;
    App_IMU_Service(0);
    assert(mpu_last_frame_size == 15 && App_IMU_GetSnapshot(&snapshot) != 0);
    assert(snapshot.raw.Gyro.Gyro_z == 3274);
    assert(fabsf(snapshot.yaw_deg - previous.yaw_deg) < 2.0f);

    app_imu_set_accel_reference(1);
    assert(app_imu_set_sample_period(10) == 10);
    assert(app_imu_set_sample_period(200) == 200);
    frames = heart_motion_frames;
    tick += 200;
    App_IMU_Service(1);
    assert(heart_motion_frames == frames && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x28);
    assert(app_imu_set_sample_period(10) == 10);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x08 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 7);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 3);

    /* 每一笔模式配置失败都不能发布半配置数据，恢复后重新应用加速度模式。 */
    for (i = 0; i < 5; ++i)
    {
        assert(my_mpu6050_init() == 0);
        mpu_fail_write_at = (int)mpu_writes + (int)i;
        assert(app_imu_set_sample_period(10) == 20);
        assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &untouched, sizeof(raw)) == 0);
        mpu_fail_write_at = -1;
        started = tick;
        do
        {
            recovery = checked_recovery_step(my_mpu6050_recover_step);
            tick += 10;
            assert((uint32_t)(tick - started) < 2000);
        } while (recovery != 0);
        assert(app_imu_set_sample_period(10) == 10);
        assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x08 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 7);
        assert(regs[MPU6050_ADDR][MPU6050_SMPLRT_DIV] == 9 && regs[LSM303DLHC_MAG_ADDR][0x02] == 3);
    }

    app_imu_set_accel_reference(0);
    assert(app_imu_set_sample_period(20) == 20);
    fail_dev = LSM303DLHC_MAG_ADDR;
    fail_reg = 0x02;
    app_imu_set_accel_reference(1);
    assert(app_imu_set_sample_period(10) == 10 && regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    fail_dev = fail_reg = -1;
    assert(app_imu_set_sample_period(10) == 10 && regs[LSM303DLHC_MAG_ADDR][0x02] == 3);
    transactions = bus_transactions;
    assert(app_imu_set_sample_period(10) == 10 && bus_transactions == transactions);

    App_IMU_MagCalibrate(1);
    assert(app_imu_set_sample_period(10) == 20);
    assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 1 && regs[MPU6050_ADDR][MPU6050_PWR_MGMT_2] == 0);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    puts("PASS: 100Hz accel-only PPG reference preserves fresh calibrated acceleration, sleeps gyro/temp/MAG, skips fusion, restores full/5Hz modes and retries partial configuration/recovery failures.");
}

static void background_mag_sleep_retry_tests(void)
{
    uint32_t i;
    unsigned transactions;
    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    assert(my_mpu6050_init() == 0 && lsm303dlhc_mag_init() == 0);
    App_IMU_Init();
    assert(app_imu_set_sample_period(20) == 20);
    fail_dev = LSM303DLHC_MAG_ADDR;
    fail_reg = 0x02;
    assert(app_imu_set_sample_period(200) == 200);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    for (i = 0; i < 4; ++i)
    {
        tick += 200;
        assert(app_imu_set_sample_period(200) == 200);
        App_IMU_Service(1);
        assert(regs[MPU6050_ADDR][MPU6050_PWR_MGMT_1] == 0x28);
    }
    fail_dev = fail_reg = -1;
    tick += 200;
    assert(app_imu_set_sample_period(200) == 200);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 3);
    transactions = bus_transactions;
    assert(app_imu_set_sample_period(200) == 200 && bus_transactions == transactions);

    /* 回到姿态档仍由原恢复流程重配磁力计；旧休眠请求不得在活动档重新生效。 */
    assert(app_imu_set_sample_period(20) == 20);
    for (i = 0; i < 4; ++i)
    {
        tick += 20;
        (void)lsm303dlhc_mag_recover_step(tick);
        assert(app_imu_set_sample_period(20) == 20);
    }
    assert(lsm303dlhc_mag_set_rate(300) == 0);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    fail_dev = LSM303DLHC_MAG_ADDR;
    fail_reg = 0x02;
    assert(app_imu_set_sample_period(200) == 200);
    fail_dev = fail_reg = -1;
    assert(app_imu_set_sample_period(20) == 20);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    puts("PASS: failed magnetic sleep retries in the same background mode, stops writing after success and follows a changed active target.");
}

int main(int argc, char **argv)
{
    if (argc > 1 && strncmp(argv[1], "mag_schedule", 12) == 0)
    {
        mag_schedule_tests(strcmp(argv[1], "mag_schedule") != 0, strcmp(argv[1], "mag_schedule_wrap") == 0);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "background_mag_sleep_retry") == 0)
    {
        background_mag_sleep_retry_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "activity") == 0)
    {
        activity_tests();
        accel_reference_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "mag_sleep_recovery") == 0)
    {
        mag_sleep_recovery_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "mpu_sleep_recovery") == 0)
    {
        mpu_sleep_recovery_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "power_profile") == 0)
    {
        power_profile_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "uptime") == 0)
    {
        uptime_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "async_save") == 0)
    {
        async_save_tests();
        euler_decimation_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "polling") == 0)
    {
        polling_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "mpu_stale") == 0)
    {
        mpu_stale_recovery_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "recovery") == 0)
    {
        recovery_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "save_failed") != 0)
    {
        restore_tests(strcmp(argv[1], "load_failed") == 0);
        return 0;
    }
    spl006_init_tests();
    spl006_sample_tests();
    baro_service_tests();
    spl006_sleep_recovery_tests();
    mpu_init_error_tests();
    init_delay_tests();
    eeprom_tests();
    imu_eeprom_tests();
    imu_save_diagnostic_tests();
    driver_tests();
    missing_frame_tests();
    calibration_tests();
    incremental_calibration_tests();
    calibration_diag_tests();
    calibration_level_tests();
    calibration_negative_gravity_tests();
    fusion_tests();
    service_tests(argc > 1);
    service_missing_frame_tests();
    return 0;
}
