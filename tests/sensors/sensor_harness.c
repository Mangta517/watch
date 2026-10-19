/* Real driver/service/math, fake bus. Inject EXTI while an I2C transaction is active. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_imu.h"
#include "app_heart.h"
#include "common_imu.h"
#include "my_i2c.h"
#include "spl006.h"
#include "app_baro.h"
#include "semphr.h"
#include "bl24c02.h"
#include "app_mag_store.h"
#include "app_imu_store.h"
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
static int scheduler_state = taskSCHEDULER_RUNNING;
static unsigned hal_delay_calls, task_delay_calls;
static uint32_t primask;
static uint8_t bus_active, mutex_taken;
static int fail_dev = -1, fail_reg = -1;
static unsigned reads, mag_frames, mag_config_writes;
static int spl_fail_read_at = -1, spl_fail_write_at = -1;
static unsigned spl_reads, spl_writes;
static unsigned mpu_reads, mpu_writes;
static int mpu_fail_read_at = -1, mpu_fail_write_at = -1;
static uint8_t spl_reset_status = SPL06_SENSOR_RDY | SPL06_COEF_RDY;
static TickType_t motion_at_tick;
static uint8_t eeprom[BL24C02_CAPACITY_BYTES];
static uint8_t eeprom_write_protected;
static TickType_t eeprom_busy_until;
static int eeprom_fail_write_after = -1;
static unsigned eeprom_writes;
static int eeprom_read_fail_after = -1;
static TickType_t eeprom_busy_ms = 3U;
extern uint32_t host_imu_save_diag[4];
static void word(uint8_t dev, uint8_t reg, int16_t value);
static uint32_t heart_motion_frames;
static int16_t heart_motion_accel[3];

void app_heart_motion_update(int16_t accel_x, int16_t accel_y, int16_t accel_z, uint32_t sample_ms)
{
    heart_motion_accel[0] = accel_x;
    heart_motion_accel[1] = accel_y;
    heart_motion_accel[2] = accel_z;
    assert(sample_ms == HAL_GetTick());
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

static void bus_enter(my_i2c_bus_t bus)
{
    assert((bus == MY_I2C_BUS_SENSOR || bus == MY_I2C_BUS_EEPROM) && !bus_active);
    assert(!primask && !mutex_taken); /* No whole-transfer IRQ mask or snapshot mutex. */
    bus_active = 1;
    HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
    assert(bus_active && !primask); /* ISR cannot nest an I2C transaction. */
}
void my_i2c_init(my_i2c_bus_t bus)
{
    assert(bus == MY_I2C_BUS_EEPROM);
}
uint8_t my_i2c_probe(my_i2c_bus_t bus, uint8_t dev)
{
    bus_enter(bus);
    assert(bus == MY_I2C_BUS_EEPROM && dev == BL24C02_ADDR);
    bus_active = 0;
    return fail_dev == dev || tick < eeprom_busy_until;
}
uint8_t my_i2c_read_bytes(my_i2c_bus_t bus, uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t error;
    bus_enter(bus);
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
        assert(tick >= eeprom_busy_until);
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
            assert(len == 15 || len == 1);
            if (len == 15 && mpu_auto_ready != 0)
            {
                data[0] |= 1U; /* 默认模拟持续100Hz输出；新鲜度用例可暂停转换。 */
            }
            regs[dev][0x3A] = 0;
        }
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
        assert((unsigned)reg + len <= sizeof(eeprom) && tick >= eeprom_busy_until);
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
        memcpy(&regs[dev][reg], data, len);
        if (dev == MPU6050_ADDR && reg == MPU6050_PWR_MGMT_1 && data[0] == 0x80)
        {
            regs[dev][reg] = 0x40; /* Device reset completes into SLEEP. */
        }
        if (dev == SPL006_ADDR && reg == SPL06_RESET)
        {
            regs[dev][SPL06_MEAS_CFG] = spl_reset_status;
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
    for (i = 0; i <= 202U; ++i)
    {
        raw = (Accel_Gyro_struct){.Gyro = {66, -33, 99}, .Accel = {20, -10, 8222}};
        my_mpu6050_zero_offset_step(&raw, i != 50U && i != 150U, i * 10U);
    }
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100U && average == 100U);
    my_mpu6050_zero_offset_begin(UINT32_MAX - 1000U);
    assert(my_mpu6050_zero_offset_step(NULL, 0, UINT32_MAX - 999U) == MPU6050_CAL_WAIT_STILL);
    assert(my_mpu6050_zero_offset_step(NULL, 0, 1999U) == MPU6050_CAL_FAILED);
    my_mpu6050_restore_offset(&zero);
    puts("PASS: DATA_RDY gates publication; missing calibration frames keep the wall-clock timeout and valid sample counts.");
}

static void calibration_tests(void)
{
    TickType_t start;
    Accel_Gyro_struct raw;
    extern int16_t offset_accel_x, offset_accel_y, offset_accel_z;
    extern int16_t offset_gyro_x, offset_gyro_y, offset_gyro_z;
    word(0x68, 0x3B, 20);
    word(0x68, 0x3D, -10);
    word(0x68, 0x3F, 8192 + 30);
    word(0x68, 0x43, 66);
    word(0x68, 0x45, -33);
    word(0x68, 0x47, 99);
    start = tick;
    assert(my_mpu6050_zero_offset() == 0 && tick - start == 2000);
    assert(offset_accel_x == 20 && offset_accel_y == -10 && offset_accel_z == 30);
    assert(offset_gyro_x == 66 && offset_gyro_y == -33 && offset_gyro_z == 99);
    assert(my_mpu6050_get_data(&raw) == 0);
    assert(raw.Accel.Accel_z == 8192 && raw.Gyro.Gyro_z == 0);
    assert(my_mpu6050_zero_offset() == 0);
    assert(offset_accel_x == 20 && offset_accel_y == -10 && offset_accel_z == 30);
    assert(offset_gyro_x == 66 && offset_gyro_y == -33 && offset_gyro_z == 99);
    word(0x68, 0x3B, 2000);
    start = tick;
    assert(my_mpu6050_zero_offset() == 1 && tick - start == 3000);
    assert(offset_accel_x == 20 && offset_gyro_z == 99);
    word(0x68, 0x3B, 20);
    motion_at_tick = tick + 1500; /* Start moving halfway through the averaging window. */
    assert(my_mpu6050_zero_offset() == 1);
    assert(offset_accel_x == 20 && offset_gyro_z == 99);
    word(0x68, 0x3B, 20);
    fail_dev = 0x68;
    fail_reg = 0x3A; /* Fail after settling, before publishing averages. */
    assert(my_mpu6050_zero_offset() == 1);
    assert(offset_accel_x == 20 && offset_gyro_z == 99);
    fail_dev = fail_reg = -1;
    offset_accel_x = offset_accel_y = offset_accel_z = 0;
    offset_gyro_x = offset_gyro_y = offset_gyro_z = 0;
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
    uint32_t now = UINT32_MAX - 500U;
    TickType_t start = tick;
    unsigned bus_reads = reads;
    extern int16_t offset_accel_x, offset_accel_y, offset_accel_z;
    extern int16_t offset_gyro_x, offset_gyro_y, offset_gyro_z;
    my_mpu6050_zero_offset_begin(now);
    for (unsigned i = 0; i <= 200; ++i)
    {
        raw = (Accel_Gyro_struct){.Gyro = {66, -33, 99}, .Accel = {20, -10, 8222}};
        my_mpu6050_zero_offset_step(&raw, 1, now);
        now += 10;
    }
    my_mpu6050_zero_offset_progress(&stable, &average);
    assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100 && average == 100);
    assert(offset_accel_x == 20 && offset_accel_y == -10 && offset_accel_z == 30);
    assert(offset_gyro_x == 66 && offset_gyro_y == -33 && offset_gyro_z == 99);
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
    assert(offset_accel_x == 20 && offset_gyro_z == 99);
    assert(my_mpu6050_zero_offset_diag()[MPU_CAL_DIAG_REJECT] == MPU_CAL_REJECT_AVERAGE_TIMEOUT);
    my_mpu6050_zero_offset_begin(now);
    assert(my_mpu6050_zero_offset_step(NULL, 0, now) == MPU6050_CAL_WAIT_STILL);
    assert(my_mpu6050_zero_offset_step(NULL, 0, now + 3000U) == MPU6050_CAL_FAILED);
    assert(offset_accel_x == 20 && offset_gyro_z == 99);
    offset_accel_x = offset_accel_y = offset_accel_z = 0;
    offset_gyro_x = offset_gyro_y = offset_gyro_z = 0;
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
        if (i >= 3U)
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
    for (sample = 0; sample <= 50U; ++sample)
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
        for (sample = 0; sample <= 200U; ++sample)
        {
            raw.Accel = accepted[pose];
            raw.Gyro = (Gyro_struct){66, -33, 99};
            my_mpu6050_zero_offset_step(&raw, 1, sample * 10U);
        }
        my_mpu6050_zero_offset_progress(&stable, &average);
        my_mpu6050_get_offset(&after);
        assert(my_mpu6050_zero_offset_state() == MPU6050_CAL_DONE && stable == 100U && average == 100U);
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
        for (sample = 0; sample <= 300U; ++sample)
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
    for (sample = 0; sample <= 300U; ++sample)
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
        for (sample = 0; sample <= 200U; ++sample)
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
            else if (sample > 100U)
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
        for (frame = 0; frame < 1000U; ++frame)
        {
            Common_IMU_GetEulerAngle(&raw, &euler, 0.01f);
            predicted[0] = -sinf(euler.pitch / RtA);
            predicted[1] = cosf(euler.pitch / RtA) * sinf(euler.roll / RtA);
            predicted[2] = cosf(euler.pitch / RtA) * cosf(euler.roll / RtA);
            for (axis = 0; axis < 3U; ++axis)
            {
                assert(fabsf(predicted[axis] - gravity[axis]) < 0.001f);
            }
        }
        if (pose == 0U)
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
    App_IMU_Service();
}
static void service_missing_frame_tests(void)
{
    app_imu_struct before, after;
    uint32_t motion_frames;
    mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};
    my_mpu6050_restore_offset(&zero);
    Common_IMU_Reset();
    fail_dev = LSM303DLHC_MAG_ADDR;
    word(MPU6050_ADDR, 0x3F, 8192);
    word(MPU6050_ADDR, 0x47, 6560); /* 200deg/s；相邻有效更新隔20ms应前进4度。 */
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
    assert(heart_motion_frames == motion_frames + 1U);
    assert(angle_error(after.yaw_deg, before.yaw_deg + 4.0f) < 0.1f);
    mpu_auto_ready = 0;
    for (unsigned i = 0; i < 5U; ++i)
    {
        service_step();
    }
    assert(!App_IMU_GetSnapshot(&after));
    mpu_auto_ready = 1;
    service_step();
    assert(App_IMU_GetSnapshot(&after));
    fail_dev = -1;
    word(MPU6050_ADDR, 0x47, 0);
    puts("PASS: no-ready frame neither publishes nor integrates; next fresh frame uses elapsed time; stale MPU goes offline and recovers.");
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
        service_step(); /* No new DRDY: expire after50ms, not100ms. */
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
    for (n = 0; n < 3; ++n)
    {
        service_step();
    }
    assert(App_IMU_GetSnapshot(&imu) && !imu.mag_online && !imu.mag_fused);
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
        float sign = n & 1 ? 1.0f : -1.0f;
        magnetic(10 + (axis == 0 ? sign * 40 : 0), -5 + (axis == 1 ? sign * 50 : 0), 7 + (axis == 2 ? sign * 60 : 0));
        service_step();
        service_step();
        service_step();
    }
    eeprom_write_protected = fail_save;
    App_IMU_MagCalibrate(0);
    service_step();
    App_IMU_GetSnapshot(&imu);
    assert(imu.mag_cal_state == APP_MAG_CAL_DONE && imu.mag_calibrated);
    assert(imu.mag_persist_state == (fail_save ? APP_MAG_PERSIST_SAVE_FAILED : APP_MAG_PERSIST_SAVED));
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
    assert(imu.mag_persist_state == (fail_save ? APP_MAG_PERSIST_SAVE_FAILED : APP_MAG_PERSIST_SAVED));
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
    for (i = 0; i < sizeof(data); ++i)
    {
        data[i] = (uint8_t)(i + 1);
    }
    before = eeprom_writes;
    assert(bl24c02_write(14, data, sizeof(data)) == 0 && eeprom_writes == before + 3);
    assert(bl24c02_read(14, readback, sizeof(readback)) == 0 && memcmp(data, readback, sizeof(data)) == 0);
    assert(bl24c02_write(250, data, 20) == 1 && bl24c02_read(255, readback, 2) == 1);
    assert(bl24c02_write(256, NULL, 0) == 0 && bl24c02_read(0, NULL, 1) == 1);
    fail_dev = BL24C02_ADDR;
    start = tick;
    assert(bl24c02_read(0, readback, 1) == 1 && tick - start == 10);
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
    const uint32_t addresses[] = {0x60U, 0x80U, 0x80U};
    unsigned i;
    memset(eeprom, 0xFF, sizeof(eeprom));
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_DONE && host_imu_save_diag[1] == BL24C02_ERROR_NONE);
    assert(host_imu_save_diag[2] == 0x60U && host_imu_save_diag[3] >= 6U);
    for (i = 0; i < 3U; ++i)
    {
        eeprom_read_fail_after = (int)i;
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
    assert(host_imu_save_diag[2] == 0x90U);
    eeprom_fail_write_after = -1;
    eeprom_busy_ms = 20U;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_WRITE && host_imu_save_diag[1] == BL24C02_ERROR_READY_TIMEOUT);
    assert(host_imu_save_diag[2] == 0x80U && host_imu_save_diag[3] >= 10U);
    tick = eeprom_busy_until;
    eeprom_busy_ms = 3U;
    memset(eeprom + 0x80, 0xFF, 32);
    eeprom_write_protected = 1;
    assert(app_imu_store_save(&b) == APP_IMU_STORE_ERROR);
    assert(host_imu_save_diag[0] == APP_IMU_SAVE_VERIFY_DATA && host_imu_save_diag[1] == BL24C02_ERROR_NONE);
    assert(host_imu_save_diag[2] == 0x80U);
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
    }
    puts(fail_load ? "PASS: EEPROM load failure does not claim restored calibration"
                   : "PASS: fresh-boot sensor service restores EEPROM coefficients");
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
        assert(spl006_init() == 1);
        pressure = -1;
        assert(spl006_process_pressure(0, 0, &pressure) == 1 && pressure == -1);
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
        spl_reset_status = failure ? SPL06_SENSOR_RDY : SPL06_COEF_RDY;
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
    const uint16_t expected[] = {0x0101U, 0x0100U, 0x0001U, 0};
    for (ready = 0; ready < 4; ++ready)
    {
        regs[SPL006_ADDR][SPL06_MEAS_CFG] = ready << 4;
        before = spl_reads;
        assert(spl006_read_sample(&pressure, &temperature) == expected[ready]);
        if (ready < 3)
        {
            assert(spl_reads == before + 1U && pressure == 123 && temperature == 456);
        }
        else
        {
            assert(spl_reads == before + 2U); /* 原整对读取需要四次事务。 */
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
    App_Baro_Init();
    memset(regs[SPL006_ADDR], 0, 6);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 100000 && isfinite(sample.alt_m));
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
    puts("PASS: real barometer service preserves compensation, partial-readiness snapshots, three-error offline threshold and recovery.");
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
    assert(writes == 11 && read_count == 3);
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
        assert(my_mpu6050_init() == 1);
        mpu_fail_read_at = -1;
        assert(my_mpu6050_get_data(&raw) == 1 && memcmp(&raw, &before, sizeof(raw)) == 0);
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
        assert(tick - started >= 70U);
        fail_dev = MPU6050_ADDR;
        fail_reg = MPU6050_WHO_AM_I;
        started = tick;
        assert(my_mpu6050_init() == 1);
        assert(tick - started >= 950U && tick - started <= 969U);
        fail_dev = fail_reg = -1;
        if (scheduler_state == taskSCHEDULER_RUNNING)
        {
            assert(task_delay_calls == 21U && hal_delay_calls == 0U);
        }
        else
        {
            assert(hal_delay_calls == 21U && task_delay_calls == 0U);
        }
        assert(my_mpu6050_init() == 0);
    }
    scheduler_state = taskSCHEDULER_RUNNING;
    puts("PASS: SPL006/MPU initialization yields only with a running scheduler, preserves minimum waits and bounded retries");
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "save_failed") != 0)
    {
        restore_tests(strcmp(argv[1], "load_failed") == 0);
        return 0;
    }
    spl006_init_tests();
    spl006_sample_tests();
    baro_service_tests();
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
