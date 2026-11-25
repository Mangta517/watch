/* Real driver/service/math, fake bus. Inject EXTI while an I2C transaction is active. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_imu.h"
#include "mpu6050.h"
#include "lsm303dlhc.h"
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
sensor_host_exti_struct sensor_host_exti;
static uint8_t regs[128][256] = {[MPU6050_ADDR] = {[MPU6050_WHO_AM_I] = 0x68}};
static TickType_t tick;
static uint8_t mpu_auto_ready = 1;
static uint32_t heart_motion_ms;
static int scheduler_state = taskSCHEDULER_RUNNING;
static unsigned hal_delay_calls, task_delay_calls;
static uint32_t primask;
static uint8_t bus_active, mutex_taken;
static uint8_t baro_request_on_unlock;
static int fail_dev = -1, fail_reg = -1;
static unsigned reads, mag_frames, mag_config_writes, bus_transactions;
static uint32_t transfer_ms, spl_reset_ended, spl_status_ended;
static uint8_t check_recovery_waits, spl_status_polls;
static int spl_fail_read_at = -1, spl_fail_write_at = -1;
static unsigned spl_reads, spl_writes;
static unsigned mpu_reads, mpu_writes;
static uint16_t mpu_last_frame_size;
static int mpu_fail_read_at = -1, mpu_fail_write_at = -1;
static uint8_t mpu_mot_test, mpu_mot_edge_on_enable;
static uint8_t mpu_write_trace[16][2];
static unsigned mpu_mot_notifications;
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
    if (baro_request_on_unlock != 0)
    {
        baro_request_on_unlock = 0;
        assert(app_baro_calibrate_altitude(110, 0) == 0);
    }
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
    if (mpu_mot_test == 0)
    {
        HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
    }
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
        uint8_t read_reg = reg;
        if (dev == LSM303DLHC_ACCEL_ADDR && reg == 0xA8)
        {
            read_reg = 0x28;
            assert(len == 6 && (regs[dev][0x23] & 0x98) == 0x98);
        }
        memcpy(data, &regs[dev][read_reg], len);
        if (dev == LSM303DLHC_ACCEL_ADDR && reg == 0x27 && regs[dev][0x20] != 0)
        {
            data[0] |= 0x08;
        }
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
        if (mpu_mot_test != 0 && mpu_writes < sizeof(mpu_write_trace) / sizeof(mpu_write_trace[0]))
        {
            mpu_write_trace[mpu_writes][0] = reg;
            mpu_write_trace[mpu_writes][1] = data[0];
        }
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
        if (mpu_mot_edge_on_enable != 0 && dev == MPU6050_ADDR && reg == MPU6050_INT_ENABLE && data[0] == 0x40)
        {
            assert((EXTI->IMR & MPU6050_INT_Pin) != 0 && EXTI->PR == (1U << 4));
            HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
        }
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
    word(LSM303DLHC_MAG_ADDR, 3, (int16_t)lroundf(x * 6.7f));
    word(LSM303DLHC_MAG_ADDR, 5, (int16_t)lroundf(z * 6.0f));
    word(LSM303DLHC_MAG_ADDR, 7, (int16_t)lroundf(y * 6.7f));
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
    assert(regs[0x1E][0] == 0x18 && regs[0x1E][1] == 0x60 && regs[0x1E][2] == 0);
    assert(mag_config_writes >= 3);
    magnetic(50, -25, 30);
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OK);
    assert(mag.raw[0] == 335 && mag.raw[1] == -168 && mag.raw[2] == 180);
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
    assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_OVERFLOW);
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
    assert(raw.Accel.Accel_x == 123 && raw.Accel.Accel_y == -456 && raw.Gyro.Gyro_x == 0);
    assert(reads == before + 1 && regs[MPU6050_ADDR][0x3A] == 0 && my_mpu6050_get_int_status() == 1);
    regs[SPL006_ADDR][8] = 0x10;
    assert(spl006_read_pressure(&pressure) == 0); /* Third device under injected EXTI. */
    word(MPU6050_ADDR, 0x3B, 0);
    word(MPU6050_ADDR, 0x3D, 0);
    word(MPU6050_ADDR, 0x43, 0);
    puts("PASS: ID/config, XZY/endian/units, NACK/saturation/LOCK, deferred MPU IRQ, shared bus");
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
    const float cases[][3] = {{0, 0, 90}, {35, 25, 90}, {-40, 30, -135}, {20, -55, 179}, {0, 0, -179}, {45, 45, 0}, {180, 0, 90}};
    Accel_Gyro_struct raw;
    Euler_struct euler = {0};
    float mag[3], gravity[3], without_mag, with_mag, before;
    unsigned c, n;
    fusion_startup_tests();
    for (c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        Common_IMU_Reset();
        truth(cases[c][0], cases[c][1], cases[c][2], &raw, mag, gravity);
        assert(Common_IMU_GetEulerAngleWithMag(&raw, mag, &euler, .01f) == 1);
        assert(angle_error(euler.roll, cases[c][0]) < .3f);
        assert(angle_error(euler.pitch, cases[c][1]) < .3f);
        assert(angle_error(euler.yaw, cases[c][2]) < .3f);
        for (n = 1; n < 12000; ++n)
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

static void compass_tests(void)
{
    static const float fields[][3] = {{30, 0, 40}, {0, 30, 40}, {-30, 0, 40}, {0, -30, 40}};
    static const float poses[][2] = {{0, 0}, {35, 25}, {-50, -40}, {80, 60}, {180, 0}, {-179, 15}};
    Accel_struct accel = {0, 0, 8192};
    Accel_Gyro_struct raw;
    float mag[3] = {30, 28.284271f, 28.284271f};
    float heading = -1.0f;
    float gravity[3];
    unsigned i, yaw;

    for (i = 0; i < 4; ++i)
    {
        assert(common_imu_compass_heading(&accel, fields[i], &heading) == 0);
        assert(angle_error(heading, (float)i * 90.0f) < 0.01f);
    }
    accel.Accel_z = 5793;
    accel.Accel_y = 5793; /* 45°横滚后Hy=0，方位角为0°。 */
    assert(common_imu_compass_heading(&accel, mag, &heading) == 0 && angle_error(heading, 0.0f) < 0.01f);
    accel.Accel_x = -5793;
    accel.Accel_y = 0;
    mag[0] = mag[2] = 21.213203f;
    mag[1] = 30.0f; /* 45°俯仰后Hx=Hy=30。 */
    assert(common_imu_compass_heading(&accel, mag, &heading) == 0 && angle_error(heading, 45.0f) < 0.01f);
    accel.Accel_x = accel.Accel_y = accel.Accel_z = INT16_MIN;
    assert(common_imu_compass_heading(&accel, fields[0], &heading) != 0);
    accel.Accel_x = accel.Accel_y = accel.Accel_z = 0;
    heading = 123.0f;
    assert(common_imu_compass_heading(&accel, fields[0], &heading) != 0 && heading == 123.0f);
    accel.Accel_z = 8192;
    mag[0] = mag[1] = 0.0f;
    mag[2] = 40.0f;
    assert(common_imu_compass_heading(&accel, mag, &heading) != 0 && heading == 123.0f);
    mag[0] = 150.0f;
    assert(common_imu_compass_heading(&accel, mag, &heading) == COMMON_COMPASS_FIELD_RANGE);
    assert(common_imu_compass_heading_with_limit(&accel, mag, 350.0f, &heading) == COMMON_COMPASS_OK &&
           angle_error(heading, 0.0f) < 0.01f);
    assert(common_imu_compass_heading_with_limit(&accel, mag, 4.0f, &heading) == COMMON_COMPASS_FIELD_RANGE);
    assert(common_imu_compass_heading_with_limit(&accel, mag, NAN, &heading) == COMMON_COMPASS_FIELD_RANGE);
    mag[0] = 400.0f;
    assert(common_imu_compass_heading_with_limit(&accel, mag, 350.0f, &heading) == COMMON_COMPASS_FIELD_RANGE);
    mag[0] = 3.0f;
    mag[1] = 0.0f;
    mag[2] = 0.0f;
    assert(common_imu_compass_heading_with_limit(&accel, mag, 350.0f, &heading) == COMMON_COMPASS_FIELD_RANGE);
    mag[0] = NAN;
    assert(common_imu_compass_heading(&accel, mag, &heading) != 0);
    accel.Accel_x = 8192;
    accel.Accel_z = 0;
    assert(common_imu_compass_heading(&accel, fields[0], &heading) != 0);
    /* 用已有Rz*Ry*Rx真值生成器验证整圈航向，覆盖复合倾角与倒置。 */
    for (i = 0; i < sizeof(poses) / sizeof(poses[0]); ++i)
    {
        for (yaw = 0; yaw < 360; yaw += 15)
        {
            truth(poses[i][0], poses[i][1], (float)yaw, &raw, mag, gravity);
            assert(common_imu_compass_heading(&raw.Accel, mag, &heading) == 0);
            assert(angle_error(heading, -(float)yaw) < 0.02f);
        }
    }

    assert(common_imu_compass_heading(NULL, fields[0], &heading) != 0);
    assert(common_imu_compass_heading(&accel, NULL, &heading) != 0);
    assert(common_imu_compass_heading(&accel, fields[0], NULL) != 0);
    accel.Accel_x = 8192;
    for (i = 0; i < 11; ++i)
    {
        accel.Accel_y = (int16_t)i - 5;
        accel.Accel_z = 5 - (int16_t)i;
        assert(common_imu_compass_heading(&accel, fields[0], &heading) != 0);
    }
    puts("PASS: direct compass cardinal/tilted/inverted poses, null/non-finite/impact input and near-vertical noisy dead zone.");
}



static void test_record_crc(uint8_t *record, unsigned offset);

static uint8_t mag_params_match(const common_mag_cal_params_struct *a, const common_mag_cal_params_struct *b)
{
    return memcmp(a->offset_uT, b->offset_uT, sizeof(a->offset_uT)) == 0 &&
           memcmp(a->soft_sym, b->soft_sym, sizeof(a->soft_sym)) == 0 &&
           a->dip_valid == b->dip_valid && a->dip_ref_deg == b->dip_ref_deg;
}

static uint8_t mag_records_match(const app_mag_calibration_struct *a, const app_mag_calibration_struct *b)
{
    return a->is_ellipsoid == b->is_ellipsoid && mag_params_match(&a->params, &b->params);
}

static void eeprom_tests(void)
{
    uint8_t data[20], readback[20], previous[96];
    app_mag_calibration_struct a = {{{10, -5, 7}, {1.25f, 1.0f, .8333333f, -0.1f, 0.2f, 0.1f}, 0.0f, 0}, 1};
    app_mag_calibration_struct b = {{{12, -3, 9}, {1.1f, .9f, 1.05f, 0.2f, 0.1f, -0.1f}, -40.0f, 1}, 1}, loaded;
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
    before = eeprom_writes;
    assert(app_mag_store_save(&a) == APP_MAG_STORE_OK && eeprom_writes == before + 3);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && mag_records_match(&loaded, &a));
    memcpy(previous, eeprom, sizeof(previous));
    eeprom_fail_write_after = 1;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    eeprom_fail_write_after = -1;
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && mag_records_match(&loaded, &a));
    assert(memcmp(previous, eeprom, 48) == 0); /* Active copy survives a torn target write. */
    assert(app_mag_store_save(&b) == APP_MAG_STORE_OK);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && mag_records_match(&loaded, &b));
    eeprom[48 + 8] ^= 1; /* Reject latest bad CRC, recover older valid copy. */
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && mag_records_match(&loaded, &a));
    eeprom_write_protected = 1;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR); /* ACK is not proof of persisted data. */
    eeprom_write_protected = 0;
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && mag_records_match(&loaded, &a));
    fail_dev = BL24C02_ADDR;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    fail_dev = -1;
    b.params.soft_sym[3] = 10.0f; /* Positive diagonal alone is insufficient. */
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    b.params.soft_sym[3] = 0.2f;
    b.params.soft_sym[0] = NAN;
    assert(app_mag_store_save(&b) == APP_MAG_STORE_ERROR);
    b.params.soft_sym[0] = 1.1f;
    eeprom[4] = eeprom[5] = 0xFF;
    test_record_crc(eeprom, 44);
    assert(app_mag_store_save(&b) == APP_MAG_STORE_OK);
    assert(eeprom[52] == 0 && eeprom[53] == 0);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && mag_records_match(&loaded, &b));

    /* 将槽0改为旧版记录，验证跨版本优先级和CRC损坏回退。 */
    eeprom[3] = '1';
    eeprom[4] = eeprom[5] = eeprom[6] = eeprom[7] = 0xFF;
    test_record_crc(eeprom, 32);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && loaded.is_ellipsoid);
    eeprom[48 + 44] ^= 1;
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && !loaded.is_ellipsoid && !loaded.params.dip_valid);
    assert(loaded.params.offset_uT[0] == 10.0f && loaded.params.soft_sym[0] == 1.25f);
    assert(loaded.params.soft_sym[3] == 0.0f && loaded.params.soft_sym[4] == 0.0f && loaded.params.soft_sym[5] == 0.0f);
    assert(app_mag_store_save(&b) == APP_MAG_STORE_OK);
    assert(app_mag_store_load(&loaded) == APP_MAG_STORE_OK && loaded.is_ellipsoid && loaded.params.dip_ref_deg == -40.0f);
    for (i = 96; i < sizeof(eeprom); ++i)
    {
        assert(eeprom[i] == 0xFF); /* Other settings region is untouched. */
    }
    memset(eeprom, 0xFF, sizeof(eeprom));
    eeprom_writes = 0;
    puts("PASS: EEPROM page splits/ACK timeout, dual-copy restore, torn write/CRC/WP failures, reserved range");
}

/* 独立按逐位多项式重算测试记录，构造CRC正确但量程/序号变化的数据。 */
static void test_record_crc(uint8_t *record, unsigned offset)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < offset; ++i)
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
        record[offset + i] = (uint8_t)(crc >> (8 * i));
    }
}

static void imu_eeprom_tests(void)
{
    mpu6050_offset_struct a = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct b = {{-21, 11, -31}, {-67, 34, -100}}, loaded, runtime;
    app_mag_calibration_struct mag = {{{10, -5, 7}, {1.25f, 1.0f, .8333333f}, 0.0f, 0}, 1}, mag_loaded;
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
    assert(app_mag_store_load(&mag_loaded) == APP_MAG_STORE_OK && mag_records_match(&mag_loaded, &mag));
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
    test_record_crc(eeprom + 0x60, 24);
    test_record_crc(eeprom + 0x80, 24);
    assert(app_imu_store_load(&loaded) == APP_IMU_STORE_EMPTY); /* CRC正确仍拒绝旧量程。 */
    assert(app_imu_store_save(&a) == APP_IMU_STORE_OK);
    memset(eeprom + 0x60 + 4, 0xFF, 4); /* 活跃记录序号变为UINT32_MAX。 */
    test_record_crc(eeprom + 0x60, 24);
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
    bl24c02_transfer_struct transfer;
    uint8_t byte;
    unsigned i;
    memset(eeprom, 0xFF, sizeof(eeprom));
    eeprom_busy_until = tick + 40;
    bl24c02_transfer_begin(&transfer, 0x60, &byte, 1, 0);
    tick += 25;
    assert(bl24c02_transfer_step(&transfer) == 2);
    tick += 25;
    assert(bl24c02_transfer_step(&transfer) == 2);
    tick += 25;
    assert(bl24c02_transfer_step(&transfer) == 0 && byte == 0xFF);
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
    eeprom_busy_ms = 80;
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
    float standard_altitude;
    App_Baro_Init();
    memset(regs[SPL006_ADDR], 0, 6);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 100000 && isfinite(sample.alt_m));
    standard_altitude = sample.alt_m;
    assert(app_baro_calibrate_altitude(110, 0) == 0);
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && !sample.reference_ready && fabsf(sample.alt_m - 110.0f) < 0.01f);
    app_baro_reset_calibration();
    app_baro_set_reference(102125, 0, 0);
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 100000);
    assert(sample.alt_m > standard_altitude + 60.0f && sample.alt_m < standard_altitude + 70.0f);
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.alt_m > standard_altitude + 60.0f);
    app_baro_set_reference(101325, 0, 0);
    assert(App_Baro_GetSnapshot(&sample) && fabsf(sample.alt_m - standard_altitude) < 0.01f);
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
        tick += APP_BARO_SAMPLE_PERIOD_MS * 3 + 999;
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
        tick += APP_BARO_SAMPLE_PERIOD_MS * 3 + 1000;
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

static void baro_calibration_tests(void)
{
    app_baro_struct sample;
    float standard_altitude, calibrated_altitude;
    unsigned reads;
    memset(regs[SPL006_ADDR], 0, 6);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    app_baro_reset_calibration();
    app_baro_set_reference(101325, 0, 0);
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample));
    standard_altitude = sample.alt_m;
    app_baro_set_reference(0, 200, 1);
    assert(App_Baro_GetSnapshot(&sample) && sample.reference_ready && sample.temperature_corrected);
    assert(sample.alt_m > standard_altitude + 1.0f);
    app_baro_set_reference(0, -100, 1);
    assert(App_Baro_GetSnapshot(&sample) && sample.alt_m < standard_altitude);
    app_baro_set_reference(0, 999, 0);
    assert(App_Baro_GetSnapshot(&sample) && !sample.temperature_corrected);
    assert(fabsf(sample.alt_m - standard_altitude) < 0.01f);
    app_baro_set_reference(101325, 200, 1);
    assert(app_baro_calibrate_altitude(-1001, 1) == 1 && app_baro_calibrate_altitude(9001, 0) == 1);
    assert(app_baro_calibrate_altitude(500, 1) == 0 && app_baro_calibration_pending());
    assert(App_Baro_GetSnapshot(&sample) && sample.alt_m < 200.0f);
    reads = spl_reads;
    App_Baro_Service();
    assert(spl_reads > reads && !app_baro_calibration_pending());
    assert(App_Baro_GetSnapshot(&sample) && sample.calibration_state == APP_BARO_CAL_APPLIED);
    assert(sample.calibrated_from_phone && sample.pressure_pa == 100000 && fabsf(sample.alt_m - 500.0f) < 0.01f);
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && fabsf(sample.alt_m - 500.0f) < 0.2f);
    app_baro_set_reference(101425, 0, 0);
    assert(App_Baro_GetSnapshot(&sample) && fabsf(sample.alt_m - 500.0f) < 0.01f);
    regs[SPL006_ADDR][SPL06_C00_BYTE1] = 0x5A;
    regs[SPL006_ADDR][SPL06_C00_BYTE0] = 0x60;
    assert(spl006_init() == 0);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.pressure_pa == 99750 && sample.alt_m > 521.0f && sample.alt_m < 522.0f);
    calibrated_altitude = sample.alt_m;
    assert(app_baro_calibrate_altitude(110, 1) == 0);
    tick += 8000;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.calibration_state == APP_BARO_CAL_FAILED);
    assert(fabsf(sample.alt_m - calibrated_altitude) < 0.01f);
    assert(app_baro_calibrate_altitude(110, 0) == 0);
    fail_dev = SPL006_ADDR;
    App_Baro_Service();
    fail_dev = -1;
    assert(!app_baro_calibration_pending());
    App_Baro_GetSnapshot(&sample);
    assert(sample.calibration_state == APP_BARO_CAL_FAILED && fabsf(sample.alt_m - calibrated_altitude) < 0.01f);
    assert(app_baro_calibrate_altitude(110, 0) == 0);
    app_baro_reset_calibration();
    assert(!app_baro_calibration_pending());
    app_baro_set_reference(101325, 0, 0);
    regs[SPL006_ADDR][SPL06_C00_BYTE1] = 0x6A;
    regs[SPL006_ADDR][SPL06_C00_BYTE0] = 0;
    assert(spl006_init() == 0);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && sample.calibration_state == APP_BARO_CAL_NONE);
    assert(fabsf(sample.alt_m - standard_altitude) < 0.01f);
    assert(app_baro_calibrate_altitude(-50, 0) == 0);
    App_Baro_Service();
    assert(App_Baro_GetSnapshot(&sample) && fabsf(sample.alt_m + 50.0f) < 0.2f);
    /* 无效气压不能更新校准点和海拔。 */
    regs[SPL006_ADDR][SPL06_C00_BYTE2] = 0;
    regs[SPL006_ADDR][SPL06_C00_BYTE1] = 0;
    assert(spl006_init() == 0);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    App_Baro_Service();
    App_Baro_GetSnapshot(&sample);
    assert(sample.pressure_pa == 100000 && fabsf(sample.alt_m + 50.0f) < 0.2f);
    regs[SPL006_ADDR][SPL06_C00_BYTE2] = 0x18;
    regs[SPL006_ADDR][SPL06_C00_BYTE1] = 0x6A;
    assert(spl006_init() == 0);
    regs[SPL006_ADDR][SPL06_MEAS_CFG] = SPL06_PRS_RDY | SPL06_TMP_RDY;
    assert(app_baro_calibrate_altitude(500, 0) == 0);
    baro_request_on_unlock = 1;
    App_Baro_Service();
    assert(app_baro_calibration_pending());
    App_Baro_GetSnapshot(&sample);
    assert(fabsf(sample.alt_m - 500.0f) < 0.2f);
    App_Baro_Service();
    assert(!app_baro_calibration_pending());
    App_Baro_GetSnapshot(&sample);
    assert(fabsf(sample.alt_m - 110.0f) < 0.2f);
    app_baro_reset_calibration();
    puts("PASS: fresh phone/manual anchor, pressure-relative ascent, weather independence, expired request, failure retention and reset.");
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

static void mpu_mot_notify(void)
{
    assert(!primask && (EXTI->IMR & MPU6050_INT_Pin) == 0 && my_mpu6050_mot_woken() != 0);
    ++mpu_mot_notifications;
}

static void mpu_mot_tests(void)
{
    const uint8_t enter[][2] = {
        {0x38, 0}, {0x6B, 0x08}, {0x1C, 0x08}, {0x1A, 0}, {0x69, 0x11}, {0x1F, 18},
        {0x20, 1}, {0x37, 0xC0}, {0x1C, 0x0F}, {0x6C, 0x87}, {0x38, 0x40}, {0x6B, 0x28}
    };
    const uint8_t exit_background[][2] = {
        {0x6B, 0x08}, {0x1C, 0x08}, {0x38, 1}, {0x6C, 0xC7}, {0x1A, 3}, {0x19, 9}, {0x6B, 0x28}
    };
    Accel_Gyro_struct raw = {0};
    unsigned before;
    int failure;
    assert(my_mpu6050_init() == 0);
    my_mpu6050_set_mot_callback(mpu_mot_notify);
    mpu_mot_test = mpu_mot_edge_on_enable = 1;
    EXTI->IMR = 1U << 4;
    EXTI->PR = MPU6050_INT_Pin | (1U << 4);
    mpu_writes = 0;
    before = tick;
    assert(my_mpu6050_enter_mot_sleep() == 0 && tick > before);
    assert(memcmp(enter, mpu_write_trace, sizeof(enter)) == 0 && mpu_writes == sizeof(enter) / sizeof(enter[0]));
    assert(mpu_mot_notifications == 1 && my_mpu6050_mot_woken());
    HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
    assert(mpu_mot_notifications == 1 && EXTI->IMR == (1U << 4));
    before = mpu_reads;
    tick += 10000;
    assert(my_mpu6050_recover_step(tick) == 0 && my_mpu6050_get_data(&raw) == 1 && mpu_reads == before);
    mpu_writes = 0;
    assert(my_mpu6050_set_sample_mode(25, 0) == 0);
    assert(memcmp(exit_background, mpu_write_trace, sizeof(exit_background)) == 0);
    assert(my_mpu6050_get_data(&raw) == 0 && !my_mpu6050_mot_woken());

    mpu_mot_edge_on_enable = 0;
    assert(my_mpu6050_enter_mot_sleep() == 0 && my_mpu6050_mot_armed());
    assert(my_mpu6050_set_sample_mode(10, 1) == 0 && EXTI->IMR == (1U << 4));
    assert(regs[MPU6050_ADDR][0x6B] == 0x08 && regs[MPU6050_ADDR][0x6C] == 0x07);
    assert(regs[MPU6050_ADDR][0x1A] == 0x03 && regs[MPU6050_ADDR][0x38] == 1 && regs[MPU6050_ADDR][0x1C] == 0x08);
    assert(my_mpu6050_set_sample_mode(25, 0) == 0 && my_mpu6050_enter_mot_sleep() == 0);
    tick += 60000;
    assert(my_mpu6050_mot_health_check() == 0 && my_mpu6050_mot_armed());
    regs[MPU6050_ADDR][0x38] = 0;
    assert(my_mpu6050_mot_health_check() == 1 && !my_mpu6050_mot_armed() && EXTI->IMR == (1U << 4));
    assert(my_mpu6050_recover_step(tick) == 2);

    for (failure = 0; failure < (int)(sizeof(enter) / sizeof(enter[0])); ++failure)
    {
        assert(my_mpu6050_init() == 0);
        mpu_writes = 0;
        mpu_fail_write_at = failure;
        assert(my_mpu6050_enter_mot_sleep() == 1 && !my_mpu6050_mot_armed() && EXTI->IMR == (1U << 4));
        mpu_fail_write_at = -1;
        assert(my_mpu6050_recover_step(tick) == 2);
    }
    for (failure = 0; failure < (int)(sizeof(exit_background) / sizeof(exit_background[0])); ++failure)
    {
        assert(my_mpu6050_init() == 0 && my_mpu6050_enter_mot_sleep() == 0);
        mpu_writes = 0;
        mpu_fail_write_at = failure;
        assert(my_mpu6050_set_sample_mode(25, 0) == 1 && !my_mpu6050_mot_armed() && EXTI->IMR == (1U << 4));
        mpu_fail_write_at = -1;
        assert(my_mpu6050_recover_step(tick) == 2);
    }
    assert(my_mpu6050_init() == 0);
    mpu_fail_read_at = (int)mpu_reads;
    assert(my_mpu6050_enter_mot_sleep() == 1 && !my_mpu6050_mot_armed());
    mpu_fail_read_at = -1;
    assert(my_mpu6050_init() == 0 && my_mpu6050_enter_mot_sleep() == 0);
    mpu_fail_read_at = (int)mpu_reads;
    assert(my_mpu6050_set_sample_mode(25, 0) == 1 && !my_mpu6050_mot_armed());
    mpu_fail_read_at = -1;
    assert(my_mpu6050_init() == 0 && my_mpu6050_enter_mot_sleep() == 0);
    mpu_fail_read_at = (int)mpu_reads;
    assert(my_mpu6050_mot_health_check() == 1 && !my_mpu6050_mot_armed());
    mpu_fail_read_at = -1;
    assert(EXTI->IMR == (1U << 4) && my_mpu6050_recover_step(tick) == 2);
    puts("PASS: 20 Hz MOT register sequence, first IRQ during arm, one-shot, no frame timeout, 40/100 Hz resume and I2C recovery.");
}

static void mpu_mot_fallback_tests(void)
{
    app_imu_struct snapshot;
    unsigned before_reads;
    uint32_t sample_ms;

    mpu_auto_ready = 1;
    assert(my_mpu6050_init() == 0);
    App_IMU_Init();
    word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, -1000);
    word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 8192);
    assert(App_IMU_Service(0) == 0);
    sample_ms = my_mpu6050_sample_ms();
    assert(app_imu_enter_mot() == 0 && my_mpu6050_mot_armed());
    mpu_auto_ready = 0;
    tick += 400;

    before_reads = mpu_reads;
    word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, -1368);
    assert(app_imu_mot_poll() == 0 && mpu_reads == before_reads + 1 && mpu_last_frame_size == 7);
    word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, -1369);
    assert(app_imu_mot_poll() == 1 && !my_mpu6050_mot_woken());
    word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, -1000);
    regs[MPU6050_ADDR][MPU6050_INT_STATUS] = 0x40;
    assert(app_imu_mot_poll() == 1 && !my_mpu6050_mot_woken());
    assert(app_imu_mot_poll() == 0 && my_mpu6050_sample_ms() == sample_ms);
    assert(!App_IMU_GetSnapshot(&snapshot) && snapshot.raw.Accel.Accel_x == -1000 && snapshot.raw.Accel.Accel_z == 8192);

    mpu_fail_read_at = (int)mpu_reads;
    assert(app_imu_mot_poll() == 1 && !my_mpu6050_mot_armed() && !(EXTI->IMR & MPU6050_INT_Pin));
    mpu_fail_read_at = -1;
    assert(my_mpu6050_recover_step(tick) == 2);
    puts("PASS: MOT fallback restores sampling for latched motion or raw acceleration without PB12, ignores quiet noise and recovers on NACK.");
}

static void mpu_mot_first_step_tests(void)
{
    app_imu_struct snapshot;
    unsigned source, elapsed, reads;
    int16_t z;

    for (source = 0; source < 2; ++source)
    {
        mpu_auto_ready = 1;
        assert(my_mpu6050_init() == 0);
        App_IMU_Init();
        word(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 0);
        word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, -8192);
        for (elapsed = 0; elapsed <= 6000; elapsed += 25)
        {
            tick += 25;
            assert(App_IMU_Service(0) == 0);
        }
        assert(app_imu_can_enter_mot(tick) && app_imu_enter_mot() == 0);
        tick += 10000;
        word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, -11141); /* 1.36g首个落地峰。 */
        if (source == 0)
        {
            HAL_GPIO_EXTI_Callback(MPU6050_INT_Pin);
            assert(my_mpu6050_mot_woken());
        }
        else
        {
            assert(app_imu_mot_poll() == 1);
        }
        reads = mpu_reads;
        app_imu_exit_mot(0);
        /* 一次读取清状态，另外只有EXTI路径需要读取动作帧。 */
        assert(mpu_reads == reads + 2 - source);
        for (elapsed = 25; elapsed < 1500; elapsed += 25)
        {
            z = -8192;
            if (elapsed == 25)
            {
                z = -9667; /* 切回40Hz后只剩首个落地峰的尾部。 */
            }
            else if (elapsed >= 500 && elapsed % 500 < 125)
            {
                z = -10240;
            }
            tick += 25;
            word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, z);
            assert(App_IMU_Service(0) == 0);
        }
        App_IMU_GetSnapshot(&snapshot);
        assert(snapshot.steps == 3);
    }
    puts("PASS: MPU MOT interrupt and fallback retain the first measured step across the 20/40Hz switch.");
}

static void mpu_stale_recovery_tests(void)
{
    Accel_Gyro_struct raw;
    mpu6050_offset_struct saved = {{20, -10, 30}, {66, -33, 99}};
    mpu6050_offset_struct after;
    uint32_t start, writes_before;
    uint32_t i;
    writes_before = mpu_writes;
    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x00;
    assert(my_mpu6050_init() == 1 && mpu_writes == writes_before);
    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x68;
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
                magnetic(30.0f, 5.0f, 20.0f); /* 唤醒后首个完整读只排空残留。 */
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
            assert(lsm303dlhc_mag_read(&mag) == LSM303DLHC_MAG_NOT_READY);
            tick += 40;
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
    *now += 25;
}

static uint32_t activity_gait(float rate, unsigned phase, unsigned kind, uint32_t now, uint8_t cold)
{
    common_activity_struct state = {0};
    unsigned elapsed, duration = (unsigned)(20000.0f / rate), delay_ms = 0;
    float angle, cycle, magnitude, ax;
    if (kind >= 2)
    {
        /* 20个完整脉冲，改变启动相位并保留前后的静止区。 */
        delay_ms = (unsigned)(1000.0f / rate * phase / 40.0f);
        duration += delay_ms;
    }
    if (cold == 0)
    {
        for (elapsed = 0; elapsed < 1000; elapsed += 25)
        {
            activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
        }
    }
    for (elapsed = 0; elapsed < duration; elapsed += 25)
    {
        cycle = rate * (float)elapsed / 1000.0f + (float)phase / 40.0f;
        if (kind >= 2)
        {
            cycle = rate * ((float)elapsed - (float)delay_ms) / 1000.0f;
        }
        angle = cycle * 6.2831853f;
        magnitude = 1.0f + 0.15f * sinf(angle);
        ax = 0.0f;
        if (kind == 1)
        {
            ax = 0.30f * sinf(angle);
            magnitude = 1.0f + 0.08f * cosf(angle);
        }
        else if (kind == 2)
        {
            magnitude = 1.0f;
            if (elapsed >= delay_ms && cycle - floorf(cycle) < 0.12f * rate)
            {
                magnitude = 1.15f;
            }
        }
        else if (kind == 3)
        {
            magnitude = 1.0f;
            if (elapsed >= delay_ms && sinf(angle) > 0.0f)
            {
                magnitude += 2.2f * sinf(angle);
            }
        }
        activity_sample(&state, ax, 0.0f, -magnitude, &now);
    }
    /* 步行停止后继续采样，允许末步滤波与回落确认完成。 */
    for (elapsed = 0; elapsed < 500; elapsed += 25)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    return state.steps;
}

static void activity_pulses(common_activity_struct *state, uint32_t *now, unsigned count, unsigned period_ms, float amplitude)
{
    unsigned elapsed;
    float magnitude;
    for (elapsed = 0; elapsed < count * period_ms; elapsed += 25)
    {
        magnitude = 1.0f;
        if (elapsed % period_ms < 125)
        {
            magnitude += amplitude;
        }
        activity_sample(state, 0.0f, 0.0f, -magnitude, now);
    }
}

static void activity_undercount_tests(void)
{
    common_activity_struct state;
    uint32_t now = UINT32_MAX - 2000, result[8];
    unsigned scenario, frame, step, phase, duration, count;
    float magnitude, orientation;

    for (scenario = 0; scenario < 8; ++scenario)
    {
        memset(&state, 0, sizeof(state));
        orientation = -1.0f;
        if (scenario == 1 || scenario == 2)
        {
            orientation = 1.0f;
        }
        for (frame = 0; frame < 241; ++frame)
        {
            activity_sample(&state, 0.0f, 0.0f, orientation, &now);
        }
        count = 40;
        if (scenario == 4 || scenario == 5)
        {
            count = 3;
        }
        else if (scenario == 6)
        {
            count = 4;
            now += 10000;
            common_activity_resume_from_mot(&state, now);
        }
        else if (scenario == 7)
        {
            count = 20;
        }
        for (step = 0; step < count; ++step)
        {
            duration = 525;
            if (scenario == 2)
            {
                duration = 775;
            }
            else if (scenario == 3)
            {
                duration = 400 + 300 * (step % 2);
            }
            else if (scenario == 5 && step == 0)
            {
                duration = 850;
            }
            else if (scenario == 7)
            {
                duration = 350;
            }
            for (phase = 0; phase < duration; phase += 25)
            {
                magnitude = 1.0f;
                if (scenario == 0 || scenario == 4)
                {
                    float amplitude = 0.055f;
                    if (scenario == 4)
                    {
                        amplitude = 0.13f;
                    }
                    magnitude += amplitude * sinf(6.2831853f * (float)phase / (float)duration);
                }
                else if (scenario == 7)
                {
                    if (phase < 75)
                    {
                        magnitude = 4.0f;
                    }
                    else if (phase < 150)
                    {
                        magnitude = 0.30f;
                    }
                }
                else if (phase < 125)
                {
                    magnitude = 1.20f;
                    if (scenario == 1 || scenario == 2)
                    {
                        magnitude = 1.16f;
                        if (step % 2 != 0)
                        {
                            magnitude = 1.065f;
                        }
                    }
                }
                activity_sample(&state, 0.0f, 0.0f, orientation * magnitude, &now);
            }
        }
        result[scenario] = state.steps;
    }
    printf("Step undercount: viewing=%lu, weak alternate=%lu/%lu, uneven=%lu, starts=%lu/%lu, MOT=%lu, clipped=%lu\n",
           (unsigned long)result[0], (unsigned long)result[1], (unsigned long)result[2], (unsigned long)result[3],
           (unsigned long)result[4], (unsigned long)result[5], (unsigned long)result[6], (unsigned long)result[7]);
    fflush(stdout);
    assert(result[0] >= 39 && result[0] <= 40);
    assert(result[1] >= 39 && result[1] <= 40 && result[2] >= 39 && result[2] <= 40);
    assert(result[3] == 40 && result[4] == 3 && result[5] == 3);
    assert(result[6] == 4 && result[7] == 20);

    memset(&state, 0, sizeof(state));
    for (frame = 0; frame < 40; ++frame)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    activity_pulses(&state, &now, 5, 500, 0.25f);
    for (frame = 0; frame < 20; ++frame)
    {
        float x = 0.055f * cosf(1.5707963f * (float)frame);
        activity_sample(&state, x, 0.0f, -sqrtf(1.0f - x * x), &now);
    }
    activity_pulses(&state, &now, 2, 500, 0.25f);
    assert(state.steps == 7 && state.walking_confirmed != 0);
    puts("PASS: a missing peak during ongoing motion preserves gait and counts only the next two measured steps.");
}

static void activity_mot_tests(void)
{
    common_activity_struct state = {0};
    uint32_t now = UINT32_MAX - 4000;
    unsigned i;
    state.steps = 42;
    for (i = 0; i < 240; ++i)
    {
        activity_sample(&state, 0.0f, 1.0f, 0.0f, &now);
    }
    assert(state.wrist_armed && !common_activity_is_mot_ready(&state, now, 6000));
    activity_sample(&state, 0.0f, 1.0f, 0.0f, &now);
    assert(common_activity_is_mot_ready(&state, now, 6000));
    assert(!common_activity_is_mot_ready(&state, now + 600, 6000));

    now += 10000;
    common_activity_resume_from_mot(&state, now);
    for (i = 0; i < 20; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.steps == 42 && state.wrist_sequence == 1 && !state.wrist_armed);
    activity_pulses(&state, &now, 3, 500, 0.25f);
    assert(state.steps == 45);

    state.wrist_armed = 1;
    now += 10000;
    common_activity_resume_from_mot(&state, now);
    for (i = 0; i < 125; ++i)
    {
        activity_sample(&state, 0.0f, 0.8f, -0.6f, &now);
    }
    assert(state.steps == 45 && state.wrist_sequence == 1 && !state.wrist_armed);

    /* 弱步态仍按原来5步确认，回补实际读到的候选。 */
    common_activity_resume_from_mot(&state, now);
    for (i = 0; i < 20; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    activity_pulses(&state, &now, 5, 500, 0.13f);
    assert(state.steps == 50);
    memset(&state, 0, sizeof(state));
    for (i = 0; i < 240; ++i)
    {
        activity_sample(&state, 0.0f, 1.0f, 0.0f, &now);
    }
    activity_sample(&state, 0.0f, 0.15f, 0.0f, &now);
    assert(!common_activity_is_mot_ready(&state, now, 6000));
    for (i = 0; i < 239; ++i)
    {
        activity_sample(&state, 0.0f, 1.0f, 0.0f, &now);
    }
    assert(!common_activity_is_mot_ready(&state, now, 6000));
    activity_sample(&state, 0.0f, 1.0f, 0.0f, &now);
    assert(common_activity_is_mot_ready(&state, now, 6000));
    puts("PASS: MOT quiet gate across wrap and after impact, first wrist raise after gap, grace expiry and 3/5-step backfill.");
}

static void activity_response_tests(void)
{
    common_activity_struct state = {0};
    uint32_t now = UINT32_MAX - 1000;
    unsigned i, cycle, axis;
    float angle, magnitude, accel[3];
    for (i = 0; i < 40; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    activity_pulses(&state, &now, 2, 500, 0.20f);
    assert(state.steps == 0 && state.walking_run == 2);
    for (i = 0; i < 20; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.still != 0 && state.walking_run == 0 && state.peak_pending == 0);
    activity_pulses(&state, &now, 1, 500, 0.20f);
    assert(state.steps == 0);
    puts("PASS: two pending steps expire after a quiet stop; a later isolated movement cannot release old steps across tick wrap.");

    memset(&state, 0, sizeof(state));
    for (i = 0; i < 40; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    activity_pulses(&state, &now, 10, 500, 0.20f);
    assert(state.steps == 10 && state.walking_confirmed != 0);
    for (i = 0; i < 20; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.walking_run == 0 && state.walking_confirmed == 0);
    for (i = 0; i < 20; ++i)
    {
        activity_sample(&state, 0.0f, 1.2f, 0.0f, &now);
    }
    for (i = 0; i < 20; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.steps == 10);
    puts("PASS: ten confirmed steps followed by a stop and wrist raise retain exactly ten steps.");

    memset(&state, 0, sizeof(state));
    for (i = 0; i < 40; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    activity_pulses(&state, &now, 4, 500, 0.13f);
    assert(state.steps == 0 && state.walking_run == 4);
    activity_pulses(&state, &now, 1, 500, 0.13f);
    assert(state.steps == 5 && state.walking_confirmed != 0);
    puts("PASS: weak periodic pulses require five candidates and backfill once.");

    memset(&state, 0, sizeof(state));
    for (i = 0; i < 40; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    for (cycle = 0; cycle < 20; ++cycle)
    {
        activity_pulses(&state, &now, 1, 400, 0.20f);
        activity_pulses(&state, &now, 1, 1100, 0.20f);
    }
    assert(state.steps == 0);
    puts("PASS: irregular strong movements cannot form a confirmed walking cadence.");

    memset(&state, 0, sizeof(state));
    for (i = 0; i < 12000; ++i)
    {
        angle = (float)i * 6.2831853f / 17.0f;
        magnitude = 1.0f + 0.025f * sinf(angle);
        if (i % 29 < 3)
        {
            magnitude += 0.085f;
        }
        activity_sample(&state, 0.02f * sinf(angle), 0.02f * cosf(angle), -magnitude, &now);
    }
    assert(state.steps == 0);
    puts("PASS: five minutes of synthetic desk micro-movements and short taps count zero steps.");

    memset(&state, 0, sizeof(state));
    for (cycle = 0; cycle < 20; ++cycle)
    {
        for (i = 0; i < 64; ++i)
        {
            angle = 0.0f;
            if (i >= 20 && i < 32)
            {
                angle = (float)(i - 20) * 1.5707963f / 12.0f;
            }
            else if (i >= 32 && i < 52)
            {
                angle = 1.5707963f;
            }
            else if (i >= 52)
            {
                angle = (float)(64 - i) * 1.5707963f / 12.0f;
            }
            magnitude = 1.0f + 0.20f * sinf(2.0f * angle);
            activity_sample(&state, 0.0f, magnitude * cosf(angle), -magnitude * sinf(angle), &now);
        }
    }
    assert(state.steps == 0 && state.wrist_sequence != 0);
    puts("PASS: repeated raise/lower wrist rotations with transient acceleration count zero steps and retain wrist wake events.");

    for (axis = 0; axis < 6; ++axis)
    {
        memset(&state, 0, sizeof(state));
        for (i = 0; i < 40 + 480 + 20; ++i)
        {
            magnitude = 1.0f;
            if (i >= 40 && i < 520)
            {
                magnitude += 0.15f * sinf(6.2831853f * (float)(i - 40) / 24.0f);
            }
            accel[0] = accel[1] = accel[2] = 0.0f;
            accel[axis / 2] = magnitude;
            if (axis % 2 != 0)
            {
                accel[axis / 2] = -magnitude;
            }
            activity_sample(&state, accel[0], accel[1], accel[2], &now);
        }
        assert(state.steps >= 19 && state.steps <= 20);
    }
    puts("PASS: identical step impacts remain countable with gravity along all six sensor directions.");
}

static void activity_tests(void)
{
    static const float rates[] = {1.25f, 1.5f, 1.8f, 2.0f, 2.2f, 2.5f};
    common_activity_struct state = {0};
    uint32_t now = UINT32_MAX - 1000, before;
    unsigned i, phase, kind, rate, cold, steps, lowest, highest;
    for (kind = 0; kind < 4; ++kind)
    {
        for (rate = 0; rate < sizeof(rates) / sizeof(rates[0]); ++rate)
        {
            lowest = 100;
            highest = 0;
            for (cold = 0; cold < 2; ++cold)
            {
                for (phase = 0; phase < 40; ++phase)
                {
                    steps = activity_gait(rates[rate], phase, kind, UINT32_MAX - 1000, cold);
                    if (steps < lowest)
                    {
                        lowest = steps;
                    }
                    if (steps > highest)
                    {
                        highest = steps;
                    }
                }
            }
            printf("gait kind=%u rate=%.2f, 40 phases warm/cold: %u..%u / 20\n", kind, rates[rate], lowest, highest);
            assert(lowest >= 19 && highest <= 20);
        }
    }
    for (phase = 0; phase < 40; ++phase)
    {
        memset(&state, 0, sizeof(state));
        for (i = 0; i < 1600; ++i)
        {
            activity_sample(&state, 0.0f, 0.0f, -(1.0f + 0.5f * sinf(6.2831853f * (17.0f * i / 40.0f + phase / 40.0f))), &now);
        }
        assert(state.steps == 0);
    }
    memset(&state, 0, sizeof(state));
    for (i = 0; i < 100; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    activity_sample(&state, 0.0f, 0.0f, -3.2f, &now);
    for (i = 0; i < 100; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.steps == 0 && state.wrist_sequence == 0);
    for (i = 0; i < 17; ++i)
    {
        activity_sample(&state, 0.0f, 1.0f, 0.0f, &now);
        if (i < 16)
        {
            assert(state.wrist_armed == 0);
        }
    }
    assert(state.wrist_armed != 0);
    for (i = 0; i < 17; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
        if (i < 16)
        {
            assert(state.wrist_sequence == 0);
        }
    }
    assert(state.wrist_sequence == 1);
    for (i = 0; i < 100; ++i)
    {
        activity_sample(&state, 0.0f, 0.0f, -1.0f, &now);
    }
    assert(state.wrist_sequence == 1 && state.steps == 0);
    before = state.sample_ms;
    common_activity_update(&state, 0.0f, 0.0f, -1.5f, before);
    common_activity_update(&state, 0.0f, 0.0f, -1.5f, before + 10);
    assert(state.sample_ms == before);
    state.steps = 123;
    common_activity_update(&state, 0.0f, 0.0f, -1.0f, before + 600);
    state.peak_pending = state.walking_run = state.wrist_armed = 1;
    common_activity_update(&state, 0.0f, 0.0f, -1.0f, state.sample_ms + 601);
    assert(state.peak_pending == 0 && state.walking_run == 0 && state.wrist_armed == 0);
    assert(state.steps == 123 && state.wrist_sequence == 1);
    puts("PASS: 40Hz phase/cold-start gait, horizontal swing, 120ms pulses, 3.2g running, 17Hz rejection, impact, timed wrist, duplicate and gap/wrap checks.");
    activity_response_tests();
    activity_undercount_tests();
}

static uint16_t boot_step_day, last_step_day;
static uint32_t boot_steps, last_steps;
void app_battery_log_steps_load(uint16_t *day, uint32_t *steps)
{
    *day = boot_step_day;
    *steps = boot_steps;
}
void app_battery_log_steps_update(uint16_t day, uint32_t steps, uint32_t now_ms)
{
    assert(now_ms == tick);
    last_step_day = day;
    last_steps = steps;
}

static void independent_setup(void)
{
    const mpu6050_offset_struct zero = {{0, 0, 0}, {0, 0, 0}};
    memset(eeprom, 0xFF, sizeof(eeprom));
    regs[MPU6050_ADDR][MPU6050_WHO_AM_I] = 0x68;
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    app_imu_init_hw();
    app_imu_init_mag_hw();
    my_mpu6050_restore_offset(&zero);
    App_IMU_Init();
    word(MPU6050_ADDR, 0x3F, 8192);
    /* LSM303 HR、±4g，1g输出为8000，小端。 */
    regs[LSM303DLHC_ACCEL_ADDR][0x28] = 0;
    regs[LSM303DLHC_ACCEL_ADDR][0x29] = 0;
    regs[LSM303DLHC_ACCEL_ADDR][0x2A] = 0;
    regs[LSM303DLHC_ACCEL_ADDR][0x2B] = 0;
    regs[LSM303DLHC_ACCEL_ADDR][0x2C] = 0x40;
    regs[LSM303DLHC_ACCEL_ADDR][0x2D] = 0x1F;
}

static void compass_acceleration(const float gravity[3])
{
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        uint16_t raw = (uint16_t)((int16_t)lroundf(gravity[axis] * 500.0f) * 16);
        regs[LSM303DLHC_ACCEL_ADDR][0x28 + axis * 2] = (uint8_t)raw;
        regs[LSM303DLHC_ACCEL_ADDR][0x29 + axis * 2] = (uint8_t)(raw >> 8);
    }
}

static void accel_warmup_tests(void)
{
    Accel_struct accel = {123, 456, 789};
    Accel_struct previous = accel;
    uint32_t wake_ms;
    unsigned i;
    independent_setup();
    tick = UINT32_MAX - 100;
    lsm303dlhc_accel_recover_begin();
    assert(lsm303dlhc_accel_recover_step(tick) == 2);
    assert(lsm303dlhc_accel_recover_step(tick) == 0);
    assert(lsm303dlhc_accel_set_sleep(0) == 0);
    wake_ms = tick;
    for (i = 0; i < 7; ++i)
    {
        tick = wake_ms + i * 40;
        assert(lsm303dlhc_accel_read(&accel) == LSM303DLHC_MAG_NOT_READY);
        assert(memcmp(&accel, &previous, sizeof(accel)) == 0);
    }
    tick = wake_ms + 280;
    assert(lsm303dlhc_accel_read(&accel) == LSM303DLHC_MAG_OK && accel.Accel_z == 8192);
    assert(lsm303dlhc_accel_set_sleep(1) == 0);
    regs[LSM303DLHC_ACCEL_ADDR][0x2C] = 0xC0;
    regs[LSM303DLHC_ACCEL_ADDR][0x2D] = 0xE0;
    assert(lsm303dlhc_accel_set_sleep(0) == 0);
    tick += 1000;
    previous = accel;
    assert(lsm303dlhc_accel_read(&accel) == LSM303DLHC_MAG_NOT_READY && memcmp(&accel, &previous, sizeof(accel)) == 0);
    tick += 40;
    assert(lsm303dlhc_accel_read(&accel) == LSM303DLHC_MAG_OK && accel.Accel_z == -8192);
    puts("PASS: LSM HR warmup drains without publishing for 280ms, first resume frame is discarded even after a late poll, signed little-endian units survive tick wrap.");
}

static void independent_tests(void)
{
    app_imu_struct snapshot;
    unsigned i, frames, transactions;
    independent_setup();
    assert(regs[MPU6050_ADDR][0x6B] == 0x28 && regs[MPU6050_ADDR][0x6C] == 0xC7);
    assert(App_IMU_Service(0) == 0 && mpu_last_frame_size == 7);
    fail_dev = MPU6050_ADDR;
    for (i = 0; i < 12; ++i)
    {
        magnetic(0, 30, 40);
        tick += 40;
        app_imu_compass_service(1, 1);
        App_IMU_Service(0);
    }
    assert(App_IMU_GetSnapshot(&snapshot) == 0);
    assert(snapshot.compass_valid != 0 && angle_error(snapshot.compass_deg, 270.0f) < 0.1f);
    /* 未校准时允许硬铁偏置导致的>100uT非溢出磁场显示指向，但<5uT仍拒绝。 */
    magnetic(0, 120, 40);
    tick += 40;
    app_imu_compass_service(1, 1);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_calibrated == 0 && snapshot.mag_online != 0 && snapshot.compass_valid != 0 &&
           snapshot.compass_state == APP_COMPASS_READY && angle_error(snapshot.compass_deg, 270.0f) < 0.1f);
    magnetic(2, 0, 0);
    tick += 40;
    app_imu_compass_service(1, 1);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_online != 0 && snapshot.compass_valid == 0 && snapshot.compass_state == APP_COMPASS_FIELD_RANGE);
    assert(regs[LSM303DLHC_ACCEL_ADDR][0x20] == 0x37 && regs[LSM303DLHC_ACCEL_ADDR][0x23] == 0x98);
    assert(regs[MPU6050_ADDR][0x6B] == 0x28 && regs[MPU6050_ADDR][0x6C] == 0xC7);
    fail_dev = LSM303DLHC_ACCEL_ADDR;
    for (i = 0; i < 3; ++i)
    {
        magnetic(0, 30, 40);
        tick += 40;
        app_imu_compass_service(1, 1);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_online != 0 && snapshot.compass_valid == 0);
    fail_dev = -1;
    for (i = 0; i < 10; ++i)
    {
        magnetic(-30, 0, 40);
        tick += 40;
        app_imu_compass_service(1, 1);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid != 0 && angle_error(snapshot.compass_deg, 0.0f) < 0.1f);
    compass_acceleration((float[3]){0.0f, 0.15f, 1.0f});
    magnetic(-30, 0, 40);
    tick += 40;
    app_imu_compass_service(1, 1);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid && angle_error(snapshot.compass_deg, 0.0f) < 4.0f);
    compass_acceleration((float[3]){0.0f, 0.0f, 1.0f});
    for (i = 0; i < 12; ++i)
    {
        magnetic(-30, 0, 40);
        tick += 40;
        app_imu_compass_service(1, 1);
    }
    /* 1.16g的一帧横向扰动只以5%权重进入重力方向。 */
    compass_acceleration((float[3]){0.0f, 0.58f, 1.0f});
    magnetic(-30, 0, 40);
    tick += 40;
    app_imu_compass_service(1, 1);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid && angle_error(snapshot.compass_deg, 0.0f) < 3.0f);
    compass_acceleration((float[3]){0.0f, 0.0f, 1.4f});
    tick += 40;
    app_imu_compass_service(1, 1);
    App_IMU_GetSnapshot(&snapshot);
    assert(!snapshot.compass_valid && snapshot.compass_state == APP_COMPASS_ACCEL_INVALID);

    /* 核对芯片倒装(Z=-1g)加180°刚体映射：倾斜与平放均保持一致航向。 */
    compass_acceleration((float[3]){0.0f, 0.0f, -1.0f});
    for (i = 0; i < 24; ++i)
    {
        magnetic(-7, -40, -25);
        tick += 40;
        app_imu_compass_service(1, 1);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid != 0 && angle_error(snapshot.compass_deg, 279.93f) < 0.2f);
    regs[LSM303DLHC_ACCEL_ADDR][0x28] = 0xD0;
    regs[LSM303DLHC_ACCEL_ADDR][0x29] = 0xEE; /* -4400 => -0.55g */
    regs[LSM303DLHC_ACCEL_ADDR][0x2C] = 0x20;
    regs[LSM303DLHC_ACCEL_ADDR][0x2D] = 0xE6; /* -6624 => -0.828g */
    for (i = 0; i < 24; ++i)
    {
        magnetic(-24.84f, 0.0f, 16.5f); /* 俯仰时北向磁场投影，同步旋转后仍指向0° */
        tick += 40;
        app_imu_compass_service(1, 1);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid != 0 && angle_error(snapshot.compass_deg, 0.0f) < 0.5f);
    regs[LSM303DLHC_ACCEL_ADDR][0x28] = 0;
    regs[LSM303DLHC_ACCEL_ADDR][0x29] = 0;
    regs[LSM303DLHC_ACCEL_ADDR][0x2C] = 0x40;
    regs[LSM303DLHC_ACCEL_ADDR][0x2D] = 0x1F;
    tick += 251;
    app_imu_compass_service(1, 0);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_online == 0 && snapshot.compass_valid == 0);
    assert(app_imu_compass_service(0, 1) == 0);
    assert(regs[LSM303DLHC_ACCEL_ADDR][0x20] == 0 && regs[LSM303DLHC_MAG_ADDR][2] == 3);
    app_imu_set_accel_reference(1);
    assert(app_imu_set_sample_period(10) == 10);
    frames = heart_motion_frames;
    for (i = 0; i < 50; ++i)
    {
        tick += 10;
        transactions = bus_transactions;
        assert(App_IMU_Service(0) == 0 && mpu_last_frame_size == 7);
        assert(bus_transactions == transactions + 1);
    }
    assert(heart_motion_frames == frames + 50);
    assert(regs[MPU6050_ADDR][0x6B] == 0x08 && regs[MPU6050_ADDR][0x6C] == 0x07);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.raw.Gyro.Gyro_x == 0 && snapshot.raw.Gyro.Gyro_y == 0 && snapshot.raw.Gyro.Gyro_z == 0);
    mpu_auto_ready = 0;
    assert(App_IMU_Service(0) == MPU6050_DATA_NOT_READY);
    app_imu_set_accel_reference(0);
    assert(app_imu_set_sample_period(25) == 25 && regs[MPU6050_ADDR][0x6B] == 0x28);
    puts("PASS: MPU offline independent compass, LSM accel recovery/freshness/power-down, two accel-only MPU modes and fresh 100Hz PPG reference.");
}

static void activity_high_rate_tests(void)
{
    app_imu_struct snapshot;
    unsigned frame, step, phase, period_ms = MPU6050_SAMPLE_PERIOD_MS;
    uint32_t motion_before;
    uint8_t reference;
    float magnitude;
    independent_setup();
    app_imu_set_accel_reference(1);
    assert(app_imu_set_sample_period(10) == 10);
    for (frame = 0; frame < 3000; ++frame)
    {
        magnitude = 1.0f + 0.5f * sinf((float)frame * 0.01f * 17.0f * 6.2831853f);
        word(MPU6050_ADDR, 0x3F, (int16_t)(magnitude * 8192.0f));
        tick += 10;
        assert(App_IMU_Service(0) == 0);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.steps == 0 && heart_motion_frames == 3000);
    for (frame = 0; frame < 3000; ++frame)
    {
        magnitude = 1.0f + 0.3f * sinf((float)frame * 0.01f * 1.25f * 6.2831853f);
        word(MPU6050_ADDR, 0x3F, (int16_t)(magnitude * 8192.0f));
        tick += 10;
        assert(App_IMU_Service(0) == 0);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.steps >= 34 && snapshot.steps <= 39 && heart_motion_frames == 6000);
    puts("PASS: 100Hz low-pass rejects 17Hz alias steps while retaining 1.25Hz gait and all raw PPG reference frames.");
    {
        static const float rates[] = {1.25f, 1.5f, 1.8f, 2.0f, 2.2f, 2.5f};
        unsigned rate, phase, elapsed, lowest, highest;
        for (rate = 0; rate < sizeof(rates) / sizeof(rates[0]); ++rate)
        {
            lowest = 100;
            highest = 0;
            for (phase = 0; phase < 40; ++phase)
            {
                independent_setup();
                app_imu_set_accel_reference(1);
                assert(app_imu_set_sample_period(10) == 10);
                motion_before = heart_motion_frames;
                for (elapsed = 0; elapsed < 1000; elapsed += 10)
                {
                    word(MPU6050_ADDR, 0x3F, 8192);
                    tick += 10;
                    assert(App_IMU_Service(0) == 0);
                }
                for (elapsed = 0; elapsed < (unsigned)(20000.0f / rates[rate]); elapsed += 10)
                {
                    magnitude = 1.0f + 0.15f * sinf(6.2831853f * (rates[rate] * elapsed / 1000.0f + phase / 40.0f));
                    word(MPU6050_ADDR, 0x3F, (int16_t)(magnitude * 8192.0f));
                    tick += 10;
                    assert(App_IMU_Service(0) == 0);
                }
                assert(heart_motion_frames - motion_before == 100 + elapsed / 10);
                for (elapsed = 0; elapsed < 500; elapsed += 10)
                {
                    word(MPU6050_ADDR, 0x3F, 8192);
                    tick += 10;
                    assert(App_IMU_Service(0) == 0);
                }
                App_IMU_GetSnapshot(&snapshot);
                if (snapshot.steps < lowest)
                {
                    lowest = snapshot.steps;
                }
                if (snapshot.steps > highest)
                {
                    highest = snapshot.steps;
                }
            }
            printf("100Hz service rate=%.2f, 40 phases: %u..%u / 20\n", rates[rate], lowest, highest);
            assert(lowest >= 19 && highest <= 20);
        }
    }

    independent_setup();
    word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, -8192);
    for (frame = 0; frame < 40; ++frame)
    {
        tick += 25;
        assert(App_IMU_Service(0) == 0);
    }
    motion_before = heart_motion_frames;
    for (step = 0; step < 20; ++step)
    {
        if (step % 5 == 0)
        {
            reference = (step / 5) % 2;
            app_imu_set_accel_reference(reference);
            period_ms = app_imu_set_sample_period(25);
        }
        for (phase = 0; phase < 500; phase += period_ms)
        {
            magnitude = 1.0f + 0.055f * sinf(6.2831853f * (float)phase / 500.0f);
            word(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, (int16_t)(-magnitude * 8192.0f));
            tick += period_ms;
            assert(App_IMU_Service(0) == 0);
        }
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.steps >= 19 && snapshot.steps <= 20 && heart_motion_frames - motion_before == 500);
    puts("PASS: low-amplitude face-up gait and raw PPG frames continue through 40/100Hz switches.");
}

static void day_tests(uint8_t retained)
{
    app_imu_struct snapshot;
    boot_step_day = 10;
    boot_steps = 1234;
    independent_setup();
    app_imu_set_day(11, retained);
    app_imu_activity_service();
    App_IMU_GetSnapshot(&snapshot);
    if (retained != 0)
    {
        assert(snapshot.steps == 0 && last_step_day == 11 && last_steps == 0);
    }
    else
    {
        assert(snapshot.steps == 1234 && last_step_day == 11 && last_steps == 1234);
    }
    app_imu_set_day(10, retained);
    app_imu_activity_service();
    assert(last_step_day == 11 && last_steps == snapshot.steps);
    app_imu_set_day(11, retained);
    app_imu_activity_service();
    assert(last_step_day == 11 && last_steps == snapshot.steps);
    app_imu_set_day(12, retained);
    app_imu_activity_service();
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.steps == 0 && last_step_day == 12 && last_steps == 0);
    puts("PASS: restored steps, retained-calendar day mismatch, cold-start first-sync binding and runtime midnight reset.");
}

static void distorted_field(const float field[3], float raw_uT[3])
{
    raw_uT[0] = 95.0f + 1.15f * field[0] + 0.24f * field[1] - 0.07f * field[2];
    raw_uT[1] = -70.0f + 0.24f * field[0] + 0.91f * field[1] + 0.11f * field[2];
    raw_uT[2] = 85.0f - 0.07f * field[0] + 0.11f * field[1] + 1.06f * field[2];
}

static void mag_ellipsoid_tests(void)
{
    common_mag_cal_struct cal = {0}, flat = {0}, oversized = {0};
    common_mag_cal_params_struct params = {0}, saved;
    static const float poses[][2] = {{0, 0}, {35, 25}, {-50, -40}, {80, 60}, {180, 0}, {-179, 15}};
    Accel_Gyro_struct accel;
    float gravity[3], field[3], raw_uT[3], corrected[3], heading, max_error = 0.0f;
    for (unsigned i = 0; i < 800; ++i)
    {
        truth(i * 7.0f, 74.0f * sinf(i * 0.041f), i * 11.0f, &accel, field, gravity);
        distorted_field(field, raw_uT);
        raw_uT[0] += 0.18f * sinf(i * 1.2f);
        raw_uT[1] += 0.18f * cosf(i * 0.7f);
        if (i == 108)
        {
            raw_uT[0] += 95.0f;
        }
        if (i == 130 || i == 326)
        {
            raw_uT[2] += 12.0f;
        }
        common_mag_cal_collect(&cal, raw_uT, gravity, 0);

        truth(0.0f, 0.0f, i * 11.0f, &accel, field, gravity);
        distorted_field(field, raw_uT);
        common_mag_cal_collect(&flat, raw_uT, gravity, 0);

        truth(i * 7.0f, 74.0f * sinf(i * 0.041f), i * 11.0f, &accel, field, gravity);
        distorted_field((float[3]){2.0f * field[0], 2.0f * field[1], 2.0f * field[2]}, raw_uT);
        common_mag_cal_collect(&oversized, raw_uT, NULL, 0);
    }
    assert(cal.count == COMMON_MAG_CAL_MAX_POINTS && cal.coverage_pct >= 69);
    assert(common_mag_cal_fit(&cal, &params) == 0);
    assert(params.dip_valid && fabsf(params.dip_ref_deg - 53.13f) < 0.5f);
    for (unsigned i = 0; i < sizeof(poses) / sizeof(poses[0]); ++i)
    {
        for (unsigned yaw = 0; yaw < 360; yaw += 15)
        {
            truth(poses[i][0], poses[i][1], (float)yaw, &accel, field, gravity);
            distorted_field(field, raw_uT);
            common_mag_cal_apply(&params, raw_uT, corrected);
            assert(common_imu_compass_heading(&accel.Accel, corrected, &heading) == 0);
            max_error = fmaxf(max_error, angle_error(heading, -(float)yaw));
        }
    }
    assert(max_error < 1.0f);
    saved = params;
    assert(common_mag_cal_fit(&flat, &params) != 0);
    assert(common_mag_cal_fit(&oversized, &params) != 0);
    cal.samples = 99;
    assert(common_mag_cal_fit(&cal, &params) != 0);
    assert(mag_params_match(&saved, &params));
    printf("PASS: hard/soft iron, noise/outliers, 144 poses max heading error %.3f deg, flat/oversized/short rejection.\n", max_error);
}

static void compass_field_sample(float x, float y, float z)
{
    float raw_uT[3];
    distorted_field((float[3]){x, y, z}, raw_uT);
    magnetic(raw_uT[0], raw_uT[1], raw_uT[2]);
    tick += 40;
    app_imu_compass_service(1, 1);
}

static void mag_calibration_tests(uint8_t fail_save, uint8_t reference)
{
    app_imu_struct snapshot;
    app_mag_calibration_struct saved;
    Accel_Gyro_struct accel;
    unsigned i;
    float field[3], gravity[3];
    independent_setup();
    if (reference != 0)
    {
        app_imu_set_accel_reference(1);
        assert(app_imu_set_sample_period(10) == 10);
    }
    App_IMU_MagCalibrate(1);
    for (i = 0; i < 640; ++i)
    {
        truth(i * 5.0f, 74.0f * sinf(i * 0.041f), i * 7.0f, &accel, field, gravity);
        compass_acceleration(gravity);
        compass_field_sample(field[0], field[1], field[2]);
        App_IMU_Service(0);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_cal_state == APP_MAG_CAL_RUNNING && snapshot.compass_valid == 0);
    assert(snapshot.mag_cal_samples >= 100 && snapshot.mag_cal_coverage_pct >= 69);
    App_IMU_MagCalibrate(0);
    tick += 40;
    app_imu_compass_service(1, 0);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_calibrated && snapshot.mag_cal_is_ellipsoid && snapshot.mag_cal_state == APP_MAG_CAL_DONE && !snapshot.mag_online);
    compass_acceleration((float[3]){0.0f, 0.0f, 1.0f});
    compass_field_sample(0, 72, 40);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_calibrated != 0 && snapshot.mag_online != 0 && snapshot.compass_valid == 0 &&
           snapshot.compass_state == APP_COMPASS_FIELD_RANGE);
    for (i = 0; i < 4; ++i)
    {
        compass_field_sample(-21.6f, 0, 28.8f); /* 36μT已回外窗，未进入内窗。 */
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(!snapshot.compass_valid && snapshot.compass_state == APP_COMPASS_FIELD_RANGE);
    for (i = 0; i < 3; ++i)
    {
        compass_field_sample(-30, 0, 40);
        App_IMU_GetSnapshot(&snapshot);
        if (i < 2)
        {
            assert(!snapshot.compass_valid && snapshot.compass_state == APP_COMPASS_FIELD_RANGE);
        }
    }
    for (i = 0; i < 20; ++i)
    {
        compass_field_sample(-30, 0, 40);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid && snapshot.mag_declination_deg == -2.0f && angle_error(snapshot.compass_deg, 358.0f) < 0.5f);
    compass_field_sample(-30.0f * cosf(2.0f / RtA), -30.0f * sinf(2.0f / RtA), 40.0f);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid && angle_error(snapshot.compass_deg, 0.0f) < 0.5f);

    compass_field_sample(-46.0f, 0, 19.6f); /* 模长约50μT，倾角与参考相差约30°。 */
    App_IMU_GetSnapshot(&snapshot);
    assert(!snapshot.compass_valid && snapshot.compass_state == APP_COMPASS_INTERFERENCE);
    compass_field_sample(-30, 0, 40);
    for (i = 0; i < 3; ++i)
    {
        tick += 5;
        app_imu_compass_service(1, 0);
    }
    App_IMU_GetSnapshot(&snapshot);
    assert(!snapshot.compass_valid && snapshot.compass_state == APP_COMPASS_INTERFERENCE);
    compass_field_sample(-30, 0, 40);
    App_IMU_GetSnapshot(&snapshot);
    assert(!snapshot.compass_valid && snapshot.compass_state == APP_COMPASS_INTERFERENCE);
    compass_field_sample(-30, 0, 40);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid);
    app_imu_set_declination(0.0f);
    compass_field_sample(-30, 0, 40);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid && snapshot.mag_declination_deg == 0.0f && angle_error(snapshot.compass_deg, 0.0f) < 0.5f);

    magnetic(0, 0, 0);
    word(LSM303DLHC_MAG_ADDR, 3, -4096);
    tick += 40;
    app_imu_compass_service(1, 1);
    App_IMU_GetSnapshot(&snapshot);
    assert(!snapshot.compass_valid && !snapshot.mag_online && snapshot.compass_state == APP_COMPASS_FIELD_RANGE);
    if (fail_save != 0)
    {
        eeprom_fail_write_after = 0;
    }
    for (i = 0; i < 40; ++i)
    {
        tick += 20;
        App_IMU_SaveService();
    }
    App_IMU_GetSnapshot(&snapshot);
    if (fail_save != 0)
    {
        assert(snapshot.mag_persist_state == APP_MAG_PERSIST_SAVE_FAILED);
    }
    else
    {
        assert(snapshot.mag_persist_state == APP_MAG_PERSIST_SAVED);
        assert(app_mag_store_load(&saved) == APP_MAG_STORE_OK && saved.is_ellipsoid && saved.params.dip_valid);
        assert(fabsf(saved.params.dip_ref_deg - 53.13f) < 0.5f);
    }
    App_IMU_MagCalibrate(1);
    compass_field_sample(-30, 0, 40);
    App_IMU_MagCalibrate(0);
    app_imu_compass_service(1, 0);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_cal_state == APP_MAG_CAL_FAILED && snapshot.mag_cal_is_ellipsoid);
    assert(App_IMU_SaveService() == 0);
    compass_field_sample(-30, 0, 40);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.compass_valid && angle_error(snapshot.compass_deg, 0.0f) < 0.5f);
    if (reference != 0)
    {
        assert(regs[MPU6050_ADDR][0x6B] == 0x08 && regs[MPU6050_ADDR][0x6C] == 0x07);
    }
    else
    {
        assert(regs[MPU6050_ADDR][0x6B] == 0x28 && regs[MPU6050_ADDR][0x6C] == 0xC7);
    }
    puts("PASS: MAG2 calibration, declination, field/dip hysteresis on fresh frames, immediate saturation and EEPROM save state.");
}

static void mag_screen_off_tests(void)
{
    app_imu_struct snapshot;
    unsigned i;
    regs[LSM303DLHC_MAG_ADDR][0x0A] = 0x48;
    regs[LSM303DLHC_MAG_ADDR][0x0B] = 0x34;
    regs[LSM303DLHC_MAG_ADDR][0x0C] = 0x33;
    app_imu_init_mag_hw();
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0x03);
    App_IMU_Init();
    App_IMU_MagCalibrate(1);
    for (i = 0; i < 5; ++i)
    {
        app_imu_compass_service(1, 1);
        tick += 40;
    }
    assert(app_imu_mag_cal_active() && regs[LSM303DLHC_MAG_ADDR][0x02] == 0);
    app_imu_cancel_mag_calibration();
    assert(app_imu_compass_service(0, 1) == 0);
    App_IMU_GetSnapshot(&snapshot);
    assert(snapshot.mag_cal_state == APP_MAG_CAL_FAILED && !app_imu_mag_cal_active() && !App_IMU_SaveService());
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0x03 && regs[LSM303DLHC_ACCEL_ADDR][0x20] == 0);
    lsm303dlhc_mag_recover_begin();
    for (i = 0; i < 4; ++i)
    {
        app_imu_compass_service(0, 1);
        assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0x03 && regs[LSM303DLHC_ACCEL_ADDR][0x20] == 0);
    }
    fail_dev = LSM303DLHC_MAG_ADDR;
    lsm303dlhc_mag_recover_begin();
    assert(app_imu_compass_service(0, 1) == APP_COMPASS_SERVICE_RETRY);
    for (i = 0; i < 480; ++i)
    {
        app_compass_service_state work;
        tick += 25;
        work = app_imu_compass_service(0, 1);
        if (work == APP_COMPASS_SERVICE_BUSY)
        {
            tick += 25;
            ++i;
            assert(app_imu_compass_service(0, 1) == APP_COMPASS_SERVICE_RETRY);
        }
        else
        {
            assert(work == APP_COMPASS_SERVICE_RETRY);
        }
    }
    fail_dev = -1;
    for (i = 0; i < 8; ++i)
    {
        tick += 400;
        app_imu_compass_service(0, 1);
    }
    assert(app_imu_compass_service(0, 1) == APP_COMPASS_SERVICE_IDLE);
    assert(regs[LSM303DLHC_MAG_ADDR][0x02] == 0x03 && regs[LSM303DLHC_ACCEL_ADDR][0x20] == 0);
    puts("PASS: screen-off cancels unfinished magnetic calibration, allows MOT during compass cooldown and recovers the magnetometer asleep.");
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "compass") == 0)
        {
            compass_tests();
        }
        else if (strcmp(argv[1], "mag_ellipsoid") == 0)
        {
            mag_ellipsoid_tests();
        }
        else if (strcmp(argv[1], "accel_warmup") == 0)
        {
            accel_warmup_tests();
        }
        else if (strcmp(argv[1], "independent") == 0)
        {
            independent_tests();
        }
        else if (strcmp(argv[1], "day_cold") == 0 || strcmp(argv[1], "day_retained") == 0)
        {
            day_tests(strcmp(argv[1], "day_retained") == 0);
        }
        else if (strcmp(argv[1], "mag_calibration") == 0 || strcmp(argv[1], "mag_save_failed") == 0 || strcmp(argv[1], "mag_calibration_reference") == 0)
        {
            mag_calibration_tests(strcmp(argv[1], "mag_save_failed") == 0, strcmp(argv[1], "mag_calibration_reference") == 0);
        }
        else if (strcmp(argv[1], "activity_high_rate") == 0)
        {
            activity_high_rate_tests();
        }
        else if (strcmp(argv[1], "activity") == 0)
        {
            activity_tests();
        }
        else if (strcmp(argv[1], "uptime") == 0)
        {
            uptime_tests();
        }
        else if (strcmp(argv[1], "async_save") == 0)
        {
            async_save_tests();
            euler_decimation_tests();
        }
        else if (strcmp(argv[1], "mpu_stale") == 0)
        {
            mpu_stale_recovery_tests();
        }
        else if (strcmp(argv[1], "mpu_mot") == 0)
        {
            mpu_mot_tests();
            mpu_mot_fallback_tests();
            mpu_mot_first_step_tests();
            activity_mot_tests();
        }
        else if (strcmp(argv[1], "mag_screen_off") == 0)
        {
            mag_screen_off_tests();
        }
        else if (strcmp(argv[1], "mpu_sleep_recovery") == 0)
        {
            mpu_sleep_recovery_tests();
        }
        else if (strcmp(argv[1], "mag_sleep_recovery") == 0)
        {
            mag_sleep_recovery_tests();
        }
        else
        {
            assert(strcmp(argv[1], "recovery") == 0);
            recovery_tests();
        }
        return 0;
    }
    spl006_init_tests();
    spl006_sample_tests();
    baro_service_tests();
    baro_calibration_tests();
    spl006_sleep_recovery_tests();
    mpu_init_error_tests();
    init_delay_tests();
    eeprom_tests();
    imu_eeprom_tests();
    imu_save_diagnostic_tests();
    driver_tests();
    fusion_tests();
    return 0;
}
