/* Real driver/service/math, fake bus. Inject EXTI while an I2C transaction is active. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_imu.h"
#include "common_imu.h"
#include "my_i2c.h"
#include "spl006.h"
#include "semphr.h"
#include "bl24c02.h"
#include "app_mag_store.h"

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
static uint32_t primask;
static uint8_t bus_active, mutex_taken;
static int fail_dev = -1, fail_reg = -1;
static unsigned reads, mag_frames, mag_config_writes;
static int spl_fail_read_at = -1, spl_fail_write_at = -1;
static unsigned spl_reads, spl_writes;
static uint8_t spl_reset_status = SPL06_SENSOR_RDY | SPL06_COEF_RDY;
static TickType_t motion_at_tick;
static uint8_t eeprom[BL24C02_CAPACITY_BYTES];
static uint8_t eeprom_write_protected;
static TickType_t eeprom_busy_until;
static int eeprom_fail_write_after = -1;
static unsigned eeprom_writes;
static void word(uint8_t dev, uint8_t reg, int16_t value);

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
    tick += ms;
}
void Error_Handler(void)
{
    abort();
}
TickType_t xTaskGetTickCount(void)
{
    return tick;
}
void vTaskDelay(TickType_t ticks)
{
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
    if (dev == SPL006_ADDR)
    {
        error |= (int)spl_reads++ == spl_fail_read_at;
    }
    if (bus == MY_I2C_BUS_EEPROM)
    {
        assert(dev == BL24C02_ADDR && (unsigned)reg + len <= sizeof(eeprom));
        assert(tick >= eeprom_busy_until);
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
            eeprom_busy_until = tick + 3;
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
    word(0x68, 0x3B, 1000);
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
static void fusion_tests(void)
{
    const float cases[][3] = {{0, 0, 90}, {35, 25, 90}, {-40, 30, -135}, {20, -55, 179}, {0, 0, -179}, {45, 45, 0}};
    Accel_Gyro_struct raw;
    Euler_struct euler = {0};
    float mag[3], gravity[3], without_mag, with_mag, before;
    unsigned c, n;
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

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "save_failed") != 0)
    {
        restore_tests(strcmp(argv[1], "load_failed") == 0);
        return 0;
    }
    spl006_init_tests();
    eeprom_tests();
    driver_tests();
    calibration_tests();
    fusion_tests();
    service_tests(argc > 1);
    return 0;
}
