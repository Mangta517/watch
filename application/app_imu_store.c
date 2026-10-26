/** MPU1记录28字节：格式、序号、量程、六轴int16零偏、CRC32。
 *  两个32字节槽位在0x60/0x80，更新失败保留旧副本；不覆盖磁力计记录。 */
#include "app_imu_store.h"
#include "app_diag.h"
#include "bl24c02.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define IMU_STORE_MAGIC        0x3155504D /* 小端字符MPU1 */
#define IMU_STORE_RECORD_BYTES 28
#define IMU_STORE_CRC_OFFSET   24
#define IMU_STORE_SLOT_BYTES   32
#define IMU_STORE_BASE         0x60
#define IMU_STORE_RANGE        ((uint32_t)MPU6050_ACCEL_RANGE << 8 | MPU6050_GYRO_RANGE)

static void imu_store_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (i * 8));
    }
}

static uint32_t imu_store_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t imu_store_crc(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            if ((crc & 1) != 0)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

static void imu_store_encode(uint8_t *record, uint32_t sequence, const mpu6050_offset_struct *offset)
{
    uint8_t i;
    uint16_t value;
    imu_store_put_u32(record, IMU_STORE_MAGIC);
    imu_store_put_u32(record + 4, sequence);
    imu_store_put_u32(record + 8, IMU_STORE_RANGE);
    for (i = 0; i < 3; ++i)
    {
        value = (uint16_t)offset->accel[i];
        record[12 + i * 2] = (uint8_t)value;
        record[13 + i * 2] = (uint8_t)(value >> 8);
        value = (uint16_t)offset->gyro[i];
        record[18 + i * 2] = (uint8_t)value;
        record[19 + i * 2] = (uint8_t)(value >> 8);
    }
    imu_store_put_u32(record + IMU_STORE_CRC_OFFSET, imu_store_crc(record, IMU_STORE_CRC_OFFSET));
}

static uint8_t imu_store_decode(const uint8_t *record, mpu6050_offset_struct *offset)
{
    uint8_t i;
    if (imu_store_get_u32(record) != IMU_STORE_MAGIC || imu_store_get_u32(record + 8) != IMU_STORE_RANGE ||
        imu_store_get_u32(record + IMU_STORE_CRC_OFFSET) != imu_store_crc(record, IMU_STORE_CRC_OFFSET))
    {
        return 1;
    }
    for (i = 0; i < 3; ++i)
    {
        offset->accel[i] = (int16_t)((uint16_t)record[12 + i * 2] | ((uint16_t)record[13 + i * 2] << 8));
        offset->gyro[i] = (int16_t)((uint16_t)record[18 + i * 2] | ((uint16_t)record[19 + i * 2] << 8));
    }
    return 0;
}

/* 两份都有效时按无符号序号差选择较新记录，兼容32位序号回绕。 */
static uint8_t imu_store_latest(const uint8_t *first, const uint8_t *second, const uint8_t valid[2])
{
    uint32_t difference;
    if (!valid[0])
    {
        return 1;
    }
    if (!valid[1])
    {
        return 0;
    }
    difference = imu_store_get_u32(second + 4) - imu_store_get_u32(first + 4);
    return difference != 0 && difference < 0x80000000U;
}

app_imu_store_result app_imu_store_load(mpu6050_offset_struct *offset)
{
    uint8_t record[2][IMU_STORE_RECORD_BYTES], valid[2] = {0};
    uint8_t i, read_error = 0;
    mpu6050_offset_struct candidate;
    if (offset == NULL)
    {
        return APP_IMU_STORE_ERROR;
    }
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(IMU_STORE_BASE + i * IMU_STORE_SLOT_BYTES, record[i], IMU_STORE_RECORD_BYTES) == 0)
        {
            valid[i] = imu_store_decode(record[i], &candidate) == 0;
        }
        else
        {
            read_error = 1;
        }
    }
    if (!valid[0] && !valid[1])
    {
        if (read_error != 0)
        {
            return APP_IMU_STORE_ERROR;
        }
        return APP_IMU_STORE_EMPTY;
    }
    (void)imu_store_decode(record[imu_store_latest(record[0], record[1], valid)], offset);
    return APP_IMU_STORE_OK;
}

/* 保存结果单独记录；不增加I2C事务或串口发送，只发布给原诊断任务。 */
static app_imu_store_result imu_store_save_finish(app_imu_save_stage stage, uint32_t started_ms, uint32_t address)
{
    bl24c02_error error;
    uint16_t driver_address;
    if (stage == APP_IMU_SAVE_ARGUMENT)
    {
        error = BL24C02_ERROR_ARGUMENT;
    }
    else
    {
        error = bl24c02_get_error(&driver_address);
        if (error != BL24C02_ERROR_NONE)
        {
            address = driver_address;
        }
    }
    app_diag_imu_save_record((uint32_t)stage, (uint32_t)error, address, HAL_GetTick() - started_ms);
    if (stage == APP_IMU_SAVE_DONE)
    {
        return APP_IMU_STORE_OK;
    }
    return APP_IMU_STORE_ERROR;
}

/* sensor串行推进；两份旧记录读完后才选择写入另一槽位。 */
static struct
{
    bl24c02_transfer_struct io;
    mpu6050_offset_struct value;
    uint8_t record[2][IMU_STORE_RECORD_BYTES], verify[IMU_STORE_RECORD_BYTES], valid[2];
    uint32_t started_ms;
    uint16_t address;
    uint8_t phase, target;
    app_imu_store_result result;
} s_save;

app_imu_store_result app_imu_store_save_begin(const mpu6050_offset_struct *offset)
{
    if (s_save.result == APP_IMU_STORE_BUSY)
    {
        return APP_IMU_STORE_BUSY;
    }
    if (offset == NULL)
    {
        return imu_store_save_finish(APP_IMU_SAVE_ARGUMENT, HAL_GetTick(), UINT32_MAX);
    }
    s_save.value = *offset;
    s_save.started_ms = HAL_GetTick();
    s_save.phase = 0;
    s_save.result = APP_IMU_STORE_BUSY;
    bl24c02_transfer_begin(&s_save.io, IMU_STORE_BASE, s_save.record[0], IMU_STORE_RECORD_BYTES, 0);
    return s_save.result;
}

app_imu_store_result app_imu_store_save_step(void)
{
    mpu6050_offset_struct candidate;
    uint32_t sequence = 1;
    uint8_t latest, result;
    if (s_save.result != APP_IMU_STORE_BUSY)
    {
        return s_save.result;
    }
    result = bl24c02_transfer_step(&s_save.io);
    if (result == 2)
    {
        return APP_IMU_STORE_BUSY;
    }
    if (result != 0)
    {
        static const app_imu_save_stage failures[] = {APP_IMU_SAVE_READ_SLOT0, APP_IMU_SAVE_READ_SLOT1,
                                                      APP_IMU_SAVE_WRITE, APP_IMU_SAVE_VERIFY_READ};
        s_save.result = imu_store_save_finish(failures[s_save.phase], s_save.started_ms, s_save.io.address);
        return s_save.result;
    }
    if (s_save.phase < 2)
    {
        s_save.valid[s_save.phase] = imu_store_decode(s_save.record[s_save.phase], &candidate) == 0;
        if (s_save.phase++ == 0)
        {
            bl24c02_transfer_begin(&s_save.io, IMU_STORE_BASE + IMU_STORE_SLOT_BYTES, s_save.record[1], IMU_STORE_RECORD_BYTES, 0);
            return APP_IMU_STORE_BUSY;
        }
        s_save.target = 0;
        if (s_save.valid[0] || s_save.valid[1])
        {
            latest = imu_store_latest(s_save.record[0], s_save.record[1], s_save.valid);
            sequence = imu_store_get_u32(s_save.record[latest] + 4) + 1;
            s_save.target = latest ^ 1;
        }
        imu_store_encode(s_save.record[s_save.target], sequence, &s_save.value);
        s_save.address = IMU_STORE_BASE + s_save.target * IMU_STORE_SLOT_BYTES;
        bl24c02_transfer_begin(&s_save.io, s_save.address, s_save.record[s_save.target], IMU_STORE_RECORD_BYTES, 1);
    }
    else if (s_save.phase == 2)
    {
        s_save.phase = 3;
        bl24c02_transfer_begin(&s_save.io, s_save.address, s_save.verify, IMU_STORE_RECORD_BYTES, 0);
    }
    else
    {
        if (memcmp(s_save.verify, s_save.record[s_save.target], IMU_STORE_RECORD_BYTES) != 0 ||
            imu_store_decode(s_save.verify, &candidate) != 0)
        {
            s_save.result = imu_store_save_finish(APP_IMU_SAVE_VERIFY_DATA, s_save.started_ms, s_save.address);
        }
        else
        {
            s_save.result = imu_store_save_finish(APP_IMU_SAVE_DONE, s_save.started_ms, s_save.address);
        }
    }
    return s_save.result;
}

/* 旧调用方兼容入口；运行期保存只使用begin/step。 */
app_imu_store_result app_imu_store_save(const mpu6050_offset_struct *offset)
{
    app_imu_store_result result = app_imu_store_save_begin(offset);
    while (result == APP_IMU_STORE_BUSY)
    {
        result = app_imu_store_save_step();
        if (result == APP_IMU_STORE_BUSY)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return result;
}
