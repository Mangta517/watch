/** MPU1记录28字节：格式、序号、量程、六轴int16零偏、CRC32。
 *  两个32字节槽位在0x60/0x80，更新失败保留旧副本；不覆盖磁力计记录。 */
#include "app_imu_store.h"
#include "app_diag.h"
#include "bl24c02.h"
#include <stddef.h>
#include <string.h>

#define IMU_STORE_MAGIC        0x3155504DU /* 小端字符MPU1 */
#define IMU_STORE_RECORD_BYTES 28U
#define IMU_STORE_CRC_OFFSET   24U
#define IMU_STORE_SLOT_BYTES   32U
#define IMU_STORE_BASE         0x60U
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
            if ((crc & 1U) != 0)
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
        return 0;
    }
    for (i = 0; i < 3; ++i)
    {
        offset->accel[i] = (int16_t)((uint16_t)record[12 + i * 2] | ((uint16_t)record[13 + i * 2] << 8));
        offset->gyro[i] = (int16_t)((uint16_t)record[18 + i * 2] | ((uint16_t)record[19 + i * 2] << 8));
    }
    return 1;
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
            valid[i] = imu_store_decode(record[i], &candidate);
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

app_imu_store_result app_imu_store_save(const mpu6050_offset_struct *offset)
{
    uint8_t record[2][IMU_STORE_RECORD_BYTES], verify[IMU_STORE_RECORD_BYTES], valid[2];
    uint8_t i, latest, target;
    uint32_t sequence = 1;
    uint32_t started_ms = HAL_GetTick();
    uint16_t address;
    mpu6050_offset_struct candidate;
    if (offset == NULL)
    {
        return imu_store_save_finish(APP_IMU_SAVE_ARGUMENT, started_ms, UINT32_MAX);
    }
    for (i = 0; i < 2; ++i)
    {
        /* 保存前必须读到两个槽位，不能把通信失败误当作空槽而覆盖旧记录。 */
        if (bl24c02_read(IMU_STORE_BASE + i * IMU_STORE_SLOT_BYTES, record[i], IMU_STORE_RECORD_BYTES) != 0)
        {
            return imu_store_save_finish((app_imu_save_stage)(APP_IMU_SAVE_READ_SLOT0 + i), started_ms,
                                         IMU_STORE_BASE + i * IMU_STORE_SLOT_BYTES);
        }
        valid[i] = imu_store_decode(record[i], &candidate);
    }
    target = 0;
    if (valid[0] || valid[1])
    {
        latest = imu_store_latest(record[0], record[1], valid);
        sequence = imu_store_get_u32(record[latest] + 4) + 1U;
        target = latest ^ 1U;
    }
    imu_store_encode(record[target], sequence, offset);
    address = IMU_STORE_BASE + target * IMU_STORE_SLOT_BYTES;
    if (bl24c02_write(address, record[target], IMU_STORE_RECORD_BYTES) != 0)
    {
        return imu_store_save_finish(APP_IMU_SAVE_WRITE, started_ms, address);
    }
    if (bl24c02_read(address, verify, IMU_STORE_RECORD_BYTES) != 0)
    {
        return imu_store_save_finish(APP_IMU_SAVE_VERIFY_READ, started_ms, address);
    }
    if (memcmp(verify, record[target], IMU_STORE_RECORD_BYTES) != 0 || !imu_store_decode(verify, &candidate))
    {
        return imu_store_save_finish(APP_IMU_SAVE_VERIFY_DATA, started_ms, address);
    }
    return imu_store_save_finish(APP_IMU_SAVE_DONE, started_ms, address);
}
