/**
 * @file app_mag_store.c
 * @brief 两份磁校准记录轮换写入，保留旧副本；CRC32和读回校验确认保存结果。
 *        记录36字节：MAG1(格式版本) + 序号 + 6个float32 + CRC32，按小端编码。
 *        槽位0x00/0x30，各保留48字节；0x60..0xFF不用于本模块。
 */
#include "app_mag_store.h"
#include "bl24c02.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MAG_STORE_MAGIC        0x3147414DU /* 小端字符MAG1 */
#define MAG_STORE_RECORD_BYTES 36U
#define MAG_STORE_CRC_OFFSET   32U
#define MAG_STORE_SLOT_BYTES   48U

typedef char mag_store_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

static void mag_store_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (i * 8));
    }
}

static uint32_t mag_store_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t mag_store_crc(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

static uint8_t mag_store_values_valid(const app_mag_calibration_struct *calibration)
{
    uint8_t i;
    for (i = 0; i < 3; ++i)
    {
        if (!isfinite(calibration->offset_uT[i]) || !isfinite(calibration->scale[i]) || calibration->scale[i] <= 0.0f)
        {
            return 0;
        }
    }
    return 1;
}

static void mag_store_encode(uint8_t *record, uint32_t sequence, const app_mag_calibration_struct *calibration)
{
    uint8_t i;
    uint32_t bits;
    mag_store_put_u32(record, MAG_STORE_MAGIC);
    mag_store_put_u32(record + 4, sequence);
    for (i = 0; i < 3; ++i)
    {
        memcpy(&bits, &calibration->offset_uT[i], 4);
        mag_store_put_u32(record + 8 + i * 4, bits);
        memcpy(&bits, &calibration->scale[i], 4);
        mag_store_put_u32(record + 20 + i * 4, bits);
    }
    mag_store_put_u32(record + MAG_STORE_CRC_OFFSET, mag_store_crc(record, MAG_STORE_CRC_OFFSET));
}

static uint8_t mag_store_decode(const uint8_t *record, app_mag_calibration_struct *calibration)
{
    uint8_t i;
    uint32_t bits;
    app_mag_calibration_struct candidate;
    if (mag_store_get_u32(record) != MAG_STORE_MAGIC ||
        mag_store_get_u32(record + MAG_STORE_CRC_OFFSET) != mag_store_crc(record, MAG_STORE_CRC_OFFSET))
    {
        return 0;
    }
    for (i = 0; i < 3; ++i)
    {
        bits = mag_store_get_u32(record + 8 + i * 4);
        memcpy(&candidate.offset_uT[i], &bits, 4);
        bits = mag_store_get_u32(record + 20 + i * 4);
        memcpy(&candidate.scale[i], &bits, 4);
    }
    if (!mag_store_values_valid(&candidate))
    {
        return 0;
    }
    *calibration = candidate;
    return 1;
}

/* 两份都有效时按无符号序号差选择较新记录，兼容32位序号回绕。 */
static uint8_t mag_store_latest(const uint8_t *first, const uint8_t *second, const uint8_t valid[2])
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
    difference = mag_store_get_u32(second + 4) - mag_store_get_u32(first + 4);
    return difference != 0 && difference < 0x80000000U;
}

app_mag_store_result app_mag_store_load(app_mag_calibration_struct *calibration)
{
    uint8_t record[2][MAG_STORE_RECORD_BYTES], valid[2] = {0};
    uint8_t i, read_error = 0;
    app_mag_calibration_struct candidate;
    if (calibration == NULL)
    {
        return APP_MAG_STORE_ERROR;
    }
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(i * MAG_STORE_SLOT_BYTES, record[i], MAG_STORE_RECORD_BYTES) == 0)
        {
            valid[i] = mag_store_decode(record[i], &candidate);
        }
        else
        {
            read_error = 1;
        }
    }
    if (!valid[0] && !valid[1])
    {
        return read_error ? APP_MAG_STORE_ERROR : APP_MAG_STORE_EMPTY;
    }
    (void)mag_store_decode(record[mag_store_latest(record[0], record[1], valid)], calibration);
    return APP_MAG_STORE_OK;
}

app_mag_store_result app_mag_store_save(const app_mag_calibration_struct *calibration)
{
    uint8_t record[2][MAG_STORE_RECORD_BYTES], verify[MAG_STORE_RECORD_BYTES], valid[2];
    uint8_t i, latest, target;
    uint32_t sequence = 1;
    app_mag_calibration_struct candidate;
    if (calibration == NULL || !mag_store_values_valid(calibration))
    {
        return APP_MAG_STORE_ERROR;
    }
    for (i = 0; i < 2; ++i)
    {
        /* 保存前必须读到两个槽位，不能把通信失败误当作空槽而覆盖旧记录。 */
        if (bl24c02_read(i * MAG_STORE_SLOT_BYTES, record[i], MAG_STORE_RECORD_BYTES) != 0)
        {
            return APP_MAG_STORE_ERROR;
        }
        valid[i] = mag_store_decode(record[i], &candidate);
    }
    target = 0;
    if (valid[0] || valid[1])
    {
        latest = mag_store_latest(record[0], record[1], valid);
        sequence = mag_store_get_u32(record[latest] + 4) + 1U;
        target = latest ^ 1U;
    }
    mag_store_encode(record[target], sequence, calibration);
    if (bl24c02_write(target * MAG_STORE_SLOT_BYTES, record[target], MAG_STORE_RECORD_BYTES) != 0 ||
        bl24c02_read(target * MAG_STORE_SLOT_BYTES, verify, MAG_STORE_RECORD_BYTES) != 0 ||
        memcmp(verify, record[target], MAG_STORE_RECORD_BYTES) != 0 || !mag_store_decode(verify, &candidate))
    {
        return APP_MAG_STORE_ERROR;
    }
    return APP_MAG_STORE_OK;
}
