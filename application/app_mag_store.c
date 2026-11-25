/**
 * @file app_mag_store.c
 * @brief 两份磁校准记录轮换写入，保留旧副本；CRC32和读回校验确认保存结果。
 *        MAG2: 格式字、16位序号、磁倾角(0.1°)、9个float32、CRC32，按小端编码。
 *        兼容36字节MAG1的32位序号、三轴偏置和三轴比例。
 *        槽位0x00/0x30，各保留48字节；0x60..0xFF不用于本模块。
 */
#include "app_mag_store.h"
#include "bl24c02.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MAG_STORE_MAGIC_V1     0x3147414D /* 小端字符MAG1 */
#define MAG_STORE_MAGIC_V2     0x3247414D /* 小端字符MAG2 */
#define MAG_STORE_CRC_V1       32
#define MAG_STORE_CRC_V2       44
#define MAG_STORE_RECORD_BYTES 48
#define MAG_STORE_SLOT_BYTES   48

typedef char mag_store_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

/* 小端写32位值：第i字节取value从8*i位起的低8位，共写4字节。 */
static void mag_store_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    /* 从最低字节开始逐字节写入。 */
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (i * 8));
    }
}

/* 小端读32位值：data[0]为最低字节，依次左移8/16/24位后按位或。 */
static uint32_t mag_store_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void mag_store_put_float(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits)); /* 按位保持float32的EEPROM格式。 */
    mag_store_put_u32(data, bits);
}

static float mag_store_get_float(const uint8_t *data)
{
    uint32_t bits = mag_store_get_u32(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* 对length字节计算CRC32，初值与最终异或均为0xFFFFFFFF。 */
static uint32_t mag_store_crc(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint16_t i;
    uint8_t bit;
    /* 逐字节异或进CRC低8位。 */
    for (i = 0; i < length; ++i)
    {
        crc ^= data[i];
        /* 每字节处理8位：最低位为1时右移并异或多项式0xEDB88320，为0时右移后异或0(即只右移)。 */
        for (bit = 0; bit < 8; ++bit)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc = (crc >> 1) ^ 0;
            }
        }
    }
    /* 结果按位取反后返回。 */
    return crc ^ 0xFFFFFFFFU;
}

static uint8_t mag_store_values_valid(const app_mag_calibration_struct *calibration)
{
    const common_mag_cal_params_struct *params = &calibration->params;
    const float *w = params->soft_sym;
    uint8_t i;
    for (i = 0; i < 3; ++i)
    {
        if (!isfinite(params->offset_uT[i]) || !isfinite(w[i]) || w[i] <= 0.0f)
        {
            return 0;
        }
    }
    if (calibration->is_ellipsoid != 0)
    {
        double xx = w[0], yy = w[1], zz = w[2], xy = w[3], xz = w[4], yz = w[5];
        double minor = xx * yy - xy * xy;
        double det = minor * zz + 2.0 * xy * xz * yz - xx * yz * yz - yy * xz * xz;
        if (!isfinite(xy) || !isfinite(xz) || !isfinite(yz) || !(minor > 0.0 && det > 0.0) ||
            (params->dip_valid != 0 && !(fabsf(params->dip_ref_deg) <= 90.0f)))
        {
            return 0;
        }
    }
    return 1;
}

static void mag_store_encode(uint8_t *record, uint32_t sequence, const app_mag_calibration_struct *calibration)
{
    const common_mag_cal_params_struct *params = &calibration->params;
    int16_t dip_x10 = INT16_MIN;
    uint8_t i;
    if (params->dip_valid != 0)
    {
        dip_x10 = (int16_t)roundf(params->dip_ref_deg * 10.0f);
    }
    mag_store_put_u32(record, MAG_STORE_MAGIC_V2);
    mag_store_put_u32(record + 4, (uint16_t)sequence | ((uint32_t)(uint16_t)dip_x10 << 16));
    for (i = 0; i < 3; ++i)
    {
        mag_store_put_float(record + 8 + i * 4, params->offset_uT[i]);
    }
    for (i = 0; i < 6; ++i)
    {
        mag_store_put_float(record + 20 + i * 4, params->soft_sym[i]);
    }
    mag_store_put_u32(record + MAG_STORE_CRC_V2, mag_store_crc(record, MAG_STORE_CRC_V2));
}

/* 校验格式字和CRC，按位还原三轴float并检查数值；通过才写calibration，返回1表示失败、0表示成功。 */
static uint8_t mag_store_decode(const uint8_t *record, app_mag_calibration_struct *calibration)
{
    uint8_t i, crc_offset, count;
    uint32_t magic = mag_store_get_u32(record);
    int16_t dip_x10;
    app_mag_calibration_struct candidate = {0};
    if (magic == MAG_STORE_MAGIC_V2)
    {
        candidate.is_ellipsoid = 1;
        crc_offset = MAG_STORE_CRC_V2;
        count = 6;
        dip_x10 = (int16_t)(mag_store_get_u32(record + 4) >> 16);
        if (dip_x10 != INT16_MIN)
        {
            candidate.params.dip_ref_deg = dip_x10 * 0.1f;
            candidate.params.dip_valid = 1;
        }
    }
    else if (magic == MAG_STORE_MAGIC_V1)
    {
        crc_offset = MAG_STORE_CRC_V1;
        count = 3;
    }
    else
    {
        return 1;
    }
    if (mag_store_get_u32(record + crc_offset) != mag_store_crc(record, crc_offset))
    {
        return 1;
    }
    for (i = 0; i < 3; ++i)
    {
        candidate.params.offset_uT[i] = mag_store_get_float(record + 8 + i * 4);
    }
    for (i = 0; i < count; ++i)
    {
        candidate.params.soft_sym[i] = mag_store_get_float(record + 20 + i * 4);
    }
    /* 还原出的数值不满足有限值/正比例要求时仍判失败。 */
    if (!mag_store_values_valid(&candidate))
    {
        return 1;
    }
    /* 全部检查通过才整体写回调用方缓冲。 */
    *calibration = candidate;
    return 0;
}

/* 混合版本时有效MAG2为迁移后的记录；同版本按对应位宽比较循环序号。 */
static uint8_t mag_store_latest(const uint8_t *first, const uint8_t *second, const uint8_t valid[2])
{
    uint32_t difference;
    /* 槽0无效时选槽1；槽0有效而槽1无效时选槽0。 */
    if (!valid[0])
    {
        return 1;
    }
    if (!valid[1])
    {
        return 0;
    }
    if (mag_store_get_u32(first) != mag_store_get_u32(second))
    {
        return mag_store_get_u32(second) == MAG_STORE_MAGIC_V2;
    }
    difference = mag_store_get_u32(second + 4) - mag_store_get_u32(first + 4);
    if (mag_store_get_u32(first) == MAG_STORE_MAGIC_V2)
    {
        uint16_t diff16 = (uint16_t)difference;
        return diff16 != 0 && diff16 < 0x8000;
    }
    return difference != 0 && difference < 0x80000000U;
}

/* 仅sensor任务调用：读两槽并逐条校验格式/CRC/数值，取较新一份；两槽都坏但读事务失败报ERROR。 */
app_mag_store_result app_mag_store_load(app_mag_calibration_struct *calibration)
{
    uint8_t record[2][MAG_STORE_RECORD_BYTES], valid[2] = {0};
    uint8_t i, read_error = 0;
    app_mag_calibration_struct candidate;
    /* 输出指针为空时直接报参数错误，不访问EEPROM。 */
    if (calibration == NULL)
    {
        return APP_MAG_STORE_ERROR;
    }
    /* 逐槽读取48字节：槽0在0x00，槽1在0x30。 */
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(i * MAG_STORE_SLOT_BYTES, record[i], MAG_STORE_RECORD_BYTES) == 0)
        {
            /* 读成功后才做格式/CRC/数值校验，通过则置该槽有效位。 */
            valid[i] = mag_store_decode(record[i], &candidate) == 0;
        }
        else
        {
            /* 记录读取失败，仅置读错误标记，另一槽仍继续判定。 */
            read_error = 1;
        }
    }
    /* 两槽都无效：至少一槽读失败报ERROR，否则都读通但内容无效报EMPTY。 */
    if (!valid[0] && !valid[1])
    {
        if (read_error)
        {
            return APP_MAG_STORE_ERROR;
        }
        else
        {
            return APP_MAG_STORE_EMPTY;
        }
    }
    /* 选出较新槽并再次解码到调用方缓冲；记录已在上方校验过。 */
    (void)mag_store_decode(record[mag_store_latest(record[0], record[1], valid)], calibration);
    return APP_MAG_STORE_OK;
}

/* sensor串行推进；两份旧记录读完后才选择写入另一槽位。 */
static struct
{
    bl24c02_transfer_struct io;
    app_mag_calibration_struct value;
    uint8_t record[2][MAG_STORE_RECORD_BYTES], verify[MAG_STORE_RECORD_BYTES], valid[2];
    uint16_t address;
    uint8_t phase, target;
    app_mag_store_result result;
} s_save;

/* 启动一次保存：校验数值后复制系数并排入读槽0事务；上一次保存仍在途时返回BUSY，不覆盖。 */
app_mag_store_result app_mag_store_save_begin(const app_mag_calibration_struct *calibration)
{
    /* 上一次保存仍在途时返回BUSY，不覆盖当前事务。 */
    if (s_save.result == APP_MAG_STORE_BUSY)
    {
        return APP_MAG_STORE_BUSY;
    }
    /* 空指针或数值不合法(非有限值/比例非正)时直接报错，不启动写事务。 */
    if (calibration == NULL || calibration->is_ellipsoid == 0 || !mag_store_values_valid(calibration))
    {
        return APP_MAG_STORE_ERROR;
    }
    /* 复制系数到模块内，避免事务期间外部数据变化。 */
    s_save.value = *calibration;
    /* 从阶段0开始：先读槽0。 */
    s_save.phase = 0;
    /* 先置BUSY，表示事务在途。 */
    s_save.result = APP_MAG_STORE_BUSY;
    /* 排入读槽0事务，读结果存入record[0]。 */
    bl24c02_transfer_begin(&s_save.io, 0, s_save.record[0], MAG_STORE_RECORD_BYTES, 0);
    return s_save.result;
}

/* 分相位推进的双副本轮换：0读槽0→1读槽1→2选新序号写较旧槽→3读回→与编码数据逐字节比对。
 * 任一bl24c02事务未完成即返回BUSY，读写校验全过才报OK，否则记ERROR。 */
app_mag_store_result app_mag_store_save_step(void)
{
    app_mag_calibration_struct candidate;
    uint32_t sequence = 1;
    uint8_t latest, result;
    /* 只有BUSY状态需要推进；其他状态直接返回上次结果。 */
    if (s_save.result != APP_MAG_STORE_BUSY)
    {
        return s_save.result;
    }
    /* 推进当前EEPROM事务，结果0完成、1失败、2仍在途。 */
    result = bl24c02_transfer_step(&s_save.io);
    /* 事务仍在途：保持阶段不变，继续返回BUSY。 */
    if (result == 2)
    {
        return APP_MAG_STORE_BUSY;
    }
    /* 事务失败：本次保存置ERROR并返回，结束轮换。 */
    if (result != 0)
    {
        s_save.result = APP_MAG_STORE_ERROR;
        return s_save.result;
    }
    /* 阶段0/1：刚读完一个槽，先校验它是否有效。 */
    if (s_save.phase < 2)
    {
        s_save.valid[s_save.phase] = mag_store_decode(s_save.record[s_save.phase], &candidate) == 0;
        /* 阶段0刚完成：自增到1，接着排入槽1读取事务。 */
        if (s_save.phase++ == 0)
        {
            bl24c02_transfer_begin(&s_save.io, MAG_STORE_SLOT_BYTES, s_save.record[1], MAG_STORE_RECORD_BYTES, 0);
            return APP_MAG_STORE_BUSY;
        }
        /* 两槽都读完：默认写槽0，序号从1起。 */
        s_save.target = 0;
        if (s_save.valid[0] || s_save.valid[1])
        {
            /* 至少一槽有效时，取较新槽序号加1作为新序号，并改写到另一槽。 */
            latest = mag_store_latest(s_save.record[0], s_save.record[1], s_save.valid);
            sequence = mag_store_get_u32(s_save.record[latest] + 4) + 1;
            s_save.target = latest ^ 1;
        }
        /* 把新序号和系数编码进目标槽缓冲。 */
        mag_store_encode(s_save.record[s_save.target], sequence, &s_save.value);
        /* 计算目标槽地址(槽号乘48字节)并排入写事务。 */
        s_save.address = s_save.target * MAG_STORE_SLOT_BYTES;
        bl24c02_transfer_begin(&s_save.io, s_save.address, s_save.record[s_save.target], MAG_STORE_RECORD_BYTES, 1);
    }
    /* 阶段2：写事务完成，切到阶段3并排入读回事务。 */
    else if (s_save.phase == 2)
    {
        s_save.phase = 3;
        bl24c02_transfer_begin(&s_save.io, s_save.address, s_save.verify, MAG_STORE_RECORD_BYTES, 0);
    }
    /* 阶段3：读回完成，与编码数据逐字节比对，再校验读回记录本身。 */
    else
    {
        if (memcmp(s_save.verify, s_save.record[s_save.target], MAG_STORE_RECORD_BYTES) != 0 ||
            mag_store_decode(s_save.verify, &candidate) != 0)
        {
            s_save.result = APP_MAG_STORE_ERROR;
        }
        else
        {
            s_save.result = APP_MAG_STORE_OK;
        }
    }
    return s_save.result;
}

/* 旧调用方兼容入口；运行期保存只使用begin/step。 */
app_mag_store_result app_mag_store_save(const app_mag_calibration_struct *calibration)
{
    /* 先启动保存，返回BUSY表示事务已排入。 */
    app_mag_store_result result = app_mag_store_save_begin(calibration);
    /* 每拍推进一步，直到不再是BUSY；步骤间延时1ms让出CPU。 */
    while (result == APP_MAG_STORE_BUSY)
    {
        result = app_mag_store_save_step();
        if (result == APP_MAG_STORE_BUSY)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    /* 返回最终OK/ERROR等结果。 */
    return result;
}
