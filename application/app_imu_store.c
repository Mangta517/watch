/** MPU1记录28字节：格式、序号、量程、六轴int16零偏、CRC32。
 *  两个32字节槽位在0x60/0x80，更新失败保留旧副本；不覆盖磁力计记录。 */
#include "app_imu_store.h"
#include "mpu6050.h"
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

/* 小端写32位值：第i字节取value从8*i位起的低8位，共写4字节。 */
static void imu_store_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    /* 从最低字节开始逐字节写入缓冲。 */
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (i * 8));
    }
}

/* 小端读32位值：data[0]为最低字节，依次左移8/16/24位后按位或。 */
static uint32_t imu_store_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

/* 对length字节计算CRC32，初值与最终异或均为0xFFFFFFFF。 */
static uint32_t imu_store_crc(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint16_t i;
    uint8_t bit;
    /* 逐字节异或进CRC低8位。 */
    for (i = 0; i < length; ++i)
    {
        crc ^= data[i];
        /* 每字节处理8位：最低位为1时右移并异或多项式0xEDB88320，为0时只右移。 */
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
    /* 结果按位取反后返回。 */
    return crc ^ 0xFFFFFFFFU;
}

/* 把序号和六轴零偏编码成28字节记录，最后对前24字节算CRC32写到偏移24。 */
static void imu_store_encode(uint8_t *record, uint32_t sequence, const mpu6050_offset_struct *offset)
{
    uint8_t i;
    uint16_t value;
    /* 偏移0写MPU1格式字，偏移4写本次序号，偏移8写加速度/陀螺仪量程组合。 */
    imu_store_put_u32(record, IMU_STORE_MAGIC);
    imu_store_put_u32(record + 4, sequence);
    imu_store_put_u32(record + 8, IMU_STORE_RANGE);
    /* 逐轴编码：加速度int16存12..17，陀螺仪int16存18..23，均为低字节在前的小端。 */
    for (i = 0; i < 3; ++i)
    {
        value = (uint16_t)offset->accel[i];
        record[12 + i * 2] = (uint8_t)value;
        record[13 + i * 2] = (uint8_t)(value >> 8);
        value = (uint16_t)offset->gyro[i];
        record[18 + i * 2] = (uint8_t)value;
        record[19 + i * 2] = (uint8_t)(value >> 8);
    }
    /* 对偏移0..23计算CRC32，写到偏移24。 */
    imu_store_put_u32(record + IMU_STORE_CRC_OFFSET, imu_store_crc(record, IMU_STORE_CRC_OFFSET));
}

/* 校验格式字、量程字和CRC，通过后把六轴零偏解到offset；返回1表示校验失败，0表示成功。 */
static uint8_t imu_store_decode(const uint8_t *record, mpu6050_offset_struct *offset)
{
    uint8_t i;
    /* 格式字不等于MPU1、量程字不匹配当前配置、或CRC不符，任一条件成立即判记录无效。 */
    if (imu_store_get_u32(record) != IMU_STORE_MAGIC || imu_store_get_u32(record + 8) != IMU_STORE_RANGE ||
        imu_store_get_u32(record + IMU_STORE_CRC_OFFSET) != imu_store_crc(record, IMU_STORE_CRC_OFFSET))
    {
        return 1;
    }
    /* 逐轴把小端两字节拼回int16：加速度12..17、陀螺仪18..23。 */
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
    /* 槽0无效时直接选槽1，即使槽1也无效也返回槽1，由调用方结合valid判断。 */
    if (!valid[0])
    {
        return 1;
    }
    /* 槽0有效而槽1无效时选槽0。 */
    if (!valid[1])
    {
        return 0;
    }
    /* 两槽都有效：用槽1序号减槽0序号，差值非0且小于0x80000000表示槽1更新。 */
    difference = imu_store_get_u32(second + 4) - imu_store_get_u32(first + 4);
    return difference != 0 && difference < 0x80000000U;
}

/* 仅sensor任务调用：读两槽并逐条校验格式/量程/CRC，取较新一份；两槽都坏但读事务失败报ERROR。 */
app_imu_store_result app_imu_store_load(mpu6050_offset_struct *offset)
{
    uint8_t record[2][IMU_STORE_RECORD_BYTES], valid[2] = {0};
    uint8_t i, read_error = 0;
    mpu6050_offset_struct candidate;
    /* 输出指针为空时直接报参数错误，不访问EEPROM。 */
    if (offset == NULL)
    {
        return APP_IMU_STORE_ERROR;
    }
    /* 逐槽读取28字节：槽0在0x60，槽1在0x80。 */
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(IMU_STORE_BASE + i * IMU_STORE_SLOT_BYTES, record[i], IMU_STORE_RECORD_BYTES) == 0)
        {
            /* 读成功后才做格式/量程/CRC校验，校验通过置该槽有效位。 */
            valid[i] = imu_store_decode(record[i], &candidate) == 0;
        }
        else
        {
            /* 记录读取失败，仅置读错误标记，不影响另一槽的判定。 */
            read_error = 1;
        }
    }
    /* 两槽都无效：若至少一槽读失败报ERROR，否则两槽都读通但内容无效报EMPTY。 */
    if (!valid[0] && !valid[1])
    {
        if (read_error != 0)
        {
            return APP_IMU_STORE_ERROR;
        }
        return APP_IMU_STORE_EMPTY;
    }
    /* 选出较新槽并再次解码到调用方缓冲；此处记录已在上面校验过。 */
    (void)imu_store_decode(record[imu_store_latest(record[0], record[1], valid)], offset);
    return APP_IMU_STORE_OK;
}

/* 保存结果单独记录；不增加I2C事务或串口发送，只发布给原诊断任务。 */
static app_imu_store_result imu_store_save_finish(app_imu_save_stage stage, uint32_t started_ms, uint32_t address)
{
    bl24c02_error error;
    uint16_t driver_address;
    /* 参数错误阶段不查询驱动，直接使用参数错误码。 */
    if (stage == APP_IMU_SAVE_ARGUMENT)
    {
        error = BL24C02_ERROR_ARGUMENT;
    }
    else
    {
        /* 其他阶段取驱动最近一次读写的错误码和页地址。 */
        error = bl24c02_get_error(&driver_address);
        if (error != BL24C02_ERROR_NONE)
        {
            /* 驱动报错时用驱动记录的地址覆盖调用方传入的地址，便于定位。 */
            address = driver_address;
        }
    }
    /* 发布诊断：阶段码、错误码、地址以及从启动到现在的毫秒耗时。 */
    app_diag_imu_save_record((uint32_t)stage, (uint32_t)error, address, HAL_GetTick() - started_ms);
    /* 只有DONE阶段算保存成功，其余阶段返回ERROR。 */
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

/* 启动一次保存：只复制零偏并排入读槽0事务；上一次保存仍在途时返回BUSY，不覆盖。 */
app_imu_store_result app_imu_store_save_begin(const mpu6050_offset_struct *offset)
{
    /* 上一次保存的结果仍是BUSY时，拒绝启动新保存，保持原事务状态。 */
    if (s_save.result == APP_IMU_STORE_BUSY)
    {
        return APP_IMU_STORE_BUSY;
    }
    /* 空指针按参数错误处理，只发布诊断，不启动EEPROM事务。 */
    if (offset == NULL)
    {
        return imu_store_save_finish(APP_IMU_SAVE_ARGUMENT, HAL_GetTick(), UINT32_MAX);
    }
    /* 复制调用方零偏到模块内，避免事务期间外部数据变化。 */
    s_save.value = *offset;
    /* 记录本次保存起始tick，供诊断计算耗时。 */
    s_save.started_ms = HAL_GetTick();
    /* 从阶段0开始：先读槽0。 */
    s_save.phase = 0;
    /* 先置BUSY，表示事务在途。 */
    s_save.result = APP_IMU_STORE_BUSY;
    /* 排入读槽0的28字节事务，读结果存入record[0]。 */
    bl24c02_transfer_begin(&s_save.io, IMU_STORE_BASE, s_save.record[0], IMU_STORE_RECORD_BYTES, 0);
    return s_save.result;
}

/* 分相位推进的双副本轮换：0读槽0→1读槽1→2选新序号写较旧槽→3读回→校验数据。
 * 任一bl24c02事务未完成即返回BUSY；失败按failures[]记诊断阶段，读写校验全过才报OK。 */
app_imu_store_result app_imu_store_save_step(void)
{
    mpu6050_offset_struct candidate;
    uint32_t sequence = 1;
    uint8_t latest, result;
    /* 只有BUSY状态需要推进；其他状态直接返回上次结果。 */
    if (s_save.result != APP_IMU_STORE_BUSY)
    {
        return s_save.result;
    }
    /* 推进当前EEPROM事务，结果0完成、1失败、2仍在途。 */
    result = bl24c02_transfer_step(&s_save.io);
    /* 事务仍在途：本拍不改变阶段，继续返回BUSY。 */
    if (result == 2)
    {
        return APP_IMU_STORE_BUSY;
    }
    /* 事务失败：按当前phase映射到读槽0/读槽1/写/读回阶段并发布诊断，结束本次保存。 */
    if (result != 0)
    {
        static const app_imu_save_stage failures[] = {APP_IMU_SAVE_READ_SLOT0, APP_IMU_SAVE_READ_SLOT1,
                                                      APP_IMU_SAVE_WRITE, APP_IMU_SAVE_VERIFY_READ};
        s_save.result = imu_store_save_finish(failures[s_save.phase], s_save.started_ms, s_save.io.address);
        return s_save.result;
    }
    /* 阶段0/1：刚读完一个槽，先校验它是否有效。 */
    if (s_save.phase < 2)
    {
        s_save.valid[s_save.phase] = imu_store_decode(s_save.record[s_save.phase], &candidate) == 0;
        /* 阶段0刚完成：自增到1，接着排入槽1读取事务。 */
        if (s_save.phase++ == 0)
        {
            bl24c02_transfer_begin(&s_save.io, IMU_STORE_BASE + IMU_STORE_SLOT_BYTES, s_save.record[1], IMU_STORE_RECORD_BYTES, 0);
            return APP_IMU_STORE_BUSY;
        }
        /* 两槽都读完：默认写槽0，序号从1起。 */
        s_save.target = 0;
        if (s_save.valid[0] || s_save.valid[1])
        {
            /* 至少一槽有效时，取较新槽序号加1作为新序号，并改写到另一槽。 */
            latest = imu_store_latest(s_save.record[0], s_save.record[1], s_save.valid);
            sequence = imu_store_get_u32(s_save.record[latest] + 4) + 1;
            s_save.target = latest ^ 1;
        }
        /* 把新序号和零偏编码进目标槽缓冲。 */
        imu_store_encode(s_save.record[s_save.target], sequence, &s_save.value);
        /* 计算目标槽EEPROM地址，并排入写事务。 */
        s_save.address = IMU_STORE_BASE + s_save.target * IMU_STORE_SLOT_BYTES;
        bl24c02_transfer_begin(&s_save.io, s_save.address, s_save.record[s_save.target], IMU_STORE_RECORD_BYTES, 1);
    }
    /* 阶段2：写事务完成，切到阶段3并排入读回事务。 */
    else if (s_save.phase == 2)
    {
        s_save.phase = 3;
        bl24c02_transfer_begin(&s_save.io, s_save.address, s_save.verify, IMU_STORE_RECORD_BYTES, 0);
    }
    /* 阶段3：读回完成，先逐字节比对，再校验格式/量程/CRC。 */
    else
    {
        if (memcmp(s_save.verify, s_save.record[s_save.target], IMU_STORE_RECORD_BYTES) != 0 ||
            imu_store_decode(s_save.verify, &candidate) != 0)
        {
            /* 内容不一致或校验失败：按校验数据阶段报错。 */
            s_save.result = imu_store_save_finish(APP_IMU_SAVE_VERIFY_DATA, s_save.started_ms, s_save.address);
        }
        else
        {
            /* 比对和校验都通过：按完成阶段发布成功。 */
            s_save.result = imu_store_save_finish(APP_IMU_SAVE_DONE, s_save.started_ms, s_save.address);
        }
    }
    return s_save.result;
}

/* 旧调用方兼容入口；运行期保存只使用begin/step。 */
app_imu_store_result app_imu_store_save(const mpu6050_offset_struct *offset)
{
    /* 先启动保存，返回BUSY表示事务已排入。 */
    app_imu_store_result result = app_imu_store_save_begin(offset);
    /* 每拍推进一步，直到不再是BUSY；步骤间延时1ms让出CPU。 */
    while (result == APP_IMU_STORE_BUSY)
    {
        result = app_imu_store_save_step();
        if (result == APP_IMU_STORE_BUSY)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    /* 返回最终OK/ERROR等结果。 */
    return result;
}
