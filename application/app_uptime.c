/**
 * @file app_uptime.c
 * @brief 每10分钟保存本次开机秒数；0xA0/0xB0各16字节，不覆盖磁/IMU校准。
 *        UPT1、小端序号、秒数、CRC32；轮换写入并读回验证，保留另一份记录。
 */
#include "app_uptime.h"
#include "bl24c02.h"
#include "main.h"
#include <string.h>

#define APP_UPTIME_STORE_BASE   0xA0
#define APP_UPTIME_RECORD_BYTES 16
#define APP_UPTIME_MAGIC        0x31545055
#define APP_UPTIME_SAVE_SECONDS 600

static struct
{
    uint32_t last_ms, seconds, remainder_ms, attempted_seconds;
    uint32_t sequence, saved_seconds;
    uint8_t valid, loaded, target, phase;
    uint8_t record[APP_UPTIME_RECORD_BYTES], verify[APP_UPTIME_RECORD_BYTES];
    bl24c02_transfer_struct io;
} s_uptime;

/* sensor写一次后发布有效位；LVGL只读，避免把本次运行的新记录当成上次记录。 */
static uint32_t s_previous_seconds;
static volatile uint32_t s_previous_valid;

/* 小端拼装32位无符号数：data[0]为最低字节，其余字节依次左移8/16/24位后按位或。 */
static uint32_t uptime_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

/* 小端拆分32位无符号数：固定写4字节，第i字节取value从8*i位起的低8位。 */
static void uptime_put_u32(uint8_t *data, uint32_t value)
{
    uint8_t i;
    /* 逐字节写入，先低字节后高字节。 */
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (8 * i));
    }
}

/* 对记录前12字节计算CRC32，返回取反后的最终值。 */
static uint32_t uptime_crc(const uint8_t *data)
{
    uint32_t crc = UINT32_MAX;
    uint8_t i, bit;
    /* 逐字节把当前数据异或进CRC低8位，再处理这8位。 */
    for (i = 0; i < 12; ++i)
    {
        crc ^= data[i];
        /* 每轮检查最低位：为1时右移并异或多项式0xEDB88320，为0时只右移。 */
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
    /* CRC32结果按位取反后返回。 */
    return crc ^ UINT32_MAX;
}

/* 读0xA0/0xB0两槽，逐槽校验MAGIC与CRC，按序号选较新记录并恢复s_uptime的序号、
 * 已存秒数和下一写入槽；返回1表示某槽读取失败，0表示两槽都已读完(记录可能都无效)。 */
static uint8_t uptime_load(void)
{
    uint8_t record[2][APP_UPTIME_RECORD_BYTES];
    uint8_t valid[2], i, latest = 0;
    uint32_t difference;
    /* 逐槽读取16字节记录：0xA0槽0、0xB0槽1。 */
    for (i = 0; i < 2; ++i)
    {
        if (bl24c02_read(APP_UPTIME_STORE_BASE + i * APP_UPTIME_RECORD_BYTES, record[i], APP_UPTIME_RECORD_BYTES) != 0)
        {
            return 1; /* 未读清旧副本时不覆盖，下一记录周期再探测。 */
        }
        /* 首4字节等于UPT1且偏移12的CRC匹配前12字节时，本槽有效。 */
        valid[i] = uptime_get_u32(record[i]) == APP_UPTIME_MAGIC && uptime_get_u32(record[i] + 12) == uptime_crc(record[i]);
    }
    /* 槽1序号减槽0序号：无符号差值小于0x80000000表示槽1更新，兼容32位序号回绕。 */
    difference = uptime_get_u32(record[1] + 4) - uptime_get_u32(record[0] + 4);
    if (valid[1] != 0 && (valid[0] == 0 || (difference != 0 && difference < 0x80000000U)))
    {
        latest = 1;
    }
    /* 记录选中槽的有效位；无效时保留清零后的状态，不恢复序号和秒数。 */
    s_uptime.valid = valid[latest];
    if (s_uptime.valid != 0)
    {
        /* 恢复选中槽的序号与已保存开机秒数。 */
        s_uptime.sequence = uptime_get_u32(record[latest] + 4);
        s_uptime.saved_seconds = uptime_get_u32(record[latest] + 8);
        /* 下次写入选另一槽，保留当前最新记录。 */
        s_uptime.target = latest ^ 1;
        /* 首次成功读取时才发布上次开机时长快照；后续周期读取不覆盖它。 */
        if (s_previous_valid == 0)
        {
            s_previous_seconds = s_uptime.saved_seconds;
            __DMB();
            s_previous_valid = 1;
        }
    }
    return 0;
}

/* sensor启动时清状态并从HAL tick起算开机秒数；启动时读取两槽，失败时保留未加载状态，
 * 待后续600秒保存周期且存储可用时再尝试读取。 */
void app_uptime_init(void)
{
    /* 清零计时、序号、槽位和事务状态。 */
    memset(&s_uptime, 0, sizeof(s_uptime));
    /* 撤下上次开机的旧时长快照，等本次读取成功后重新发布。 */
    s_previous_valid = 0;
    /* 以当前HAL毫秒tick作为本次计时起点。 */
    s_uptime.last_ms = HAL_GetTick();
    /* 起点tick换算成整秒作为计时初值，余下不足1秒的部分单独保存，后续累加。 */
    s_uptime.seconds = s_uptime.last_ms / 1000;
    s_uptime.remainder_ms = s_uptime.last_ms % 1000;
    /* 读两槽恢复记录；读取失败时loaded保持0，后续保存周期再试。 */
    s_uptime.loaded = uptime_load() == 0;
}

/* 每拍累加本次开机秒数；跨过600s且EEPROM空闲时排入写事务(phase=1写、phase=2读回校验)。
 * 返回1表示占用EEPROM，其他保存须等到返回0；失败不连续重试，下个周期再写。 */
uint8_t app_uptime_service(uint8_t storage_available)
{
    /* 取当前tick，与上次记录的时刻相减得到本拍经过的毫秒数(无符号相减处理回绕)。 */
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - s_uptime.last_ms;
    uint8_t result;
    /* Tickless补齐Sleep/STOP的HAL时间；跨49.7天tick回绕仍连续计时。 */
    s_uptime.last_ms = now_ms;
    /* 本拍整秒部分累加进开机秒数。 */
    s_uptime.seconds += elapsed_ms / 1000;
    /* 本拍不足1秒的余数与之前累计的余数合并。 */
    s_uptime.remainder_ms += elapsed_ms % 1000;
    /* 余数满1000ms时进位成1秒，再把余数限制在0..999。 */
    s_uptime.seconds += s_uptime.remainder_ms / 1000;
    s_uptime.remainder_ms %= 1000;
    /* phase=0表示当前没有进行中的EEPROM事务，检查是否到保存点。 */
    if (s_uptime.phase == 0)
    {
        /* 存储不可用，或距上次尝试保存还不足600秒时，本拍不启动写事务。 */
        if (storage_available == 0 || (uint32_t)(s_uptime.seconds - s_uptime.attempted_seconds) < APP_UPTIME_SAVE_SECONDS)
        {
            return 0;
        }
        /* 把本次尝试点对齐到600秒整数倍，避免每个周期重复触发。 */
        s_uptime.attempted_seconds = s_uptime.seconds - s_uptime.seconds % APP_UPTIME_SAVE_SECONDS;
        /* 启动时未读清记录的话，这里补读两槽；仍失败就放弃本轮写入。 */
        if (s_uptime.loaded == 0)
        {
            s_uptime.loaded = uptime_load() == 0;
            if (s_uptime.loaded == 0)
            {
                return 0;
            }
        }
        /* 组装16字节记录：0..3为UPT1，4..7为下一序号，8..11为当前开机秒数，12..15为前12字节CRC32。 */
        uptime_put_u32(s_uptime.record, APP_UPTIME_MAGIC);
        uptime_put_u32(s_uptime.record + 4, s_uptime.sequence + 1);
        uptime_put_u32(s_uptime.record + 8, s_uptime.seconds);
        uptime_put_u32(s_uptime.record + 12, uptime_crc(s_uptime.record));
        /* 在target指向的槽启动写事务，写入刚组好的16字节记录。 */
        bl24c02_transfer_begin(&s_uptime.io, APP_UPTIME_STORE_BASE + s_uptime.target * APP_UPTIME_RECORD_BYTES,
                               s_uptime.record, APP_UPTIME_RECORD_BYTES, 1);
        /* 标记已进入写阶段，并告知调用方EEPROM被占用。 */
        s_uptime.phase = 1;
        return 1;
    }
    /* 已有事务：推进至多一笔EEPROM操作。 */
    result = bl24c02_transfer_step(&s_uptime.io);
    /* 返回2表示事务仍在进行，继续占用EEPROM。 */
    if (result == 2)
    {
        return 1;
    }
    /* 非0非2表示本笔失败：回到空闲，等下一个600秒周期再写。 */
    if (result != 0)
    {
        s_uptime.phase = 0; /* 不连续重试故障器件，下一10分钟周期再写。 */
        return 0;
    }
    /* 写阶段完成：从同一槽启动读回事务，准备逐字节比对。 */
    if (s_uptime.phase == 1)
    {
        bl24c02_transfer_begin(&s_uptime.io, APP_UPTIME_STORE_BASE + s_uptime.target * APP_UPTIME_RECORD_BYTES,
                               s_uptime.verify, APP_UPTIME_RECORD_BYTES, 0);
        /* 切到读回校验阶段。 */
        s_uptime.phase = 2;
        return 1;
    }
    /* 读回内容与写入内容一致时，确认本槽记录有效。 */
    if (memcmp(s_uptime.record, s_uptime.verify, APP_UPTIME_RECORD_BYTES) == 0)
    {
        s_uptime.valid = 1;
        /* 更新对外可读的最近保存秒数。 */
        s_uptime.saved_seconds = uptime_get_u32(s_uptime.record + 8);
        /* 序号递增，供下次写入。 */
        ++s_uptime.sequence;
        /* 下次写另一槽，保留本次记录。 */
        s_uptime.target ^= 1;
    }
    /* 无论校验是否一致，事务结束回到空闲。 */
    s_uptime.phase = 0;
    return 0;
}

/* 取本次开机已计时秒数；sensor任务读写，跨tick回绕仍连续。 */
uint32_t app_uptime_get_seconds(void)
{
    /* 返回sensor任务最近一拍累加出的本次开机秒数。 */
    return s_uptime.seconds;
}

/* 通过seconds输出最近已保存或启动时恢复的记录值，可能属于上次开机；
 * 返回1表示有有效记录，0表示无记录或空指针。 */
uint8_t app_uptime_get_saved(uint32_t *seconds)
{
    /* 空指针或当前没有有效记录时返回0，不写输出。 */
    if (seconds == NULL || s_uptime.valid == 0)
    {
        return 0;
    }
    /* 输出最近一次成功保存或启动时恢复的开机秒数。 */
    *seconds = s_uptime.saved_seconds;
    return 1;
}

/* 通过seconds输出启动时恢复的上次开机时长快照，不随本次保存更新；
 * 返回1表示有记录，0表示无记录或空指针。 */
uint8_t app_uptime_get_previous(uint32_t *seconds)
{
    /* 空指针或快照无效(valid=0)时返回0，不写输出。 */
    if (seconds == NULL || s_previous_valid == 0)
    {
        return 0;
    }
    /* 先执行数据存储屏障，再取启动时恢复的上次开机秒数快照。 */
    __DMB();
    *seconds = s_previous_seconds;
    return 1;
}
