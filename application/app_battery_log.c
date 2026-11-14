/**
 * @file app_battery_log.c
 * @brief 逐分钟电压记录与复位现场，写入 sector 7 日志区。
 *        每条记录绑定本次开机编号(session)，电压字段0xFF00..0xFF0F存复位现场、0xFF10为提交槽，
 *        0xFFFF是session标记；sensor在日志锁内串行追加，LVGL清空时取同一把锁，
 *        掉电或坏槽不重复编程，读取以CRC校验区分有效记录。
 */
#include "app_battery_log.h"
#include "app_battery.h"
#include "int_battery_log.h"
#include "kt6368a.h"
#include "tps3823.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

static SemaphoreHandle_t s_log_mutex;
/* s_log 只在日志锁内推进：sensor 任务负责追加，LVGL 显式清空时取同一把锁，导出方只读取。 */
static struct
{
    app_battery_log_status_struct status;
    uint32_t session;
    uint32_t last_minute;
    uint8_t session_written;
    uint8_t reset_written;
} s_log;

#define BATTERY_LOG_RESET_COMMIT 0x52535431

static uint8_t battery_log_reset_decode(uint32_t slot, uint32_t session, app_reset_info_struct *info);

/* CRC16-CCITT多项式0x1021；格式及session参与初值，坏开机标记不会串接到旧曲线。 */
static uint16_t battery_log_crc(const uint8_t *data, uint32_t session)
{
    uint16_t crc = 0xB471; /* CRC初值，后续按字节和位逐级混入。 */
    uint8_t i, bit;
    /* 电压字段不是0xFFFF(session标记)时，把开机编号高、低16位异或进初值，使CRC绑定本次开机。 */
    if (data[4] != 0xFF || data[5] != 0xFF)
    {
        crc ^= (uint16_t)session ^ (uint16_t)(session >> 16);
    }
    /* 前6字节为数据区(4字节时间+2字节电压)，逐字节参与CRC。 */
    for (i = 0; i < 6; ++i)
    {
        crc ^= (uint16_t)data[i] << 8; /* 当前字节移到高8位，与CRC高字节对齐后异或。 */
        /* 每个字节固定处理8位。 */
        for (bit = 0; bit < 8; ++bit)
        {
            /* 最高位为1时左移后异或多项式0x1021；否则只左移一位。 */
            if ((crc & 0x8000) != 0)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc; /* 返回本记录的16位CRC值。 */
}

/* 解码指定槽；CRC与session不符(坏槽/串号)返回1，0=已填入record。 */
static uint8_t battery_log_decode(uint32_t slot, uint32_t session, app_battery_log_record_struct *record)
{
    uint8_t data[BATTERY_LOG_RECORD_BYTES];
    uint16_t crc;
    int_battery_log_read(slot, data); /* 从指定槽读出8字节原始记录。 */
    /* 记录内CRC按小端存放：data[6]为低字节，data[7]为高字节。 */
    crc = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
    /* 与按当前session重算的CRC比较；不等说明是坏槽或属于别的开机，返回1。 */
    if (crc != battery_log_crc(data, session))
    {
        return 1;
    }
    /* 秒数字段按小端4字节还原为32位。 */
    record->seconds = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    /* 电压字段按小端2字节还原为毫伏值。 */
    record->voltage_mv = (uint16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    return 0; /* CRC通过，record已填入。 */
}

/* 开机扫描一遍日志区：以最后一条session标记为当前开机编号，统计电压条数与完整现场组数。
 * 只扫描不擦写，因此旧固件残留区只置NEED_CLEAR，等用户显式清空。 */
void app_battery_log_init(void)
{
    uint32_t slot, i;
    uint8_t data[BATTERY_LOG_RECORD_BYTES], occupied;
    app_battery_log_record_struct record;
    int_battery_log_set_rx_callbacks(kt6368a_rx_is_enabled, kt6368a_receive_polled);
    /* 首次调用时创建互斥锁；后续调用直接复用已建好的锁。 */
    if (s_log_mutex == NULL)
    {
        s_log_mutex = xSemaphoreCreateMutex();
    }
    /* 锁创建失败则把状态置为ERROR，后续写日志入口都会因无锁而不再推进。 */
    if (s_log_mutex == NULL)
    {
        s_log.status.state = APP_BATTERY_LOG_ERROR;
        return;
    }
    /* 扫描期间独占日志状态，等所有槽统计完再一次性放开。 */
    if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == 0)
    {
        return;
    }
    /* 清空上次运行留下的状态/session/游标，作为本次扫描的起点。 */
    memset(&s_log, 0, sizeof(s_log));
    s_log.status.state = APP_BATTERY_LOG_READY;
    /* 逐槽扫描整个日志区，slot同时是槽号。 */
    for (slot = 0; slot < BATTERY_LOG_CAPACITY; ++slot)
    {
        int_battery_log_read(slot, data); /* 读出该槽8字节原始内容。 */
        occupied = 0;
        /* 任一字节不是0xFF即认为该槽已被写过；全0xFF视为擦除态空槽。 */
        for (i = 0; i < sizeof(data); ++i)
        {
            occupied |= (uint8_t)(data[i] != 0xFF);
        }
        if (occupied == 0)
        {
            continue; /* 空槽不占用编号，继续看后面的槽。 */
        }
        s_log.status.slots = slot + 1; /* 掉电留下的坏槽也不重复编程。 */
        /* 只有CRC校验通过的槽才继续解释内容。 */
        if (battery_log_decode(slot, s_log.session, &record) == 0)
        {
            /* 电压字段为0xFFFF表示session标记，其seconds即开机编号，记下来供后续记录校验。 */
            if (record.voltage_mv == UINT16_MAX)
            {
                s_log.session = record.seconds;
            }
            /* 属于已知session且电压低于复位现场标记的，是普通分钟电压记录。 */
            else if (s_log.session != 0 && record.voltage_mv < APP_BATTERY_LOG_RESET_TAG)
            {
                ++s_log.status.samples;
            }
            /* 电压字段恰为0xFF00的是复位现场组首槽，向后校验整组17槽是否完整。 */
            else if (s_log.session != 0 && record.voltage_mv == APP_BATTERY_LOG_RESET_TAG)
            {
                app_reset_info_struct info;
                if (slot + APP_BATTERY_LOG_RESET_SLOTS <= BATTERY_LOG_CAPACITY &&
                    battery_log_reset_decode(slot, s_log.session, &info) == 0)
                {
                    ++s_log.status.resets;
                }
            }
        }
    }
    /* 旧固件/未知格式区域不自动擦除；首条必须是有效session记录。 */
    if (s_log.status.slots != 0 && (battery_log_decode(0, 0, &record) != 0 || record.voltage_mv != UINT16_MAX || record.seconds == 0))
    {
        s_log.status.state = APP_BATTERY_LOG_NEED_CLEAR;
    }
    /* 已无空槽则标记FULL，后续追加会被拒绝。 */
    else if (s_log.status.slots == BATTERY_LOG_CAPACITY)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
    }
    /* session已到32位最大值，再自增会回绕为0。 */
    else if (s_log.session == UINT32_MAX)
    {
        /* 再自增就回到0，与"尚无session"混淆，本轮开机不再写入。 */
        s_log.status.state = APP_BATTERY_LOG_ERROR;
    }
    ++s_log.session; /* 本次开机的新编号；首条电压记录前会先写一条session标记 */
    xSemaphoreGive(s_log_mutex); /* 扫描完成，放开日志锁。 */
}

/* 在下一个空槽追加一条记录；写满或Flash写失败置对应状态并返回1，0=已落盘。 */
static uint8_t battery_log_append(uint32_t seconds, uint16_t voltage_mv)
{
    uint8_t data[BATTERY_LOG_RECORD_BYTES];
    uint32_t i;
    uint16_t crc;
    /* 已无空槽时置FULL并返回失败，不再尝试写入。 */
    if (s_log.status.slots >= BATTERY_LOG_CAPACITY)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
        return 1;
    }
    /* 秒数按小端写入前4字节：低字节在前。 */
    for (i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(seconds >> (8 * i));
    }
    /* 电压毫伏值按小端写入第4、5字节。 */
    data[4] = (uint8_t)voltage_mv;
    data[5] = (uint8_t)(voltage_mv >> 8);
    crc = battery_log_crc(data, s_log.session); /* 以本次开机编号计算CRC。 */
    /* CRC按小端写入最后两字节。 */
    data[6] = (uint8_t)crc;
    data[7] = (uint8_t)(crc >> 8);
    /* 写入当前空槽；失败则置ERROR并返回，槽号不前进。 */
    if (int_battery_log_write(s_log.status.slots, data) != 0)
    {
        s_log.status.state = APP_BATTERY_LOG_ERROR;
        return 1;
    }
    ++s_log.status.slots; /* 写入成功，占用槽数加一，即指向下一空槽。 */
    /* 刚好写满时置FULL，后续追加直接拒绝。 */
    if (s_log.status.slots == BATTERY_LOG_CAPACITY)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
    }
    return 0; /* 记录已落盘。 */
}

/* 校验连续17槽(16个现场字+1个提交标记)；任一坏槽或提交标记不符返回1，0=现场完整。 */
static uint8_t battery_log_reset_decode(uint32_t slot, uint32_t session, app_reset_info_struct *info)
{
    uint32_t words[16];
    uint32_t i;
    app_battery_log_record_struct record;
    /* session为0或剩余槽不足一整组17槽时，不可能有完整现场，直接返回失败。 */
    if (session == 0 || slot > BATTERY_LOG_CAPACITY - APP_BATTERY_LOG_RESET_SLOTS)
    {
        return 1;
    }
    /* 连续校验17个槽：前16槽装现场字，第17槽是提交标记。 */
    for (i = 0; i < APP_BATTERY_LOG_RESET_SLOTS; ++i)
    {
        /* 该槽CRC/编号不符，或电压标记不是0xFF00+i的连续编号，则整组无效。 */
        if (battery_log_decode(slot + i, session, &record) != 0 || record.voltage_mv != APP_BATTERY_LOG_RESET_TAG + i)
        {
            return 1;
        }
        /* 前16槽的seconds字段是现场数据字，按顺序收集。 */
        if (i < 16)
        {
            words[i] = record.seconds;
        }
        /* 第17槽的seconds必须等于提交常量，否则说明这组没写完。 */
        else if (record.seconds != BATTERY_LOG_RESET_COMMIT)
        {
            return 1;
        }
    }
    memcpy(info, words, sizeof(*info)); /* 16个字即一份完整的 app_reset_info_struct。 */
    return 0;
}

/* 开机后首次把复位现场(上次备份RCC标志)写进日志；无现场或空间不足则不写。 */
static void battery_log_save_reset(void)
{
    app_reset_info_struct info;
    uint32_t words[16];
    uint32_t i, needed = APP_BATTERY_LOG_RESET_SLOTS;
    app_reset_get_info(&info); /* 启动时已固定的RCC标志及上次备份现场，不读取本次运行检查点。 */
    /* 既无复位标志也无有效现场时没有内容可存，标记已处理并返回。 */
    if (info.reset_flags == 0 && info.valid == 0)
    {
        s_log.reset_written = 1;
        return;
    }
    /* 若还没写过session标记，本次要多占一个槽，预留空间时一并算上。 */
    if (s_log.session_written == 0)
    {
        ++needed;
    }
    /* 剩余空槽不够(session标记+17槽现场)时置FULL，整组都不写，避免半组残留。 */
    if (BATTERY_LOG_CAPACITY - s_log.status.slots < needed)
    {
        s_log.status.state = APP_BATTERY_LOG_FULL;
        return;
    }
    /* 现场组之前必须先有一条session标记，后续记录的CRC才能按同一编号校验。 */
    if (s_log.session_written == 0)
    {
        if (battery_log_append(s_log.session, UINT16_MAX) != 0)
        {
            return;
        }
        s_log.session_written = 1;
    }
    memcpy(words, &info, sizeof(info)); /* 把结构体按16个32位字展开，逐槽写入。 */
    /* 16个现场字依次写入，电压字段用0xFF00..0xFF0F标识槽序。 */
    for (i = 0; i < 16; ++i)
    {
        if (battery_log_append(words[i], (uint16_t)(APP_BATTERY_LOG_RESET_TAG + i)) != 0)
        {
            return;
        }
    }
    /* 每槽CRC绑定开机编号，最后提交版本标记；掉电残留不能当成完整现场。 */
    if (battery_log_append(BATTERY_LOG_RESET_COMMIT, APP_BATTERY_LOG_RESET_TAG + 16) == 0)
    {
        s_log.reset_written = 1; /* 整组提交成功，本次开机不再重复写现场。 */
        ++s_log.status.resets;
    }
}

/* 日志锁忙则本轮跳过；首条写入前先补session标记，之后每跨过一分钟存一条电压。 */
void app_battery_log_service(uint32_t seconds)
{
    app_battery_struct battery;
    /* 未建锁或本轮锁被占用(非阻塞获取失败)时直接跳过，不阻塞sensor任务。 */
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return;
    }
    /* 同一分钟不重复落盘；还没写过复位现场且已过10秒时不能跳过本轮，现场优先于电压记录。 */
    if (s_log.status.state != APP_BATTERY_LOG_READY ||
        ((s_log.reset_written != 0 || seconds < 10) && seconds / 60 == s_log.last_minute))
    {
        xSemaphoreGive(s_log_mutex);
        return;
    }
    app_battery_get_snapshot(&battery); /* valid=启动平均完成，result=HAL_OK=本轮转换成功，两者都过才落盘 */
    /* 已过10秒、电压快照有效且尚未保存现场时，先把上次复位现场写进日志。 */
    if (s_log.status.state == APP_BATTERY_LOG_READY && s_log.reset_written == 0 && seconds >= 10 &&
        battery.valid != 0 && battery.result == HAL_OK)
    {
        battery_log_save_reset();
    }
    /* 已跨入新的一分钟(秒数/60与上次不同)才追加普通电压记录。 */
    if (s_log.status.state == APP_BATTERY_LOG_READY && seconds / 60 != s_log.last_minute)
    {
        /* 快照有效、转换成功且电压落在普通记录范围(低于0xFF00)时才写。 */
        if (battery.valid != 0 && battery.result == HAL_OK && battery.voltage_mv < APP_BATTERY_LOG_RESET_TAG)
        {
            s_log.last_minute = seconds / 60; /* 先记下本分钟，后续同分钟请求不再重复进入。 */
            /* 本次开机还没有session标记时先补一条，其seconds为开机编号。 */
            if (s_log.session_written == 0 && battery_log_append(s_log.session, UINT16_MAX) == 0)
            {
                s_log.session_written = 1;
            }
            /* session标记已就位后再写电压记录，成功后累计样本数。 */
            if (s_log.session_written != 0 && battery_log_append(seconds, (uint16_t)battery.voltage_mv) == 0)
            {
                ++s_log.status.samples;
            }
        }
    }
    xSemaphoreGive(s_log_mutex); /* 本轮处理结束，释放日志锁。 */
}

/* 导出方取状态快照；锁忙返回1让调用方本轮重试，不阻塞蓝牙时序。
 * 尚未建锁时返回当前静态状态；建锁后以非阻塞方式取得快照。 */
uint8_t app_battery_log_get_status(app_battery_log_status_struct *status)
{
    /* 未建锁时直接返回当前静态状态，调用方无需等待。 */
    if (s_log_mutex == NULL)
    {
        *status = s_log.status;
        return 0;
    }
    /* 非阻塞取锁；锁被占用返回1，让调用方本轮重试。 */
    if (xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return 1;
    }
    *status = s_log.status; /* 在锁内整体复制状态，四个字段来自同一次更新。 */
    xSemaphoreGive(s_log_mutex);
    return 0;
}

/* 越界或未扫描到的槽直接返回1；CRC按传入session校验，故导出方必须跟着session标记换编号。 */
uint8_t app_battery_log_read(uint32_t slot, uint32_t session, app_battery_log_record_struct *record)
{
    uint8_t result = 1; /* 默认按失败返回，只有实际读到并通过CRC才改成0。 */
    /* 未建锁或锁忙时返回2，导出方据此保留游标、下一轮重试。 */
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return 2;
    }
    /* 槽号必须在已占用范围内，否则保持result=1(越界失败)。 */
    if (slot < s_log.status.slots)
    {
        result = battery_log_decode(slot, session, record); /* 读原始槽并按传入session校验CRC。 */
    }
    xSemaphoreGive(s_log_mutex);
    return result;
}

/* 只有连续17槽全部CRC正确才置complete=1；否则info保持调用方原值，由导出方跳过该现场组。 */
uint8_t app_battery_log_read_reset(uint32_t slot, uint32_t session, app_reset_info_struct *info, uint8_t *complete)
{
    /* 未建锁或锁忙返回1，调用方下一轮重试，info/complete保持原值。 */
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return 1;
    }
    *complete = 0; /* 先按"无完整现场"处理。 */
    /* 槽号已占用且从该槽起剩余至少17槽时，才可能有一整组现场。 */
    if (slot < s_log.status.slots && s_log.status.slots - slot >= APP_BATTERY_LOG_RESET_SLOTS)
    {
        /* 整组17槽逐槽校验通过才置complete=1，info此时才被填入。 */
        if (battery_log_reset_decode(slot, session, info) == 0)
        {
            *complete = 1;
        }
    }
    xSemaphoreGive(s_log_mutex);
    return 0;
}

void app_battery_log_clear_and_reset(void)
{
    /* 显式维护操作由LVGL发起：取得同一日志锁后擦除并立即重启，避免暂停造成HAL/RTOS时基失真。
     * 擦除期间屏蔽外部看门狗；禁止升级程序擦除此扇区。 */
    /* 未建锁或锁忙时本轮不执行清空，直接返回。 */
    if (s_log_mutex == NULL || xSemaphoreTake(s_log_mutex, 0) == 0)
    {
        return;
    }
    tps3823_disable(); /* 屏蔽外部看门狗复位输出，并把WDI置高阻，供擦除期间使用。 */
    __disable_irq(); /* 关闭中断，使擦除与重启之间不被其它任务打断。 */
    int_battery_log_erase(); /* 擦除sector 7日志区。 */
    NVIC_SystemReset(); /* 擦除后立即复位，由重启后的扫描重新建立日志状态。 */
}
