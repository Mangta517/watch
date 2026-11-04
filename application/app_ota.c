/**
 * @file app_ota.c
 * @brief APP内OTA请求处理：收到'OTA,START'只把sensor侧的写标志请求置位，绝不在此擦写APP自身。
 *        写标志+读回校验由sensor在storage_service完成(EEPROM归sensor独占)，成功后复位，
 *        由主引导据标志选择跳转保底收固件；只有映射到0x08010000的APP才提供此能力。
 */
#include "app_ota.h"
#include "boot_layout.h"
#include "int_bootloader.h"
#include "bl24c02.h"
#include "kt6368a.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

typedef enum
{
    OTA_STORE_IDLE,
    OTA_STORE_REQUESTED,
    OTA_STORE_READY,
    OTA_STORE_FAILED
} ota_store_state_enum;

/* 双任务仅在短临界区交接状态；EEPROM始终由sensor操作。 */
static ota_store_state_enum s_store_state;
static struct
{
    char line[16];
    uint8_t length, overflow, pending, reboot, replied;
    uint32_t rx_ms, reply_ms;
    const char *reply;
} s_ota;

void app_ota_reset_receive(void)
{
    /* 清空行缓冲长度，下一条命令从行首重新累积。 */
    s_ota.length = 0;
    /* 清超长/非法字符标记，避免影响下一行。 */
    s_ota.overflow = 0;
    /* 完整命令已交给sensor时不能因蓝牙断开撤销持久化/复位。 */
}

void app_ota_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    /* i为本批数据的字节下标。 */
    uint16_t i;
    /* 1秒无新字节视为上一段命令作废，避免断连前的半行接到新会话上。 */
    if ((uint32_t)(now_ms - s_ota.rx_ms) >= 1000)
    {
        app_ota_reset_receive();
    }
    /* 刷新本批字节的接收时刻，作为下一次1秒超时的基准。 */
    s_ota.rx_ms = now_ms;
    /* 逐字节累积命令行：换行触发命令判断，非法字符或超长置溢出。 */
    for (i = 0; i < size; ++i)
    {
        /* CR或LF表示一行结束：补结束符后判断是否为完整OTA命令。 */
        if (data[i] == '\r' || data[i] == '\n')
        {
            /* 在当前长度处补字符串结束符，供strcmp比较。 */
            s_ota.line[s_ota.length] = '\0';
            /* 仅当未溢出、整行等于"OTA,START"、当前没有待发回执且没有在途发送时受理。 */
            if (s_ota.overflow == 0 && strcmp(s_ota.line, "OTA,START") == 0 && s_ota.reply == NULL && s_ota.pending == 0)
            {
#if defined(VECT_TAB_OFFSET) && VECT_TAB_OFFSET == (APP_START_ADDR - BOOTLOADER_START_ADDR)
                /* 进入短临界区，与sensor任务对存储状态做互斥交接。 */
                taskENTER_CRITICAL();
                /* 仅在存储状态空闲时把请求置为已请求，避免覆盖进行中的升级流程。 */
                if (s_store_state == OTA_STORE_IDLE)
                {
                    s_store_state = OTA_STORE_REQUESTED;
                }
                /* 退出临界区，后续EEPROM写入交给sensor任务。 */
                taskEXIT_CRITICAL();
#else
                /* 独立普通APP占用0x08000000，不能复位后误入不存在的Bootloader。 */
                s_ota.reply = "OTA,UNAVAILABLE,INSTALL_BOOTLOADER_AND_UPGRADE_APP\r\n";
#endif
            }
            /* 一行处理完，清空长度与溢出标记准备下一行。 */
            app_ota_reset_receive();
        }
        /* 非可打印ASCII或缓冲已满(留1字节结束符)：本行作废，只置溢出标记。 */
        else if (data[i] < 32 || data[i] > 126 || s_ota.length >= sizeof(s_ota.line) - 1)
        {
            s_ota.overflow = 1;
        }
        else
        {
            /* 合法可打印字符：追加到行尾并后移长度。 */
            s_ota.line[s_ota.length++] = (char)data[i];
        }
    }
}

/* 仅sensor调用；存储可用且有请求时写入并读回升级标志。返回1表示标志已确认，
 * 调用方应停止后续持久化操作并等待复位；其余返回0。 */
uint8_t app_ota_storage_service(uint8_t storage_available)
{
    /* 待写入的4字节升级标志记录：'B'、'L'、UPDATING(0x5A)、VALID(0xA5)。 */
    const uint8_t record[4] = {0x42, 0x4C, BOOTLOADER_FLAG_UPDATING, BOOTLOADER_FLAG_VALID};
    /* verify用于读回同一地址，与record逐字节比较。 */
    uint8_t verify[4];
    /* result保存写/读的中间结果，0表示成功。 */
    uint8_t result;
    /* state为临界区内取出的存储状态副本。 */
    ota_store_state_enum state;

    /* 进入短临界区，读取sensor与LVGL共享的存储状态。 */
    taskENTER_CRITICAL();
    state = s_store_state;
    /* 退出临界区，后续EEPROM操作在临界区外进行。 */
    taskEXIT_CRITICAL();
    if (state == OTA_STORE_READY)
    {
        return 1; /* 复位前不再开始其他EEPROM写入。 */
    }
    /* 没有待处理请求，或存储当前不可用：本次不写EEPROM，返回0。 */
    if (state != OTA_STORE_REQUESTED || storage_available == 0)
    {
        return 0;
    }
    /* 把4字节升级标志写到EEPROM固定标志地址，返回0表示写入成功。 */
    result = bl24c02_write(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record));
    if (result == 0)
    {
        /* 写成功后读回同一地址的4字节，返回0表示读取成功。 */
        result = bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, verify, sizeof(verify));
        /* 读回内容与写入记录不一致时，把结果改为1表示校验失败。 */
        if (result == 0 && memcmp(record, verify, sizeof(record)) != 0)
        {
            result = 1;
        }
    }
    /* 再次进入短临界区，提交本次写标志的最终状态。 */
    taskENTER_CRITICAL();
    if (result == 0)
    {
        /* 写入并读回一致：状态置READY，通知LVGL排队复位回执。 */
        s_store_state = OTA_STORE_READY;
    }
    else
    {
        /* 写或读回失败：状态置FAILED，由LVGL回报错误并退回IDLE。 */
        s_store_state = OTA_STORE_FAILED;
    }
    /* 退出临界区。 */
    taskEXIT_CRITICAL();
    /* 返回1表示标志已确认(调用方应停止后续持久化并等待复位)，其余返回0。 */
    return result == 0;
}

/* LVGL每轮调用：READY时排队REBOOTING回执，FAILED时改回IDLE并回EEPROM_ERROR。
 * 回收发送结果后留500ms窗口，尚未回收至2s时也复位。返回1表示请求仍在处理。 */
uint8_t app_ota_service(uint8_t ready, uint32_t now_ms)
{
    /* state为临界区内取出的存储状态副本。 */
    ota_store_state_enum state;
    /* result接收异步发送的完成结果或启动结果。 */
    HAL_StatusTypeDef result;
    /* 进入短临界区，读取sensor更新后的存储状态。 */
    taskENTER_CRITICAL();
    state = s_store_state;
    /* 退出临界区，回执与复位流程在临界区外执行。 */
    taskEXIT_CRITICAL();

    /* 标志已确认且尚未安排复位：置复位流程标志、清回执完成标记并挂上REBOOTING文本。 */
    if (state == OTA_STORE_READY && s_ota.reboot == 0)
    {
        s_ota.reboot = 1;
        s_ota.replied = 0;
        /* 以当前毫秒tick作为回执/复位等待的计时起点。 */
        s_ota.reply_ms = now_ms;
        s_ota.reply = "OTA,REBOOTING,SPP\r\n";
    }
    /* 存储失败：挂EEPROM错误回执，并把状态退回IDLE以便后续请求重新尝试。 */
    else if (state == OTA_STORE_FAILED)
    {
        s_ota.reply = "OTA,EEPROM_ERROR\r\n";
        /* 进入短临界区修改共享状态。 */
        taskENTER_CRITICAL();
        s_store_state = OTA_STORE_IDLE;
        /* 退出临界区。 */
        taskEXIT_CRITICAL();
    }
    /* 有在途发送且已取回完成结果：清在途标记并释放回执文本；发送结果可能成功或失败。 */
    if (s_ota.pending != 0 && kt6368a_take_tx_result(&result) != 0)
    {
        s_ota.pending = 0;
        s_ota.reply = NULL;
        s_ota.replied = 1;
        /* 记录回执完成时刻，作为500ms复位窗口的起点。 */
        s_ota.reply_ms = now_ms;
    }
    /* 复位等待：已取回发送结果满500ms，或尚未取回结果已满2s，即执行系统复位。 */
    if (s_ota.reboot != 0 && ((s_ota.replied != 0 && (uint32_t)(now_ms - s_ota.reply_ms) >= 500) ||
        (s_ota.replied == 0 && (uint32_t)(now_ms - s_ota.reply_ms) >= 2000)))
    {
        /* 即使手机断开/回执失败，已确认的升级标志仍必须交给Bootloader处理。 */
        NVIC_SystemReset();
        return 1;
    }
    /* 链路就绪、有待发文本、无在途发送且UART空闲时，异步启动一次回执发送。 */
    if (ready != 0 && s_ota.reply != NULL && s_ota.pending == 0 && kt6368a_tx_is_busy() == 0)
    {
        /* 按文本长度启动异步发送，100为本次发送的超时毫秒数。 */
        result = kt6368a_write_async((const uint8_t *)s_ota.reply, (uint16_t)strlen(s_ota.reply), 100);
        if (result == HAL_OK)
        {
            /* 启动成功：置在途标记，等待后续取发送结果。 */
            s_ota.pending = 1;
        }
        /* 非忙的启动失败：丢弃该回执，避免反复重试同一文本。 */
        else if (result != HAL_BUSY)
        {
            s_ota.reply = NULL;
        }
    }
    /* 仍有待处理请求、复位流程、待发文本或在途发送时返回1，表示OTA仍在进行。 */
    return state == OTA_STORE_REQUESTED || s_ota.reboot != 0 || s_ota.reply != NULL || s_ota.pending != 0;
}
