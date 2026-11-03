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
    s_ota.length = 0;
    s_ota.overflow = 0;
    /* 完整命令已交给sensor时不能因蓝牙断开撤销持久化/复位。 */
}

void app_ota_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    uint16_t i;
    if ((uint32_t)(now_ms - s_ota.rx_ms) >= 1000)
    {
        app_ota_reset_receive();
    }
    s_ota.rx_ms = now_ms;
    for (i = 0; i < size; ++i)
    {
        if (data[i] == '\r' || data[i] == '\n')
        {
            s_ota.line[s_ota.length] = '\0';
            if (s_ota.overflow == 0 && strcmp(s_ota.line, "OTA,START") == 0 && s_ota.reply == NULL && s_ota.pending == 0)
            {
#if defined(VECT_TAB_OFFSET) && VECT_TAB_OFFSET == (APP_START_ADDR - BOOTLOADER_START_ADDR)
                taskENTER_CRITICAL();
                if (s_store_state == OTA_STORE_IDLE)
                {
                    s_store_state = OTA_STORE_REQUESTED;
                }
                taskEXIT_CRITICAL();
#else
                /* 独立普通APP占用0x08000000，不能复位后误入不存在的Bootloader。 */
                s_ota.reply = "OTA,UNAVAILABLE,INSTALL_BOOTLOADER_AND_UPGRADE_APP\r\n";
#endif
            }
            app_ota_reset_receive();
        }
        else if (data[i] < 32 || data[i] > 126 || s_ota.length >= sizeof(s_ota.line) - 1)
        {
            s_ota.overflow = 1;
        }
        else
        {
            s_ota.line[s_ota.length++] = (char)data[i];
        }
    }
}

uint8_t app_ota_storage_service(uint8_t storage_available)
{
    const uint8_t record[4] = {0x42, 0x4C, BOOTLOADER_FLAG_UPDATING, BOOTLOADER_FLAG_VALID};
    uint8_t verify[4];
    uint8_t result;
    ota_store_state_enum state;

    taskENTER_CRITICAL();
    state = s_store_state;
    taskEXIT_CRITICAL();
    if (state == OTA_STORE_READY)
    {
        return 1; /* 复位前不再开始其他EEPROM写入。 */
    }
    if (state != OTA_STORE_REQUESTED || storage_available == 0)
    {
        return 0;
    }
    result = bl24c02_write(BOOTLOADER_EEPROM_FLAG_ADDR, record, sizeof(record));
    if (result == 0)
    {
        result = bl24c02_read(BOOTLOADER_EEPROM_FLAG_ADDR, verify, sizeof(verify));
        if (result == 0 && memcmp(record, verify, sizeof(record)) != 0)
        {
            result = 1;
        }
    }
    taskENTER_CRITICAL();
    if (result == 0)
    {
        s_store_state = OTA_STORE_READY;
    }
    else
    {
        s_store_state = OTA_STORE_FAILED;
    }
    taskEXIT_CRITICAL();
    return result == 0;
}

uint8_t app_ota_service(uint8_t ready, uint32_t now_ms)
{
    ota_store_state_enum state;
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    state = s_store_state;
    taskEXIT_CRITICAL();

    if (state == OTA_STORE_READY && s_ota.reboot == 0)
    {
        s_ota.reboot = 1;
        s_ota.replied = 0;
        s_ota.reply_ms = now_ms;
        s_ota.reply = "OTA,REBOOTING,SPP\r\n";
    }
    else if (state == OTA_STORE_FAILED)
    {
        s_ota.reply = "OTA,EEPROM_ERROR\r\n";
        taskENTER_CRITICAL();
        s_store_state = OTA_STORE_IDLE;
        taskEXIT_CRITICAL();
    }
    if (s_ota.pending != 0 && kt6368a_take_tx_result(&result) != 0)
    {
        s_ota.pending = 0;
        s_ota.reply = NULL;
        s_ota.replied = 1;
        s_ota.reply_ms = now_ms;
    }
    if (s_ota.reboot != 0 && ((s_ota.replied != 0 && (uint32_t)(now_ms - s_ota.reply_ms) >= 500) ||
        (s_ota.replied == 0 && (uint32_t)(now_ms - s_ota.reply_ms) >= 2000)))
    {
        /* 即使手机断开/回执失败，已确认的升级标志仍必须交给Bootloader处理。 */
        NVIC_SystemReset();
        return 1;
    }
    if (ready != 0 && s_ota.reply != NULL && s_ota.pending == 0 && kt6368a_tx_is_busy() == 0)
    {
        result = kt6368a_write_async((const uint8_t *)s_ota.reply, (uint16_t)strlen(s_ota.reply), 100);
        if (result == HAL_OK)
        {
            s_ota.pending = 1;
        }
        else if (result != HAL_BUSY)
        {
            s_ota.reply = NULL;
        }
    }
    return state == OTA_STORE_REQUESTED || s_ota.reboot != 0 || s_ota.reply != NULL || s_ota.pending != 0;
}
