"""Execute the real APP OTA command, EEPROM handoff and UART/reboot lifecycle."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
header = r'''
#ifndef OTA_HOST_H
#define OTA_HOST_H
#include <stdint.h>
#define BOOTLOADER_EEPROM_FLAG_ADDR 0xC0
#define BOOTLOADER_FLAG_UPDATING 0x5A
#define BOOTLOADER_FLAG_VALID 0xA5
typedef enum
{
    HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT
} HAL_StatusTypeDef;
void host_enter(void);
void host_exit(void);
#define taskENTER_CRITICAL() host_enter()
#define taskEXIT_CRITICAL() host_exit()
uint8_t bl24c02_write(uint16_t address, const uint8_t *data, uint16_t length);
uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length);
uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result);
uint8_t kt6368a_tx_is_busy(void);
HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout);
void NVIC_SystemReset(void);
#endif
'''
harness = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_ota.c"
static uint8_t eeprom[256], error, tx_busy, tx_done;
static unsigned writes, reads, sends, resets, critical;
static const uint8_t *pending_data;
static char message[128];
void host_enter(void)
{
    ++critical;
}
void host_exit(void)
{
    assert(critical != 0);
    --critical;
}
uint8_t bl24c02_write(uint16_t address, const uint8_t *data, uint16_t length)
{
    assert(critical == 0 && address == 0xC0 && length == 4);
    ++writes;
    if (error == 1)
    {
        return 1;
    }
    for (unsigned i = 0; i < length; ++i)
    {
        eeprom[address + i] = data[i];
    }
    return 0;
}
uint8_t bl24c02_read(uint16_t address, uint8_t *data, uint16_t length)
{
    assert(critical == 0 && address == 0xC0 && length == 4);
    ++reads;
    for (unsigned i = 0; i < length; ++i)
    {
        data[i] = eeprom[address + i];
    }
    if (error == 3)
    {
        data[2] ^= 1;
    }
    return error == 2;
}
uint8_t kt6368a_tx_is_busy(void)
{
    return tx_busy;
}
uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    assert(pending_data != NULL && strcmp((const char *)pending_data, message) == 0);
    if (tx_done == 0)
    {
        return 0;
    }
    *result = HAL_OK;
    tx_busy = 0;
    return 1;
}
HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(tx_busy == 0 && timeout == 100 && size < sizeof(message));
    for (unsigned i = 0; i < size; ++i)
    {
        message[i] = (char)data[i];
    }
    message[size] = 0;
    pending_data = data;
    tx_busy = 1;
    ++sends;
    return HAL_OK;
}
void NVIC_SystemReset(void)
{
    assert(critical == 0);
    ++resets;
}
static void init(void)
{
    memset(&s_ota, 0, sizeof(s_ota));
    memset(eeprom, 0x3C, sizeof(eeprom));
    s_store_state = OTA_STORE_IDLE;
    writes = reads = sends = resets = 0;
    error = tx_busy = tx_done = 0;
}
static void feed(const char *text, uint32_t now)
{
    app_ota_receive((const uint8_t *)text, (uint16_t)strlen(text), now);
}
int main(void)
{
    init();
    feed("OTA,STA", 10);
    app_ota_reset_receive();
    feed("RT\n", 11);
    assert(app_ota_storage_service(1) == 0 && writes == 0);
    feed("OTA,STARTxxxxxxxxxxxxxxxxxx\nOTA,START", 20);
    assert(app_ota_storage_service(1) == 0 && writes == 0);
    feed("\n", 21);
#if VECT_TAB_OFFSET == 0
    assert(app_ota_storage_service(1) == 0 && writes == 0);
    assert(app_ota_service(1, 22) == 1);
    assert(strstr(message, "UNAVAILABLE") != NULL && resets == 0);
    puts("PASS: standalone APP refuses OTA without a Bootloader partition");
#else
    assert(app_ota_storage_service(0) == 0 && writes == 0);
    assert(app_ota_service(1, 22) == 1 && sends == 0);
    assert(app_ota_storage_service(1) == 1 && writes == 1 && reads == 1);
    assert(memcmp(eeprom + 0xC0, "\x42\x4C\x5A\xA5", 4) == 0);
    for (unsigned i = 0; i < sizeof(eeprom); ++i)
    {
        if (i < 0xC0 || i >= 0xC4)
        {
            assert(eeprom[i] == 0x3C);
        }
    }
    assert(app_ota_service(1, 30) == 1 && sends == 1);
    app_ota_reset_receive();
    app_ota_service(0, 31);
    tx_done = 1;
    app_ota_service(0, 40);
    assert(app_ota_storage_service(1) == 1 && writes == 1);
    app_ota_service(0, 539);
    assert(resets == 0);
    app_ota_service(0, 540);
    assert(resets == 1);
    for (unsigned failure = 1; failure <= 3; ++failure)
    {
        init();
        feed("OTA,START\r\n", 0);
        error = failure;
        assert(app_ota_storage_service(1) == 0);
        app_ota_service(1, 1);
        assert(strstr(message, "EEPROM_ERROR") != NULL && resets == 0);
    }
    init();
    feed("OTA,START\n", UINT32_MAX - 20);
    assert(app_ota_storage_service(1) == 1);
    app_ota_service(0, UINT32_MAX - 10);
    app_ota_service(0, 1988);
    assert(resets == 0);
    app_ota_service(0, 1989);
    assert(resets == 1 && sends == 0);
    puts("PASS: fragmented commands, loss reset, EEPROM ownership/readback/errors, stable async reply, disconnect and tick wrap");
#endif
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="watch-ota-") as folder:
    output = Path(folder)
    for name in ["int_bootloader.h", "bl24c02.h", "kt6368a.h", "FreeRTOS.h", "task.h"]:
        (output / name).write_text(header, encoding="utf-8")
    source = output / "test.c"
    source.write_text(harness, encoding="utf-8")
    for offset in ["0", "0x10000"]:
        exe = output / ("ota_" + offset + ".exe")
        subprocess.run([shutil.which("gcc"), "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "-DVECT_TAB_OFFSET=" + offset, "-I" + str(output), "-I" + str(root / "application"), "-I" + str(root / "common"),
                        str(source), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
