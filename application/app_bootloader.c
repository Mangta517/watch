#include "app_bootloader.h"
#include "kt6368a.h"
#include <stdio.h>
#include <string.h>

Bootloader_state boot_state = BOOTLOADER_STATE_DEFAULT;

static uint32_t uart1_rx_app_len;
static uint32_t uart1_rx_app_crc;
static uint8_t uart1_rx_crc_enabled;
static uint32_t command_last_tick;
static uint32_t command_errors;
static uint32_t command_dropped;
static uint32_t reply_complete_tick;
static char command[64];
static uint8_t command_length;
static uint8_t command_overflow;

static void app_bootloader_message(const char *message)
{
    (void)kt6368a_write((const uint8_t *)message, (uint16_t)strlen(message), 1000U);
}

static void app_bootloader_fail(void)
{
    int_bootloader_stop();
    app_bootloader_message("app rec error,system reset; reconnect and send start again\n");
    HAL_Delay(500U); /* 失败路径同样先给模块发送回执，再复位断开连接。 */
    NVIC_SystemReset();
}

void app_bootloader_init(void)
{
    command_length = command_overflow = 0;
    command_errors = kt6368a_get_rx_errors();
    command_dropped = kt6368a_get_rx_dropped();
    app_bootloader_message("Bootloader started, waiting for user data transfer...\r\n"
                           "Please use the bootloader tool to send data to the device.\r\n"
                           "send 'start:len[:crc32]' to start\r\n");
    boot_state = BOOTLOADER_STATE_INIT;
}

static void app_bootloader_parse_command(void)
{
    uint32_t length = 0;
    uint32_t crc = 0;
    uint8_t i;
    uint8_t digit;
    uint8_t crc_enabled = 0;
    uint8_t valid = 1;
    char response[80];

    if (command_overflow != 0U || command_length <= 6U || memcmp(command, "start:", 6) != 0)
    {
        valid = 0;
    }
    else
    {
        for (i = 6; i < command_length; ++i)
        {
            if (command[i] == ':')
            {
                break;
            }
            if (command[i] < '0' || command[i] > '9' || length > (APP_MAX_SIZE - (uint32_t)(command[i] - '0')) / 10U)
            {
                valid = 0;
                break;
            }
            length = length * 10U + (uint32_t)(command[i] - '0');
        }
        if (valid != 0U && i < command_length)
        {
            /* CRC字段必须恰好8位十六进制，不能把残缺字段降级为旧协议。 */
            if (i == 6U || command_length - i != 9U)
            {
                valid = 0;
            }
            else
            {
                crc_enabled = 1;
                for (++i; i < command_length; ++i)
                {
                    if (command[i] >= '0' && command[i] <= '9')
                    {
                        digit = (uint8_t)(command[i] - '0');
                    }
                    else if (command[i] >= 'a' && command[i] <= 'f')
                    {
                        digit = (uint8_t)(command[i] - 'a' + 10);
                    }
                    else if (command[i] >= 'A' && command[i] <= 'F')
                    {
                        digit = (uint8_t)(command[i] - 'A' + 10);
                    }
                    else
                    {
                        valid = 0;
                        break;
                    }
                    crc = (crc << 4) | digit;
                }
            }
        }
    }
    if (valid != 0U && length >= 8U && length <= APP_MAX_SIZE)
    {
        uart1_rx_app_len = length;
        uart1_rx_app_crc = crc;
        uart1_rx_crc_enabled = crc_enabled;
        (void)snprintf(response, sizeof(response), "Received start command, program length: %lu len\r\n", (unsigned long)length);
        app_bootloader_message(response);
        boot_state = BOOTLOADER_STATE_RUN;
    }
    else
    {
        app_bootloader_message("Invalid length/command, use 'start:len[:crc32]'\r\n");
    }
    command_length = command_overflow = 0;
}

static void app_bootloader_run(void)
{
    uint8_t byte;
    uint16_t budget = KT6368A_RX_BUFFER_SIZE;
    if (command_errors != kt6368a_get_rx_errors() || command_dropped != kt6368a_get_rx_dropped())
    {
        kt6368a_flush_rx();
        command_length = 0;
        command_overflow = 1;
        command_last_tick = HAL_GetTick();
        command_errors = kt6368a_get_rx_errors();
        command_dropped = kt6368a_get_rx_dropped();
    }
    while (budget-- != 0U && kt6368a_read(&byte, 1) != 0U)
    {
        command_last_tick = HAL_GetTick();
        if (byte == '\r' || byte == '\n')
        {
            if (command_length != 0U || command_overflow != 0U)
            {
                app_bootloader_parse_command();
                return;
            }
        }
        else if (command_length < sizeof(command))
        {
            command[command_length++] = (char)byte;
        }
        else
        {
            command_overflow = 1;
        }
    }
    /* 保留F407的空闲结束命令用法，不强制主机添加换行。 */
    if ((command_length != 0U || command_overflow != 0U) &&
        (uint32_t)(HAL_GetTick() - command_last_tick) >= BOOTLOADER_RX_IDLE_MS)
    {
        app_bootloader_parse_command();
    }
}

void app_bootloader_work(void)
{
    char response[48];
    switch (boot_state)
    {
        case BOOTLOADER_STATE_DEFAULT:
            app_bootloader_init();
            break;
        case BOOTLOADER_STATE_INIT:
            app_bootloader_run();
            break;
        case BOOTLOADER_STATE_RUN:
            /* 长度决定擦除范围；擦除结束前主机必须等待ready，不能提前发送BIN。 */
            kt6368a_flush_rx();
            if (int_bootloader_stop() != HAL_OK || int_bootloader_erase(uart1_rx_app_len) != HAL_OK ||
                bootloader_start_dma(uart1_rx_app_len) != HAL_OK)
            {
                app_bootloader_fail();
                break;
            }
            if (uart1_rx_crc_enabled != 0U)
            {
                int_bootloader_expect_crc(uart1_rx_app_crc);
            }
            else
            {
                app_bootloader_message("Warning: legacy transfer without CRC\r\n");
            }
            boot_state = BOOTLOADER_STATE_REC_DATA;
            app_bootloader_message("Flash erased, ready to receive data...\r\n");
            break;
        case BOOTLOADER_STATE_REC_DATA:
            if (int_bootloader_process() != HAL_OK)
            {
                app_bootloader_fail();
            }
            else if (uart1_rx_full_len == uart1_rx_app_len && uart1_rx_idle != 0U &&
                     (uint32_t)(HAL_GetTick() - uart1_rx_complete_tick) >= BOOTLOADER_COMPLETE_IDLE_MS)
            {
                boot_state = BOOTLOADER_STATE_CHECK_DATA;
            }
            break;
        case BOOTLOADER_STATE_CHECK_DATA:
            if (int_bootloader_finish() != HAL_OK)
            {
                app_bootloader_fail();
                break;
            }
            (void)snprintf(response, sizeof(response), "ok,len:%lu\n", (unsigned long)uart1_rx_app_len);
            app_bootloader_message(response);
            reply_complete_tick = HAL_GetTick();
            boot_state = BOOTLOADER_STATE_JUMP_TO_APP;
            break;
        case BOOTLOADER_STATE_JUMP_TO_APP:
            /* APP启动会重新切换模块电源，先给SPP回执留500ms发送窗口。 */
            if ((uint32_t)(HAL_GetTick() - reply_complete_tick) >= 500U)
            {
                int_bootloader_jump_to_app();
                app_bootloader_fail();
            }
            break;
        default:
            break;
    }
}
