/**
 * @file app_bootloader.c
 * @brief 保底程序收固件状态机：文本命令(start:len[:crc32]/OTA,INFO) + 循环DMA分块接收 + CRC校验。
 *        整幅镜像不驻留RAM：int层用8KiB双半区环形缓冲按已接收进度分块搬运并写APP区Flash，本层只看进度与结果。
 *        启用CRC协议时才校验整幅CRC；start:len为旧协议，允许不带CRC。
 *        int层擦除/编程/DMA回报0成功、1失败；本层只做协议解析、回执和跳转/复位决策。
 */
#include "app_bootloader.h"
#include "kt6368a.h"
#include <stdio.h>
#include <string.h>

Bootloader_state boot_state = BOOTLOADER_STATE_DEFAULT;

/* 本轮传输参数：start命令解析成功后才有效，直到复位或跳转。 */
static uint32_t uart1_rx_app_len;
static uint32_t uart1_rx_app_crc;
static uint8_t uart1_rx_crc_enabled;
/* 行缓冲状态：last_tick用于无换行时按BOOTLOADER_COMMAND_IDLE_MS做静默判定，
 * errors/dropped快照发现接收丢字节后整帧作废。 */
static uint32_t command_last_tick;
static uint32_t command_errors;
static uint32_t command_dropped;
static uint32_t reply_complete_tick;
static char command[64];
static uint8_t command_length;
static uint8_t command_overflow;

/* 回执统一走带1000ms超时的阻塞发送；返回值忽略，提示丢失不改变状态机。 */
static void app_bootloader_message(const char *message)
{
    /* 以字符串实际长度作为发送字节数，超时1000ms；返回值丢弃，回执发不出也继续状态机。 */
    (void)kt6368a_write((const uint8_t *)message, (uint16_t)strlen(message), 1000);
}

static void app_bootloader_fail(void)
{
    /* 先停掉int层的DMA/接收流程，避免复位前继续写Flash。 */
    int_bootloader_stop();
    /* 给上位机发一条错误提示，说明需要重新连接并再发start。 */
    app_bootloader_message("app rec error,system reset; reconnect and send start again\n");
    HAL_Delay(500); /* 失败路径同样先给模块发送回执，再复位断开连接。 */
    /* 软件复位MCU，重新走启动选择流程。 */
    NVIC_SystemReset();
}

void app_bootloader_init(void)
{
    /* 清空行长度与溢出标志，从空命令开始收。 */
    command_length = command_overflow = 0;
    /* 先取接收故障快照作基线，避免把开机前已发生的丢字节算到第一条命令上。 */
    command_errors = kt6368a_get_rx_errors();
    command_dropped = kt6368a_get_rx_dropped();
    /* 发送欢迎与用法提示，告诉上位机用start:len[:crc32]开始。 */
    app_bootloader_message("Independent recovery started, waiting for user data transfer...\r\n"
                           "Please use the bootloader tool to send data to the device.\r\n"
                           "send 'start:len[:crc32]' to start\r\n");
    /* 状态切到INIT，后续work()每轮解析命令行。 */
    boot_state = BOOTLOADER_STATE_INIT;
}

/* 解析一行命令：OTA,INFO回报布局；start:len[:crc32]校验长度区间与8位十六进制CRC，
 * 通过后切到RUN。非法或残缺命令回执提示，CRC字段残缺被明确拒绝，start:len旧协议可无CRC。 */
static void app_bootloader_parse_command(void)
{
    uint32_t length = 0; /* 从命令文本解析出的镜像长度(字节)。 */
    uint32_t crc = 0; /* 从8位十六进制文本解析出的CRC32。 */
    uint8_t i; /* 命令缓冲区扫描下标。 */
    uint8_t digit; /* 单个十六进制字符换算出的4位数值。 */
    uint8_t crc_enabled = 0; /* 本命令是否带CRC字段。 */
    uint8_t valid = 1; /* 解析是否仍然有效，遇非法字符置0。 */
    char response[80]; /* 回执文本缓冲区。 */

    /* 长度正好8且内容为OTA,INFO：回报APP区起始地址与最大长度。 */
    if (command_overflow == 0 && command_length == 8 && memcmp(command, "OTA,INFO", 8) == 0)
    {
        /* 按%08lX输出8位大写十六进制地址和大小，行尾CRLF。 */
        (void)snprintf(response, sizeof(response), "OTA,LAYOUT,%08lX,%08lX\r\n",
                       (unsigned long)APP_START_ADDR, (unsigned long)APP_MAX_SIZE);
        /* 把布局文本发给上位机。 */
        app_bootloader_message(response);
        /* 命令已消费，清空行缓冲状态后返回。 */
        command_length = command_overflow = 0;
        return;
    }
    /* 溢出、长度不足或前缀不是start:：判为非法命令。 */
    if (command_overflow != 0 || command_length <= 6 || memcmp(command, "start:", 6) != 0)
    {
        valid = 0;
    }
    else
    {
        /* 从第6字节起逐字符解析十进制长度，遇冒号结束。 */
        for (i = 6; i < command_length; ++i)
        {
            if (command[i] == ':')
            {
                break;
            }
            /* 非数字，或乘10加本位会超过APP_MAX_SIZE，直接判非法。 */
            if (command[i] < '0' || command[i] > '9' || length > (APP_MAX_SIZE - (uint32_t)(command[i] - '0')) / 10)
            {
                valid = 0;
                break;
            }
            /* 字符减'0'得到0..9，再累加到长度。 */
            length = length * 10 + (uint32_t)(command[i] - '0');
        }
        /* 长度字段后面确实有冒号时，继续解析CRC字段。 */
        if (valid != 0 && i < command_length)
        {
            /* CRC字段必须恰好8位十六进制，不能把残缺字段降级为旧协议。 */
            if (i == 6 || command_length - i != 9)
            {
                valid = 0;
            }
            else
            {
                /* 标记本命令启用CRC校验。 */
                crc_enabled = 1;
                /* 跳过冒号，逐位把十六进制字符累进crc：先左移4位再或入本位数。 */
                for (++i; i < command_length; ++i)
                {
                    if (command[i] >= '0' && command[i] <= '9')
                    {
                        /* 数字字符换算为0..9。 */
                        digit = (uint8_t)(command[i] - '0');
                    }
                    else if (command[i] >= 'a' && command[i] <= 'f')
                    {
                        /* 小写a..f换算为10..15。 */
                        digit = (uint8_t)(command[i] - 'a' + 10);
                    }
                    else if (command[i] >= 'A' && command[i] <= 'F')
                    {
                        /* 大写A..F换算为10..15。 */
                        digit = (uint8_t)(command[i] - 'A' + 10);
                    }
                    else
                    {
                        /* 出现非十六进制字符，整条命令作废。 */
                        valid = 0;
                        break;
                    }
                    /* 已解析的4位左移，腾出低4位后填入本字符数值。 */
                    crc = (crc << 4) | digit;
                }
            }
        }
    }
    /* 解析有效且长度在8字节到APP_MAX_SIZE之间：保存参数并进入RUN。 */
    if (valid != 0 && length >= 8 && length <= APP_MAX_SIZE)
    {
        /* 保存长度、CRC值以及是否启用CRC，供RUN/CHECK阶段使用。 */
        uart1_rx_app_len = length;
        uart1_rx_app_crc = crc;
        uart1_rx_crc_enabled = crc_enabled;
        /* 回执确认收到的长度，然后切到RUN准备擦除。 */
        (void)snprintf(response, sizeof(response), "Received start command, program length: %lu len\r\n", (unsigned long)length);
        app_bootloader_message(response);
        boot_state = BOOTLOADER_STATE_RUN;
    }
    else
    {
        /* 非法或越界：回执用法提示，状态保持INIT继续等命令。 */
        app_bootloader_message("Invalid length/command, use 'start:len[:crc32]'\r\n");
    }
    /* 无论成功失败都清空行缓冲，准备接收下一条命令。 */
    command_length = command_overflow = 0;
}

/* INIT阶段攒一行文本：收到CR/LF立即解析；主机不发换行时，靠BOOTLOADER_COMMAND_IDLE_MS静默超时收尾。 */
static void app_bootloader_run(void)
{
    uint8_t byte; /* 本轮从接收队列取出的单个字节。 */
    /* 每轮最多取走一整块接收缓冲，剩余字节下一轮继续，不让主机卡住本循环。 */
    uint16_t budget = KT6368A_RX_BUFFER_SIZE;
    /* 接收错误计数或丢字节计数与基线不同：说明有数据丢失，整行作废。 */
    if (command_errors != kt6368a_get_rx_errors() || command_dropped != kt6368a_get_rx_dropped())
    {
        /* 清空接收队列，重置行缓冲并置溢出标志，让本行不会被误解析。 */
        kt6368a_flush_rx();
        command_length = 0;
        command_overflow = 1;
        /* 以当前时刻重起静默计时，并刷新错误/丢字节基线。 */
        command_last_tick = HAL_GetTick();
        command_errors = kt6368a_get_rx_errors();
        command_dropped = kt6368a_get_rx_dropped();
    }
    /* 在预算内逐字节读取，直到队列空或预算用完。 */
    while (budget-- != 0 && kt6368a_read(&byte, 1) != 0)
    {
        /* 每收到一个字节就刷新静默计时起点。 */
        command_last_tick = HAL_GetTick();
        /* 收到CR或LF视为一行结束。 */
        if (byte == '\r' || byte == '\n')
        {
            /* 行内已有内容或已溢出才解析，空行直接忽略。 */
            if (command_length != 0 || command_overflow != 0)
            {
                /* 解析本条命令后返回，剩余字节留到下一轮。 */
                app_bootloader_parse_command();
                return;
            }
        }
        /* 普通字节且缓冲区未满：追加到命令行末尾并递增长度。 */
        else if (command_length < sizeof(command))
        {
            command[command_length++] = (char)byte;
        }
        else
        {
            /* 行缓冲已满仍继续来字节：只置溢出标志，不再写入。 */
            command_overflow = 1;
        }
    }
    /* 无换行的命令靠 BOOTLOADER_COMMAND_IDLE_MS 静默超时收尾，不强制主机添加换行；
     * BLE分片送来的命令片间隔远大于DMA数据的10ms空闲阈值，用后者会把半条命令当整条解析。 */
    if ((command_length != 0 || command_overflow != 0) &&
        (uint32_t)(HAL_GetTick() - command_last_tick) >= BOOTLOADER_COMMAND_IDLE_MS)
    {
        /* 已积累内容且静默超时，按完整一行解析。 */
        app_bootloader_parse_command();
    }
}

void app_bootloader_work(void)
{
    char response[48]; /* CHECK_DATA成功后的回执文本缓冲区。 */
    /* 按当前状态机状态分派本轮动作。 */
    switch (boot_state)
    {
    case BOOTLOADER_STATE_DEFAULT:
        /* 上电首轮：发欢迎提示并进入INIT。 */
        app_bootloader_init();
        break;
    case BOOTLOADER_STATE_INIT:
        /* 等待并解析上位机命令。 */
        app_bootloader_run();
        break;
    case BOOTLOADER_STATE_RUN:
        /* 长度决定擦除范围；擦除结束前主机必须等待ready，不能提前发送BIN。 */
        /* 丢掉残留命令字节，避免被当成固件数据。 */
        kt6368a_flush_rx();
        /* 依次停旧流程、按长度擦除APP区、启动循环DMA；任一步失败走复位。 */
        if (int_bootloader_stop() != 0 || int_bootloader_erase(uart1_rx_app_len) != 0 ||
            bootloader_start_dma(uart1_rx_app_len) != 0)
        {
            app_bootloader_fail();
            break;
        }
        /* 本命令带CRC：把期望值交给int层，收完数据后整幅校验。 */
        if (uart1_rx_crc_enabled != 0)
        {
            int_bootloader_expect_crc(uart1_rx_app_crc);
        }
        else
        {
            /* 旧协议无CRC：只回执警告，不设置期望值。 */
            app_bootloader_message("Warning: legacy transfer without CRC\r\n");
        }
        /* 进入收数据状态，并告知主机Flash已擦除可以发送。 */
        boot_state = BOOTLOADER_STATE_REC_DATA;
        app_bootloader_message("Flash erased, ready to receive data...\r\n");
        break;
    case BOOTLOADER_STATE_REC_DATA:
        /* int层按已接收进度分块搬运并写Flash；接收长度满足且静默达到收尾条件后再检查完成状态，
         * 提前结束会把慢速链路的尾字节当成完整镜像。 */
        if (int_bootloader_process() != 0)
        {
            /* 接收/写Flash过程报错：复位重来。 */
            app_bootloader_fail();
        }
        /* 已收满声明长度、链路已静默且静默超过收尾阈值：转入完成检查。 */
        else if (uart1_rx_full_len == uart1_rx_app_len && uart1_rx_idle != 0 &&
                 (uint32_t)(HAL_GetTick() - uart1_rx_complete_tick) >= BOOTLOADER_COMPLETE_IDLE_MS)
        {
            boot_state = BOOTLOADER_STATE_CHECK_DATA;
        }
        break;
    case BOOTLOADER_STATE_CHECK_DATA:
        /* finish完成收尾检查，启用CRC协议时才校验整幅CRC，再做写后回读和向量提交，
         * 全过才把EEPROM标志改成VALID。 */
        if (int_bootloader_finish() != 0)
        {
            /* 校验/回读/提交失败：复位。 */
            app_bootloader_fail();
            break;
        }
        /* 全部通过：回执ok与本次长度，记录回执时刻供跳转前留发送窗口。 */
        (void)snprintf(response, sizeof(response), "ok,len:%lu\n", (unsigned long)uart1_rx_app_len);
        app_bootloader_message(response);
        reply_complete_tick = HAL_GetTick();
        boot_state = BOOTLOADER_STATE_JUMP_TO_APP;
        break;
    case BOOTLOADER_STATE_JUMP_TO_APP:
        /* APP启动会重新切换模块电源，先给BLE回执留500ms发送窗口。 */
        if ((uint32_t)(HAL_GetTick() - reply_complete_tick) >= 500)
        {
            /* jump只在镜像向量校验通过时生效；返回即失败，走复位让主引导重新选择目标。 */
            int_bootloader_jump_to_app();
            app_bootloader_fail();
        }
        break;
    default:
        /* 未知状态不动作，下一轮继续按原状态进入。 */
        break;
    }
}
