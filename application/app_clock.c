/**
 * @file app_clock.c
 * @brief 手机校时文本帧(H:M / HH:MM / HH:MM:SS)解析并写入RTC；只在LVGL任务收发，不访问UART。
 *        一次只保留一行(最长8字符)在制帧；解析结果与合法帧计数留给UI与诊断读取。
 */
#include "app_clock.h"
#include "watch_rtc.h"
#include <stddef.h>

/* 行组装状态：记录在制帧、溢出标记与最近一次处理结论。 */
typedef struct
{
    uint8_t frame[8];
    uint8_t length;
    uint8_t overflow;
    uint8_t has_colon;
    uint32_t last_byte_ms; /* 最近一个非结束符字节的到达时刻，用于空闲收尾 */
    uint32_t received_count;
    app_clock_result result;
} app_clock_parser_struct;

/* 无并发保护：读写方都是LVGL任务(app_ui的蓝牙接收入口与显示轮询)。 */
static app_clock_parser_struct parser;

/* 只废弃在制帧，不动result/received_count：蓝牙丢字节不应抹掉已成功的校时记录。 */
void app_clock_reset_receive(void)
{
    /* 清空在制帧长度，下一批字节从帧首重新写入。 */
    parser.length = 0;
    /* 清除超长标记，避免上一段残帧的溢出结论带到下一帧。 */
    parser.overflow = 0;
    /* 清除冒号标记，新帧需重新出现冒号才会进入校时判断。 */
    parser.has_colon = 0;
}

/* 仅处理含冒号的帧：范围/格式非法记INVALID，合法才写RTC；无冒号帧忽略并保留上次结论，RTC写失败记RTC_ERROR。 */
static void app_clock_finish(void)
{
    /* i为帧内字节下标；part为当前字段序号(0时/1分/2秒)；digits为本段已收数字位数；value按段累计数值。 */
    uint8_t i, part = 0, digits = 0, value[3] = {0};
    /* 先给合法性一个初值：未溢出且帧长在3..8字符之间才可能继续校验。 */
    uint8_t valid = !parser.overflow && parser.length >= 3 && parser.length <= 8;
    /* 没有冒号(含空行)的帧直接丢弃，不记INVALID，避免其他用途的文本干扰校时结论。 */
    if (parser.has_colon)
    {
        /* 逐字节扫描在制帧；一旦valid被置0，循环条件不再成立并提前结束。 */
        for (i = 0; valid && i < parser.length; ++i)
        {
            /* 遇到冒号且当前字段未超过秒段、本段已收过数字：结束本段并切到下一段。 */
            if (parser.frame[i] == ':' && part < 2 && digits != 0)
            {
                ++part;
                digits = 0;
            }
            /* 普通十进制数字且本段不足2位：按原值乘10加当前位累加到对应字段。 */
            else if (parser.frame[i] >= '0' && parser.frame[i] <= '9' && digits < 2)
            {
                value[part] = value[part] * 10 + parser.frame[i] - '0';
                ++digits;
            }
            else
            {
                /* 其他字符(字母、多余冒号、第3位数字等)使整帧非法。 */
                valid = 0;
            }
        }
        /* 收尾校验：至少包含时、分两个字段、末段有数字，且时0..23、分0..59、秒0..59；缺省秒保持0。 */
        valid = valid && part >= 1 && digits != 0 && value[0] <= 23 && value[1] <= 59 && value[2] <= 59;
        if (!valid)
        {
            /* 格式或范围不合法：只记录INVALID，不触碰RTC与合法帧计数。 */
            parser.result = APP_CLOCK_INVALID;
        }
        else
        {
            ++parser.received_count; /* 合法手机消息确认通信已建立，RTC写入失败也计入。 */
            /* 把解析出的时/分/秒交给RTC驱动写入，返回0表示写入成功。 */
            if (watch_rtc_set_time(value[0], value[1], value[2]) == 0)
            {
                parser.result = APP_CLOCK_SYNCED;
            }
            else
            {
                /* 校验通过但RTC写失败，结论与成功区分开，便于UI/诊断显示。 */
                parser.result = APP_CLOCK_RTC_ERROR;
            }
        }
    }
    /* 一帧处理完毕：清空长度，下一批字节重新从帧首开始。 */
    parser.length = 0;
    /* 清溢出标记，避免影响下一帧。 */
    parser.overflow = 0;
    /* 清冒号标记，下一帧需重新出现冒号才进入校时分支。 */
    parser.has_colon = 0;
}

/* 无换行的残帧靠 APP_CLOCK_FRAME_IDLE_MS 静默收尾；一帧只处理一次，不会重复写RTC。 */
void app_clock_service(uint32_t now_ms)
{
    /* 有在制字节或已溢出时，计算距最近非结束符字节的毫秒差；达到空闲门限即按一帧收尾。 */
    if ((parser.length || parser.overflow) && (uint32_t)(now_ms - parser.last_byte_ms) >= APP_CLOCK_FRAME_IDLE_MS)
    {
        app_clock_finish();
    }
}

/* 解析一行：H:M / HH:MM / HH:MM:SS，省略秒时按0写入RTC。 */
void app_clock_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    /* i为本批数据的字节下标。 */
    uint16_t i;
    /* 无数据指针时直接返回，不改变在制帧。 */
    if (data == NULL)
    {
        return;
    }
    /* 先补收尾再喂新字节：距上一字节已满200ms的分段帧按已完成处理，本批字节只组成下一帧。 */
    app_clock_service(now_ms);
    /* 逐字节处理本批数据：换行符收尾，其他字节追加到在制帧。 */
    for (i = 0; i < size; ++i)
    {
        /* CR或LF表示一帧结束：立即解析并尝试写RTC，随后由finish清空在制帧。 */
        if (data[i] == '\r' || data[i] == '\n')
        {
            app_clock_finish();
        }
        else
        {
            /* 帧缓冲未满时把当前字节追加到尾部并后移长度。 */
            if (parser.length < sizeof(parser.frame))
            {
                parser.frame[parser.length++] = data[i];
            }
            else
            {
                /* 超出8字节缓冲：不再写入，只置溢出标记；含冒号的校时帧在收尾时判为非法，无冒号帧仍按原规则忽略。 */
                parser.overflow = 1;
            }
            /* 记录本帧已出现冒号，供收尾时决定是否按校时文本解析。 */
            if (data[i] == ':')
            {
                parser.has_colon = 1;
            }
            /* 记录最近非结束符字节的到达时刻，作为空闲收尾的计时基准。 */
            parser.last_byte_ms = now_ms;
        }
    }
}

/* 返回最近一帧的解析结果；只读解析状态，无并发保护。 */
app_clock_result app_clock_get_result(void)
{
    /* 返回最近一帧的解析结论(SYNCED/INVALID/RTC_ERROR/NONE)，不改变解析状态。 */
    return parser.result;
}

/* 合法帧计数：只有通过范围校验并尝试写RTC的帧才累加，写失败也计入。 */
uint32_t app_clock_get_received_count(void)
{
    /* 返回通过校验并尝试写RTC的累计帧数，只读计数。 */
    return parser.received_count;
}
