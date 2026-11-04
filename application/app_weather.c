/**
 * @file app_weather.c
 * @brief 手机下发的天气帧解析(WEATHER,WMO码,温度x10,湿度\n)，全部在LVGL任务读写。
 *        天气来自手机而非本地AHT传感器；CR/LF与200ms接收空闲都可收尾，非法帧保留旧快照。
 */
#include "app_weather.h"
#include <stddef.h>
#include <string.h>

static app_weather_struct s_weather;
static char s_frame[32];
static uint8_t s_length;
static uint8_t s_overflow;
static uint32_t s_last_byte_ms;

void app_weather_reset_receive(void)
{
    /* 清空在制帧长度，下一批字节从帧首重新拼起。 */
    s_length = 0;
    /* 清超长标记，避免上一段残帧的溢出结论影响下一帧。 */
    s_overflow = 0;
}

/* 解析一个可选负号、至多4位十进制数；游标停在结束处。无数字或位数超限返回1，0=已取value。 */
static uint8_t parse_number(const char **cursor, int32_t *value)
{
    /* magnitude暂存按十进制累加出的绝对值，最后按符号写入value。 */
    int32_t magnitude = 0;
    /* 先看游标处是否为负号，决定是否跳过符号并按负值输出。 */
    uint8_t negative = **cursor == '-';
    /* 记录本数字已收的位数，用于限制至多4位。 */
    uint8_t digits = 0;
    if (negative)
    {
        /* 跳过负号，游标指向第一个数字。 */
        ++*cursor;
    }
    /* 连续读取十进制数字，直到遇到非数字字符。 */
    while (**cursor >= '0' && **cursor <= '9')
    {
        /* 第5位数字即超限：返回1表示解析失败，游标停在超限位。 */
        if (++digits > 4)
        {
            return 1;
        }
        /* 把当前位累加到绝对值，原值乘10再加新数字。 */
        magnitude = magnitude * 10 + **cursor - '0';
        /* 游标前进到下一字符。 */
        ++*cursor;
    }
    if (negative)
    {
        /* 带负号时写入负的绝对值。 */
        *value = -magnitude;
    }
    else
    {
        /* 无负号时直接写入绝对值。 */
        *value = magnitude;
    }
    /* 一个数字都没读到算失败(返回1)，否则返回0表示已取出value。 */
    return digits == 0;
}

/* 校验整帧：三段数字以逗号分隔且游标恰在末尾，码0..99、温度±1000、湿度0..100才更新快照。 */
static void finish_frame(void)
{
    /* 游标从"WEATHER,"之后的第8字节开始，依次读取三个数字字段。 */
    const char *cursor = s_frame + 8;
    /* code为WMO天气码，temperature为摄氏温度乘10的整数，humidity为相对湿度百分比。 */
    int32_t code, temperature, humidity;
    /* 在有效长度处补字符串结束符，供memcmp与游标判断使用。 */
    s_frame[s_length] = '\0';
    /* 整帧条件：未溢出且长度至少8；前缀为"WEATHER,"；依次解析码/温度/湿度且逗号位置正确；
     * 游标恰停在帧尾且该处为结束符；码0..99、温度-1000..1000、湿度0..100 才接受。 */
    if (!s_overflow && s_length >= 8 && memcmp(s_frame, "WEATHER,", 8) == 0 && parse_number(&cursor, &code) == 0 && *cursor++ == ',' &&
        parse_number(&cursor, &temperature) == 0 && *cursor++ == ',' && parse_number(&cursor, &humidity) == 0 && cursor == s_frame + s_length &&
        *cursor == '\0' && code >= 0 && code <= 99 && temperature >= -1000 && temperature <= 1000 && humidity >= 0 && humidity <= 100)
    {
        /* 天气码收窄为8位存入快照。 */
        s_weather.code = (uint8_t)code;
        /* 温度按原“乘10”定点值收窄为16位，单位仍为0.1摄氏度。 */
        s_weather.temperature_x10 = (int16_t)temperature;
        /* 湿度百分比收窄为8位。 */
        s_weather.humidity_rh = (uint8_t)humidity;
        /* 记录本帧最后字节的到达毫秒tick，供显示判断数据新鲜度。 */
        s_weather.received_ms = s_last_byte_ms;
        /* 标记快照来自手机实时帧。 */
        s_weather.valid = 1;
        /* 新帧已覆盖，清除“EEPROM恢复的旧天气”标记。 */
        s_weather.cached = 0;
    }
    /* 无论接受与否都清空在制帧长度，下一批字节重新拼帧。 */
    s_length = 0;
    /* 清溢出标记，避免影响下一帧。 */
    s_overflow = 0;
}

/* 无换行残帧靠200ms接收空闲收尾；有CR/LF时receive直接处理，同一帧不重复更新。 */
void app_weather_service(uint32_t now_ms)
{
    /* 有在制字节或已溢出时，计算距最近非结束符字节的毫秒差；达到200ms空闲即收尾本帧。 */
    if ((s_length || s_overflow) && (uint32_t)(now_ms - s_last_byte_ms) >= 200)
    {
        finish_frame();
    }
}

/* 拼接分包字节：CR/LF成形并解析，超过31字符置overflow；先补收尾再喂新字节。 */
void app_weather_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    /* i为本批数据的字节下标。 */
    uint16_t i;
    /* 无数据指针时直接返回，不改变在制帧。 */
    if (data == NULL)
    {
        return;
    }
    /* 先按空闲门限补收尾上一段残帧，本批字节只拼成后续帧。 */
    app_weather_service(now_ms);
    /* 逐字节处理本批数据：换行收尾，其他字节追加到帧缓冲。 */
    for (i = 0; i < size; ++i)
    {
        /* CR或LF表示一帧结束：立即校验并更新快照，随后由finish_frame清空在制帧。 */
        if (data[i] == '\r' || data[i] == '\n')
        {
            finish_frame();
        }
        else
        {
            /* 缓冲留出1字节结束符；未满时把当前字节写入并后移长度。 */
            if (s_length < sizeof(s_frame) - 1)
            {
                s_frame[s_length++] = (char)data[i];
            }
            else
            {
                /* 超出31字符上限：不再写入，只置溢出标记，收尾时整帧丢弃。 */
                s_overflow = 1;
            }
            /* 记录最近非结束符字节的到达时刻，作为200ms空闲收尾的计时基准。 */
            s_last_byte_ms = now_ms;
        }
    }
}

/* LVGL任务内整体复制当前天气；NULL不处理，不做跨任务同步。 */
void app_weather_get_snapshot(app_weather_struct *snapshot)
{
    /* 调用方给了输出指针才整体复制当前快照，否则不做任何动作。 */
    if (snapshot != NULL)
    {
        *snapshot = s_weather;
    }
}

void app_weather_restore(const app_weather_struct *snapshot)
{
    /* 仅在当前天气快照无效、且待恢复记录本身有效时执行恢复。 */
    if (s_weather.valid == 0 && snapshot->valid != 0)
    {
        /* 整体复制EEPROM恢复的天气字段。 */
        s_weather = *snapshot;
        /* 标记这份天气来自缓存，更新时间未知。 */
        s_weather.cached = 1;
        /* 旧tick属于上次开机，清零避免被当作本次运行的新鲜时间。 */
        s_weather.received_ms = 0;
    }
}

/* WMO天气码到显示字符串的映射；未知码返回"UNKNOWN"。 */
const char *app_weather_description(uint8_t code)
{
    /* WMO 0：晴。 */
    if (code == 0)
    {
        return "CLEAR";
    }
    /* WMO 1：大部晴朗。 */
    if (code == 1)
    {
        return "MAINLY CLEAR";
    }
    /* WMO 2/3：云量递增，其中3为阴。 */
    if (code <= 3)
    {
        if (code == 3)
        {
            return "OVERCAST";
        }
        else
        {
            return "PARTLY CLOUDY";
        }
    }
    /* WMO 45/48：雾与雾凇。 */
    if (code == 45 || code == 48)
    {
        return "FOG";
    }
    /* WMO 51..57：毛毛雨各强度。 */
    if (code >= 51 && code <= 57)
    {
        return "DRIZZLE";
    }
    /* WMO 61..67 连续雨与 80..82 阵雨，统一显示为雨。 */
    if ((code >= 61 && code <= 67) || (code >= 80 && code <= 82))
    {
        return "RAIN";
    }
    /* WMO 71..77 连续雪、85/86 阵雪，统一显示为雪。 */
    if ((code >= 71 && code <= 77) || code == 85 || code == 86)
    {
        return "SNOW";
    }
    /* WMO 95..99：雷暴(含冰雹强度)。 */
    if (code >= 95 && code <= 99)
    {
        return "THUNDERSTORM";
    }
    /* 未覆盖的码值：返回未知占位文本。 */
    return "UNKNOWN";
}
