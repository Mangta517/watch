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
    s_length = 0;
    s_overflow = 0;
}

static uint8_t parse_number(const char **cursor, int32_t *value)
{
    int32_t magnitude = 0;
    uint8_t negative = **cursor == '-';
    uint8_t digits = 0;
    if (negative)
    {
        ++*cursor;
    }
    while (**cursor >= '0' && **cursor <= '9')
    {
        if (++digits > 4)
        {
            return 1;
        }
        magnitude = magnitude * 10 + **cursor - '0';
        ++*cursor;
    }
    if (negative)
    {
        *value = -magnitude;
    }
    else
    {
        *value = magnitude;
    }
    return digits == 0;
}

static void finish_frame(void)
{
    const char *cursor = s_frame + 8;
    int32_t code, temperature, humidity;
    s_frame[s_length] = '\0';
    if (!s_overflow && s_length >= 8 && memcmp(s_frame, "WEATHER,", 8) == 0 && parse_number(&cursor, &code) == 0 && *cursor++ == ',' &&
        parse_number(&cursor, &temperature) == 0 && *cursor++ == ',' && parse_number(&cursor, &humidity) == 0 && cursor == s_frame + s_length &&
        *cursor == '\0' && code >= 0 && code <= 99 && temperature >= -1000 && temperature <= 1000 && humidity >= 0 && humidity <= 100)
    {
        s_weather.code = (uint8_t)code;
        s_weather.temperature_x10 = (int16_t)temperature;
        s_weather.humidity_rh = (uint8_t)humidity;
        s_weather.received_ms = s_last_byte_ms;
        s_weather.valid = 1;
    }
    s_length = 0;
    s_overflow = 0;
}

void app_weather_service(uint32_t now_ms)
{
    if ((s_length || s_overflow) && (uint32_t)(now_ms - s_last_byte_ms) >= 200)
    {
        finish_frame();
    }
}

void app_weather_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    uint16_t i;
    if (data == NULL)
    {
        return;
    }
    app_weather_service(now_ms);
    for (i = 0; i < size; ++i)
    {
        if (data[i] == '\r' || data[i] == '\n')
        {
            finish_frame();
        }
        else
        {
            if (s_length < sizeof(s_frame) - 1)
            {
                s_frame[s_length++] = (char)data[i];
            }
            else
            {
                s_overflow = 1;
            }
            s_last_byte_ms = now_ms;
        }
    }
}

void app_weather_get_snapshot(app_weather_struct *snapshot)
{
    if (snapshot != NULL)
    {
        *snapshot = s_weather;
    }
}

const char *app_weather_description(uint8_t code)
{
    if (code == 0)
    {
        return "CLEAR";
    }
    if (code == 1)
    {
        return "MAINLY CLEAR";
    }
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
    if (code == 45 || code == 48)
    {
        return "FOG";
    }
    if (code >= 51 && code <= 57)
    {
        return "DRIZZLE";
    }
    if ((code >= 61 && code <= 67) || (code >= 80 && code <= 82))
    {
        return "RAIN";
    }
    if ((code >= 71 && code <= 77) || code == 85 || code == 86)
    {
        return "SNOW";
    }
    if (code >= 95 && code <= 99)
    {
        return "THUNDERSTORM";
    }
    return "UNKNOWN";
}
