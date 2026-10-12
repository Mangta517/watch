#include "app_clock.h"
#include "watch_rtc.h"
#include <stddef.h>

typedef struct
{
    uint8_t frame[8];
    uint8_t length;
    uint8_t overflow;
    uint8_t has_colon;
    uint32_t last_byte_ms;
    app_clock_result result;
} app_clock_parser_struct;

static app_clock_parser_struct parser;

static void app_clock_finish(void)
{
    uint8_t i, part = 0, digits = 0, value[2] = {0};
    uint8_t valid = !parser.overflow && parser.length >= 3 && parser.length <= 5;
    if (parser.has_colon)
    {
        for (i = 0; valid && i < parser.length; ++i)
        {
            if (parser.frame[i] == ':' && part == 0 && digits != 0)
            {
                part = 1;
                digits = 0;
            }
            else if (parser.frame[i] >= '0' && parser.frame[i] <= '9' && digits < 2)
            {
                value[part] = value[part] * 10 + parser.frame[i] - '0';
                ++digits;
            }
            else
            {
                valid = 0;
            }
        }
        valid = valid && part == 1 && digits != 0 && value[0] <= 23 && value[1] <= 59;
        if (!valid)
        {
            parser.result = APP_CLOCK_INVALID;
        }
        else
        {
            parser.result = watch_rtc_set_time(value[0], value[1]) == HAL_OK ? APP_CLOCK_SYNCED : APP_CLOCK_RTC_ERROR;
        }
    }
    parser.length = 0;
    parser.overflow = 0;
    parser.has_colon = 0;
}

void app_clock_service(uint32_t now_ms)
{
    if ((parser.length || parser.overflow) && (uint32_t)(now_ms - parser.last_byte_ms) >= APP_CLOCK_FRAME_IDLE_MS)
    {
        app_clock_finish();
    }
}

void app_clock_receive(const uint8_t *data, uint16_t size, uint32_t now_ms)
{
    uint16_t i;
    if (data == NULL)
    {
        return;
    }
    app_clock_service(now_ms);
    for (i = 0; i < size; ++i)
    {
        if (data[i] == '\r' || data[i] == '\n')
        {
            app_clock_finish();
        }
        else
        {
            if (parser.length < sizeof(parser.frame))
            {
                parser.frame[parser.length++] = data[i];
            }
            else
            {
                parser.overflow = 1;
            }
            if (data[i] == ':')
            {
                parser.has_colon = 1;
            }
            parser.last_byte_ms = now_ms;
        }
    }
}

app_clock_result app_clock_get_result(void)
{
    return parser.result;
}
