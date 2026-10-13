#include "app_weather.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void feed(const char *text, uint32_t now_ms)
{
    app_weather_receive((const uint8_t *)text, (uint16_t)strlen(text), now_ms);
}

int main(void)
{
    const char *frame = "WEATHER,3,-53,81\r\n";
    const char *invalid[] = {"WEATHER,100,263,80\n",   "WEATHER,3,1001,80\n", "WEATHER,3,263,101\n",
                             "WEATHER,3,263,80junk\n", "WEATHER,3,,80\n",     "WEATHER,9999999999999999999999999999999999999,0,0\n"};
    app_weather_struct weather;
    unsigned split, i;
    app_weather_get_snapshot(&weather);
    assert(!weather.valid);
    for (split = 0; split <= strlen(frame); ++split)
    {
        app_weather_receive((const uint8_t *)frame, (uint16_t)split, 100);
        app_weather_receive((const uint8_t *)frame + split, (uint16_t)(strlen(frame) - split), 150);
        app_weather_get_snapshot(&weather);
        assert(weather.valid && weather.code == 3 && weather.temperature_x10 == -53 && weather.humidity_rh == 81);
    }
    for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        feed(invalid[i], 200);
        app_weather_get_snapshot(&weather);
        assert(weather.code == 3 && weather.temperature_x10 == -53 && weather.humidity_rh == 81);
    }
    {
        static const uint8_t embedded_nul[] = "WEATHER,0,300,70\0junk\n";
        app_weather_receive(embedded_nul, sizeof(embedded_nul) - 1, 250);
        app_weather_get_snapshot(&weather);
        assert(weather.code == 3 && weather.temperature_x10 == -53);
    }
    feed("09:05\nhello\nWEATHER,0,263,70\n", 300);
    app_weather_get_snapshot(&weather);
    assert(weather.code == 0 && weather.temperature_x10 == 263 && weather.humidity_rh == 70);
    feed("WEATHER,99,-1000,100", UINT32_MAX - 100U);
    app_weather_service(98);
    app_weather_get_snapshot(&weather);
    assert(weather.code == 0);
    app_weather_service(99);
    app_weather_get_snapshot(&weather);
    assert(weather.code == 99 && weather.temperature_x10 == -1000 && weather.humidity_rh == 100);
    assert(strcmp(app_weather_description(0), "CLEAR") == 0);
    assert(strcmp(app_weather_description(3), "OVERCAST") == 0);
    assert(strcmp(app_weather_description(61), "RAIN") == 0);
    assert(strcmp(app_weather_description(95), "THUNDERSTORM") == 0);
    app_weather_receive(NULL, 1, 0);
    app_weather_get_snapshot(NULL);
    puts("PASS: weather fragmentation, ranges, invalid-frame preservation, clock/noise coexistence, idle/tick wrap and WMO labels.");
    return 0;
}
