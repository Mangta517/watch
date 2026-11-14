package dev.mangta.wave;

/** 天气同步附加海平面气压，连接会话仍由原WatchSyncSession校验。 */
public final class WeatherPressure
{
    private static final long REFRESH_PERIOD_MS = 15 * 60 * 1000;
    private static Object last_peer;
    private static int last_token;
    private static long last_refresh_ms;

    private WeatherPressure()
    {
    }

    /** Open-Meteo的pressure_msl单位为hPa，手表帧使用整数Pa。 */
    public static String append_reference(String weather_frame, double sea_level_hpa)
    {
        if (Double.isNaN(sea_level_hpa) || Double.isInfinite(sea_level_hpa) || sea_level_hpa < 800 || sea_level_hpa > 1200)
        {
            return weather_frame;
        }
        return weather_frame + "BARO,REF," + Math.round(sea_level_hpa * 100) + "\n";
    }

    /** 首次连接立即同步，保持连接每15分钟刷新；断连后清除旧会话的定时状态。 */
    public static synchronized int refresh_token(Object peer, boolean ready, int token, long now_ms)
    {
        if (!ready)
        {
            last_peer = null;
            last_token = 0;
            return 0;
        }
        if (token != 0)
        {
            last_peer = peer;
            last_token = token;
            last_refresh_ms = now_ms;
            return token;
        }
        if (peer == last_peer && last_token != 0 && now_ms - last_refresh_ms >= REFRESH_PERIOD_MS)
        {
            last_refresh_ms = now_ms;
            return last_token;
        }
        return 0;
    }
}
