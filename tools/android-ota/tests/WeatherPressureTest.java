package dev.mangta.wave;

public final class WeatherPressureTest
{
    public static void main(String[] args)
    {
        String weather = "WEATHER,3,263,80\n";
        check(WeatherPressure.append_reference(weather, 1021.25).equals(weather + "BARO,REF,102125\n"));
        check(WeatherPressure.append_reference(weather, 1013.254).equals(weather + "BARO,REF,101325\n"));
        for (double value : new double[] {Double.NaN, Double.POSITIVE_INFINITY, 799.99, 1200.01})
        {
            check(WeatherPressure.append_reference(weather, value).equals(weather));
        }
        Object peer = new Object();
        Object other = new Object();
        check(WeatherPressure.refresh_token(peer, true, 7, 100) == 7);
        check(WeatherPressure.refresh_token(peer, true, 0, 900099) == 0);
        check(WeatherPressure.refresh_token(peer, true, 0, 900100) == 7);
        check(WeatherPressure.refresh_token(peer, true, 0, 900101) == 0);
        check(WeatherPressure.refresh_token(other, true, 0, 1800100) == 0);
        check(WeatherPressure.refresh_token(peer, false, 0, 1800100) == 0);
        check(WeatherPressure.refresh_token(peer, true, 0, 2700100) == 0);
        check(WeatherPressure.refresh_token(peer, true, 8, 2700100) == 8);
        check(WeatherPressure.refresh_token(peer, true, 0, 3600100) == 8);
        System.out.println("PASS: pressure units, legacy weather fallback and 15-minute refresh across disconnect/reconnect.");
    }

    private static void check(boolean condition)
    {
        if (!condition)
        {
            throw new AssertionError();
        }
    }
}
