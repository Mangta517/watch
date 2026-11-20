package dev.mangta.wave;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothGatt;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.OnNmeaMessageListener;
import android.location.altitude.AltitudeConverter;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.Toast;

/** 用户发起时短时定位，将海平面海拔交给手表与新气压配对。 */
final class AltitudeCalibration
{
    private static final int GNSS_TIMEOUT_MS = 20000;
    private static final int MAX_FIX_AGE_MS = 8000;
    private static final String BUTTON_TEXT = "用手机定位校准手表海拔";

    private AltitudeCalibration()
    {
    }

    static void install(Activity activity, LinearLayout container)
    {
        if (Build.VERSION.SDK_INT < 26)
        {
            return;
        }
        Button button = new Button(activity);
        button.setText(BUTTON_TEXT);
        button.setOnClickListener(view -> start(activity, button));
        container.addView(button, 2);
    }

    private static void start(Activity activity, Button button)
    {
        BluetoothGatt link = OtaBridge.calibrationLink(activity);
        if (link == null)
        {
            message(activity, "请先连接手表 BLE 并保持亮屏");
            return;
        }
        if (activity.checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED)
        {
            message(activity, "允许精确位置后再点击校准");
            activity.requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION}, 16411);
            return;
        }

        LocationManager manager = (LocationManager)activity.getSystemService(Context.LOCATION_SERVICE);
        if (manager == null || !manager.isProviderEnabled(LocationManager.GPS_PROVIDER))
        {
            message(activity, "请打开系统定位并在室外站定");
            return;
        }

        Handler handler = new Handler(Looper.getMainLooper());
        boolean use_nmea = Build.VERSION.SDK_INT < 34;
        class CalibrationListener implements LocationListener, OnNmeaMessageListener
        {
            private boolean stopped;
            private Location fix;
            private long gga_time_ms;
            private double gga_altitude_m = Double.NaN;
            private final Runnable timeout = () ->
            {
                if (stop())
                {
                    finish(activity, button, "定位精度不足，请到开阔处站定后重试");
                }
            };

            private boolean stop()
            {
                if (stopped)
                {
                    return false;
                }
                stopped = true;
                handler.removeCallbacks(timeout);
                manager.removeUpdates(this);
                if (use_nmea)
                {
                    manager.removeNmeaListener(this);
                }
                return true;
            }

            private void send_fix(Location location, double nmea_altitude_m)
            {
                if (!activity.hasWindowFocus())
                {
                    stop();
                    finish(activity, button, "校准已取消");
                    return;
                }
                stop();
                new Thread(() -> send(activity, button, link, location, nmea_altitude_m), "watch-altitude").start();
            }

            private void pair_gga()
            {
                if (fix != null && fix_age_ms(fix) <= 2000 && fix_age_ms(fix) >= 0 &&
                    Math.abs(fix.getTime() - gga_time_ms) <= 1500 && !Double.isNaN(gga_altitude_m))
                {
                    send_fix(fix, gga_altitude_m);
                }
            }

            @Override
            public void onLocationChanged(Location location)
            {
                if (stopped)
                {
                    return;
                }
                if (!activity.hasWindowFocus())
                {
                    stop();
                    finish(activity, button, "校准已取消");
                    return;
                }
                long age_ms = fix_age_ms(location);
                if (age_ms < 0 || age_ms > 2000 || location.isFromMockProvider() || !location.hasAltitude() || !location.hasAccuracy() ||
                    !location.hasVerticalAccuracy() || !(location.getAccuracy() <= 15.0f) ||
                    !(location.getVerticalAccuracyMeters() <= 12.0f) ||
                    !location.hasSpeed() || !(location.getSpeed() <= 1.0f))
                {
                    fix = null;
                    return;
                }

                if (use_nmea)
                {
                    fix = location;
                    pair_gga();
                }
                else
                {
                    send_fix(location, Double.NaN);
                }
            }

            @Override
            public void onNmeaMessage(String sentence, long timestamp)
            {
                if (stopped)
                {
                    return;
                }
                double altitude_m = GgaAltitude.parse(sentence);
                if (!Double.isNaN(altitude_m))
                {
                    gga_altitude_m = altitude_m;
                    gga_time_ms = timestamp;
                    pair_gga();
                }
            }

            @Override
            public void onStatusChanged(String provider, int status, Bundle extras)
            {
            }

            @Override
            public void onProviderEnabled(String provider)
            {
            }

            @Override
            public void onProviderDisabled(String provider)
            {
                if (stop())
                {
                    finish(activity, button, "系统定位已关闭");
                }
            }

            void begin()
            {
                if (use_nmea && !manager.addNmeaListener(this, handler))
                {
                    finish(activity, button, "手机未提供 GNSS 海拔数据，可在手表手动校准");
                    return;
                }
                manager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000, 0, this, handler.getLooper());
                handler.postDelayed(timeout, GNSS_TIMEOUT_MS);
            }
        }
        button.setEnabled(false);
        button.setText("正在获取海拔，请站定");
        CalibrationListener listener = new CalibrationListener();
        try
        {
            listener.begin();
        }
        catch (SecurityException | IllegalArgumentException error)
        {
            listener.stop();
            finish(activity, button, "无法定位，请检查系统定位与精确位置权限");
        }
    }

    private static long fix_age_ms(Location location)
    {
        return (SystemClock.elapsedRealtimeNanos() - location.getElapsedRealtimeNanos()) / 1000000L;
    }

    private static void send(Activity activity, Button button, BluetoothGatt link, Location location, double altitude_m)
    {
        String result;
        try
        {
            if (Build.VERSION.SDK_INT >= 34)
            {
                altitude_m = mean_sea_level_altitude(activity, location);
            }
            long age_ms = fix_age_ms(location);
            if (age_ms < 0 || age_ms > MAX_FIX_AGE_MS || !(altitude_m >= -1000 && altitude_m <= 9000))
            {
                result = "未取得精度合格的新海拔，请站定后重试";
            }
            else
            {
                long rounded_m = Math.round(altitude_m);
                OtaBridge.sendCalibration(activity, link, (int)rounded_m);
                result = "已发送 " + rounded_m + " m，请在手表气压页确认";
            }
        }
        catch (Exception error)
        {
            result = "校准发送未完成，请检查定位和手表 BLE 后重试";
        }
        String text = result;
        activity.runOnUiThread(() -> finish(activity, button, text));
    }

    @android.annotation.TargetApi(34)
    private static double mean_sea_level_altitude(Activity activity, Location location) throws java.io.IOException
    {
        if (!location.hasMslAltitude() || !location.hasMslAltitudeAccuracy())
        {
            new AltitudeConverter().addMslAltitudeToLocation(activity, location);
        }
        if (location.hasMslAltitude() && location.hasMslAltitudeAccuracy() && location.getMslAltitudeAccuracyMeters() <= 12.0f)
        {
            return location.getMslAltitudeMeters();
        }
        return Double.NaN;
    }

    private static void finish(Activity activity, Button button, String text)
    {
        button.setEnabled(true);
        button.setText(BUTTON_TEXT);
        message(activity, text);
    }

    private static void message(Activity activity, String text)
    {
        Toast.makeText(activity, text, Toast.LENGTH_LONG).show();
    }
}
