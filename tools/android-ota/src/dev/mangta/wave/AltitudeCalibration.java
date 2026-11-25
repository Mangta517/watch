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
    private static final int GNSS_TIMEOUT_MS = 30000;
    private static final int MAX_FIX_AGE_MS = 8000;
    private static final String BUTTON_TEXT = "用手机定位校准手表海拔";

    private static final class MslResult
    {
        final double altitudeM;
        final float mslAccuracyM;
        final String errorMessage;

        MslResult(double altitudeM, float mslAccuracyM, String errorMessage)
        {
            this.altitudeM = altitudeM;
            this.mslAccuracyM = mslAccuracyM;
            this.errorMessage = errorMessage;
        }
    }

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
        boolean require_nmea = Build.VERSION.SDK_INT < 34;
        AltitudeFixSelector<Location> selector = new AltitudeFixSelector<>(require_nmea);
        class CalibrationListener implements LocationListener, OnNmeaMessageListener
        {
            private boolean stopped;
            private boolean nmea_registered;
            private boolean converge_scheduled;
            private String last_status = "正在搜星，请站定...";
            private final Runnable converge_check = () ->
            {
                converge_scheduled = false;
                if (stopped)
                {
                    return;
                }
                long now_ms = SystemClock.elapsedRealtime();
                AltitudeFixSelector.Selected<Location> ready = selector.onConvergeTimer(now_ms);
                if (ready != null)
                {
                    send_selected(ready);
                    return;
                }
                schedule_converge(now_ms);
            };
            private final Runnable timeout = () ->
            {
                if (stopped)
                {
                    return;
                }
                AltitudeFixSelector.Selected<Location> fallback = selector.onTimeout(SystemClock.elapsedRealtime());
                if (fallback != null)
                {
                    send_selected(fallback);
                    return;
                }
                if (stop())
                {
                    finish(activity, button, selector.timeoutMessage());
                }
            };

            private boolean stop()
            {
                if (stopped)
                {
                    return false;
                }
                stopped = true;
                converge_scheduled = false;
                handler.removeCallbacks(timeout);
                handler.removeCallbacks(converge_check);
                manager.removeUpdates(this);
                if (nmea_registered)
                {
                    nmea_registered = false;
                    manager.removeNmeaListener(this);
                }
                return true;
            }

            private void schedule_converge(long now_ms)
            {
                long remaining_ms = selector.remainingConvergeMs(now_ms);
                if (remaining_ms < 0L)
                {
                    if (converge_scheduled)
                    {
                        converge_scheduled = false;
                        handler.removeCallbacks(converge_check);
                    }
                    return;
                }
                if (!converge_scheduled)
                {
                    converge_scheduled = true;
                    handler.postDelayed(converge_check, remaining_ms);
                }
            }

            private void send_selected(AltitudeFixSelector.Selected<Location> selected)
            {
                if (!activity.hasWindowFocus())
                {
                    stop();
                    finish(activity, button, "校准已取消");
                    return;
                }
                stop();
                button.setText("正在发送海拔...");
                new Thread(() -> send(activity, button, link, selected), "watch-altitude").start();
            }

            private void update_status()
            {
                String status = selector.statusLabel();
                if (!status.equals(last_status))
                {
                    last_status = status;
                    button.setText(status);
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
                long now_ms = SystemClock.elapsedRealtime();
                AltitudeFixSelector.Sample<Location> sample = to_sample(location, now_ms);
                AltitudeFixSelector.Selected<Location> selected = selector.onLocation(sample, now_ms);
                if (selected != null)
                {
                    send_selected(selected);
                    return;
                }
                update_status();
                schedule_converge(now_ms);
            }

            @Override
            public void onNmeaMessage(String sentence, long timestamp)
            {
                if (stopped)
                {
                    return;
                }
                long now_ms = SystemClock.elapsedRealtime();
                AltitudeFixSelector.Selected<Location> selected = selector.onNmea(sentence, timestamp, now_ms);
                if (selected != null)
                {
                    send_selected(selected);
                    return;
                }
                update_status();
                schedule_converge(now_ms);
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
                if (require_nmea)
                {
                    nmea_registered = manager.addNmeaListener(this, handler);
                    if (!nmea_registered)
                    {
                        finish(activity, button, "手机未提供 GNSS 海拔数据，可在手表手动校准");
                        return;
                    }
                }
                else
                {
                    try
                    {
                        nmea_registered = manager.addNmeaListener(this, handler);
                    }
                    catch (RuntimeException ignored)
                    {
                        nmea_registered = false;
                    }
                }
                manager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000, 0, this, handler.getLooper());
                handler.postDelayed(timeout, GNSS_TIMEOUT_MS);
            }
        }
        button.setEnabled(false);
        button.setText("正在搜星，请站定...");
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

    private static AltitudeFixSelector.Sample<Location> to_sample(Location location, long now_ms)
    {
        boolean has_vertical = location.hasVerticalAccuracy();
        float vertical_m = has_vertical ? location.getVerticalAccuracyMeters() : Float.NaN;
        boolean has_speed = location.hasSpeed();
        float speed_mps = has_speed ? location.getSpeed() : Float.NaN;
        return new AltitudeFixSelector.Sample<>(
            location,
            location.getElapsedRealtimeNanos() / 1000000L,
            now_ms,
            location.getTime(),
            location.isFromMockProvider(),
            location.hasAltitude(),
            location.hasAccuracy(),
            location.hasAccuracy() ? location.getAccuracy() : Float.NaN,
            has_vertical,
            vertical_m,
            has_speed,
            speed_mps
        );
    }

    private static long fix_age_ms(Location location)
    {
        return (SystemClock.elapsedRealtimeNanos() - location.getElapsedRealtimeNanos()) / 1000000L;
    }

    private static void send(Activity activity, Button button, BluetoothGatt link,
        AltitudeFixSelector.Selected<Location> selected)
    {
        Location location = selected.sample.payload;
        double altitude_m = selected.nmeaAltitudeM;
        float msl_accuracy_m = Float.NaN;
        String result;
        try
        {
            String msl_error = null;
            if (Build.VERSION.SDK_INT >= 34)
            {
                MslResult msl = mean_sea_level_altitude(activity, location, selected.nmeaAltitudeM);
                altitude_m = msl.altitudeM;
                msl_accuracy_m = msl.mslAccuracyM;
                msl_error = msl.errorMessage;
            }
            long age_ms = fix_age_ms(location);
            if (msl_error != null)
            {
                result = msl_error;
            }
            else if (age_ms < AltitudeFixSelector.MIN_LIVE_FIX_AGE_MS || age_ms > MAX_FIX_AGE_MS ||
                !(altitude_m >= -1000 && altitude_m <= 9000))
            {
                result = "未取得精度合格的新海拔，请站定后重试";
            }
            else
            {
                long rounded_m = Math.round(altitude_m);
                OtaBridge.sendCalibration(activity, link, (int)rounded_m);
                result = AltitudeFixSelector.formatSuccess(rounded_m, selected.sample, msl_accuracy_m);
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
    private static MslResult mean_sea_level_altitude(Activity activity, Location location, double fallback_nmea_m)
    {
        try
        {
            if (!location.hasMslAltitude() || !location.hasMslAltitudeAccuracy())
            {
                new AltitudeConverter().addMslAltitudeToLocation(activity, location);
            }
            if (location.hasMslAltitude())
            {
                if (location.hasMslAltitudeAccuracy())
                {
                    float accuracy_m = location.getMslAltitudeAccuracyMeters();
                    if (Float.isFinite(accuracy_m) && accuracy_m <= AltitudeFixSelector.MAX_MSL_ACCURACY_M)
                    {
                        return new MslResult(location.getMslAltitudeMeters(), accuracy_m, null);
                    }
                    return new MslResult(Double.NaN, accuracy_m,
                        "海平面海拔换算精度不足(垂直±" + Math.round(accuracy_m) + "m)，请到开阔处重试");
                }
                return new MslResult(location.getMslAltitudeMeters(), Float.NaN, null);
            }
        }
        catch (Exception ignored)
        {
            // 部分精简ROM缺少离线大地水准面表时回退到已配对的NMEA GGA正高。
        }
        if (!Double.isNaN(fallback_nmea_m))
        {
            return new MslResult(fallback_nmea_m, Float.NaN, null);
        }
        return new MslResult(Double.NaN, Float.NaN, "系统未能换算海平面海拔，可在手表手动校准");
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
