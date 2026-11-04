package dev.mangta.wave;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

/** 旧蓝牙管理线程创建前完成Android 12+附近设备授权。 */
public final class OtaLaunchActivity extends Activity
{
    @Override
    public void onCreate(Bundle saved)
    {
        super.onCreate(saved);
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        layout.setPadding(40, 60, 40, 40);
        TextView text = new TextView(this);
        text.setText("蓝牙调试器 · 波形 OTA\n\n请允许BLE扫描和连接权限（Android 6–11为位置权限），"
            + "用于BLE日常连接和SPP升级；首次升级时系统可能再弹出蓝牙配对确认。");
        layout.addView(text);
        Button retry = new Button(this);
        retry.setText("允许附近设备并打开App");
        retry.setOnClickListener(view -> open());
        layout.addView(retry);
        setContentView(layout);
        open();
    }

    private boolean granted(String permission)
    {
        return checkSelfPermission(permission) == PackageManager.PERMISSION_GRANTED;
    }

    private void open()
    {
        if (Build.VERSION.SDK_INT >= 31)
        {
            if (!granted(Manifest.permission.BLUETOOTH_CONNECT) || !granted(Manifest.permission.BLUETOOTH_SCAN))
            {
                requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN}, 102);
                return;
            }
        }
        else if (Build.VERSION.SDK_INT >= 23 && !granted(Manifest.permission.ACCESS_FINE_LOCATION))
        {
            // Android 6–11的BLE扫描受位置权限和系统位置服务约束。
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 102);
            return;
        }
        Intent intent = new Intent();
        intent.setClassName(this, "xnj.lazydog.btcontroller.SplashActivity");
        startActivity(intent);
        finish();
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] permissions, int[] results)
    {
        super.onRequestPermissionsResult(code, permissions, results);
        if (code != 102 || results.length == 0)
        {
            return;
        }
        for (int result : results)
        {
            if (result != PackageManager.PERMISSION_GRANTED)
            {
                return;
            }
        }
        open();
    }
}
