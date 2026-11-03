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
        text.setText("蓝牙调试器 · 波形 OTA\n\n请允许附近设备权限，以连接手表BLE和升级SPP。");
        layout.addView(text);
        Button retry = new Button(this);
        retry.setText("允许附近设备并打开App");
        retry.setOnClickListener(view -> open());
        layout.addView(retry);
        setContentView(layout);
        open();
    }

    private void open()
    {
        if (Build.VERSION.SDK_INT >= 31 &&
            (checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED ||
             checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED))
        {
            requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN}, 102);
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
        if (code == 102 && results.length == 2 && results[0] == PackageManager.PERMISSION_GRANTED &&
            results[1] == PackageManager.PERMISSION_GRANTED)
        {
            open();
        }
    }
}
