package dev.mangta.wave;

import android.app.Activity;
import android.content.Intent;
import android.widget.Button;
import android.widget.LinearLayout;
import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;

/** 复用旧App的BLE发送入口，升级数据由独立页面独占SPP连接。 */
public final class OtaBridge
{
    private static WeakReference<Activity> owner = new WeakReference<>(null);

    private OtaBridge()
    {
    }

    public static void install(Activity activity, LinearLayout container)
    {
        owner = new WeakReference<>(activity);
        Button button = new Button(activity);
        button.setText("手表固件升级 · OTA");
        button.setOnClickListener(view -> activity.startActivity(new Intent(activity, OtaActivity.class)));
        container.addView(button, 1);
    }

    private static Object field(Object object, String name) throws Exception
    {
        Field item = object.getClass().getDeclaredField(name);
        item.setAccessible(true);
        return item.get(object);
    }

    public static void requestUpgrade() throws Exception
    {
        Activity activity = owner.get();
        if (activity == null)
        {
            throw new IllegalStateException("请返回主界面并连接手表BLE，也可按住KEY_1复位进入升级");
        }
        Object master = field(activity, "master");
        if (master == null || field(master, "deviceConnected") == null || field(master, "tx_characteristic") == null)
        {
            throw new IllegalStateException("请先在原主界面连接手表BLE并选择FFF1发送特征");
        }
        master.getClass().getMethod("sendWatchSync", byte[].class)
            .invoke(master, (Object)"OTA,START\n".getBytes(StandardCharsets.US_ASCII));
    }
}
