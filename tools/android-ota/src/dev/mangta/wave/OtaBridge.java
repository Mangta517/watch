package dev.mangta.wave;

import android.app.Activity;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.content.Intent;
import android.os.Handler;
import android.os.Looper;
import android.widget.Button;
import android.widget.LinearLayout;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/** 复用旧App的BLE发送入口；OTA页先释放旧GATT，再独占FFF0/FFF1升级。 */
public final class OtaBridge
{
    private static final int OLD_STATE_CONNECTED = 3;
    private static final int RELEASE_WAIT_MS = 5000;
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

    private static void setField(Object object, String name, Object value) throws Exception
    {
        Field item = object.getClass().getDeclaredField(name);
        item.setAccessible(true);
        item.set(object, value);
    }

    private static Object optionalField(Object object, String name)
    {
        try
        {
            return field(object, name);
        }
        catch (NoSuchFieldException error)
        {
            return null;
        }
        catch (Exception error)
        {
            return null;
        }
    }

    private static java.lang.reflect.Method method(Object object, String name, Class<?>... types) throws Exception
    {
        java.lang.reflect.Method item = object.getClass().getMethod(name, types);
        try
        {
            item.setAccessible(true);
        }
        catch (RuntimeException error)
        {
            // 公共方法无需强制放开访问。
        }
        return item;
    }

    private static int state(Object master)
    {
        Object value = optionalField(master, "state");
        if (value instanceof Number)
        {
            return ((Number)value).intValue();
        }
        return -1;
    }

    /** 原WatchSync的天气令牌随连接失效；只调用reset_connection，不detach常驻监听。 */
    private static void resetWatchSync()
    {
        Runnable reset = () ->
        {
            try
            {
                Class<?> type = Class.forName("dev.mangta.wave.WatchSync");
                type.getMethod("reset_connection").invoke(null);
            }
            catch (Throwable error)
            {
                // 旧类或方法缺失时不影响OTA释放流程。
            }
        };
        if (Looper.myLooper() == Looper.getMainLooper())
        {
            reset.run();
            return;
        }
        CountDownLatch done = new CountDownLatch(1);
        new Handler(Looper.getMainLooper()).post(() ->
        {
            reset.run();
            done.countDown();
        });
        try
        {
            done.await(2, TimeUnit.SECONDS);
        }
        catch (InterruptedException error)
        {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * 停旧扫描并断开旧GATT，等状态回0后关闭残留句柄；之后OTA才能建立自己的连接。
     * 不重连旧master，用户升级失败返回原界面后需手动重连。
     */
    public static void releaseConnection() throws Exception
    {
        resetWatchSync();
        Activity activity = owner.get();
        if (activity == null)
        {
            return;
        }
        Object master = optionalField(activity, "master");
        if (master == null)
        {
            return;
        }
        try
        {
            method(master, "stopScan").invoke(master);
        }
        catch (NoSuchMethodException error)
        {
            // 旧版本没有stopScan时仍继续断开。
        }
        try
        {
            method(master, "disconnect").invoke(master);
        }
        catch (NoSuchMethodException error)
        {
            // 没有disconnect时只能关闭保留的GATT。
        }
        // 与releaseConnection一致：状态未知(-1)时不能无限等待，只清理句柄。
        if (state(master) >= 0)
        {
            long deadline = System.nanoTime() + RELEASE_WAIT_MS * 1000000L;
            while (state(master) != 0 && System.nanoTime() < deadline)
            {
                try
                {
                    Thread.sleep(100);
                }
                catch (InterruptedException error)
                {
                    Thread.currentThread().interrupt();
                    throw new IOException("等待原BLE连接释放被中断", error);
                }
            }
        }
        Object retained = optionalField(master, "gatt");
        if (retained instanceof BluetoothGatt)
        {
            BluetoothGatt gatt = (BluetoothGatt)retained;
            try
            {
                gatt.disconnect();
            }
            catch (RuntimeException error)
            {
                // 系统可能已断开，继续close本地句柄。
            }
            try
            {
                gatt.close();
            }
            catch (RuntimeException error)
            {
                // close失败不重试，避免阻塞升级线程。
            }
        }
        resetWatchSync();
        if (state(master) > 0)
        {
            throw new IOException("原BLE连接未能在5秒内释放，请返回原界面手动断开后重试");
        }
    }

    /**
     * 释放旧连接前记录当前已连接的手表；state!=3或旧master不存在时返回null。
     * 这里只读字段，不向旧连接发送任何数据。
     */
    public static BluetoothDevice connectedDevice() throws Exception
    {
        Activity activity = owner.get();
        if (activity == null)
        {
            return null;
        }
        Object master = optionalField(activity, "master");
        if (master == null)
        {
            return null;
        }
        if (state(master) != OLD_STATE_CONNECTED)
        {
            return null;
        }
        Object device = optionalField(master, "deviceConnected");
        if (device instanceof BluetoothDevice)
        {
            return (BluetoothDevice)device;
        }
        return null;
    }

    /** 恢复失败时停掉重连并关闭本次恢复建立的GATT，避免旧连接残留在后台。 */
    private static void abortRestore()
    {
        try
        {
            releaseConnection();
        }
        catch (Exception error)
        {
            // 释放失败不覆盖取消/超时的原始原因。
        }
    }

    /** 取消查询接口；minSdk21不能使用API24才有的java.util.function.BooleanSupplier。 */
    public interface CancelCheck
    {
        boolean isCancelled();
    }

    /**
     * 用旧master重新连接手表，轮询到state==3且rx/tx特征已按MAC加载完成。
     * 只能在后台线程调用；取消或超时会断开本次重连，不留下后台连接。
     */
    public static void restoreConnection(BluetoothDevice device, CancelCheck cancelled, int timeoutMs) throws Exception
    {
        if (Looper.myLooper() == Looper.getMainLooper())
        {
            throw new IllegalStateException("恢复旧BLE连接必须在后台线程执行");
        }
        Activity activity = owner.get();
        if (activity == null)
        {
            throw new IllegalStateException("原界面已不存在，无法恢复旧BLE连接");
        }
        Object master = optionalField(activity, "master");
        if (master == null)
        {
            throw new IllegalStateException("原BLE连接对象已不存在，无法恢复旧BLE连接");
        }
        if (device == null)
        {
            throw new IllegalArgumentException("没有可恢复的手表BLE设备");
        }
        if (cancelled.isCancelled())
        {
            throw new IOException("已取消，未恢复旧BLE连接");
        }
        resetWatchSync();
        setField(master, "toShowUUIDDialog", false);
        method(master, "connect", BluetoothDevice.class).invoke(master, device);
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        while (true)
        {
            if (cancelled.isCancelled())
            {
                abortRestore();
                throw new IOException("已取消，旧BLE连接恢复中止");
            }
            if (state(master) == OLD_STATE_CONNECTED &&
                optionalField(master, "rx_characteristic") != null &&
                optionalField(master, "tx_characteristic") != null &&
                sameDevice(optionalField(master, "deviceConnected"), device))
            {
                return;
            }
            if (System.nanoTime() >= deadline)
            {
                abortRestore();
                throw new IOException("恢复旧BLE连接超时（" + timeoutMs + "ms），请返回原界面手动连接");
            }
            try
            {
                Thread.sleep(100);
            }
            catch (InterruptedException error)
            {
                abortRestore();
                Thread.currentThread().interrupt();
                throw new IOException("等待旧BLE连接恢复被中断", error);
            }
        }
    }

    private static boolean sameDevice(Object connected, BluetoothDevice device)
    {
        if (!(connected instanceof BluetoothDevice))
        {
            return false;
        }
        BluetoothDevice current = (BluetoothDevice)connected;
        if (current.getAddress() == null || device.getAddress() == null)
        {
            return false;
        }
        return current.getAddress().equals(device.getAddress());
    }

    /** 仅在原BLE已连接(state==3)时发送OTA,START；发送可能阻塞，不能占用UI线程。 */
    public static void requestUpgrade() throws Exception
    {
        Activity activity = owner.get();
        if (activity == null)
        {
            throw new IllegalStateException("请返回主界面并连接手表BLE，也可按住KEY_1复位进入升级");
        }
        Object master = optionalField(activity, "master");
        if (master == null || optionalField(master, "deviceConnected") == null ||
            optionalField(master, "tx_characteristic") == null)
        {
            throw new IllegalStateException("请先在原主界面连接手表BLE并选择FFF1发送特征");
        }
        int current = state(master);
        if (current != OLD_STATE_CONNECTED)
        {
            throw new IllegalStateException("原BLE未连接完成（state=" + current + "），请等待连接成功后再请求升级");
        }
        byte[] command = "OTA,START\n".getBytes(StandardCharsets.US_ASCII);
        if (Looper.myLooper() == Looper.getMainLooper())
        {
            // sendWatchSync可能等待BLE回调，必须在升级页的后台线程调用。
            throw new IllegalStateException("OTA,START必须在后台线程发送");
        }
        try
        {
            method(master, "sendWatchSync", byte[].class).invoke(master, (Object)command);
        }
        catch (java.lang.reflect.InvocationTargetException error)
        {
            Throwable cause = error.getCause();
            if (cause == null)
            {
                cause = error;
            }
            throw new IOException("发送OTA,START失败：" + cause.getMessage(), cause);
        }
    }
}
