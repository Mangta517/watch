package dev.mangta.wave;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.database.Cursor;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.provider.OpenableColumns;
import android.provider.Settings;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public final class OtaActivity extends Activity
{
    private static final UUID SPP = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
    private final Handler ui = new Handler(Looper.getMainLooper());
    private TextView status;
    private ProgressBar progress;
    private Button choose, request, transfer, settings;
    private byte[] image;
    private volatile BluetoothSocket socket;
    private volatile boolean running, cancelled;
    private volatile long deadline;

    private final Runnable watchdog = new Runnable()
    {
        @Override
        public void run()
        {
            if (running && deadline != 0 && SystemClock.elapsedRealtime() >= deadline)
            {
                stopTransfer("通信超时，已停止；重新进入升级模式后完整重传");
            }
            ui.postDelayed(this, 500);
        }
    };

    private Button button(LinearLayout layout, String text, android.view.View.OnClickListener action)
    {
        Button item = new Button(this);
        item.setText(text);
        item.setOnClickListener(action);
        layout.addView(item);
        return item;
    }

    @Override
    public void onCreate(Bundle saved)
    {
        super.onCreate(saved);
        setTitle("F411 手表蓝牙 OTA");
        ScrollView scroll = new ScrollView(this);
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        int padding = (int)(16 * getResources().getDisplayMetrics().density);
        layout.setPadding(padding, padding, padding, padding);
        scroll.addView(layout);
        TextView help = new TextView(this);
        help.setText("1. 选择升级APP的BIN（0x08010000）\n2. 通过已连接的BLE请求进入升级，或按住KEY_1复位\n"
            + "3. 等待约5秒，在系统蓝牙设置中配对模块的经典蓝牙SPP\n4. 返回本页，选择对应设备并升级\n\n"
            + "升级会覆盖APP；中断后可重新完整升级。升级期间保持本页前台、手表供电，完成后重新连接BLE。");
        layout.addView(help);
        choose = button(layout, "1 · 选择 BIN 文件", view -> chooseImage());
        request = button(layout, "2 · 让手表进入升级模式（当前BLE连接）", view -> requestUpgrade());
        settings = button(layout, "3 · 打开系统蓝牙设置 / 配对SPP", view -> startActivity(new Intent(Settings.ACTION_BLUETOOTH_SETTINGS)));
        transfer = button(layout, "4 · 选择已配对设备并开始升级", view -> chooseDevice());
        button(layout, "停止本次传输", view -> stopTransfer("已停止；Bootloader保留升级状态，需重新完整发送"));
        progress = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progress.setMax(100);
        layout.addView(progress);
        status = new TextView(this);
        status.setText("等待选择BIN。首次使用需通过SWD部署Bootloader和升级APP。");
        layout.addView(status);
        setContentView(scroll);
        ui.post(watchdog);
    }

    private void chooseImage()
    {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*");
        startActivityForResult(intent, 100);
    }

    @Override
    protected void onActivityResult(int code, int result, Intent data)
    {
        super.onActivityResult(code, result, data);
        if (code != 100 || result != RESULT_OK || data == null || data.getData() == null)
        {
            return;
        }
        Uri uri = data.getData();
        image = null;
        try (InputStream input = getContentResolver().openInputStream(uri);
             ByteArrayOutputStream output = new ByteArrayOutputStream())
        {
            if (input == null)
            {
                throw new IOException("无法打开所选文件");
            }
            byte[] buffer = new byte[4096];
            int count;
            while ((count = input.read(buffer)) != -1)
            {
                if (output.size() + count > OtaProtocol.MAX_IMAGE)
                {
                    throw new IOException("BIN超过320KiB的APP分区");
                }
                output.write(buffer, 0, count);
            }
            byte[] selected = output.toByteArray();
            String command = OtaProtocol.command(selected);
            String name = "所选BIN";
            try (Cursor cursor = getContentResolver().query(uri, new String[]{OpenableColumns.DISPLAY_NAME}, null, null, null))
            {
                if (cursor != null && cursor.moveToFirst())
                {
                    name = cursor.getString(0);
                }
            }
            image = selected;
            status.setText(name + "\n" + selected.length + " 字节\n" + command.trim() + "\n文件检查通过，尚未发送。");
            progress.setProgress(0);
        }
        catch (Exception error)
        {
            status.setText("文件检查失败：" + error.getMessage());
        }
    }

    private void requestUpgrade()
    {
        if (image == null)
        {
            status.setText("请先选择并检查BIN文件");
            return;
        }
        new AlertDialog.Builder(this).setTitle("进入手表升级模式")
            .setMessage("手表将保存升级标志并重启，BLE会断开；随后配对SPP并发送所选BIN。")
            .setNegativeButton("取消", null).setPositiveButton("进入升级", (dialog, which) ->
            {
                try
                {
                    OtaBridge.requestUpgrade();
                    status.setText("已提交OTA,START；请等待手表重启并开放SPP。此提示不代表手表已受理，"
                        + "若未重启，请检查原接收窗口的OTA错误，或按住KEY_1复位。");
                }
                catch (Exception error)
                {
                    status.setText("无法请求升级：" + error.getMessage());
                }
            }).show();
    }

    private boolean permitted()
    {
        if (Build.VERSION.SDK_INT >= 31 && checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED)
        {
            requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT}, 101);
            status.setText("请允许附近设备权限，然后再次选择设备");
            return false;
        }
        return true;
    }

    private void chooseDevice()
    {
        if (image == null)
        {
            status.setText("请先选择有效的升级BIN");
            return;
        }
        if (!permitted())
        {
            return;
        }
        try
        {
            BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
            if (adapter == null || !adapter.isEnabled())
            {
                status.setText("请先打开手机蓝牙");
                return;
            }
            List<BluetoothDevice> devices = new ArrayList<>();
            List<String> names = new ArrayList<>();
            for (BluetoothDevice device : adapter.getBondedDevices())
            {
                if (device.getType() != BluetoothDevice.DEVICE_TYPE_LE)
                {
                    devices.add(device);
                    names.add(String.valueOf(device.getName()) + "\n" + device.getAddress());
                }
            }
            if (devices.isEmpty())
            {
                status.setText("没有已配对的经典蓝牙设备；先让手表进入升级模式，再在系统设置配对模块SPP");
                return;
            }
            new AlertDialog.Builder(this).setTitle("选择手表模块的SPP设备")
                .setItems(names.toArray(new String[0]), (dialog, index) ->
                    new AlertDialog.Builder(this).setTitle("开始固件升级")
                        .setMessage("向 " + names.get(index) + "\n发送 " + image.length + " 字节；Bootloader会擦除原APP。")
                        .setNegativeButton("取消", null)
                        .setPositiveButton("开始升级", (confirm, which) -> startTransfer(devices.get(index))).show())
                .setNegativeButton("取消", null).show();
        }
        catch (SecurityException error)
        {
            status.setText("请在系统设置允许此App的蓝牙/附近设备权限");
        }
    }

    private void setRunning(boolean active)
    {
        running = active;
        choose.setEnabled(!active);
        request.setEnabled(!active);
        transfer.setEnabled(!active);
        settings.setEnabled(!active);
        if (active)
        {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }
        else
        {
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }
    }

    private void closeSocket()
    {
        BluetoothSocket active = socket;
        if (active != null)
        {
            try
            {
                active.close();
            }
            catch (IOException error)
            {
                // close只用于结束本次连接，不自动重试传输。
            }
        }
    }

    private void stopTransfer(String reason)
    {
        if (running)
        {
            cancelled = true;
            closeSocket();
            status.setText(reason);
        }
    }

    private void startTransfer(BluetoothDevice device)
    {
        if (running)
        {
            return;
        }
        cancelled = false;
        setRunning(true);
        deadline = SystemClock.elapsedRealtime() + 20000;
        status.setText("连接SPP…");
        final byte[] selected = image;
        new Thread(() ->
        {
            boolean success = false;
            try
            {
                // 独立socket只服务OTA，原收发窗口不读取这条连接。
                BluetoothSocket active = device.createRfcommSocketToServiceRecord(SPP);
                socket = active;
                if (cancelled)
                {
                    throw new IOException("已取消");
                }
                active.connect();
                InputStream input = active.getInputStream();
                OutputStream output = active.getOutputStream();
                OtaProtocol.send(new OtaProtocol.Transport()
                {
                    private void check() throws IOException
                    {
                        if (cancelled)
                        {
                            throw new IOException("已取消");
                        }
                    }

                    @Override
                    public void write(byte[] data, int offset, int length) throws IOException
                    {
                        check();
                        deadline = SystemClock.elapsedRealtime() + 10000;
                        output.write(data, offset, length);
                    }

                    @Override
                    public String readLine(int timeoutMs) throws IOException
                    {
                        check();
                        deadline = SystemClock.elapsedRealtime() + timeoutMs;
                        StringBuilder line = new StringBuilder();
                        while (true)
                        {
                            int value = input.read();
                            if (value == -1)
                            {
                                throw new IOException("设备连接已断开");
                            }
                            if (value == '\n')
                            {
                                return line.toString();
                            }
                            if (line.length() >= 256)
                            {
                                throw new IOException("设备回执过长");
                            }
                            line.append((char)value);
                        }
                    }

                    @Override
                    public void pause(int milliseconds) throws IOException
                    {
                        check();
                        try
                        {
                            Thread.sleep(milliseconds);
                        }
                        catch (InterruptedException error)
                        {
                            Thread.currentThread().interrupt();
                            throw new IOException("传输已中断", error);
                        }
                    }
                }, selected, (sent, total, stage) -> ui.post(() ->
                {
                    progress.setProgress(sent * 100 / total);
                    status.setText(stage + "\n" + sent + " / " + total + " 字节");
                }));
                success = true;
            }
            catch (Exception error)
            {
                if (!cancelled)
                {
                    ui.post(() -> status.setText("升级未确认：" + error.getMessage() + "\n重新进入升级模式后，从头发送完整BIN。"));
                }
            }
            finally
            {
                deadline = 0;
                closeSocket();
                socket = null;
                final boolean completed = success;
                ui.post(() ->
                {
                    setRunning(false);
                    if (completed)
                    {
                        status.setText("升级成功：收到设备ok,len:" + selected.length + "\n手表正在启动，返回原主界面重新连接BLE。");
                    }
                });
            }
        }, "f411-ota").start();
    }

    @Override
    protected void onStop()
    {
        stopTransfer("已离开升级页面，停止本次传输；重新进入后完整重传");
        super.onStop();
    }

    @Override
    protected void onDestroy()
    {
        ui.removeCallbacks(watchdog);
        closeSocket();
        super.onDestroy();
    }
}
