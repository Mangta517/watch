package dev.mangta.wave;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.database.Cursor;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.provider.OpenableColumns;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/** 选择APP BIN，一键完成SPP OTA：主BLE控制、切换SPP发送、恢复日常连接。 */
public final class OtaActivity extends Activity
{
    private static final long SCAN_MS = 8000;
    private static final int RESTORE_TIMEOUT_MS = 45000;
    private static final long WORK_TIMEOUT_MS = 60000;
    // SPP阶段最长90秒，看门狗必须更长；阶段回调和进度都会重新计时。
    private static final long WATCHDOG_MS = RESTORE_TIMEOUT_MS + WORK_TIMEOUT_MS;
    private static final long PAUSE_SLICE_MS = 50;
    private final Handler ui = new Handler(Looper.getMainLooper());
    private final List<BluetoothDevice> found = new ArrayList<>();
    private final Set<String> addresses = new HashSet<>();
    private TextView status;
    private ProgressBar progress;
    private Button choose, scan, start, stop;
    private byte[] image;
    private BluetoothDevice target;
    private volatile OtaUpgrade.Connection ble;
    private BluetoothLeScanner scanner;
    private AlertDialog deviceDialog;
    private volatile boolean running, scanning, cancelled, destroyed, foreground;
    private volatile long deadline;
    private volatile Thread worker;
    private boolean scanForStart;
    private int runId;
    private int dialogSession;

    private final Runnable watchdog = new Runnable()
    {
        @Override
        public void run()
        {
            if (running && !cancelled && deadline != 0 && SystemClock.elapsedRealtime() >= deadline)
            {
                stopTransfer("通信超时，已停止；重新开始会从头完整重传");
            }
            ui.postDelayed(this, 500);
        }
    };

    private final Runnable scanTimeout = new Runnable()
    {
        @Override
        public void run()
        {
            if (!scanning)
            {
                return;
            }
            boolean autoStart = scanForStart;
            stopScan();
            showDevices(autoStart);
        }
    };

    private final ScanCallback scanCallback = new ScanCallback()
    {
        @Override
        public void onScanResult(int type, ScanResult result)
        {
            addDevice(result);
        }

        @Override
        public void onBatchScanResults(List<ScanResult> results)
        {
            if (results == null)
            {
                return;
            }
            for (ScanResult result : results)
            {
                addDevice(result);
            }
        }

        @Override
        public void onScanFailed(int code)
        {
            ui.post(() ->
            {
                if (!scanning)
                {
                    return;
                }
                stopScan();
                status.setText("BLE扫描失败，错误码 " + code + "；请关闭再打开手机蓝牙后重试");
            });
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
        setTitle("F411 手表SPP OTA");
        ScrollView scroll = new ScrollView(this);
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        int padding = (int)(16 * getResources().getDisplayMetrics().density);
        layout.setPadding(padding, padding, padding, padding);
        scroll.addView(layout);
        TextView help = new TextView(this);
        help.setText("1. 先在原主界面连接手表BLE\n"
            + "2. 选择升级APP的BIN文件\n"
            + "3. 点“开始升级”一次：页面自动释放原连接、让手表进入升级模式、经SPP发送并恢复\n\n"
            + "自动流程：连接主BLE → 控制进入升级 → 切换SPP → 发送 → 校验 → 恢复原连接。\n"
            + "首次升级可能弹出系统蓝牙配对确认，请允许；升级期间保持本页前台和手表供电。\n"
            + "中断后重新开始会从头完整重传。首次仍需用SWD部署主引导、保底和APP；"
            + "手机只发送APP BIN，不能写HEX或替代SWD恢复。");
        layout.addView(help);
        choose = button(layout, "选择APP BIN 文件", view -> chooseImage());
        scan = button(layout, "扫描 / 更换手表设备", view -> scanDevices(false));
        start = button(layout, "开始升级", view -> startUpgrade());
        stop = button(layout, "停止本次升级 / 扫描", view -> stopTransfer("已停止；重新开始会从头完整重传"));
        progress = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progress.setMax(100);
        layout.addView(progress);
        status = new TextView(this);
        layout.addView(status);
        setContentView(scroll);
        ui.post(watchdog);
        try
        {
            target = OtaBridge.connectedDevice();
        }
        catch (Exception error)
        {
            target = null;
        }
        if (target == null)
        {
            status.setText("等待选择APP BIN；开始升级时会自动扫描手表。");
        }
        else
        {
            status.setText("已获取当前手表连接。选择APP BIN后点“开始升级”。");
        }
        setButtons();
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
            OtaProtocol.command(selected);
            String name = "所选BIN";
            try (Cursor cursor = getContentResolver().query(uri, new String[]{OpenableColumns.DISPLAY_NAME}, null, null, null))
            {
                if (cursor != null && cursor.moveToFirst())
                {
                    name = cursor.getString(0);
                }
            }
            image = selected;
            status.setText(name + "\n" + selected.length + " 字节\n文件检查通过，可以开始升级。");
            progress.setProgress(0);
        }
        catch (Exception error)
        {
            status.setText("文件检查失败：" + error.getMessage());
        }
    }

    private boolean permitted()
    {
        if (Build.VERSION.SDK_INT >= 31)
        {
            boolean scanAllowed = checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED;
            boolean connectAllowed = checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
            if (!scanAllowed || !connectAllowed)
            {
                requestPermissions(new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT}, 101);
                status.setText("请允许“附近设备”扫描和连接权限，然后重试");
                return false;
            }
        }
        if (Build.VERSION.SDK_INT >= 23 && Build.VERSION.SDK_INT < 31 &&
            checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED)
        {
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 101);
            status.setText("Android 6–11扫描BLE需要“位置”权限，请允许后重试");
            return false;
        }
        return true;
    }

    private boolean locationReady()
    {
        if (Build.VERSION.SDK_INT < 23 || Build.VERSION.SDK_INT >= 31)
        {
            return true;
        }
        LocationManager manager = (LocationManager)getSystemService(LOCATION_SERVICE);
        if (manager == null)
        {
            status.setText("无法读取系统位置服务；Android 6–11扫描BLE前请打开“位置信息”");
            return false;
        }
        boolean gps;
        boolean network;
        try
        {
            gps = manager.isProviderEnabled(LocationManager.GPS_PROVIDER);
            network = manager.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
        }
        catch (RuntimeException error)
        {
            status.setText("无法读取系统位置服务；请在系统设置打开“位置信息”后重试");
            return false;
        }
        if (gps || network)
        {
            return true;
        }
        status.setText("Android 6–11扫描BLE需要打开系统“位置信息”服务；打开后返回本页重试");
        return false;
    }

    private void addDevice(ScanResult result)
    {
        if (result == null || result.getDevice() == null)
        {
            return;
        }
        BluetoothDevice device = result.getDevice();
        String address;
        try
        {
            address = device.getAddress();
        }
        catch (SecurityException error)
        {
            return;
        }
        if (address == null)
        {
            return;
        }
        synchronized (found)
        {
            if (!addresses.add(address))
            {
                return;
            }
            found.add(device);
        }
    }

    private void scanDevices(boolean forStart)
    {
        if (running || scanning || deviceDialog != null)
        {
            return;
        }
        if (forStart && image == null)
        {
            status.setText("请先选择有效的升级APP BIN");
            return;
        }
        if (!permitted() || !locationReady())
        {
            return;
        }
        BluetoothManager manager = (BluetoothManager)getSystemService(BLUETOOTH_SERVICE);
        BluetoothAdapter adapter = null;
        try
        {
            if (manager != null)
            {
                adapter = manager.getAdapter();
            }
            if (adapter == null || !adapter.isEnabled())
            {
                status.setText("请先打开手机蓝牙");
                return;
            }
        }
        catch (SecurityException error)
        {
            status.setText("读取蓝牙状态被拒绝，请在系统设置允许BLE扫描和连接权限");
            return;
        }
        BluetoothLeScanner active;
        try
        {
            active = adapter.getBluetoothLeScanner();
        }
        catch (SecurityException error)
        {
            status.setText("BLE扫描权限被拒绝，请在系统设置允许“附近设备”权限");
            return;
        }
        if (active == null)
        {
            status.setText("此手机不支持BLE扫描");
            return;
        }
        synchronized (found)
        {
            found.clear();
            addresses.clear();
        }
        scanForStart = forStart;
        scanner = active;
        scanning = true;
        setButtons();
        status.setText("正在扫描手表BLE设备8秒…请让手表停留在升级模式");
        try
        {
            ScanSettings settings = new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
            active.startScan(null, settings, scanCallback);
        }
        catch (SecurityException error)
        {
            stopScan();
            status.setText("BLE扫描权限被拒绝，请在系统设置允许“附近设备”权限");
            return;
        }
        catch (RuntimeException error)
        {
            stopScan();
            status.setText("无法启动BLE扫描：" + error.getMessage());
            return;
        }
        ui.postDelayed(scanTimeout, SCAN_MS);
    }

    private void stopScan()
    {
        boolean wasScanning = scanning;
        scanning = false;
        ui.removeCallbacks(scanTimeout);
        BluetoothLeScanner active = scanner;
        scanner = null;
        if (active != null)
        {
            try
            {
                active.stopScan(scanCallback);
            }
            catch (RuntimeException error)
            {
                // 蓝牙可能已关闭，扫描随适配器停止，不再重试。
            }
        }
        if (wasScanning)
        {
            setButtons();
        }
    }

    private String deviceName(BluetoothDevice device)
    {
        String name = null;
        try
        {
            name = device.getName();
        }
        catch (SecurityException error)
        {
            name = null;
        }
        if (name == null || name.length() == 0)
        {
            name = "未命名BLE设备";
        }
        return name;
    }

    private String deviceAddress(BluetoothDevice device)
    {
        try
        {
            String address = device.getAddress();
            if (address != null)
            {
                return address;
            }
        }
        catch (SecurityException error)
        {
            return "地址不可用";
        }
        return "地址不可用";
    }

    private void showDevices(boolean autoStart)
    {
        if (destroyed || !foreground)
        {
            return;
        }
        final List<BluetoothDevice> devices;
        synchronized (found)
        {
            devices = new ArrayList<>(found);
        }
        if (devices.isEmpty())
        {
            status.setText("未发现手表设备；请确认手表已进入升级模式、手机蓝牙已打开，然后重试");
            return;
        }
        final List<String> names = new ArrayList<>();
        for (BluetoothDevice device : devices)
        {
            names.add(deviceName(device) + "\n" + deviceAddress(device));
        }
        status.setText("发现 " + devices.size() + " 个BLE设备，请选择手表");
        final int session = ++dialogSession;
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("选择手表BLE设备");
        builder.setItems(names.toArray(new String[0]), (dialog, index) ->
        {
            if (destroyed || !foreground || session != dialogSession)
            {
                return;
            }
            BluetoothDevice device = devices.get(index);
            target = device;
            dismissDeviceDialog();
            if (autoStart)
            {
                beginRun(device);
            }
            else
            {
                status.setText("已选择 " + names.get(index) + "，点“开始升级”。");
            }
        });
        builder.setNegativeButton("取消", (dialog, which) -> dismissDeviceDialog());
        AlertDialog dialog = builder.create();
        deviceDialog = dialog;
        dialog.setOnDismissListener(dismissed ->
        {
            if (deviceDialog == dismissed)
            {
                deviceDialog = null;
                dialogSession++;
                setButtons();
            }
        });
        dialog.show();
        setButtons();
    }

    private void dismissDeviceDialog()
    {
        AlertDialog dialog = deviceDialog;
        deviceDialog = null;
        dialogSession++;
        if (dialog != null)
        {
            dialog.dismiss();
        }
        setButtons();
    }

    private void setButtons()
    {
        boolean idle = !running && !scanning && deviceDialog == null;
        choose.setEnabled(idle);
        scan.setEnabled(idle);
        start.setEnabled(idle);
        stop.setEnabled(running || scanning || deviceDialog != null);
    }

    private void setRunning(boolean active)
    {
        running = active;
        setButtons();
        if (active)
        {
            getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }
        else
        {
            getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        }
    }

    private void stopTransfer(String reason)
    {
        boolean busy = running || scanning || deviceDialog != null;
        cancelled = true;
        stopScan();
        dismissDeviceDialog();
        OtaUpgrade.Connection active = ble;
        if (active != null)
        {
            active.close();
        }
        Thread current = worker;
        if (current != null)
        {
            current.interrupt();
        }
        if (busy)
        {
            status.setText(reason);
        }
    }

    private boolean previousWorkerAlive()
    {
        Thread current = worker;
        if (current == null)
        {
            return false;
        }
        return current.isAlive();
    }

    private void startUpgrade()
    {
        if (running || scanning || deviceDialog != null)
        {
            return;
        }
        if (previousWorkerAlive())
        {
            status.setText("上一次升级仍在退出，请稍候再开始。");
            return;
        }
        if (image == null)
        {
            status.setText("请先选择并检查APP BIN文件");
            return;
        }
        if (target == null)
        {
            try
            {
                target = OtaBridge.connectedDevice();
            }
            catch (Exception error)
            {
                target = null;
            }
        }
        if (target == null)
        {
            status.setText("未获取到当前手表连接，正在扫描…");
            scanDevices(true);
            return;
        }
        beginRun(target);
    }

    private void beginRun(BluetoothDevice device)
    {
        if (running || scanning || device == null || !foreground)
        {
            return;
        }
        if (previousWorkerAlive())
        {
            status.setText("上一次升级仍在退出，请稍候再开始。");
            return;
        }
        final byte[] selected = image;
        if (selected == null)
        {
            status.setText("请先选择并检查APP BIN文件");
            return;
        }
        cancelled = false;
        final int id = ++runId;
        setRunning(true);
        deadline = SystemClock.elapsedRealtime() + WATCHDOG_MS;
        status.setText("正在准备升级：释放原连接并让手表进入升级模式…");
        progress.setProgress(0);
        final RunActions actions = new RunActions(device, id);
        worker = new Thread(() ->
        {
            String warning = null;
            String failure = null;
            boolean confirmed = false;
            try
            {
                warning = OtaUpgrade.run(actions, selected, (sent, total, stage) ->
                {
                    if (total <= 0)
                    {
                        return;
                    }
                    int percent = sent * 100 / total;
                    if (destroyed || cancelled || id != runId)
                    {
                        return;
                    }
                    deadline = SystemClock.elapsedRealtime() + WATCHDOG_MS;
                    ui.post(() ->
                    {
                        if (destroyed || cancelled || id != runId)
                        {
                            return;
                        }
                        progress.setProgress(percent);
                        status.setText(stage + "\n" + sent + " / " + total + " 字节");
                    });
                });
                confirmed = true;
            }
            catch (Exception error)
            {
                failure = error.getMessage();
            }
            finally
            {
                deadline = 0;
                actions.close();
            }
            final boolean done = confirmed;
            final String note = warning;
            final String reason = failure;
            ui.post(() ->
            {
                if (destroyed || id != runId)
                {
                    return;
                }
                deadline = 0;
                setRunning(false);
                if (done)
                {
                    progress.setProgress(100);
                    String text = "升级成功：设备已确认固件校验和写入完成。";
                    if (note == null)
                    {
                        text += "\n原BLE日常连接已恢复。";
                    }
                    else
                    {
                        text += "\n注意：" + note;
                    }
                    status.setText(text);
                }
                else if (cancelled)
                {
                    status.setText("已停止；重新开始会从头完整重传。");
                }
                else
                {
                    status.setText("升级未确认：" + reason + "\n重新开始会从头完整重传。");
                }
            });
        }, "f411-ota-oneclick");
        worker.start();
    }

    private final class RunActions implements OtaUpgrade.Actions
    {
        private final BluetoothDevice device;
        private final int id;
        private volatile OtaUpgrade.Connection transport;

        RunActions(BluetoothDevice device, int id)
        {
            this.device = device;
            this.id = id;
        }

        @Override
        public OtaUpgrade.Connection create()
        {
            OtaBleTransport created = new OtaBleTransport(OtaActivity.this, device);
            return publish(created);
        }

        @Override
        public OtaUpgrade.Connection createRecovery()
        {
            BluetoothAdapter adapter;
            try
            {
                BluetoothManager manager = (BluetoothManager)getSystemService(BLUETOOTH_SERVICE);
                if (manager == null)
                {
                    adapter = BluetoothAdapter.getDefaultAdapter();
                }
                else
                {
                    adapter = manager.getAdapter();
                }
            }
            catch (SecurityException denied)
            {
                throw new IllegalStateException("读取蓝牙适配器缺少权限，无法建立SPP连接", denied);
            }
            if (adapter == null)
            {
                throw new IllegalStateException("手机蓝牙适配器不可用，无法建立SPP连接");
            }
            BluetoothDevice sppDevice;
            try
            {
                // KT6368A的SPP地址与BLE地址不同：首字节加1，其余五字节不变。
                sppDevice = adapter.getRemoteDevice(Kt6368Address.spp(device.getAddress()));
            }
            catch (SecurityException denied)
            {
                throw new IllegalStateException("读取手表地址缺少权限，无法建立SPP连接", denied);
            }
            catch (IllegalArgumentException invalid)
            {
                throw new IllegalStateException("无法换算手表SPP地址：" + invalid.getMessage(), invalid);
            }
            OtaSppTransport created = new OtaSppTransport(sppDevice);
            return publish(created);
        }

        /** 先替换共享句柄再返回；已取消或页面销毁时立即关闭新连接。 */
        private OtaUpgrade.Connection publish(OtaUpgrade.Connection created)
        {
            OtaUpgrade.Connection previous = transport;
            if (previous != null)
            {
                previous.close();
            }
            transport = created;
            ble = created;
            if (cancelled || destroyed)
            {
                created.close();
                if (transport == created)
                {
                    transport = null;
                }
                if (ble == created)
                {
                    ble = null;
                }
            }
            return created;
        }

        @Override
        public void release() throws Exception
        {
            OtaBridge.releaseConnection();
        }

        @Override
        public void restore() throws Exception
        {
            OtaBridge.restoreConnection(device, () -> cancelled || destroyed, RESTORE_TIMEOUT_MS);
        }

        @Override
        public boolean cancelled()
        {
            return cancelled || destroyed;
        }

        @Override
        public long nowMs()
        {
            return SystemClock.elapsedRealtime();
        }

        @Override
        public void pause(int ms) throws IOException
        {
            long end = nowMs() + ms;
            while (true)
            {
                if (cancelled())
                {
                    throw new IOException("已取消");
                }
                long left = end - nowMs();
                if (left <= 0)
                {
                    return;
                }
                long slice;
                if (left > PAUSE_SLICE_MS)
                {
                    slice = PAUSE_SLICE_MS;
                }
                else
                {
                    slice = left;
                }
                try
                {
                    Thread.sleep(slice);
                }
                catch (InterruptedException error)
                {
                    Thread.currentThread().interrupt();
                    throw new IOException("已取消");
                }
            }
        }

        @Override
        public void stage(String text)
        {
            if (destroyed || cancelled || id != runId)
            {
                return;
            }
            deadline = SystemClock.elapsedRealtime() + WATCHDOG_MS;
            ui.post(() ->
            {
                if (destroyed || cancelled || id != runId)
                {
                    return;
                }
                status.setText(text);
            });
        }

        void close()
        {
            OtaUpgrade.Connection active = transport;
            transport = null;
            if (active != null)
            {
                active.close();
            }
            if (ble == active)
            {
                ble = null;
            }
        }
    }

    @Override
    public void onRequestPermissionsResult(int code, String[] permissions, int[] results)
    {
        super.onRequestPermissionsResult(code, permissions, results);
        if (code != 101)
        {
            return;
        }
        boolean granted = results.length > 0;
        for (int result : results)
        {
            if (result != PackageManager.PERMISSION_GRANTED)
            {
                granted = false;
            }
        }
        if (granted)
        {
            status.setText("权限已允许，请再次点“开始升级”");
        }
        else
        {
            status.setText("权限被拒绝；BLE扫描需要“附近设备”，Android 6–11还需要“位置”权限");
        }
    }

    @Override
    protected void onStart()
    {
        super.onStart();
        foreground = true;
        setButtons();
    }

    @Override
    protected void onStop()
    {
        foreground = false;
        stopTransfer("已离开升级页面，停止扫描/升级；重新进入后从头完整重传");
        super.onStop();
    }

    @Override
    protected void onDestroy()
    {
        destroyed = true;
        cancelled = true;
        stopScan();
        dismissDeviceDialog();
        OtaUpgrade.Connection active = ble;
        if (active != null)
        {
            active.close();
        }
        Thread current = worker;
        if (current != null)
        {
            current.interrupt();
        }
        ui.removeCallbacks(watchdog);
        ui.removeCallbacks(scanTimeout);
        super.onDestroy();
    }
}
