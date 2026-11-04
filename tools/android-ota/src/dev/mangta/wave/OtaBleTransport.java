package dev.mangta.wave;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.content.Context;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.util.ArrayDeque;
import java.util.UUID;

/** 独立BLE GATT通道：FFF0服务、FFF1写入和通知，供OtaProtocol逐字节发送BIN。 */
public final class OtaBleTransport implements OtaUpgrade.Connection
{
    public static final UUID SERVICE = UUID.fromString("0000fff0-0000-1000-8000-00805f9b34fb");
    public static final UUID CHARACTERISTIC = UUID.fromString("0000fff1-0000-1000-8000-00805f9b34fb");
    public static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    public static final int MAX_PACKET = 20;

    private static final int WRITE_TIMEOUT_MS = 10000;
    private static final int PACING_MS = 20;
    private static final int MAX_LINE = 256;
    private static final int MAX_LINES = 32;

    private final Object lock = new Object();
    private final Context context;
    private final BluetoothDevice device;
    private final ArrayDeque<String> lines = new ArrayDeque<>();
    private final StringBuilder partial = new StringBuilder();
    private final BluetoothGattCallback callback = new BluetoothGattCallback()
    {
        @Override
        public void onConnectionStateChange(BluetoothGatt callbackGatt, int status, int newState)
        {
            if (status != BluetoothGatt.GATT_SUCCESS || newState != BluetoothGatt.STATE_CONNECTED)
            {
                fail("BLE连接断开");
                return;
            }
            synchronized (lock)
            {
                if (closed)
                {
                    return;
                }
                // connectGatt可能在赋值gatt前同步回调，先认领本次连接。
                if (gatt == null)
                {
                    gatt = callbackGatt;
                }
                if (callbackGatt != gatt)
                {
                    return;
                }
            }
            if (!callbackGatt.discoverServices())
            {
                fail("请求BLE服务发现失败");
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt callbackGatt, int status)
        {
            if (status != BluetoothGatt.GATT_SUCCESS)
            {
                fail("BLE服务发现失败");
                return;
            }
            String failure = null;
            synchronized (lock)
            {
                if (closed || callbackGatt != gatt)
                {
                    return;
                }
                BluetoothGattService service = callbackGatt.getService(SERVICE);
                if (service == null)
                {
                    failure = "未找到BLE服务FFF0";
                }
                else
                {
                    BluetoothGattCharacteristic characteristic = service.getCharacteristic(CHARACTERISTIC);
                    if (characteristic == null)
                    {
                        failure = "未找到BLE特征FFF1";
                    }
                    else
                    {
                        int properties = characteristic.getProperties();
                        boolean writable = (properties & BluetoothGattCharacteristic.PROPERTY_WRITE) != 0 ||
                            (properties & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0;
                        boolean notify = (properties & BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0;
                        if (!writable || !notify)
                        {
                            failure = "FFF1必须同时支持写入和通知";
                        }
                        else
                        {
                            tx = characteristic;
                            writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT;
                            if ((properties & BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE) != 0)
                            {
                                writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE;
                            }
                            BluetoothGattDescriptor descriptor = characteristic.getDescriptor(CCCD);
                            if (!callbackGatt.setCharacteristicNotification(characteristic, true))
                            {
                                failure = "启用FFF1本地通知失败";
                            }
                            else if (descriptor == null)
                            {
                                failure = "FFF1缺少CCCD描述符2902";
                            }
                            else
                            {
                                descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                                try
                                {
                                    if (!callbackGatt.writeDescriptor(descriptor))
                                    {
                                        failure = "写入CCCD描述符失败";
                                    }
                                }
                                catch (RuntimeException error)
                                {
                                    failure = "写入CCCD描述符异常";
                                }
                            }
                        }
                    }
                }
            }
            if (failure != null)
            {
                fail(failure);
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt callbackGatt, BluetoothGattDescriptor descriptor, int status)
        {
            boolean rejected = false;
            synchronized (lock)
            {
                if (closed || callbackGatt != gatt)
                {
                    return;
                }
                if (descriptor == null || !CCCD.equals(descriptor.getUuid()))
                {
                    return;
                }
                if (status == BluetoothGatt.GATT_SUCCESS)
                {
                    ready = true;
                    lock.notifyAll();
                    return;
                }
                rejected = true;
            }
            if (rejected)
            {
                fail("CCCD通知回执失败");
            }
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt callbackGatt, BluetoothGattCharacteristic characteristic, int status)
        {
            synchronized (lock)
            {
                if (closed || callbackGatt != gatt || characteristic == null ||
                    !CHARACTERISTIC.equals(characteristic.getUuid()))
                {
                    return;
                }
                writeBusy = false;
                writeStatus = status;
                lock.notifyAll();
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt callbackGatt, BluetoothGattCharacteristic characteristic)
        {
            if (characteristic != null)
            {
                onNotification(characteristic.getValue());
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt callbackGatt, BluetoothGattCharacteristic characteristic, byte[] value)
        {
            onNotification(value);
        }
    };

    private BluetoothGatt gatt;
    private BluetoothGatt closedGatt;
    private BluetoothGattCharacteristic tx;
    private int writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT;
    private boolean ready;
    private boolean closed;
    private boolean writeBusy;
    private int writeStatus = BluetoothGatt.GATT_SUCCESS;
    private long lastWriteNanos;
    private IOException error;
    // 普通APP每100ms推送WDG1二进制诊断；先由原APP解析器剥离，避免撑爆文本行缓冲。
    private final OtaDiagnosticFilter diagnostic = new OtaDiagnosticFilter();

    public OtaBleTransport(Context context, BluetoothDevice device)
    {
        this.context = context;
        this.device = device;
    }

    /** 建立GATT并等待服务、FFF1属性、通知和CCCD全部就绪。 */
    public void open(int timeoutMs) throws IOException
    {
        synchronized (lock)
        {
            if (closed)
            {
                throw new IOException("BLE连接已关闭");
            }
            if (gatt != null || ready)
            {
                throw new IOException("BLE连接已打开");
            }
        }
        BluetoothGatt created;
        try
        {
            created = device.connectGatt(context, false, callback);
        }
        catch (SecurityException failure)
        {
            throw new IOException("缺少BLE连接权限", failure);
        }
        catch (RuntimeException failure)
        {
            throw new IOException("无法创建BLE GATT连接", failure);
        }
        if (created == null)
        {
            throw new IOException("无法创建BLE GATT连接");
        }
        boolean alreadyClosed = false;
        boolean canceled = false;
        synchronized (lock)
        {
            if (closed)
            {
                canceled = true;
                alreadyClosed = created == closedGatt;
            }
            else
            {
                gatt = created;
            }
        }
        if (canceled)
        {
            if (!alreadyClosed)
            {
                created.disconnect();
                created.close();
            }
            throw new IOException("BLE连接已取消");
        }
        boolean timedOut = false;
        String failure = null;
        synchronized (lock)
        {
            long deadline = System.nanoTime() + timeoutMs * 1000000L;
            while (!ready)
            {
                if (error != null)
                {
                    failure = error.getMessage();
                    break;
                }
                if (closed)
                {
                    failure = "BLE连接已关闭";
                    break;
                }
                long remaining = (deadline - System.nanoTime()) / 1000000L;
                if (remaining <= 0)
                {
                    timedOut = true;
                    break;
                }
                try
                {
                    lock.wait(remaining);
                }
                catch (InterruptedException interrupted)
                {
                    Thread.currentThread().interrupt();
                    failure = "BLE连接等待被中断";
                    break;
                }
            }
        }
        if (timedOut)
        {
            fail("连接BLE服务FFF0超时");
            throw new IOException("连接BLE服务FFF0超时");
        }
        if (failure != null)
        {
            // 取消或回调失败可能发生在gatt赋值之后，必须释放本地句柄。
            BluetoothGatt active;
            synchronized (lock)
            {
                if (closed)
                {
                    throw new IOException(failure);
                }
                closed = true;
                ready = false;
                writeBusy = false;
                active = gatt;
                gatt = null;
                tx = null;
                if (active != null)
                {
                    closedGatt = active;
                }
                lock.notifyAll();
            }
            closeGatt(active);
            throw new IOException(failure);
        }
    }

    @Override
    public void write(byte[] data, int offset, int length) throws IOException
    {
        if (data == null || offset < 0 || length < 0 || length > data.length - offset)
        {
            throw new IOException("BLE写入参数无效");
        }
        int position = 0;
        while (position < length)
        {
            int count = Math.min(MAX_PACKET, length - position);
            writePacket(data, offset + position, count);
            position += count;
        }
    }

    private void writePacket(byte[] data, int offset, int length) throws IOException
    {
        byte[] packet = new byte[length];
        System.arraycopy(data, offset, packet, 0, length);
        boolean timedOut = false;
        int status;
        synchronized (lock)
        {
            checkReadyLocked();
            if (writeBusy)
            {
                throw new IOException("BLE上一包尚未写入完成");
            }
            waitForSpacingLocked();
            tx.setValue(packet);
            tx.setWriteType(writeType);
            writeBusy = true;
            writeStatus = BluetoothGatt.GATT_SUCCESS;
            boolean started;
            try
            {
                started = gatt.writeCharacteristic(tx);
            }
            catch (RuntimeException failure)
            {
                writeBusy = false;
                throw new IOException("BLE写入请求异常", failure);
            }
            if (!started)
            {
                writeBusy = false;
                throw new IOException("BLE写入请求被系统拒绝");
            }
            long deadline = System.nanoTime() + WRITE_TIMEOUT_MS * 1000000L;
            while (writeBusy)
            {
                if (error != null)
                {
                    throw new IOException(error.getMessage());
                }
                if (closed)
                {
                    throw new IOException("BLE连接已断开");
                }
                long remaining = (deadline - System.nanoTime()) / 1000000L;
                if (remaining <= 0)
                {
                    timedOut = true;
                    writeBusy = false;
                    break;
                }
                try
                {
                    lock.wait(remaining);
                }
                catch (InterruptedException interrupted)
                {
                    Thread.currentThread().interrupt();
                    writeBusy = false;
                    throw new IOException("BLE写入等待被中断", interrupted);
                }
            }
            // fail/close会把writeBusy清零并唤醒，必须重新核对断开或取消，不能按默认成功状态继续。
            if (error != null)
            {
                throw new IOException(error.getMessage());
            }
            if (closed)
            {
                throw new IOException("BLE连接已断开");
            }
            status = writeStatus;
            lastWriteNanos = System.nanoTime();
        }
        if (timedOut)
        {
            fail("BLE写入超时");
            throw new IOException("BLE写入超时");
        }
        if (status != BluetoothGatt.GATT_SUCCESS)
        {
            fail("BLE写入失败，状态 " + status);
            throw new IOException("BLE写入失败，状态 " + status);
        }
    }

    @Override
    public String readLine(int timeoutMs) throws IOException
    {
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        synchronized (lock)
        {
            while (true)
            {
                if (!lines.isEmpty())
                {
                    return lines.removeFirst();
                }
                if (error != null)
                {
                    throw new IOException(error.getMessage());
                }
                if (closed)
                {
                    throw new IOException("BLE连接已断开");
                }
                long remaining = (deadline - System.nanoTime()) / 1000000L;
                if (remaining <= 0)
                {
                    // 只有类型化读超时代表普通APP无应答；断开仍走IOException。
                    throw new SocketTimeoutException("等待设备回执超时");
                }
                try
                {
                    lock.wait(remaining);
                }
                catch (InterruptedException interrupted)
                {
                    Thread.currentThread().interrupt();
                    throw new IOException("等待设备回执被中断", interrupted);
                }
            }
        }
    }

    @Override
    public void pause(int milliseconds) throws IOException
    {
        synchronized (lock)
        {
            long deadline = System.nanoTime() + milliseconds * 1000000L;
            while (true)
            {
                checkReadyLocked();
                long remaining = (deadline - System.nanoTime()) / 1000000L;
                if (remaining <= 0)
                {
                    return;
                }
                try
                {
                    lock.wait(Math.min(remaining, 20));
                }
                catch (InterruptedException interrupted)
                {
                    Thread.currentThread().interrupt();
                    throw new IOException("BLE发送等待被中断", interrupted);
                }
            }
        }
    }

    private void checkReadyLocked() throws IOException
    {
        if (error != null)
        {
            throw new IOException(error.getMessage());
        }
        if (closed)
        {
            throw new IOException("BLE连接已断开");
        }
        if (!ready || gatt == null || tx == null)
        {
            throw new IOException("BLE通知尚未就绪");
        }
    }

    /** 所有GATT写入共用20ms单调间隔，跨write()调用也不能连续下发。 */
    private void waitForSpacingLocked() throws IOException
    {
        while (true)
        {
            checkReadyLocked();
            if (lastWriteNanos == 0)
            {
                return;
            }
            long remaining = PACING_MS - (System.nanoTime() - lastWriteNanos) / 1000000L;
            if (remaining <= 0)
            {
                return;
            }
            try
            {
                lock.wait(remaining);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
                throw new IOException("BLE发送等待被中断", interrupted);
            }
        }
    }

    private void onNotification(byte[] value)
    {
        if (value == null)
        {
            return;
        }
        byte[] filtered;
        try
        {
            // 原解析器自带同步，放在文本锁外调用，避免回调线程持锁进入解析器。
            filtered = diagnostic.feed(value);
        }
        catch (IOException failure)
        {
            fail(failure.getMessage());
            return;
        }
        if (filtered == null || filtered.length == 0)
        {
            return;
        }
        value = filtered;
        String failure = null;
        synchronized (lock)
        {
            if (closed)
            {
                return;
            }
            for (int i = 0; i < value.length; ++i)
            {
                char item = (char)(value[i] & 255);
                if (item == '\n')
                {
                    if (lines.size() >= MAX_LINES)
                    {
                        failure = "设备回执队列溢出";
                        break;
                    }
                    lines.add(partial.toString());
                    partial.setLength(0);
                }
                else
                {
                    if (partial.length() >= MAX_LINE)
                    {
                        failure = "设备回执单行超过256字节";
                        break;
                    }
                    partial.append(item);
                }
            }
            lock.notifyAll();
        }
        if (failure != null)
        {
            fail(failure);
        }
    }

    /** 可重复调用；唤醒读写等待并只关闭一次GATT。 */
    public void close()
    {
        BluetoothGatt active;
        synchronized (lock)
        {
            if (closed)
            {
                return;
            }
            closed = true;
            ready = false;
            writeBusy = false;
            active = gatt;
            gatt = null;
            tx = null;
            if (active != null)
            {
                closedGatt = active;
            }
            lock.notifyAll();
        }
        closeGatt(active);
    }

    private void fail(String message)
    {
        BluetoothGatt active;
        synchronized (lock)
        {
            if (closed)
            {
                return;
            }
            if (error == null)
            {
                error = new IOException(message);
            }
            closed = true;
            ready = false;
            writeBusy = false;
            active = gatt;
            gatt = null;
            tx = null;
            if (active != null)
            {
                closedGatt = active;
            }
            lock.notifyAll();
        }
        closeGatt(active);
    }

    private void closeGatt(BluetoothGatt active)
    {
        if (active == null)
        {
            return;
        }
        try
        {
            active.disconnect();
        }
        catch (RuntimeException error)
        {
            // 连接可能已由系统断开，继续close释放本地句柄。
        }
        try
        {
            active.close();
        }
        catch (RuntimeException error)
        {
            // close失败不重试，避免阻塞升级线程。
        }
    }
}
