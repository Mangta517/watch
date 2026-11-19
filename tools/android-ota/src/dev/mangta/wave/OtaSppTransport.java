package dev.mangta.wave;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.SocketTimeoutException;
import java.util.ArrayDeque;
import java.util.UUID;
import java.util.concurrent.TimeUnit;

/**
 * 独立SPP通道：用系统给定的手表BluetoothDevice建立安全RFCOMM连接。
 * 安全连接由Android在connect时弹配对框，不在应用内静默接受PIN或猜测通道。
 * 每个连接只有一个守护线程看门狗，在截止点关闭socket，使connect/write不会无限阻塞。
 */
public final class OtaSppTransport implements OtaUpgrade.Connection
{
    private static final UUID SPP = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");

    private static final int MAX_LINE = 256;
    private static final int MAX_QUEUED_LINES = 32;
    private static final int WAIT_SLICE_MS = 10;
    private static final int WRITE_TIMEOUT_MS = 10000;

    private final Object lock = new Object();
    private final BluetoothDevice device;
    private final ArrayDeque<String> lines = new ArrayDeque<>();
    private IOException readFailure;

    private BluetoothSocket socket;
    private OutputStream output;
    private Thread watchdog;
    private long watchdogDeadline;
    private boolean connectExpired;
    private boolean writeExpired;
    private boolean opened;
    private boolean closed;
    private IOException error;

    public OtaSppTransport(BluetoothDevice device)
    {
        this.device = device;
    }

    boolean isConnecting()
    {
        synchronized (lock)
        {
            return socket != null && !opened && !closed;
        }
    }

    /** 在timeoutMs内完成RFCOMM连接；看门狗在截止点关闭本次socket以解除connect阻塞。 */
    public void open(int timeoutMs) throws IOException
    {
        synchronized (lock)
        {
            if (closed)
            {
                throw new IOException("SPP连接已关闭");
            }
            if (opened || socket != null)
            {
                throw new IOException("SPP连接已打开");
            }
            connectExpired = false;
            writeExpired = false;
        }
        BluetoothSocket created;
        try
        {
            created = device.createRfcommSocketToServiceRecord(SPP);
        }
        catch (SecurityException failure)
        {
            throw new IOException("缺少蓝牙连接权限", failure);
        }
        catch (RuntimeException failure)
        {
            throw new IOException("无法创建SPP连接", failure);
        }
        if (created == null)
        {
            throw new IOException("无法创建SPP连接");
        }
        synchronized (lock)
        {
            if (closed)
            {
                closeQuietly(created);
                throw new IOException("SPP连接已取消");
            }
            socket = created;
        }
        long connectDeadline = System.nanoTime() + timeoutMs * 1000000L;
        try
        {
            cancelDiscovery();
            startWatchdog();
            armWatchdog(connectDeadline);
            boolean connected = false;
            IOException failure = null;
            try
            {
                created.connect();
                connected = true;
            }
            catch (IOException failureFromConnect)
            {
                failure = failureFromConnect;
            }
            catch (RuntimeException failureFromConnect)
            {
                failure = new IOException("SPP连接失败", failureFromConnect);
            }
            finally
            {
                disarmWatchdog();
            }
            if (!connected)
            {
                boolean timeout;
                boolean canceled;
                synchronized (lock)
                {
                    timeout = connectExpired;
                    canceled = closed;
                }
                stopWatchdog();
                abandon();
                if (timeout)
                {
                    throw new SocketTimeoutException("连接SPP超时");
                }
                if (canceled)
                {
                    throw new IOException("SPP连接已取消");
                }
                if (failure != null)
                {
                    throw failure;
                }
                throw new IOException("SPP连接失败");
            }
            // connect即使正常返回，也可能已被看门狗判超时；先核对截止点，不能发布看似成功的连接。
            boolean expired;
            synchronized (lock)
            {
                expired = connectExpired;
            }
            if (expired || System.nanoTime() - connectDeadline >= 0)
            {
                stopWatchdog();
                abandon();
                throw new SocketTimeoutException("连接SPP超时");
            }
            InputStream activeInput;
            OutputStream activeOutput;
            try
            {
                activeInput = created.getInputStream();
                activeOutput = created.getOutputStream();
            }
            catch (IOException failureFromStreams)
            {
                stopWatchdog();
                abandon();
                throw failureFromStreams;
            }
            synchronized (lock)
            {
                if (closed)
                {
                    closeQuietly(created);
                    throw new IOException("SPP连接已取消");
                }
                output = activeOutput;
                opened = true;
                Thread reader = new Thread(() -> readReplies(activeInput), "ota-spp-reader");
                reader.setDaemon(true);
                reader.start();
            }
        }
        catch (RuntimeException unexpected)
        {
            // 看门狗启动等运行时失败也必须释放本次socket。
            stopWatchdog();
            abandon();
            throw new IOException("SPP连接失败", unexpected);
        }
        catch (IOException failed)
        {
            // 取消发现等连接前失败同样要释放本次socket。
            stopWatchdog();
            abandon();
            throw failed;
        }
    }

    /** 完整写入传入长度；256B一包由OtaProtocol控制40ms间隔，这里不再拆包。 */
    public void write(byte[] data, int offset, int length) throws IOException
    {
        if (data == null || offset < 0 || length < 0 || length > data.length - offset)
        {
            throw new IOException("SPP写入参数无效");
        }
        OutputStream active;
        synchronized (lock)
        {
            checkReadyLocked();
            active = output;
        }
        armWatchdog(System.nanoTime() + WRITE_TIMEOUT_MS * 1000000L);
        IOException failure = null;
        try
        {
            active.write(data, offset, length);
            active.flush();
        }
        catch (IOException writeFailure)
        {
            failure = writeFailure;
        }
        finally
        {
            disarmWatchdog();
        }
        boolean timeout;
        synchronized (lock)
        {
            timeout = writeExpired;
            writeExpired = false;
        }
        if (timeout)
        {
            fail("SPP写入超时");
            throw new IOException("SPP写入超时");
        }
        if (failure != null)
        {
            // 最后一包写入可能失败而ACK已到；保留socket，让调用方仍能读走缓冲的回执。
            throw failure;
        }
    }

    /** 在截止时间内取出下一行；远端断开时先交付已收齐的回执。 */
    public String readLine(int timeoutMs) throws IOException
    {
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        IOException failure;
        synchronized (lock)
        {
            while (true)
            {
                checkReadyLocked();
                if (!lines.isEmpty())
                {
                    String line = lines.removeFirst();
                    lock.notifyAll();
                    return line;
                }
                if (readFailure != null)
                {
                    failure = readFailure;
                    break;
                }
                long remaining = deadline - System.nanoTime();
                if (remaining <= 0)
                {
                    throw new SocketTimeoutException("等待设备回执超时");
                }
                try
                {
                    TimeUnit.NANOSECONDS.timedWait(lock, remaining);
                }
                catch (InterruptedException interrupted)
                {
                    Thread.currentThread().interrupt();
                    throw new IOException("等待设备回执被中断", interrupted);
                }
            }
        }
        fail(failure.getMessage());
        throw failure;
    }

    /** 阻塞读取能立即发现EOF；最多缓存32行及当前未完成的一行。 */
    private void readReplies(InputStream stream)
    {
        StringBuilder line = new StringBuilder();
        try
        {
            while (true)
            {
                int value = stream.read();
                if (value < 0)
                {
                    throw new IOException("设备连接已断开");
                }
                if (value == '\n')
                {
                    synchronized (lock)
                    {
                        while (!closed && lines.size() == MAX_QUEUED_LINES)
                        {
                            lock.wait();
                        }
                        if (closed)
                        {
                            return;
                        }
                        lines.addLast(line.toString());
                        lock.notifyAll();
                    }
                    line.setLength(0);
                }
                else
                {
                    if (line.length() >= MAX_LINE)
                    {
                        throw new IOException("设备回执单行超过256字节");
                    }
                    line.append((char)(value & 255));
                }
            }
        }
        catch (InterruptedException interrupted)
        {
            Thread.currentThread().interrupt();
        }
        catch (IOException failure)
        {
            synchronized (lock)
            {
                readFailure = failure;
                lock.notifyAll();
            }
        }
    }

    /** 发送间隔等待；每10ms核对一次关闭和错误，取消时立即结束。 */
    public void pause(int milliseconds) throws IOException
    {
        long deadline = System.nanoTime() + milliseconds * 1000000L;
        while (true)
        {
            synchronized (lock)
            {
                checkReadyLocked();
            }
            long remaining = (deadline - System.nanoTime()) / 1000000L;
            if (remaining <= 0)
            {
                return;
            }
            int slice = (int)Math.min(WAIT_SLICE_MS, remaining);
            try
            {
                Thread.sleep(slice);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
                throw new IOException("SPP发送等待被中断", interrupted);
            }
        }
    }

    /**
     * 可重复调用；先置关闭状态并立即关闭本次socket，唤醒connect/read/write等待。
     * 这里不读取InputStream：最终ACK必须由调用方在close前通过readLine取走。
     */
    public void close()
    {
        BluetoothSocket active;
        boolean already;
        synchronized (lock)
        {
            already = closed;
            active = null;
            if (!already)
            {
                closed = true;
                opened = false;
                active = socket;
                output = null;
                socket = null;
                lock.notifyAll();
            }
        }
        // 失败路径已标记closed时仍要收回看门狗线程。
        stopWatchdog();
        if (already)
        {
            return;
        }
        closeQuietly(active);
    }

    private void checkReadyLocked() throws IOException
    {
        if (error != null)
        {
            throw new IOException(error.getMessage(), error);
        }
        if (closed)
        {
            throw new IOException("SPP连接已断开");
        }
        if (!opened)
        {
            throw new IOException("SPP连接尚未打开");
        }
    }

    /** 尽力取消发现以加快首次连接；适配器不存在时直接交给connect报告。 */
    private void cancelDiscovery() throws IOException
    {
        BluetoothAdapter adapter;
        try
        {
            adapter = BluetoothAdapter.getDefaultAdapter();
        }
        catch (SecurityException denied)
        {
            throw new IOException("访问蓝牙适配器缺少权限", denied);
        }
        catch (RuntimeException unavailable)
        {
            throw new IOException("无法访问蓝牙适配器", unavailable);
        }
        if (adapter == null)
        {
            return;
        }
        try
        {
            if (adapter.isDiscovering())
            {
                adapter.cancelDiscovery();
            }
        }
        catch (SecurityException denied)
        {
            throw new IOException("取消蓝牙发现缺少权限", denied);
        }
        catch (RuntimeException unavailable)
        {
            throw new IOException("无法取消蓝牙发现", unavailable);
        }
    }

    /** 每个连接只建一个守护看门狗线程；它不持锁执行阻塞调用，只在截止点关闭socket。 */
    private void startWatchdog()
    {
        Thread timer = new Thread(() ->
        {
            BluetoothSocket expired = null;
            synchronized (lock)
            {
                while (watchdog == Thread.currentThread())
                {
                    if (watchdogDeadline == 0)
                    {
                        try
                        {
                            lock.wait();
                        }
                        catch (InterruptedException interrupted)
                        {
                            Thread.currentThread().interrupt();
                            return;
                        }
                        continue;
                    }
                    long remaining = (watchdogDeadline - System.nanoTime()) / 1000000L;
                    if (remaining > 0)
                    {
                        try
                        {
                            lock.wait(remaining);
                        }
                        catch (InterruptedException interrupted)
                        {
                            Thread.currentThread().interrupt();
                            return;
                        }
                        continue;
                    }
                    if (opened)
                    {
                        writeExpired = true;
                    }
                    else
                    {
                        connectExpired = true;
                    }
                    watchdogDeadline = 0;
                    expired = socket;
                    socket = null;
                    break;
                }
            }
            closeQuietly(expired);
        }, "ota-spp-watchdog");
        timer.setDaemon(true);
        synchronized (lock)
        {
            if (closed)
            {
                return;
            }
            watchdog = timer;
        }
        timer.start();
    }

    private void armWatchdog(long deadlineNanos)
    {
        synchronized (lock)
        {
            if (watchdog == null)
            {
                return;
            }
            watchdogDeadline = deadlineNanos;
            lock.notifyAll();
        }
    }

    private void disarmWatchdog()
    {
        synchronized (lock)
        {
            watchdogDeadline = 0;
            lock.notifyAll();
        }
    }

    private void stopWatchdog()
    {
        Thread timer;
        synchronized (lock)
        {
            timer = watchdog;
            watchdog = null;
            watchdogDeadline = 0;
            lock.notifyAll();
        }
        if (timer != null && timer != Thread.currentThread())
        {
            try
            {
                timer.join(200);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
            }
        }
    }

    /** 连接失败或取消后统一释放；保留已记录的取消/超时原因。 */
    private void abandon()
    {
        BluetoothSocket active;
        synchronized (lock)
        {
            opened = false;
            active = socket;
            socket = null;
            output = null;
            watchdogDeadline = 0;
            watchdog = null;
            lock.notifyAll();
        }
        closeQuietly(active);
    }

    private void fail(String message)
    {
        BluetoothSocket active;
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
            opened = false;
            active = socket;
            socket = null;
            output = null;
            watchdogDeadline = 0;
            lock.notifyAll();
        }
        closeQuietly(active);
    }

    private void closeQuietly(BluetoothSocket active)
    {
        if (active == null)
        {
            return;
        }
        try
        {
            active.close();
        }
        catch (IOException ignored)
        {
            // close失败不重试，避免阻塞升级线程。
        }
        catch (RuntimeException ignored)
        {
            // 系统可能已释放socket，继续收尾。
        }
    }
}
