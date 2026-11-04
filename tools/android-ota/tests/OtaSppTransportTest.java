package dev.mangta.wave;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.UUID;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/** 用最小RFCOMM socket桩验证真实SPP传输的UUID、超时、拼行、取消和最终ACK。 */
public final class OtaSppTransportTest
{
    private static final String MAC = "AA:BB:CC:DD:EE:FF";
    private static final UUID SPP = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");

    private static void check(boolean value, String message)
    {
        if (!value)
        {
            throw new AssertionError(message);
        }
    }

    private static byte[] bytes(String text)
    {
        return text.getBytes(StandardCharsets.US_ASCII);
    }

    private static BluetoothDevice device(BluetoothSocket socket)
    {
        BluetoothDevice device = new BluetoothDevice(MAC);
        device.sppSocket = socket;
        return device;
    }

    private static OtaSppTransport opened(BluetoothSocket socket) throws IOException
    {
        OtaSppTransport transport = new OtaSppTransport(device(socket));
        transport.open(1000);
        return transport;
    }

    /** 可阻塞的输入流：close唤醒等待中的read，模拟Android关闭socket。 */
    private static final class BlockingInput extends InputStream
    {
        private final Object lock = new Object();
        private final ByteArrayOutputStream buffered = new ByteArrayOutputStream();
        private boolean closed;
        private boolean eof;
        private boolean fail;

        void add(byte[] data)
        {
            synchronized (lock)
            {
                buffered.write(data, 0, data.length);
                lock.notifyAll();
            }
        }

        void finish()
        {
            synchronized (lock)
            {
                eof = true;
                lock.notifyAll();
            }
        }

        void breakStream()
        {
            synchronized (lock)
            {
                fail = true;
                lock.notifyAll();
            }
        }

        void shutdown()
        {
            synchronized (lock)
            {
                closed = true;
                lock.notifyAll();
            }
        }

        @Override
        public int available() throws IOException
        {
            synchronized (lock)
            {
                if (closed)
                {
                    throw new IOException("input已关闭");
                }
                if (fail)
                {
                    throw new IOException("input读取失败");
                }
                return buffered.size();
            }
        }

        @Override
        public int read() throws IOException
        {
            synchronized (lock)
            {
                while (buffered.size() == 0)
                {
                    if (closed)
                    {
                        throw new IOException("input已关闭");
                    }
                    if (fail)
                    {
                        throw new IOException("input读取失败");
                    }
                    if (eof)
                    {
                        return -1;
                    }
                    try
                    {
                        lock.wait();
                    }
                    catch (InterruptedException interrupted)
                    {
                        Thread.currentThread().interrupt();
                        throw new IOException("input等待被中断", interrupted);
                    }
                }
                byte[] data = buffered.toByteArray();
                int value = data[0] & 255;
                buffered.reset();
                buffered.write(data, 1, data.length - 1);
                return value;
            }
        }
    }

    /** 记录真实写入长度，用来证明256B不会被拆成BLE 20B包。 */
    private static final class RecordingOutput extends OutputStream
    {
        final ByteArrayOutputStream data = new ByteArrayOutputStream();
        int writes;
        int flushes;
        int largest;

        @Override
        public void write(int value)
        {
            data.write(value);
            ++writes;
            if (largest < 1)
            {
                largest = 1;
            }
        }

        @Override
        public void write(byte[] buffer, int offset, int length)
        {
            data.write(buffer, offset, length);
            ++writes;
            if (largest < length)
            {
                largest = length;
            }
        }

        @Override
        public void flush()
        {
            ++flushes;
        }
    }

    /**
     * 持续可读的对抗流：available恒大于0，read不返回数据。
     * close若还去排空输入，就会卡在read并留下计数，测试据此证明close完全不碰InputStream。
     */
    private static final class AdversarialInput extends InputStream
    {
        volatile boolean closed;
        int availableCalls;
        int readCalls;

        @Override
        public int available()
        {
            ++availableCalls;
            return 64;
        }

        @Override
        public int read() throws IOException
        {
            ++readCalls;
            while (!closed)
            {
                try
                {
                    Thread.sleep(5);
                }
                catch (InterruptedException interrupted)
                {
                    Thread.currentThread().interrupt();
                    throw new IOException("adversarial input被中断", interrupted);
                }
            }
            throw new IOException("adversarial input已关闭");
        }
    }

    private static void awaitLatch(CountDownLatch latch, String message) throws InterruptedException
    {
        if (!latch.await(3, TimeUnit.SECONDS))
        {
            throw new AssertionError(message);
        }
    }

    private static void testSecureUuidAndPinnedDevice() throws Exception
    {
        BluetoothSocket socket = new BluetoothSocket();
        BluetoothAdapter adapter = new BluetoothAdapter();
        adapter.discovering = true;
        BluetoothAdapter.install(adapter);
        BluetoothDevice device = device(socket);
        OtaSppTransport transport = new OtaSppTransport(device);
        transport.open(1000);
        check(device.rfcommCalls == 1, "必须通过给定device创建RFCOMM");
        check(SPP.equals(device.lastSppUuid), "必须使用安全SPP UUID");
        check(MAC.equals(device.getAddress()), "必须保留给定设备MAC");
        check(adapter.cancelDiscoveryCalls == 1, "连接前应取消发现");
        check(socket.connectCalled, "必须调用connect");
        transport.close();
        transport.close();
        check(socket.closes == 1, "close必须幂等");
        BluetoothAdapter.install(null);
    }

    private static void testConnectTimeout() throws Exception
    {
        final CountDownLatch closed = new CountDownLatch(1);
        BluetoothSocket socket = new BluetoothSocket();
        socket.closeAction = closed::countDown;
        socket.connectAction = () ->
        {
            try
            {
                closed.await(5, TimeUnit.SECONDS);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
            }
            throw new RuntimeException(new java.io.InterruptedIOException("closed"));
        };
        OtaSppTransport transport = new OtaSppTransport(device(socket));
        long start = System.nanoTime();
        try
        {
            transport.open(200);
            throw new AssertionError("阻塞connect必须超时");
        }
        catch (SocketTimeoutException expected)
        {
        }
        long elapsedMs = (System.nanoTime() - start) / 1000000L;
        check(elapsedMs < 1500, "connect超时必须按截止点结束");
        check(socket.closes == 1, "超时线程必须关闭本次socket");
        transport.close();
        check(socket.closes == 1, "超时后close不得重复关闭");
    }

    private static void testCancelDuringConnect() throws Exception
    {
        final CountDownLatch entered = new CountDownLatch(1);
        final CountDownLatch closed = new CountDownLatch(1);
        BluetoothSocket socket = new BluetoothSocket();
        socket.closeAction = closed::countDown;
        socket.connectAction = () ->
        {
            entered.countDown();
            try
            {
                closed.await(5, TimeUnit.SECONDS);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
            }
            throw new RuntimeException(new java.io.InterruptedIOException("closed"));
        };
        final OtaSppTransport transport = new OtaSppTransport(device(socket));
        final AtomicBoolean failed = new AtomicBoolean(false);
        Thread opener = new Thread(() ->
        {
            try
            {
                transport.open(5000);
            }
            catch (IOException expected)
            {
                failed.set(true);
            }
        });
        opener.start();
        awaitLatch(entered, "connect未进入");
        transport.close();
        opener.join(2000);
        check(!opener.isAlive(), "取消必须解除connect阻塞");
        check(failed.get(), "取消connect必须失败");
        check(socket.closes == 1, "取消必须关闭本次socket");
    }

    private static void testSplitAndCoalescedLines() throws Exception
    {
        BlockingInput input = new BlockingInput();
        RecordingOutput output = new RecordingOutput();
        OtaSppTransport transport = opened(new BluetoothSocket(input, output));
        input.add(bytes("OTA,LA"));
        try
        {
            transport.readLine(80);
            throw new AssertionError("半行必须超时");
        }
        catch (SocketTimeoutException expected)
        {
        }
        input.add(bytes("YOUT,08010000,00050000\nok,len:512\nnext"));
        check(transport.readLine(500).equals("OTA,LAYOUT,08010000,00050000"), "半行必须跨调用保留");
        check(transport.readLine(500).equals("ok,len:512"), "连续两行必须拆开");
        input.add(bytes("\n"));
        check(transport.readLine(500).equals("next"), "拼接后的行必须返回");
        input.add(bytes("0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789"
            + "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789"
            + "0123456789012345678901234567890123456789012345678901234567890123456789"));
        try
        {
            transport.readLine(500);
            throw new AssertionError("超过256字节的单行必须失败");
        }
        catch (IOException expected)
        {
        }
        transport.close();
    }

    private static void testReadTimeoutAndErrors() throws Exception
    {
        BlockingInput input = new BlockingInput();
        OtaSppTransport transport = opened(new BluetoothSocket(input, new RecordingOutput()));
        long start = System.nanoTime();
        try
        {
            transport.readLine(120);
            throw new AssertionError("无数据必须读超时");
        }
        catch (SocketTimeoutException expected)
        {
        }
        long elapsedMs = (System.nanoTime() - start) / 1000000L;
        check(elapsedMs >= 100 && elapsedMs < 1500, "读超时必须受截止时间限制");
        transport.pause(50);
        transport.close();
        try
        {
            transport.readLine(100);
            throw new AssertionError("关闭后readLine必须失败");
        }
        catch (IOException expected)
        {
        }

        BlockingInput broken = new BlockingInput();
        OtaSppTransport brokenTransport = opened(new BluetoothSocket(broken, new RecordingOutput()));
        broken.breakStream();
        try
        {
            brokenTransport.readLine(100);
            throw new AssertionError("IO错误必须失败");
        }
        catch (IOException expected)
        {
        }
        brokenTransport.close();

        BlockingInput ended = new BlockingInput();
        OtaSppTransport endedTransport = opened(new BluetoothSocket(ended, new RecordingOutput()));
        ended.finish();
        try
        {
            endedTransport.readLine(500);
            throw new AssertionError("EOF必须失败");
        }
        catch (IOException expected)
        {
        }
        endedTransport.close();
    }

    private static void testLargeWriteUnchanged() throws Exception
    {
        RecordingOutput output = new RecordingOutput();
        BluetoothSocket socket = new BluetoothSocket(new BlockingInput(), output);
        OtaSppTransport transport = opened(socket);
        byte[] packet = new byte[256];
        for (int i = 0; i < packet.length; ++i)
        {
            packet[i] = (byte)(i * 7 + 3);
        }
        transport.write(packet, 0, packet.length);
        check(output.writes == 1, "256B必须一次写入");
        check(output.largest == 256, "不得拆成20B包");
        check(output.data.size() == 256, "写入长度必须不变");
        byte[] actual = output.data.toByteArray();
        for (int i = 0; i < packet.length; ++i)
        {
            if (actual[i] != packet[i])
            {
                throw new AssertionError("写入内容被改写");
            }
        }
        transport.write(packet, 8, 100);
        check(output.data.size() == 356, "带偏移写入必须完整");
        transport.close();
    }

    private static void testCancelReadAndPause() throws Exception
    {
        BlockingInput input = new BlockingInput();
        BluetoothSocket socket = new BluetoothSocket(input, new RecordingOutput());
        socket.closeAction = input::shutdown;
        OtaSppTransport transport = opened(socket);
        final AtomicBoolean failed = new AtomicBoolean(false);
        Thread reader = new Thread(() ->
        {
            try
            {
                transport.readLine(5000);
            }
            catch (IOException expected)
            {
                failed.set(true);
            }
        });
        reader.start();
        Thread.sleep(40);
        transport.close();
        reader.join(1000);
        check(!reader.isAlive(), "关闭必须解除readLine等待");
        check(failed.get(), "关闭时readLine必须失败");

        BluetoothSocket pauseSocket = new BluetoothSocket(new BlockingInput(), new RecordingOutput());
        OtaSppTransport pauser = opened(pauseSocket);
        final AtomicBoolean pauseFailed = new AtomicBoolean(false);
        Thread pauseThread = new Thread(() ->
        {
            try
            {
                pauser.pause(5000);
            }
            catch (IOException expected)
            {
                pauseFailed.set(true);
            }
        });
        pauseThread.start();
        Thread.sleep(40);
        pauser.close();
        pauseThread.join(1000);
        check(!pauseThread.isAlive(), "关闭必须解除pause等待");
        check(pauseFailed.get(), "关闭时pause必须失败");
    }

    /** 可阻塞的输出流：close唤醒等待中的write，模拟Android关闭socket。 */
    private static final class BlockingOutput extends OutputStream
    {
        private final Object lock = new Object();
        private final ByteArrayOutputStream data = new ByteArrayOutputStream();
        private boolean closed;
        private boolean block;

        void blockWrites()
        {
            synchronized (lock)
            {
                block = true;
            }
        }

        void shutdown()
        {
            synchronized (lock)
            {
                closed = true;
                lock.notifyAll();
            }
        }

        @Override
        public void write(int value) throws IOException
        {
            write(new byte[]{(byte)value}, 0, 1);
        }

        @Override
        public void write(byte[] buffer, int offset, int length) throws IOException
        {
            synchronized (lock)
            {
                while (block && !closed)
                {
                    try
                    {
                        lock.wait();
                    }
                    catch (InterruptedException interrupted)
                    {
                        Thread.currentThread().interrupt();
                        throw new IOException("output等待被中断", interrupted);
                    }
                }
                if (closed)
                {
                    throw new IOException("output已关闭");
                }
                data.write(buffer, offset, length);
            }
        }
    }

    private static void testCancelWrite() throws Exception
    {
        BlockingOutput output = new BlockingOutput();
        output.blockWrites();
        BluetoothSocket socket = new BluetoothSocket(new BlockingInput(), output);
        socket.closeAction = output::shutdown;
        OtaSppTransport transport = opened(socket);
        final AtomicBoolean failed = new AtomicBoolean(false);
        Thread writer = new Thread(() ->
        {
            try
            {
                transport.write(new byte[256], 0, 256);
            }
            catch (IOException expected)
            {
                failed.set(true);
            }
        });
        writer.start();
        Thread.sleep(40);
        transport.close();
        writer.join(1000);
        check(!writer.isAlive(), "关闭必须解除write阻塞");
        check(failed.get(), "关闭时write必须失败");
    }

    /** close必须立即关闭socket且完全不读InputStream，即使输入永远可读。 */
    private static void testCloseNeverReadsInput() throws Exception
    {
        AdversarialInput input = new AdversarialInput();
        BluetoothSocket socket = new BluetoothSocket(input, new RecordingOutput());
        socket.closeAction = () ->
        {
            input.closed = true;
        };
        final OtaSppTransport transport = opened(socket);
        Thread closer = new Thread(transport::close);
        long start = System.nanoTime();
        closer.start();
        closer.join(2000);
        long elapsedMs = (System.nanoTime() - start) / 1000000L;
        if (closer.isAlive())
        {
            // 旧实现在close里排空输入会卡死在这里；放行线程后按断言失败报告。
            input.closed = true;
            closer.join(1000);
            throw new AssertionError("close不得阻塞在InputStream读取");
        }
        check(elapsedMs < 500, "close必须立即返回，不能排空输入");
        check(input.readCalls == 0, "close不得调用InputStream.read");
        check(input.availableCalls == 0, "close不得调用InputStream.available");
        check(socket.closes == 1, "close必须恰好关闭socket一次");
        transport.close();
        check(socket.closes == 1, "重复close不得再次关闭socket");
    }

    /**
     * 看门狗在截止点关闭socket后connect仍正常返回（不抛异常）：
     * open必须按超时失败，不能发布看似成功的连接。
     */
    private static final class DeadlineRaceSocket extends BluetoothSocket
    {
        private final CountDownLatch closedLatch = new CountDownLatch(1);

        DeadlineRaceSocket()
        {
            closeAction = closedLatch::countDown;
        }

        @Override
        public void connect() throws IOException
        {
            ++connects;
            connectCalled = true;
            try
            {
                closedLatch.await(5, TimeUnit.SECONDS);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
            }
            // 截止点被看门狗关闭后仍正常返回，模拟connect不因关闭而抛异常。
        }
    }

    private static void testConnectDeadlineRace() throws Exception
    {
        DeadlineRaceSocket socket = new DeadlineRaceSocket();
        OtaSppTransport transport = new OtaSppTransport(device(socket));
        long start = System.nanoTime();
        try
        {
            transport.open(200);
            throw new AssertionError("截止后正常返回的connect也必须判超时");
        }
        catch (SocketTimeoutException expected)
        {
        }
        long elapsedMs = (System.nanoTime() - start) / 1000000L;
        check(elapsedMs < 1500, "connect超时必须按截止点结束");
        check(socket.connects == 1, "必须调用过connect");
        check(socket.closes == 1, "超时必须关闭本次socket");
        check(!socket.isConnected(), "超时后不得保留已连接socket");
        transport.close();
        check(socket.closes == 1, "超时后close不得重复关闭");
    }

    private static void testBufferedAckSurvivesClose() throws Exception
    {
        // 最后一包写入后设备立即回ACK并断开；ACK必须在close前由readLine取走，close本身不排空输入。
        BlockingInput input = new BlockingInput();
        input.add(bytes("ok,len:512\n"));
        input.finish();
        BluetoothSocket socket = new BluetoothSocket(input, new RecordingOutput());
        OtaSppTransport transport = opened(socket);
        check(transport.readLine(1000).equals("ok,len:512"), "断开前已缓冲的ACK必须可读");
        transport.close();
        check(socket.closes == 1, "close必须关闭socket");
    }

    /** 只在指定次数的写入上失败，用来模拟最后一包正文写入时socket断开。 */
    private static final class ScriptedOutput extends OutputStream
    {
        final ByteArrayOutputStream data = new ByteArrayOutputStream();
        int calls;
        int failOnCall;
        Runnable beforeFail;

        @Override
        public void write(int value) throws IOException
        {
            write(new byte[]{(byte)value}, 0, 1);
        }

        @Override
        public void write(byte[] buffer, int offset, int length) throws IOException
        {
            ++calls;
            if (calls == failOnCall)
            {
                if (beforeFail != null)
                {
                    beforeFail.run();
                }
                throw new IOException("socket已断开");
            }
            data.write(buffer, offset, length);
        }
    }

    /**
     * 用真实OtaProtocol.send跑真实SPP传输：只有最后一包正文写入失败，
     * 设备已回ok,len:N时仍必须确认成功；没有ACK时由InputStream的IO错误快速失败。
     */
    private static void testFinalAckThroughProtocol() throws Exception
    {
        byte[] image = new byte[512];
        for (int i = 0; i < image.length; ++i)
        {
            image[i] = (byte)(i * 13 + 5);
        }
        byte[] vectors = {0, 0, 2, 0x20, 9, 0, 1, 8};
        System.arraycopy(vectors, 0, image, 0, 8);
        // INFO、start之后是两包正文；第4次写入即最后一包。
        int lastBodyWrite = 4;

        BlockingInput ackInput = new BlockingInput();
        ackInput.add(bytes("OTA,LAYOUT,08010000,00050000\nFlash erased, ready to receive data...\n"));
        ScriptedOutput ackOutput = new ScriptedOutput();
        ackOutput.failOnCall = lastBodyWrite;
        ackOutput.beforeFail = () -> ackInput.add(bytes("ok,len:512\n"));
        BluetoothSocket ackSocket = new BluetoothSocket(ackInput, ackOutput);
        OtaSppTransport acked = opened(ackSocket);
        OtaProtocol.send(acked, image, (sent, total, stage) ->
        {
        });
        check(ackOutput.calls == lastBodyWrite, "必须走到最后一包正文写入");
        check(ackSocket.closes == 0, "ACK确认前socket不得被提前关闭");
        acked.close();
        check(ackSocket.closes == 1, "成功后close必须关闭socket");

        BlockingInput missingInput = new BlockingInput();
        missingInput.add(bytes("OTA,LAYOUT,08010000,00050000\nFlash erased, ready to receive data...\n"));
        ScriptedOutput missingOutput = new ScriptedOutput();
        missingOutput.failOnCall = lastBodyWrite;
        missingOutput.beforeFail = missingInput::breakStream;
        BluetoothSocket missingSocket = new BluetoothSocket(missingInput, missingOutput);
        OtaSppTransport missing = opened(missingSocket);
        long start = System.nanoTime();
        try
        {
            OtaProtocol.send(missing, image, (sent, total, stage) ->
            {
            });
            throw new AssertionError("缺少ok,len时必须失败");
        }
        catch (IOException expected)
        {
        }
        long elapsedMs = (System.nanoTime() - start) / 1000000L;
        check(elapsedMs < 3000, "缺少ACK必须由IO错误快速失败，不能等60秒");
        missing.close();
    }

    private static void testProtocolOverTransport() throws Exception
    {
        BlockingInput input = new BlockingInput();
        input.add(bytes("OTA,LAYOUT,08010000,00050000\nFlash erased, ready to receive data...\nok,len:512\n"));
        input.finish();
        RecordingOutput output = new RecordingOutput();
        OtaSppTransport transport = opened(new BluetoothSocket(input, output));
        byte[] image = new byte[512];
        for (int i = 0; i < image.length; ++i)
        {
            image[i] = (byte)(i * 13 + 5);
        }
        byte[] vectors = {0, 0, 2, 0x20, 9, 0, 1, 8};
        System.arraycopy(vectors, 0, image, 0, 8);
        OtaProtocol.send(transport, image, (sent, total, stage) ->
        {
        });
        // INFO + start + 两包256B：每包必须保持256B，不按BLE 20B拆分。
        check(output.largest == 256, "协议包必须整包写入");
        check(output.data.size() == 9 + OtaProtocol.command(image).length() + 512, "写入总长必须等于协议内容");
        transport.close();
    }

    public static void main(String[] args) throws Exception
    {
        testSecureUuidAndPinnedDevice();
        testConnectTimeout();
        testCancelDuringConnect();
        testSplitAndCoalescedLines();
        testReadTimeoutAndErrors();
        testLargeWriteUnchanged();
        testCancelReadAndPause();
        testCancelWrite();
        testCloseNeverReadsInput();
        testConnectDeadlineRace();
        testBufferedAckSurvivesClose();
        testProtocolOverTransport();
        testFinalAckThroughProtocol();
        System.out.println("PASS: SPP secure UUID/pinned device, bounded connect timeout and cancel, deadline-close connect race, "
            + "split/coalesced lines, typed read timeout, 256B single write, close-never-reads-input, buffered final ACK, "
            + "protocol send with last-write ACK and fast missing-ACK failure");
    }
}
