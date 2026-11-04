package dev.mangta.wave;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.content.Context;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.UUID;

/** 用最小Android桩验证真实20B分包、回调等待、通知拼行和断开唤醒。 */
public final class OtaBleTransportTest
{
    private static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private static void check(boolean value)
    {
        if (!value)
        {
            throw new AssertionError();
        }
    }

    private static BluetoothGattCharacteristic characteristic(int properties)
    {
        BluetoothGattCharacteristic tx = new BluetoothGattCharacteristic(OtaBleTransport.CHARACTERISTIC, properties);
        tx.descriptor = new BluetoothGattDescriptor(CCCD);
        return tx;
    }

    private static BluetoothGatt gatt(BluetoothGattCharacteristic tx)
    {
        BluetoothGatt gatt = new BluetoothGatt();
        gatt.service = new BluetoothGattService(OtaBleTransport.SERVICE, tx);
        gatt.connectAction = () -> gatt.callback.onConnectionStateChange(gatt, BluetoothGatt.GATT_SUCCESS,
            BluetoothGatt.STATE_CONNECTED);
        return gatt;
    }

    private static BluetoothDevice device(BluetoothGatt gatt)
    {
        BluetoothDevice device = new BluetoothDevice();
        device.gatt = gatt;
        return device;
    }

    private static void await(java.util.concurrent.CountDownLatch latch, String message) throws Exception
    {
        if (!latch.await(3, java.util.concurrent.TimeUnit.SECONDS))
        {
            throw new AssertionError(message);
        }
    }

    private static byte[] join(List<byte[]> packets)
    {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        for (byte[] packet : packets)
        {
            output.write(packet, 0, packet.length);
        }
        return output.toByteArray();
    }

    private static byte[] image(int length)
    {
        byte[] data = new byte[length];
        for (int i = 0; i < length; ++i)
        {
            data[i] = (byte)(i * 37 + 11);
        }
        byte[] vectors = {0, 0, 2, 0x20, 9, 0, 1, 8};
        System.arraycopy(vectors, 0, data, 0, 8);
        return data;
    }

    private interface ChunkHandler
    {
        void chunk(BluetoothGatt gatt, BluetoothGattCharacteristic tx, int sent, int total);
    }

    private static void notifyLine(BluetoothGatt gatt, BluetoothGattCharacteristic tx, String line)
    {
        gatt.callback.onCharacteristicChanged(gatt, tx, (line + "\n").getBytes(StandardCharsets.US_ASCII));
    }

    /** 按协议回答OTA,INFO和start，数据包交给测试决定回执、通知或断开。 */
    private static BluetoothGatt scripted(BluetoothGattCharacteristic tx, int total, ChunkHandler handler)
    {
        BluetoothGatt gatt = gatt(tx);
        int[] sent = {0};
        gatt.writeAction = () ->
        {
            String text = new String(tx.getValue(), StandardCharsets.US_ASCII);
            if (text.equals("OTA,INFO\n"))
            {
                notifyLine(gatt, tx, "OTA,LAYOUT,08010000,00050000");
                gatt.callback.onCharacteristicWrite(gatt, tx, BluetoothGatt.GATT_SUCCESS);
            }
            else if (text.startsWith("start:"))
            {
                notifyLine(gatt, tx, "Flash erased, ready to receive data...");
                gatt.callback.onCharacteristicWrite(gatt, tx, BluetoothGatt.GATT_SUCCESS);
            }
            else
            {
                sent[0] += tx.getValue().length;
                handler.chunk(gatt, tx, sent[0], total);
            }
        };
        return gatt;
    }

    public static void main(String[] args) throws Exception
    {
        // 20B固定分包；每包等待onCharacteristicWrite后才发下一包。
        BluetoothGattCharacteristic tx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt gatt = gatt(tx);
        List<byte[]> packets = new ArrayList<>();
        gatt.writeAction = () ->
        {
            packets.add(Arrays.copyOf(tx.getValue(), tx.getValue().length));
            gatt.callback.onCharacteristicWrite(gatt, tx, BluetoothGatt.GATT_SUCCESS);
        };
        OtaBleTransport transport = new OtaBleTransport(new Context(), device(gatt));
        transport.open(3000);
        byte[] data = new byte[45];
        for (int i = 0; i < data.length; ++i)
        {
            data[i] = (byte)i;
        }
        long started = System.nanoTime();
        transport.write(data, 0, data.length);
        long elapsed = (System.nanoTime() - started) / 1000000L;
        check(packets.size() == 3);
        check(packets.get(0).length == 20 && packets.get(1).length == 20 && packets.get(2).length == 5);
        check(Arrays.equals(data, join(packets)));
        check(elapsed >= 30);
        transport.close();
        transport.close();
        check(gatt.closes == 1);

        // 通知被拆包和合并：先保留半行，再由下一包补全。
        BluetoothGattCharacteristic notifyTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt notifyGatt = gatt(notifyTx);
        OtaBleTransport notified = new OtaBleTransport(new Context(), device(notifyGatt));
        notified.open(3000);
        notifyGatt.callback.onCharacteristicChanged(notifyGatt, notifyTx, "OTA,LAY".getBytes("US-ASCII"));
        notifyGatt.callback.onCharacteristicChanged(notifyGatt, notifyTx,
            "OUT,08010000,00050000\nok,len:7\npartial".getBytes("US-ASCII"));
        check(notified.readLine(1000).equals("OTA,LAYOUT,08010000,00050000"));
        check(notified.readLine(1000).equals("ok,len:7"));
        notified.close();

        // CCCD失败必须在open阶段报错，不能开始OTA。
        BluetoothGattCharacteristic badTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt badGatt = gatt(badTx);
        badGatt.descriptorStatus = BluetoothGatt.GATT_FAILURE;
        OtaBleTransport rejected = new OtaBleTransport(new Context(), device(badGatt));
        try
        {
            rejected.open(3000);
            throw new AssertionError("CCCD failure accepted");
        }
        catch (IOException expected)
        {
            check(badGatt.closes == 1);
        }

        // 断开通知必须唤醒阻塞的readLine。
        BluetoothGattCharacteristic lostTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt lostGatt = gatt(lostTx);
        final OtaBleTransport waiting = new OtaBleTransport(new Context(), device(lostGatt));
        waiting.open(3000);
        final java.util.concurrent.CountDownLatch woke = new java.util.concurrent.CountDownLatch(1);
        Thread reader = new Thread(() ->
        {
            try
            {
                waiting.readLine(60000);
            }
            catch (IOException expected)
            {
                woke.countDown();
            }
        });
        reader.start();
        Thread.sleep(100);
        lostGatt.callback.onConnectionStateChange(lostGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
        await(woke, "disconnect did not wake readLine");

        // 已排队的最终ok必须在断开后先被读出，再报告断开。
        BluetoothGattCharacteristic finalTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt finalGatt = gatt(finalTx);
        OtaBleTransport drained = new OtaBleTransport(new Context(), device(finalGatt));
        drained.open(3000);
        finalGatt.callback.onCharacteristicChanged(finalGatt, finalTx, "ok,len:9\n".getBytes("US-ASCII"));
        finalGatt.callback.onConnectionStateChange(finalGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
        check(drained.readLine(1000).equals("ok,len:9"));
        try
        {
            drained.readLine(1000);
            throw new AssertionError("disconnect not reported after queue drain");
        }
        catch (IOException expected)
        {
        }

        // close必须唤醒等待写入回执的线程。
        BluetoothGattCharacteristic hangTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt hangGatt = gatt(hangTx);
        hangGatt.writeAction = () ->
        {
        };
        final OtaBleTransport hanging = new OtaBleTransport(new Context(), device(hangGatt));
        hanging.open(3000);
        final java.util.concurrent.CountDownLatch writeWoke = new java.util.concurrent.CountDownLatch(1);
        Thread writer = new Thread(() ->
        {
            try
            {
                hanging.write(new byte[20], 0, 20);
            }
            catch (IOException expected)
            {
                writeWoke.countDown();
            }
        });
        writer.start();
        Thread.sleep(100);
        hanging.close();
        await(writeWoke, "close did not wake write");

        // 不支持WRITE_NO_RESPONSE时使用带响应的DEFAULT写入。
        BluetoothGattCharacteristic responseTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt responseGatt = gatt(responseTx);
        OtaBleTransport responded = new OtaBleTransport(new Context(), device(responseGatt));
        responded.open(3000);
        responded.write(new byte[1], 0, 1);
        check(responseTx.getWriteType() == BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        responded.close();

        // 支持WWR时使用NO_RESPONSE，且系统仍回调onCharacteristicWrite时继续下一包。
        BluetoothGattCharacteristic noResponseTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE | BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt noResponseGatt = gatt(noResponseTx);
        OtaBleTransport noResponse = new OtaBleTransport(new Context(), device(noResponseGatt));
        noResponse.open(3000);
        noResponse.write(new byte[1], 0, 1);
        check(noResponseTx.getWriteType() == BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE);
        noResponse.close();

        // 断开必须唤醒等待写入回执的线程并抛错，不能按默认成功状态继续。
        BluetoothGattCharacteristic dropTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt dropGatt = gatt(dropTx);
        dropGatt.writeAction = () ->
        {
        };
        final OtaBleTransport dropped = new OtaBleTransport(new Context(), device(dropGatt));
        dropped.open(3000);
        final java.util.concurrent.CountDownLatch dropWoke = new java.util.concurrent.CountDownLatch(1);
        Thread dropWriter = new Thread(() ->
        {
            try
            {
                dropped.write(new byte[20], 0, 20);
            }
            catch (IOException expected)
            {
                dropWoke.countDown();
            }
        });
        dropWriter.start();
        Thread.sleep(100);
        dropGatt.callback.onConnectionStateChange(dropGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
        await(dropWoke, "disconnect did not wake write");
        check(dropGatt.closes == 1);

        // 两次独立write()之间同样保持20ms，不能只在单次write()内暂停。
        BluetoothGattCharacteristic paceTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt paceGatt = gatt(paceTx);
        List<Long> starts = new ArrayList<>();
        paceGatt.writeAction = () ->
        {
            starts.add(System.nanoTime());
            paceGatt.callback.onCharacteristicWrite(paceGatt, paceTx, BluetoothGatt.GATT_SUCCESS);
        };
        OtaBleTransport paced = new OtaBleTransport(new Context(), device(paceGatt));
        paced.open(3000);
        paced.write(new byte[20], 0, 20);
        paced.write(new byte[20], 0, 20);
        check(starts.size() == 2);
        check((starts.get(1) - starts.get(0)) / 1000000L >= 18);
        paced.close();

        // connectGatt返回前同步回调CONNECTED时，open必须认领同一个GATT并正常写入。
        BluetoothGattCharacteristic earlyTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt earlyGatt = gatt(earlyTx);
        List<byte[]> earlyPackets = new ArrayList<>();
        earlyGatt.writeAction = () ->
        {
            earlyPackets.add(Arrays.copyOf(earlyTx.getValue(), earlyTx.getValue().length));
            earlyGatt.callback.onCharacteristicWrite(earlyGatt, earlyTx, BluetoothGatt.GATT_SUCCESS);
        };
        BluetoothDevice earlyDevice = device(earlyGatt);
        earlyDevice.inlineConnect = true;
        OtaBleTransport early = new OtaBleTransport(new Context(), earlyDevice);
        early.open(3000);
        early.write(new byte[20], 0, 20);
        check(earlyPackets.size() == 1 && earlyPackets.get(0).length == 20);
        early.close();
        check(earlyGatt.closes == 1);

        // connectGatt返回前已认领GATT又遇到close：open只报取消，不能二次关闭。
        BluetoothGattCharacteristic cancelTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt cancelGatt = gatt(cancelTx);
        final java.util.concurrent.CountDownLatch adopted = new java.util.concurrent.CountDownLatch(1);
        final java.util.concurrent.CountDownLatch release = new java.util.concurrent.CountDownLatch(1);
        cancelGatt.discoverAction = () ->
        {
            adopted.countDown();
            try
            {
                release.await(3, java.util.concurrent.TimeUnit.SECONDS);
            }
            catch (InterruptedException interrupted)
            {
                Thread.currentThread().interrupt();
            }
        };
        final OtaBleTransport canceled = new OtaBleTransport(new Context(), device(cancelGatt));
        final java.util.concurrent.CountDownLatch openStopped = new java.util.concurrent.CountDownLatch(1);
        Thread opener = new Thread(() ->
        {
            try
            {
                canceled.open(3000);
            }
            catch (IOException expected)
            {
                openStopped.countDown();
            }
        });
        opener.start();
        await(adopted, "connect did not reach service discovery");
        canceled.close();
        release.countDown();
        await(openStopped, "close during connect did not cancel open");
        check(cancelGatt.disconnects == 1 && cancelGatt.closes == 1);

        // 空队列等待到期必须是SocketTimeoutException，供协调器识别普通APP；断开仍是IOException。
        BluetoothGattCharacteristic timeoutTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt timeoutGatt = gatt(timeoutTx);
        OtaBleTransport timeout = new OtaBleTransport(new Context(), device(timeoutGatt));
        timeout.open(3000);
        try
        {
            timeout.readLine(10);
            throw new AssertionError("read timeout not reported");
        }
        catch (SocketTimeoutException expected)
        {
        }
        timeout.close();

        // 最后一包写入回执前设备已断开，但ok,len:N已排队：仍按严格ACK完成。
        BluetoothGattCharacteristic beforeTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt beforeGatt = scripted(beforeTx, 512, (testGatt, testTx, sent, total) ->
        {
            if (sent < total)
            {
                testGatt.callback.onCharacteristicWrite(testGatt, testTx, BluetoothGatt.GATT_SUCCESS);
                return;
            }
            notifyLine(testGatt, testTx, "ok,len:" + total);
            testGatt.callback.onConnectionStateChange(testGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
        });
        OtaBleTransport before = new OtaBleTransport(new Context(), device(beforeGatt));
        before.open(3000);
        int[] confirmed = {0};
        OtaProtocol.send(before, image(512), (sent, total, stage) ->
        {
            if (stage.startsWith("设备已确认"))
            {
                ++confirmed[0];
            }
        });
        check(confirmed[0] == 1);

        // 最后一包写回执成功后设备才断开并补发ok：同样只读一次ACK并完成。
        BluetoothGattCharacteristic afterTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt afterGatt = scripted(afterTx, 512, (testGatt, testTx, sent, total) ->
        {
            testGatt.callback.onCharacteristicWrite(testGatt, testTx, BluetoothGatt.GATT_SUCCESS);
            if (sent >= total)
            {
                notifyLine(testGatt, testTx, "ok,len:" + total);
                testGatt.callback.onConnectionStateChange(testGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
            }
        });
        OtaBleTransport after = new OtaBleTransport(new Context(), device(afterGatt));
        after.open(3000);
        int[] afterConfirmed = {0};
        OtaProtocol.send(after, image(512), (sent, total, stage) ->
        {
            if (stage.startsWith("设备已确认"))
            {
                ++afterConfirmed[0];
            }
        });
        check(afterConfirmed[0] == 1);

        // 最后一包写入失败且没有ok,len:N时必须失败，不能凭默认成功状态放行。
        BluetoothGattCharacteristic missingTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt missingGatt = scripted(missingTx, 512, (testGatt, testTx, sent, total) ->
        {
            if (sent < total)
            {
                testGatt.callback.onCharacteristicWrite(testGatt, testTx, BluetoothGatt.GATT_SUCCESS);
                return;
            }
            testGatt.callback.onConnectionStateChange(testGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
        });
        OtaBleTransport missing = new OtaBleTransport(new Context(), device(missingGatt));
        missing.open(3000);
        try
        {
            OtaProtocol.send(missing, image(512), (sent, total, stage) ->
            {
            });
            throw new AssertionError("final write without ACK accepted");
        }
        catch (IOException expected)
        {
        }

        // 非最后一包写入失败必须立即失败，不能等到最后再确认。
        BluetoothGattCharacteristic earlyFailTx = characteristic(BluetoothGattCharacteristic.PROPERTY_WRITE |
            BluetoothGattCharacteristic.PROPERTY_NOTIFY);
        BluetoothGatt earlyFailGatt = scripted(earlyFailTx, 512, (testGatt, testTx, sent, total) ->
        {
            if (sent < total)
            {
                testGatt.callback.onConnectionStateChange(testGatt, BluetoothGatt.GATT_FAILURE, BluetoothGatt.STATE_DISCONNECTED);
                return;
            }
            testGatt.callback.onCharacteristicWrite(testGatt, testTx, BluetoothGatt.GATT_SUCCESS);
        });
        OtaBleTransport earlyFail = new OtaBleTransport(new Context(), device(earlyFailGatt));
        earlyFail.open(3000);
        try
        {
            OtaProtocol.send(earlyFail, image(512), (sent, total, stage) ->
            {
            });
            throw new AssertionError("non-final write failure accepted");
        }
        catch (IOException expected)
        {
        }
        System.out.println("PASS: BLE 20B packets, serial write callback, CCCD gate, notify split/coalesce, drain, "
            + "disconnect/close wakeups, global write pacing, early CONNECTED adoption, final ACK after disconnect");
    }
}
