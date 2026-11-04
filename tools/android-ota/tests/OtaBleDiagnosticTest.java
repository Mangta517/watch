package dev.mangta.wave;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.content.Context;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.UUID;

/**
 * 用受控DiagnosticParser替身验证真实OtaBleTransport已接入诊断过滤。
 * 替身不是WDG1 CRC解析器；本测试只证明超长二进制通知经过滤后不再撑爆文本行，
 * 以及解析器失败会沿既有fail路径报告，不证明真实CRC帧剥离。
 */
public final class OtaBleDiagnosticTest
{
    private static final UUID CCCD = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private static void check(boolean value, String message)
    {
        if (!value)
        {
            throw new AssertionError(message);
        }
    }

    private static BluetoothGattCharacteristic characteristic()
    {
        BluetoothGattCharacteristic tx = new BluetoothGattCharacteristic(OtaBleTransport.CHARACTERISTIC,
            BluetoothGattCharacteristic.PROPERTY_WRITE | BluetoothGattCharacteristic.PROPERTY_NOTIFY);
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

    private static OtaBleTransport opened(BluetoothGatt gatt) throws IOException
    {
        OtaBleTransport transport = new OtaBleTransport(new Context(), device(gatt));
        transport.open(3000);
        return transport;
    }

    private static void reset(byte[] expected, byte[] response, boolean fail)
    {
        DiagnosticParser.expectedInput = expected;
        DiagnosticParser.response = response;
        DiagnosticParser.fail = fail;
        DiagnosticParser.calls = 0;
        DiagnosticParser.capturedInput = null;
        DiagnosticParser.capturedLength = -1;
        DiagnosticParser.expectedTimeMinMs = Long.MIN_VALUE;
        DiagnosticParser.expectedTimeMaxMs = Long.MAX_VALUE;
    }

    private static void oversizedBinaryChunkIsFiltered() throws Exception
    {
        // 1024B二进制通知远超256B文本行上限；替身把它脚本化替换为布局文本。
        byte[] binary = new byte[1024];
        for (int i = 0; i < binary.length; ++i)
        {
            binary[i] = (byte)(i * 31 + 7);
        }
        byte[] layout = (OtaProtocol.LAYOUT + "\n").getBytes(StandardCharsets.US_ASCII);
        reset(binary, layout, false);
        BluetoothGattCharacteristic tx = characteristic();
        BluetoothGatt gatt = gatt(tx);
        OtaBleTransport transport = opened(gatt);
        gatt.callback.onCharacteristicChanged(gatt, tx, binary);
        check(DiagnosticParser.calls == 1, "notification must pass through the diagnostic filter");
        check(DiagnosticParser.capturedLength == binary.length, "whole binary chunk must reach the parser");
        check(transport.readLine(1000).equals(OtaProtocol.LAYOUT), "filtered layout text must be readable");
        transport.close();
    }

    private static void emptyFilteredResultIsIgnored() throws Exception
    {
        byte[] binary = new byte[512];
        reset(binary, new byte[0], false);
        BluetoothGattCharacteristic tx = characteristic();
        BluetoothGatt gatt = gatt(tx);
        OtaBleTransport transport = opened(gatt);
        gatt.callback.onCharacteristicChanged(gatt, tx, binary);
        try
        {
            transport.readLine(10);
            throw new AssertionError("empty filtered notification must not create a text line");
        }
        catch (java.net.SocketTimeoutException expected)
        {
            check(DiagnosticParser.calls == 1, "filter must be called before the empty result is dropped");
        }
        transport.close();
    }

    private static void parserFailureIsPropagated() throws Exception
    {
        byte[] binary = new byte[512];
        reset(binary, null, true);
        BluetoothGattCharacteristic tx = characteristic();
        BluetoothGatt gatt = gatt(tx);
        OtaBleTransport transport = opened(gatt);
        gatt.callback.onCharacteristicChanged(gatt, tx, binary);
        try
        {
            transport.readLine(1000);
            throw new AssertionError("parser failure must fail the transport");
        }
        catch (IOException expected)
        {
            check("原诊断解析器失败".equals(expected.getMessage()), "parser failure message must be preserved");
        }
        check(gatt.closes == 1, "parser failure must close the GATT link");
        transport.close();
        check(gatt.closes == 1, "close after failure must stay idempotent");
    }

    public static void main(String[] args) throws Exception
    {
        oversizedBinaryChunkIsFiltered();
        emptyFilteredResultIsIgnored();
        parserFailureIsPropagated();
        System.out.println("PASS: transport diagnostic integration drops oversized binary text, ignores empty results, "
            + "and propagates parser failure (fake parser, not real WDG1 CRC stripping)");
    }
}
