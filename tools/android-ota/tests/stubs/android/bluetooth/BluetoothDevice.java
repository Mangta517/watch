package android.bluetooth;

import android.content.Context;
import java.io.IOException;
import java.util.UUID;

public class BluetoothDevice
{
    public BluetoothGatt gatt;
    public boolean inlineConnect;
    public BluetoothSocket sppSocket;
    public UUID lastSppUuid;
    public int rfcommCalls;
    private String address;

    public BluetoothDevice()
    {
    }

    public BluetoothDevice(String address)
    {
        this.address = address;
    }

    public String getAddress()
    {
        return address;
    }

    public BluetoothGatt connectGatt(Context context, boolean autoConnect, BluetoothGattCallback callback)
    {
        gatt.callback = callback;
        if (gatt.connectAction != null)
        {
            if (inlineConnect)
            {
                gatt.connectAction.run();
            }
            else
            {
                new Thread(gatt.connectAction).start();
            }
        }
        return gatt;
    }

    public BluetoothSocket createRfcommSocketToServiceRecord(UUID uuid) throws IOException
    {
        ++rfcommCalls;
        lastSppUuid = uuid;
        if (sppSocket == null)
        {
            throw new IOException("未配置SPP socket");
        }
        sppSocket.uuid = uuid;
        return sppSocket;
    }
}
