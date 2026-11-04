package android.bluetooth;

import java.util.UUID;

public class BluetoothGatt
{
    public static final int GATT_SUCCESS = 0;
    public static final int GATT_FAILURE = 257;
    public static final int STATE_CONNECTED = 2;
    public static final int STATE_DISCONNECTED = 0;

    public BluetoothGattCallback callback;
    public Runnable connectAction;
    public Runnable discoverAction;
    public Runnable writeAction;
    public BluetoothGattService service;
    public boolean discoverResult = true;
    public boolean notificationResult = true;
    public boolean descriptorWriteResult = true;
    public boolean writeResult = true;
    public int descriptorStatus = GATT_SUCCESS;
    public int writeStatus = GATT_SUCCESS;
    public int disconnects;
    public int closes;

    public BluetoothGattService getService(UUID uuid)
    {
        if (service != null && service.getUuid().equals(uuid))
        {
            return service;
        }
        return null;
    }

    public boolean discoverServices()
    {
        if (!discoverResult)
        {
            return false;
        }
        if (discoverAction != null)
        {
            discoverAction.run();
        }
        callback.onServicesDiscovered(this, GATT_SUCCESS);
        return true;
    }

    public boolean setCharacteristicNotification(BluetoothGattCharacteristic characteristic, boolean enable)
    {
        return notificationResult;
    }

    public boolean writeDescriptor(BluetoothGattDescriptor descriptor)
    {
        if (!descriptorWriteResult)
        {
            return false;
        }
        callback.onDescriptorWrite(this, descriptor, descriptorStatus);
        return true;
    }

    public boolean writeCharacteristic(BluetoothGattCharacteristic characteristic)
    {
        if (!writeResult)
        {
            return false;
        }
        if (writeAction != null)
        {
            writeAction.run();
        }
        else
        {
            callback.onCharacteristicWrite(this, characteristic, writeStatus);
        }
        return true;
    }

    public void disconnect()
    {
        ++disconnects;
    }

    public void close()
    {
        ++closes;
    }
}
