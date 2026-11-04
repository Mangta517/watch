package android.bluetooth;

import java.util.UUID;

public class BluetoothGattService
{
    private final UUID uuid;
    private final BluetoothGattCharacteristic characteristic;

    public BluetoothGattService(UUID uuid, BluetoothGattCharacteristic characteristic)
    {
        this.uuid = uuid;
        this.characteristic = characteristic;
    }

    public UUID getUuid()
    {
        return uuid;
    }

    public BluetoothGattCharacteristic getCharacteristic(UUID id)
    {
        if (characteristic != null && characteristic.getUuid().equals(id))
        {
            return characteristic;
        }
        return null;
    }
}
