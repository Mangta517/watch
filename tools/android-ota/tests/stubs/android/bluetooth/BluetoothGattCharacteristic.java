package android.bluetooth;

import java.util.UUID;

public class BluetoothGattCharacteristic
{
    public static final int PROPERTY_WRITE = 0x08;
    public static final int PROPERTY_WRITE_NO_RESPONSE = 0x04;
    public static final int PROPERTY_NOTIFY = 0x10;
    public static final int WRITE_TYPE_DEFAULT = 2;
    public static final int WRITE_TYPE_NO_RESPONSE = 1;

    private final UUID uuid;
    private final int properties;
    private byte[] value;
    private int writeType = WRITE_TYPE_DEFAULT;
    public BluetoothGattDescriptor descriptor;

    public BluetoothGattCharacteristic(UUID uuid, int properties)
    {
        this.uuid = uuid;
        this.properties = properties;
    }

    public UUID getUuid()
    {
        return uuid;
    }

    public int getProperties()
    {
        return properties;
    }

    public void setValue(byte[] data)
    {
        value = data;
    }

    public byte[] getValue()
    {
        return value;
    }

    public void setWriteType(int type)
    {
        writeType = type;
    }

    public int getWriteType()
    {
        return writeType;
    }

    public BluetoothGattDescriptor getDescriptor(UUID id)
    {
        if (descriptor != null && descriptor.getUuid().equals(id))
        {
            return descriptor;
        }
        return null;
    }
}
