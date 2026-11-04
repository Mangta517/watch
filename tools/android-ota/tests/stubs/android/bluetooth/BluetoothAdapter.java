package android.bluetooth;

/** 主机测试用的最小适配器：只提供发现取消入口。 */
public class BluetoothAdapter
{
    private static BluetoothAdapter current;

    public int cancelDiscoveryCalls;
    public boolean discovering;

    public static void install(BluetoothAdapter adapter)
    {
        current = adapter;
    }

    public static BluetoothAdapter getDefaultAdapter()
    {
        return current;
    }

    public boolean isDiscovering()
    {
        return discovering;
    }

    public boolean cancelDiscovery()
    {
        ++cancelDiscoveryCalls;
        discovering = false;
        return true;
    }
}
