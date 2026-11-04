package dev.mangta.wave;

import android.app.Activity;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import java.io.IOException;
import java.util.UUID;

/** 用最小旧master桩验证连接捕获、成功恢复、取消和超时的收尾。 */
public final class OtaBridgeTest
{
    private static final String MAC = "AA:BB:CC:DD:EE:FF";
    private static final UUID RX = UUID.fromString("0000fff1-0000-1000-8000-00805f9b34fb");
    private static final UUID TX = UUID.fromString("0000fff1-0000-1000-8000-00805f9b34fb");

    private static void check(boolean value)
    {
        if (!value)
        {
            throw new AssertionError();
        }
    }

    /** 模拟BlueToothMaster的字段与connect/disconnect语义。 */
    public static final class FakeMaster
    {
        public int state;
        public BluetoothDevice deviceConnected;
        public BluetoothDevice deviceToConnect;
        public BluetoothGatt gatt;
        public BluetoothGattCharacteristic rx_characteristic;
        public BluetoothGattCharacteristic tx_characteristic;
        public boolean toShowUUIDDialog = true;
        public int connects;
        public int disconnects;
        public int delayedState = -1;
        public boolean mismatch;

        public void connect(BluetoothDevice device)
        {
            ++connects;
            deviceToConnect = device;
        }

        public void disconnect()
        {
            ++disconnects;
            deviceToConnect = null;
            state = 0;
        }

        public void stopScan()
        {
        }

        /** 首次轮询后按脚本到达目标状态，模拟ManagerThread后台重连。 */
        public void settle(int targetState, boolean ready)
        {
            delayedState = targetState;
            if (targetState == 3 && ready)
            {
                rx_characteristic = new BluetoothGattCharacteristic(RX, 0);
                tx_characteristic = new BluetoothGattCharacteristic(TX, 0);
            }
        }

        public void step()
        {
            if (delayedState >= 0 && state == 0)
            {
                state = delayedState;
            }
            if (state == 3 && deviceToConnect != null && !mismatch)
            {
                deviceConnected = deviceToConnect;
            }
        }
    }

    private static final class FakeActivity extends Activity
    {
        public Object master;
    }

    private static FakeActivity install(FakeMaster master)
    {
        FakeActivity activity = new FakeActivity();
        activity.master = master;
        OtaBridge.install(activity, new android.widget.LinearLayout(activity));
        return activity;
    }

    private static BluetoothDevice device(String mac)
    {
        return new BluetoothDevice(mac);
    }

    private static void connectedDeviceCapture() throws Exception
    {
        FakeMaster master = new FakeMaster();
        BluetoothDevice watch = device(MAC);
        master.deviceConnected = watch;
        install(master);
        check(OtaBridge.connectedDevice() == null);
        master.state = 3;
        check(OtaBridge.connectedDevice() == watch);
        check(master.connects == 0);
        master.state = 2;
        check(OtaBridge.connectedDevice() == null);
        master.state = 3;
        master.deviceConnected = null;
        check(OtaBridge.connectedDevice() == null);
        OtaBridge.install(null, new android.widget.LinearLayout(new FakeActivity()));
        check(OtaBridge.connectedDevice() == null);
    }

    private static void restoreSucceeds() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        BluetoothDevice watch = device(MAC);
        master.settle(3, true);
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(watch, () -> false, 3000);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        long deadline = System.currentTimeMillis() + 3000;
        while (worker.isAlive() && System.currentTimeMillis() < deadline)
        {
            master.step();
            Thread.sleep(10);
        }
        worker.join(1000);
        check(!worker.isAlive());
        check(failure[0] == null);
        check(master.connects == 1);
        check(master.disconnects == 0);
        check(master.state == 3);
        check(!master.toShowUUIDDialog);
        check(master.rx_characteristic != null);
        check(master.tx_characteristic != null);
        check(master.deviceConnected == watch);
    }

    private static void restoreRejectsMainThread() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(device(MAC), () -> false, 100);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        }, "main");
        worker.start();
        worker.join(1000);
        check(failure[0] instanceof IllegalStateException);
        check(master.connects == 0);
    }

    private static void restoreCancelBeforeConnect() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(device(MAC), () -> true, 1000);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        worker.join(1000);
        check(failure[0] instanceof IOException);
        check(master.connects == 0);
    }

    private static void restoreCancelDuringWait() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        BluetoothGatt gatt = new BluetoothGatt();
        master.gatt = gatt;
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(device(MAC), () -> master.connects > 0, 3000);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        worker.join(3000);
        check(!worker.isAlive());
        check(failure[0] instanceof IOException);
        check(master.connects == 1);
        check(master.disconnects > 0);
        check(gatt.disconnects > 0);
        check(gatt.closes > 0);
    }

    private static void restoreTimeout() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(device(MAC), () -> false, 250);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        worker.join(3000);
        check(!worker.isAlive());
        check(failure[0] instanceof IOException);
        check(master.disconnects > 0);
    }

    private static void restoreRequiresRxTx() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        BluetoothDevice watch = device(MAC);
        master.settle(3, false);
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(watch, () -> false, 250);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        long deadline = System.currentTimeMillis() + 2000;
        while (worker.isAlive() && System.currentTimeMillis() < deadline)
        {
            master.step();
            Thread.sleep(10);
        }
        worker.join(1000);
        check(failure[0] instanceof IOException);
        check(master.disconnects > 0);
    }

    private static void restoreRejectsWrongDevice() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        BluetoothDevice watch = device(MAC);
        master.settle(3, true);
        master.mismatch = true;
        master.deviceConnected = device("11:22:33:44:55:66");
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(watch, () -> false, 250);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        long deadline = System.currentTimeMillis() + 2000;
        while (worker.isAlive() && System.currentTimeMillis() < deadline)
        {
            master.step();
            Thread.sleep(10);
        }
        worker.join(1000);
        check(failure[0] instanceof IOException);
    }

    private static void restoreHonorsInterrupt() throws Exception
    {
        FakeMaster master = new FakeMaster();
        install(master);
        BluetoothGatt gatt = new BluetoothGatt();
        master.gatt = gatt;
        final Throwable[] failure = new Throwable[1];
        Thread worker = new Thread(() ->
        {
            try
            {
                OtaBridge.restoreConnection(device(MAC), () -> false, 5000);
            }
            catch (Throwable error)
            {
                failure[0] = error;
            }
        });
        worker.start();
        long deadline = System.currentTimeMillis() + 1000;
        while (master.connects == 0 && System.currentTimeMillis() < deadline)
        {
            Thread.sleep(10);
        }
        worker.interrupt();
        worker.join(3000);
        check(!worker.isAlive());
        check(failure[0] instanceof IOException);
        check(gatt.closes > 0);
    }

    public static void main(String[] args) throws Exception
    {
        connectedDeviceCapture();
        restoreSucceeds();
        restoreRejectsMainThread();
        restoreCancelBeforeConnect();
        restoreCancelDuringWait();
        restoreTimeout();
        restoreRequiresRxTx();
        restoreRejectsWrongDevice();
        restoreHonorsInterrupt();
        System.out.println("OtaBridgeTest: OK");
    }
}
