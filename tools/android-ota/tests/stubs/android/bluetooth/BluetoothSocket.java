package android.bluetooth;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

/** 主机测试用的最小RFCOMM socket：脚本化connect阻塞、输入输出和关闭唤醒。 */
public class BluetoothSocket
{
    public UUID uuid;
    public boolean connectCalled;
    public boolean closed;
    public boolean connected;
    public int connects;
    public int closes;
    public volatile InputStream input;
    public volatile OutputStream output;
    public volatile Runnable connectAction;
    public volatile Runnable closeAction;

    public BluetoothSocket()
    {
        this.input = new ByteArrayInputStream(new byte[0]);
        this.output = new ByteArrayOutputStream();
    }

    public BluetoothSocket(InputStream input, OutputStream output)
    {
        this.input = input;
        this.output = output;
    }

    public UUID getUuid()
    {
        return uuid;
    }

    public boolean isConnected()
    {
        return connected && !closed;
    }

    public InputStream getInputStream() throws IOException
    {
        if (closed)
        {
            throw new IOException("socket已关闭");
        }
        return input;
    }

    public OutputStream getOutputStream() throws IOException
    {
        if (closed)
        {
            throw new IOException("socket已关闭");
        }
        return output;
    }

    public void connect() throws IOException
    {
        ++connects;
        connectCalled = true;
        if (connectAction != null)
        {
            connectAction.run();
        }
        if (closed)
        {
            throw new IOException("socket已关闭");
        }
        connected = true;
    }

    public void close() throws IOException
    {
        ++closes;
        closed = true;
        connected = false;
        Runnable action = closeAction;
        if (action != null)
        {
            action.run();
        }
    }
}
