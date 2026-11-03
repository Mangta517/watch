package dev.mangta.wave;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.zip.CRC32;

/** F411 Bootloader字节流协议；SPP分包不改变BIN内容。 */
public final class OtaProtocol
{
    public static final int MAX_IMAGE = 0x50000;

    public interface Transport
    {
        void write(byte[] data, int offset, int length) throws IOException;
        String readLine(int timeoutMs) throws IOException;
        void pause(int milliseconds) throws IOException;
    }

    public interface Progress
    {
        void update(int sent, int total, String stage);
    }

    private OtaProtocol()
    {
    }

    private static long word(byte[] image, int offset)
    {
        long value = 0;
        for (int i = 0; i < 4; ++i)
        {
            value |= (long)(image[offset + i] & 255) << (i * 8);
        }
        return value;
    }

    public static String command(byte[] image) throws IOException
    {
        if (image.length < 8 || image.length > MAX_IMAGE)
        {
            throw new IOException("BIN长度必须为8～327680字节");
        }
        long stack = word(image, 0);
        long reset = word(image, 4);
        if (stack <= 0x20000000L || stack > 0x20020000L || (stack & 7) != 0 ||
            (reset & 1) == 0 || (reset & ~1L) < 0x08010008L || (reset & ~1L) >= 0x08010000L + image.length)
        {
            throw new IOException("请选择链接到0x08010000的升级APP BIN，不能使用普通APP/HEX/AXF");
        }
        CRC32 crc = new CRC32();
        crc.update(image);
        return String.format(Locale.ROOT, "start:%d:%08x\n", image.length, crc.getValue());
    }

    private static void expect(Transport transport, String expected, int timeoutMs) throws IOException
    {
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        while (true)
        {
            long remaining = (deadline - System.nanoTime()) / 1000000;
            if (remaining <= 0)
            {
                throw new IOException("等待设备回执超时：" + expected);
            }
            String line = transport.readLine((int)Math.min(remaining + 1, Integer.MAX_VALUE)).trim();
            if (line.startsWith("app rec error") || line.startsWith("Invalid") || line.startsWith("Warning: legacy"))
            {
                throw new IOException("Bootloader拒绝升级：" + line);
            }
            if (expected.startsWith("OTA,LAYOUT,") && line.startsWith("OTA,LAYOUT,") && !line.equals(expected))
            {
                throw new IOException("设备Flash分区不匹配，请先部署本版主引导和独立保底程序");
            }
            if (line.equals(expected))
            {
                return;
            }
        }
    }

    public static void send(Transport transport, byte[] image, Progress progress) throws IOException
    {
        byte[] start = command(image).getBytes(StandardCharsets.US_ASCII);
        byte[] query = "OTA,INFO\n".getBytes(StandardCharsets.US_ASCII);
        progress.update(0, image.length, "确认独立保底程序的Flash分区");
        transport.write(query, 0, query.length);
        expect(transport, "OTA,LAYOUT,08010000,00050000", 10000);
        progress.update(0, image.length, "等待Bootloader擦除完成");
        transport.write(start, 0, start.length);
        expect(transport, "Flash erased, ready to receive data...", 60000);
        for (int offset = 0; offset < image.length; offset += 256)
        {
            int length = Math.min(256, image.length - offset);
            transport.write(image, offset, length);
            progress.update(offset + length, image.length, "发送固件");
            // 256B/40ms低于115200 UART吞吐，给模块缓存和MCU写Flash留处理时间。
            transport.pause(40);
        }
        progress.update(image.length, image.length, "等待长度、CRC和向量校验");
        expect(transport, "ok,len:" + image.length, 60000);
        progress.update(image.length, image.length, "设备已确认升级，正在启动APP；随后重新连接BLE");
    }
}
