package dev.mangta.wave;

import java.io.IOException;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.zip.CRC32;

/** F411 Bootloader字节流协议；BLE分包不改变BIN内容。 */
public final class OtaProtocol
{
    public static final int MAX_IMAGE = 0x50000;
    public static final String LAYOUT = "OTA,LAYOUT,08010000,00050000";

    /** OTA,INFO查询结果：保底布局或正在重启。普通APP以读超时表示。 */
    public enum Probe
    {
        RECOVERY,
        REBOOTING
    }

    /** 设备明确拒绝或布局不符；调用方不得把它当成普通APP超时继续。 */
    public static final class ProbeException extends IOException
    {
        public ProbeException(String message)
        {
            super(message);
        }
    }

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

    private static void checkDeviceError(String line) throws ProbeException
    {
        if (line.startsWith("OTA,EEPROM_ERROR"))
        {
            throw new ProbeException("手表EEPROM写入失败，APP未被擦除");
        }
        if (line.startsWith("OTA,UNAVAILABLE"))
        {
            throw new ProbeException("当前手表APP不支持BLE OTA，请先通过SWD部署本版保底程序");
        }
        if (line.startsWith("app rec error") || line.startsWith("Invalid") || line.startsWith("Warning: legacy"))
        {
            throw new ProbeException("Bootloader拒绝升级：" + line);
        }
    }

    /**
     * 查询设备是否已在保底。普通APP可能持续输出日常诊断，非OTA行在截止前忽略；
     * 布局不符和明确错误立即抛出，只有读超时表示普通APP无应答。
     */
    public static Probe probe(Transport transport, int timeoutMs) throws IOException
    {
        byte[] query = "OTA,INFO\n".getBytes(StandardCharsets.US_ASCII);
        transport.write(query, 0, query.length);
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        while (true)
        {
            long remaining = (deadline - System.nanoTime()) / 1000000L;
            if (remaining <= 0)
            {
                throw new SocketTimeoutException("等待设备OTA,INFO回执超时");
            }
            String line = transport.readLine((int)Math.min(remaining + 1, Integer.MAX_VALUE)).trim();
            if (line.length() == 0)
            {
                continue;
            }
            if (line.equals(LAYOUT))
            {
                return Probe.RECOVERY;
            }
            if (line.startsWith("OTA,LAYOUT,"))
            {
                throw new ProbeException("设备Flash分区不匹配，请先部署本版主引导和独立保底程序");
            }
            checkDeviceError(line);
            if (line.startsWith("OTA,REBOOTING"))
            {
                return Probe.REBOOTING;
            }
            if (!line.startsWith("OTA,"))
            {
                continue;
            }
            throw new ProbeException("设备响应不符合OTA查询：" + line);
        }
    }

    /**
     * 发送OTA,START后等待设备明确受理。收到新保底的OTA,REBOOTING,SPP或旧APP的
     * OTA,REBOOTING,BLE都返回true；读超时或断开返回false，由后续严格布局探测决定是否继续。
     */
    public static boolean awaitReboot(Transport transport, int timeoutMs) throws IOException
    {
        long deadline = System.nanoTime() + timeoutMs * 1000000L;
        while (true)
        {
            long remaining = (deadline - System.nanoTime()) / 1000000L;
            if (remaining <= 0)
            {
                return false;
            }
            String line;
            try
            {
                line = transport.readLine((int)Math.min(remaining + 1, Integer.MAX_VALUE)).trim();
            }
            catch (ProbeException rejection)
            {
                throw rejection;
            }
            catch (SocketTimeoutException timeout)
            {
                return false;
            }
            catch (IOException disconnected)
            {
                // 设备可能在回执前后复位；随后只以严格布局作为进入保底的证据。
                return false;
            }
            if (line.length() == 0)
            {
                continue;
            }
            if (line.equals("OTA,REBOOTING,SPP") || line.equals("OTA,REBOOTING,BLE"))
            {
                return true;
            }
            if (line.equals(LAYOUT))
            {
                return false;
            }
            if (line.startsWith("OTA,LAYOUT,"))
            {
                throw new ProbeException("设备Flash分区不匹配，请先部署本版主引导和独立保底程序");
            }
            checkDeviceError(line);
        }
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
            String line;
            try
            {
                line = transport.readLine((int)Math.min(remaining + 1, Integer.MAX_VALUE)).trim();
            }
            catch (SocketTimeoutException timeout)
            {
                throw new IOException("等待设备回执超时：" + expected, timeout);
            }
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
        expect(transport, LAYOUT, 10000);
        progress.update(0, image.length, "等待Bootloader擦除完成");
        transport.write(start, 0, start.length);
        expect(transport, "Flash erased, ready to receive data...", 60000);
        boolean finalWriteFailed = false;
        for (int offset = 0; offset < image.length; offset += 256)
        {
            int length = Math.min(256, image.length - offset);
            boolean last = offset + length >= image.length;
            try
            {
                transport.write(image, offset, length);
            }
            catch (IOException failure)
            {
                if (!last)
                {
                    throw failure;
                }
                // 设备可能在最后一包的onCharacteristicWrite前断开；ok,len:N若已到达仍可确认完成。
                finalWriteFailed = true;
            }
            progress.update(offset + length, image.length, "发送固件");
            if (!last)
            {
                // 256B/40ms不是实际BLE吞吐；20B包由transport按写入回执和全局间隔控制。
                transport.pause(40);
            }
        }
        progress.update(image.length, image.length, "等待长度、CRC和向量校验");
        if (finalWriteFailed)
        {
            progress.update(image.length, image.length, "最后一包写入未确认，核对设备ok,len回执");
        }
        // 唯一的ACK读取：最后一包失败时也不能放宽，必须精确匹配ok,len:N。
        expect(transport, "ok,len:" + image.length, 60000);
        progress.update(image.length, image.length, "设备已确认升级，正在启动APP；随后重新连接BLE");
    }
}
