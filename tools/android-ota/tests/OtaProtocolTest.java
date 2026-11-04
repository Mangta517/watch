package dev.mangta.wave;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.zip.CRC32;

public final class OtaProtocolTest
{
    private static void check(boolean value)
    {
        if (!value)
        {
            throw new AssertionError();
        }
    }

    private static byte[] image(int length)
    {
        byte[] data = new byte[length];
        for (int i = 0; i < length; ++i)
        {
            data[i] = (byte)(i * 37 + 11);
        }
        byte[] vectors = {0, 0, 2, 0x20, 9, 0, 1, 8};
        System.arraycopy(vectors, 0, data, 0, 8);
        return data;
    }

    private static final class Fake implements OtaProtocol.Transport
    {
        final ByteArrayOutputStream payload = new ByteArrayOutputStream();
        byte[] command;
        boolean ready, reject, mismatch, oldLayout;
        int reads, pauses, writes;

        @Override
        public void write(byte[] data, int offset, int length)
        {
            if (writes == 0)
            {
                check(new String(data, offset, length, StandardCharsets.US_ASCII).equals("OTA,INFO\n"));
            }
            else if (command == null)
            {
                command = Arrays.copyOfRange(data, offset, offset + length);
            }
            else
            {
                check(ready && length <= 256);
                payload.write(data, offset, length);
            }
            ++writes;
        }

        @Override
        public String readLine(int timeout) throws IOException
        {
            check(timeout > 0 && timeout <= 60001);
            ++reads;
            if (reject)
            {
                return "app rec error,system reset";
            }
            if (reads == 1)
            {
                if (oldLayout)
                {
                    return "OTA,LAYOUT,08008000,00058000";
                }
                return "OTA,LAYOUT,08010000,00050000";
            }
            if (!ready)
            {
                check(payload.size() == 0);
                if (reads == 2)
                {
                    return "Received start command";
                }
                ready = true;
                return "Flash erased, ready to receive data...";
            }
            if (mismatch)
            {
                mismatch = false;
                return "ok,len:1";
            }
            return "ok,len:" + payload.size();
        }

        @Override
        public void pause(int ms)
        {
            check(ms == 40);
            ++pauses;
        }
    }

    private static final class Scripted implements OtaProtocol.Transport
    {
        final ArrayDeque<String> lines = new ArrayDeque<>();
        int writes;

        Scripted(String... script)
        {
            for (String line : script)
            {
                lines.add(line);
            }
        }

        @Override
        public void write(byte[] data, int offset, int length)
        {
            check(new String(data, offset, length, StandardCharsets.US_ASCII).equals("OTA,INFO\n"));
            ++writes;
        }

        @Override
        public String readLine(int timeout) throws IOException
        {
            check(timeout > 0);
            if (lines.isEmpty())
            {
                throw new SocketTimeoutException("普通APP无应答");
            }
            return lines.removeFirst();
        }

        @Override
        public void pause(int ms)
        {
            check(ms > 0);
        }
    }

    public static void main(String[] args) throws Exception
    {
        for (int length : new int[]{101, 512, 10001, 327680})
        {
            byte[] data = image(length);
            Fake fake = new Fake();
            fake.mismatch = true;
            int[] completed = {0};
            OtaProtocol.send(fake, data, (sent, total, stage) ->
            {
                if (stage.startsWith("设备已确认"))
                {
                    ++completed[0];
                }
            });
            CRC32 crc = new CRC32();
            crc.update(data);
            check(new String(fake.command, StandardCharsets.US_ASCII).equals(String.format(java.util.Locale.ROOT,
                "start:%d:%08x\n", length, crc.getValue())));
            check(Arrays.equals(data, fake.payload.toByteArray()) && completed[0] == 1);
            // 40ms只出现在数据块之间，最后一包后不再暂停，避免丢掉紧跟的ok,len:N。
            check(fake.pauses == (length + 255) / 256 - 1 && fake.reads == 5);
        }
        for (byte[] bad : new byte[][]{new byte[0], new byte[8], new byte[327681]})
        {
            Fake fake = new Fake();
            try
            {
                OtaProtocol.send(fake, bad, (sent, total, stage) ->
                {
                });
                throw new AssertionError("invalid image accepted");
            }
            catch (IOException expected)
            {
                check(fake.command == null);
            }
        }
        Fake fake = new Fake();
        fake.reject = true;
        try
        {
            OtaProtocol.send(fake, image(101), (sent, total, stage) ->
            {
            });
            throw new AssertionError("device error accepted");
        }
        catch (IOException expected)
        {
            check(fake.payload.size() == 0);
        }
        Fake old = new Fake();
        old.oldLayout = true;
        try
        {
            OtaProtocol.send(old, image(101), (sent, total, stage) ->
            {
            });
            throw new AssertionError("old Flash layout accepted");
        }
        catch (IOException expected)
        {
            check(old.command == null && old.writes == 1 && old.payload.size() == 0);
        }

        // 普通APP的日常诊断行在截止前忽略，随后仍能确认保底布局。
        Scripted normal = new Scripted("battery 87%", "heart 72", OtaProtocol.LAYOUT);
        check(OtaProtocol.probe(normal, 1000) == OtaProtocol.Probe.RECOVERY && normal.writes == 1);
        Scripted rebooting = new Scripted("battery 87%", "OTA,REBOOTING,BLE");
        check(OtaProtocol.probe(rebooting, 1000) == OtaProtocol.Probe.REBOOTING);
        // 非OTA行之后没有布局时，读超时仍是普通APP，不能被诊断行掩盖。
        Scripted silent = new Scripted("battery 87%");
        try
        {
            OtaProtocol.probe(silent, 1000);
            throw new AssertionError("diagnostics alone accepted as recovery");
        }
        catch (SocketTimeoutException expected)
        {
        }
        // 明确的OTA错误和错误布局不能被诊断忽略逻辑吞掉。
        for (String rejection : new String[]{"OTA,EEPROM_ERROR", "OTA,UNAVAILABLE", "OTA,LAYOUT,08008000,00058000",
            "OTA,WHAT"})
        {
            try
            {
                OtaProtocol.probe(new Scripted("battery 87%", rejection), 1000);
                throw new AssertionError("rejection accepted: " + rejection);
            }
            catch (OtaProtocol.ProbeException expected)
            {
            }
        }
        // START后的回执等待：日常诊断忽略，SPP/BLE两种REBOOTING都可确认；明确拒绝立即抛出。
        Scripted rebootAck = new Scripted("battery 87%", "OTA,REBOOTING,SPP");
        check(OtaProtocol.awaitReboot(rebootAck, 1000));
        // 旧APP只会回BLE后缀；新SPP保底仍必须能识别，升级流程不能只认一种。
        Scripted legacyAck = new Scripted("battery 87%", "OTA,REBOOTING,BLE");
        check(OtaProtocol.awaitReboot(legacyAck, 1000));
        Scripted rebootMissing = new Scripted("battery 87%");
        check(!OtaProtocol.awaitReboot(rebootMissing, 1000));
        try
        {
            OtaProtocol.awaitReboot(new Scripted("OTA,UNAVAILABLE"), 1000);
            throw new AssertionError("reboot rejection accepted");
        }
        catch (OtaProtocol.ProbeException expected)
        {
        }
        try
        {
            OtaProtocol.awaitReboot(new Scripted("OTA,LAYOUT,08008000,00058000"), 1000);
            throw new AssertionError("reboot layout mismatch accepted");
        }
        catch (OtaProtocol.ProbeException expected)
        {
        }
        System.out.println("PASS: phone CRC/vectors/length, ready-before-BIN, exact maximum/odd payload, pacing, strict ack and device errors, "
            + "probe diagnostics and typed timeout");
    }
}
