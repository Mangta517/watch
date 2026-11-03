package dev.mangta.wave;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
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
            check(fake.pauses == (length + 255) / 256 && fake.reads == 5);
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
        System.out.println("PASS: phone CRC/vectors/length, ready-before-BIN, exact maximum/odd payload, pacing, strict ack and device errors");
    }
}
