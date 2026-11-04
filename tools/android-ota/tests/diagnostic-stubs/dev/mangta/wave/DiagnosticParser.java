package dev.mangta.wave;

import java.io.IOException;
import java.util.Arrays;

/**
 * 原APP dev.mangta.wave.DiagnosticParser的受控契约替身，只用于主机测试。
 * 它不是WDG1解析实现：真正的CRC帧过滤在原始APK的smali类中，本替身只验证
 * 适配器是否按原接口传入正确字节、长度和单调毫秒时间，并原样返回脚本结果。
 */
public final class DiagnosticParser
{
    public static byte[] expectedInput;
    public static byte[] response;
    public static boolean fail;
    public static int calls;
    public static int instances;
    public static byte[] capturedInput;
    public static int capturedLength = -1;
    public static long capturedTimeMs = -1;
    public static long expectedTimeMinMs = Long.MIN_VALUE;
    public static long expectedTimeMaxMs = Long.MAX_VALUE;

    public DiagnosticParser()
    {
        ++instances;
    }

    public synchronized byte[] feed(byte[] value, int length, long timeMs) throws IOException
    {
        ++calls;
        capturedInput = value;
        capturedLength = length;
        capturedTimeMs = timeMs;
        if (expectedInput != null && !Arrays.equals(expectedInput, value))
        {
            throw new AssertionError("诊断输入字节不一致");
        }
        if (length != value.length)
        {
            throw new AssertionError("诊断输入长度不一致");
        }
        if (timeMs < expectedTimeMinMs || timeMs > expectedTimeMaxMs)
        {
            throw new AssertionError("诊断时间戳不在预期区间");
        }
        if (fail)
        {
            throw new IOException("原诊断解析器失败");
        }
        return response;
    }
}
