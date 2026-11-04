package dev.mangta.wave;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/**
 * 用受控契约替身验证反射适配器的调用契约。
 * 注意：替身不是WDG1解析器，本测试不证明CRC校验或真实帧剥离；
 * 那部分属于原始APK内的DiagnosticParser，需要在整机环境验证。
 */
public final class OtaDiagnosticFilterTest
{
    private static void check(boolean value)
    {
        if (!value)
        {
            throw new AssertionError();
        }
    }

    private static void reset(byte[] expected, byte[] response, boolean fail)
    {
        DiagnosticParser.expectedInput = expected;
        DiagnosticParser.response = response;
        DiagnosticParser.fail = fail;
        DiagnosticParser.calls = 0;
        DiagnosticParser.capturedInput = null;
        DiagnosticParser.capturedLength = -1;
        DiagnosticParser.capturedTimeMs = -1;
        DiagnosticParser.expectedTimeMinMs = Long.MIN_VALUE;
        DiagnosticParser.expectedTimeMaxMs = Long.MAX_VALUE;
    }

    public static void main(String[] args) throws Exception
    {
        // 原解析器剥离WDG1后只剩可读文本：适配器必须原样返回它。
        byte[] wdg = new byte[]{(byte)0xA5, 0x5A, 0x01, 0x02, 0x03};
        byte[] clean = "OTA,LAYOUT,08010000,00050000\n".getBytes(StandardCharsets.US_ASCII);
        reset(wdg, clean, false);
        OtaDiagnosticFilter filter = new OtaDiagnosticFilter();
        long before = System.nanoTime() / 1000000L;
        DiagnosticParser.expectedTimeMinMs = before;
        DiagnosticParser.expectedTimeMaxMs = before + 5000;
        byte[] filtered = filter.feed(wdg);
        long after = System.nanoTime() / 1000000L;
        check(DiagnosticParser.calls == 1);
        check(Arrays.equals(clean, filtered));
        check(Arrays.equals(wdg, DiagnosticParser.capturedInput));
        check(DiagnosticParser.capturedLength == wdg.length);
        check(DiagnosticParser.capturedTimeMs >= before && DiagnosticParser.capturedTimeMs <= after);

        // null不进入原解析器。
        check(filter.feed(null) == null);
        check(DiagnosticParser.calls == 1);

        // 每个适配器各自构造解析器实例，连接之间不共享状态。
        DiagnosticParser.instances = 0;
        reset(wdg, clean, false);
        OtaDiagnosticFilter first = new OtaDiagnosticFilter();
        OtaDiagnosticFilter second = new OtaDiagnosticFilter();
        check(DiagnosticParser.instances == 2);
        first.feed(wdg);
        second.feed(wdg);
        check(DiagnosticParser.calls == 2);

        // 原解析器抛出的IOException必须原样上抛，不能吞掉后继续当文本处理。
        reset(wdg, clean, true);
        try
        {
            filter.feed(wdg);
            throw new AssertionError("诊断过滤失败被吞掉");
        }
        catch (IOException expected)
        {
            check("原诊断解析器失败".equals(expected.getMessage()));
        }
        System.out.println("PASS: diagnostic filter reflection contract, per-instance parser, failure propagation "
            + "(WDG1 CRC parser itself is external to this stub)");
    }
}
