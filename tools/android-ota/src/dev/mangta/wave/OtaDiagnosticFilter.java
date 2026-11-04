package dev.mangta.wave;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * 过滤普通APP持续输出的WDG1二进制诊断，避免其撑爆OTA文本回执行缓冲。
 * 帧解析复用原APK内的dev.mangta.wave.DiagnosticParser；OTA模块只用android.jar编译，
 * 不能直接引用原APK的类，因此通过反射调用其public feed(byte[], int, long)。
 * 每个连接各自持有一个解析器实例，不共享跨连接的帧状态。
 */
final class OtaDiagnosticFilter
{
    private final Object parser;
    private final Method feed;

    OtaDiagnosticFilter()
    {
        Object created = null;
        Method method = null;
        try
        {
            Class<?> type = Class.forName("dev.mangta.wave.DiagnosticParser");
            Constructor<?> constructor = type.getConstructor();
            created = constructor.newInstance();
            method = type.getMethod("feed", byte[].class, int.class, long.class);
        }
        catch (ClassNotFoundException missing)
        {
            // 旧基座APK没有诊断解析器：保持原文本通路，现有主机测试也走这里。
            created = null;
            method = null;
        }
        catch (ReflectiveOperationException failure)
        {
            throw new IllegalStateException("无法初始化原APP诊断解析器", failure);
        }
        catch (RuntimeException failure)
        {
            throw new IllegalStateException("无法初始化原APP诊断解析器", failure);
        }
        parser = created;
        feed = method;
    }

    /**
     * 返回可继续交给文本行解析的字节，CRC有效的WDG1帧由原解析器剥离。
     * 原解析器失败必须向上报告，不能把诊断二进制当文本放行。
     */
    public byte[] feed(byte[] value) throws IOException
    {
        if (value == null)
        {
            return null;
        }
        if (parser == null)
        {
            return value;
        }
        long nowMs = System.nanoTime() / 1000000L;
        try
        {
            return (byte[])feed.invoke(parser, value, value.length, nowMs);
        }
        catch (InvocationTargetException failure)
        {
            Throwable cause = failure.getCause();
            if (cause instanceof IOException)
            {
                throw (IOException)cause;
            }
            throw new IOException("诊断过滤失败", cause);
        }
        catch (IllegalAccessException failure)
        {
            throw new IOException("无法调用原APP诊断解析器", failure);
        }
    }
}
