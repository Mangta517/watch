package dev.mangta.wave;

import java.io.IOException;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;

/** 一键OTA协调器：BLE停止原APP并请求重启，关闭BLE后经SPP严格确认保底布局，再只发送一次固件。 */
public final class OtaUpgrade
{
    private static final int PROBE_TIMEOUT_MS = 1500;
    // 普通APP静默时固件1秒无收发就切断蓝牙（UI_BT_ECO_QUIET_MS=1000）；
    // 初次BLE探测必须短于该窗口，否则等不到OTA,START就已被ECO断电。
    private static final int BLE_PROBE_TIMEOUT_MS = 500;
    private static final int OPEN_TIMEOUT_MS = 5000;
    private static final int REBOOT_ACK_TIMEOUT_MS = 3000;
    // SPP升级复位后kt6368a_init要20ms放电+1200ms启动、改SPP 00->01再断电1200ms启动；
    // 旧SPP OtaActivity也是等约5秒。这5秒自动等，避免用户手动等5秒再连。
    private static final int REBOOT_WAIT_MS = 5000;
    // SPP首次配对可能弹系统对话框；总等待必须覆盖用户确认和随后重试。
    private static final int RECOVERY_TIMEOUT_MS = 90000;
    private static final int SPP_OPEN_TIMEOUT_MS = 60000;
    private static final int RETRY_PAUSE_MS = 250;
    private static final int WAIT_SLICE_MS = 100;
    // 固件在最终ok,len后还要500ms才跳APP，跳转时BLE掉电；必须等过这段再恢复原连接。
    private static final int APP_START_WAIT_MS = 1000;

    public interface Connection extends OtaProtocol.Transport
    {
        void open(int timeoutMs) throws java.io.IOException;
        void close();
    }

    public interface Actions
    {
        /** BLE控制链路：用于OTA,INFO探测和唯一的OTA,START请求。 */
        Connection create();
        /** SPP保底链路：只能用于awaitRecovery和固件正文，不能复用BLE。 */
        Connection createRecovery();
        void release() throws Exception;
        void restore() throws Exception;
        boolean cancelled();
        long nowMs();
        void pause(int milliseconds) throws java.io.IOException;
        void stage(String text);
    }

    private OtaUpgrade()
    {
    }

    /**
     * 返回null表示固件ACK已确认且原APP连接恢复成功；非null表示固件已成功但恢复被取消或失败。
     * 固件未确认时抛异常，绝不把恢复结果当成升级结果。
     */
    public static String run(Actions actions, byte[] image, OtaProtocol.Progress progress) throws Exception
    {
        // 先校验BIN长度和向量，避免为无效镜像断开原APP。
        OtaProtocol.command(image);
        if (actions.cancelled())
        {
            throw new IOException("升级已取消");
        }
        actions.stage("停止原APP连接");
        actions.release();
        if (actions.cancelled())
        {
            throw new IOException("升级已取消");
        }
        Connection control = null;
        Connection link = null;
        try
        {
            // 第一阶段只走BLE：探测普通APP，必要时只发一次OTA,START。
            boolean needRecovery = false;
            boolean controlOpen = false;
            try
            {
                control = actions.create();
                control.open(OPEN_TIMEOUT_MS);
                controlOpen = true;
            }
            catch (IOException openFailure)
            {
                // BLE打不开时设备可能已在保底；关闭链路后交给SPP严格布局判断，不重发START。
                needRecovery = true;
            }
            if (controlOpen)
            {
                if (actions.cancelled())
                {
                    throw new IOException("升级已取消");
                }
                OtaProtocol.Probe probe = null;
                boolean normalApp = false;
                try
                {
                    probe = OtaProtocol.probe(control, BLE_PROBE_TIMEOUT_MS);
                }
                catch (SocketTimeoutException normal)
                {
                    // 普通APP不应答OTA,INFO；只有这个类型化读超时才允许发送OTA,START。
                    normalApp = true;
                }
                catch (OtaProtocol.ProbeException rejection)
                {
                    throw rejection;
                }
                catch (IOException disconnected)
                {
                    // 设备可能已自行重启；仍必须由后续SPP严格布局证明保底。
                    needRecovery = true;
                }
                if (normalApp)
                {
                    // 探测超时期间用户可能已取消或线程被中断；不能继续发送会重启手表的请求。
                    if (actions.cancelled())
                    {
                        throw new IOException("升级已取消");
                    }
                    if (Thread.currentThread().isInterrupted())
                    {
                        throw new InterruptedException("升级等待被中断");
                    }
                    actions.stage("请求手表进入升级模式");
                    byte[] start = "OTA,START\n".getBytes(StandardCharsets.US_ASCII);
                    try
                    {
                        control.write(start, 0, start.length);
                        OtaProtocol.awaitReboot(control, REBOOT_ACK_TIMEOUT_MS);
                    }
                    catch (OtaProtocol.ProbeException rejection)
                    {
                        throw rejection;
                    }
                    catch (IOException lost)
                    {
                        // 回执前后都可能复位；不以写失败或ACK丢失判断成功。
                    }
                    needRecovery = true;
                }
                else if (probe == OtaProtocol.Probe.REBOOTING)
                {
                    needRecovery = true;
                }
            }
            // 任何进入保底的路径都先彻底关闭BLE控制链路，正文绝不复用这条连接。
            if (control != null)
            {
                control.close();
                control = null;
            }
            if (needRecovery)
            {
                actions.stage("等待手表重启并重连升级通道");
                // 已确认保底时不会进入本分支；只有请求重启、自行重启或BLE打不开才等复位窗口。
                wait(actions, REBOOT_WAIT_MS);
            }
            if (actions.cancelled())
            {
                throw new IOException("升级已取消");
            }
            // 保底探测和正文始终使用SPP，即使初始BLE已看到RECOVERY也不复用BLE。
            link = awaitRecovery(actions);
            if (actions.cancelled())
            {
                throw new IOException("升级已取消");
            }
            // 固件正文只发送一次；协调器不自动重发。
            OtaProtocol.send(link, image, progress);
        }
        finally
        {
            if (control != null)
            {
                control.close();
            }
            if (link != null)
            {
                link.close();
            }
        }
        // 固件ACK已确认、OTA链路已关闭；等过跳APP的500ms掉电窗口再恢复原连接。
        actions.stage("等待手表启动APP");
        try
        {
            wait(actions, APP_START_WAIT_MS);
            if (actions.cancelled())
            {
                return "升级已完成，但已取消自动恢复原APP连接，请手动重新连接手表。";
            }
            actions.stage("恢复原APP连接");
            actions.restore();
            return null;
        }
        catch (Exception failure)
        {
            // 等待期间取消/中断或恢复失败都不能把已确认的固件成功改判为失败。
            if (actions.cancelled())
            {
                return "升级已完成，但已取消自动恢复原APP连接，请手动重新连接手表。";
            }
            if (failure instanceof InterruptedException)
            {
                return "升级已完成，但自动恢复原APP连接被中断，请手动重新连接手表。";
            }
            String detail = failure.getMessage();
            if (detail == null || detail.length() == 0)
            {
                detail = failure.getClass().getSimpleName();
            }
            return "升级已完成，但恢复原APP连接失败：" + detail + "，请手动重新连接手表。";
        }
    }

    /**
     * 在总截止时间内反复建立SPP连接，只以完整布局作为进入保底的证据。
     * 单次open取剩余时间与60秒的较小值，覆盖首次配对对话框；重试只发生在正文之前。
     */
    private static Connection awaitRecovery(Actions actions) throws Exception
    {
        long deadline = actions.nowMs() + RECOVERY_TIMEOUT_MS;
        while (true)
        {
            if (actions.cancelled())
            {
                throw new IOException("升级已取消");
            }
            if (Thread.currentThread().isInterrupted())
            {
                throw new InterruptedException("升级等待被中断");
            }
            long remaining = deadline - actions.nowMs();
            if (remaining <= 0)
            {
                throw new IOException("等待手表进入SPP升级模式超时，请重试");
            }
            int openTimeout = (int)Math.min(SPP_OPEN_TIMEOUT_MS, remaining);
            Connection link = actions.createRecovery();
            boolean recovered = false;
            try
            {
                link.open(openTimeout);
                long probeRemaining = deadline - actions.nowMs();
                if (probeRemaining <= 0)
                {
                    throw new SocketTimeoutException("等待手表进入SPP升级模式超时");
                }
                int probeTimeout = (int)Math.min(PROBE_TIMEOUT_MS, probeRemaining);
                OtaProtocol.Probe probe = OtaProtocol.probe(link, probeTimeout);
                if (probe == OtaProtocol.Probe.RECOVERY)
                {
                    recovered = true;
                    return link;
                }
            }
            catch (OtaProtocol.ProbeException rejection)
            {
                // 明确拒绝或布局不符不能靠重试掩盖。
                throw rejection;
            }
            catch (SocketTimeoutException notReady)
            {
                // 普通APP或重启尚未完成，继续等待。
            }
            catch (IOException disconnected)
            {
                // 配对未完成或设备仍在重启，继续等待。
            }
            finally
            {
                if (!recovered)
                {
                    link.close();
                }
            }
            wait(actions, RETRY_PAUSE_MS);
        }
    }

    /** 分片等待，确保取消和线程中断都能及时结束。 */
    private static void wait(Actions actions, int milliseconds) throws IOException, InterruptedException
    {
        long deadline = actions.nowMs() + milliseconds;
        while (true)
        {
            if (actions.cancelled())
            {
                throw new IOException("升级已取消");
            }
            if (Thread.currentThread().isInterrupted())
            {
                throw new InterruptedException("升级等待被中断");
            }
            long remaining = deadline - actions.nowMs();
            if (remaining <= 0)
            {
                return;
            }
            int slice = (int)Math.min(WAIT_SLICE_MS, remaining);
            actions.pause(slice);
        }
    }
}
