package dev.mangta.wave;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/** 用假时钟、独立假BLE和假SPP链路验证真实一键OTA协调流程，不依赖手机或真实无线。 */
public final class OtaUpgradeTest
{
    private static final String TIMEOUT = "!timeout";
    private static final String READY = "Flash erased, ready to receive data...";
    private static final int PINNED_TARGET = 7;
    // 与固件application/app_ui.c的UI_BT_ECO_QUIET_MS一致：普通APP静默1秒即断电。
    private static final int ECO_QUIET_MS = 1000;

    private static void check(boolean value, String message)
    {
        if (!value)
        {
            throw new AssertionError(message);
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

    private static final class Link implements OtaUpgrade.Connection
    {
        final int target = PINNED_TARGET;
        final boolean spp;
        final ArrayDeque<String> lines = new ArrayDeque<>();
        final ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        Actions actions;
        boolean open;
        boolean failOpen;
        boolean failStartWrite;
        boolean cancelOnLayout;
        boolean cancelOnTimeout;
        // 普通APP静默1秒后固件ECO会直接断电；模拟该窗口，读等待不得跨过。
        boolean ecoCutoff;
        int ecoCutoffs;
        int opens;
        int closes;
        int infoWrites;
        int startWrites;
        int sendStartWrites;
        int payloadBytes;

        Link(boolean spp, String... script)
        {
            this.spp = spp;
            for (String line : script)
            {
                lines.add(line);
            }
        }

        @Override
        public void open(int timeoutMs) throws IOException
        {
            if (spp)
            {
                check(timeoutMs > 0 && timeoutMs <= 60000, "SPP open timeout out of range");
            }
            else
            {
                check(timeoutMs > 0 && timeoutMs <= 5000, "BLE open timeout out of range");
            }
            ++opens;
            if (failOpen)
            {
                throw new IOException("链路打开失败");
            }
            open = true;
        }

        @Override
        public void close()
        {
            if (open)
            {
                open = false;
                ++closes;
            }
        }

        @Override
        public void write(byte[] data, int offset, int length) throws IOException
        {
            String text = new String(data, offset, length, StandardCharsets.US_ASCII);
            if (text.equals("OTA,INFO\n"))
            {
                ++infoWrites;
            }
            else if (text.startsWith("OTA,START"))
            {
                if (failStartWrite)
                {
                    throw new IOException("BLE连接断开");
                }
                ++startWrites;
            }
            else if (text.startsWith("start:"))
            {
                ++sendStartWrites;
            }
            else
            {
                payloadBytes += length;
            }
            bytes.write(data, offset, length);
        }

        @Override
        public String readLine(int timeoutMs) throws IOException
        {
            check(timeoutMs > 0, "read timeout must be positive");
            // START被受理后app_ota_service已busy并保持蓝牙供电，ECO只影响START之前的静默期。
            if (ecoCutoff && startWrites == 0 && timeoutMs >= ECO_QUIET_MS)
            {
                // 等满ECO静默窗口就会掉电；这里返回IO而不是普通读超时，暴露协调器探测过长。
                ++ecoCutoffs;
                open = false;
                throw new IOException("BLE已进入ECO断电");
            }
            if (lines.isEmpty())
            {
                if (cancelOnTimeout)
                {
                    actions.cancelled = true;
                }
                throw new SocketTimeoutException("普通APP无应答");
            }
            String line = lines.removeFirst();
            if (line.equals(TIMEOUT))
            {
                if (cancelOnTimeout)
                {
                    actions.cancelled = true;
                }
                throw new SocketTimeoutException("普通APP无应答");
            }
            if (line.equals(OtaProtocol.LAYOUT) && cancelOnLayout)
            {
                actions.cancelled = true;
            }
            return line;
        }

        @Override
        public void pause(int milliseconds) throws IOException
        {
            actions.pause(milliseconds);
        }
    }

    private static final class Actions implements OtaUpgrade.Actions
    {
        final ArrayDeque<Link> pendingBle = new ArrayDeque<>();
        final ArrayDeque<Link> pendingSpp = new ArrayDeque<>();
        final List<Link> createdBle = new ArrayList<>();
        final List<Link> createdSpp = new ArrayList<>();
        final List<String> stages = new ArrayList<>();
        long now;
        boolean cancelled;
        boolean restoreFailure;
        int cancelAfterPause;
        int cancelAtPause;
        int releases;
        int restores;
        int pauses;
        long restoreAtMs = -1;
        int createRecoveryAfterBleCloseViolation;

        Actions(Link... script)
        {
            for (Link link : script)
            {
                if (link.spp)
                {
                    pendingSpp.add(link);
                }
                else
                {
                    pendingBle.add(link);
                }
            }
        }

        @Override
        public OtaUpgrade.Connection create()
        {
            Link link = pendingBle.removeFirst();
            link.actions = this;
            createdBle.add(link);
            return link;
        }

        @Override
        public OtaUpgrade.Connection createRecovery()
        {
            // 只要仍有BLE链路未关闭，就记录违约；正文绝不允许从BLE继续。
            for (Link link : createdBle)
            {
                if (link.open)
                {
                    ++createRecoveryAfterBleCloseViolation;
                }
            }
            Link link = pendingSpp.removeFirst();
            link.actions = this;
            createdSpp.add(link);
            return link;
        }

        @Override
        public void release()
        {
            ++releases;
        }

        @Override
        public void restore() throws Exception
        {
            ++restores;
            restoreAtMs = now;
            for (Link link : createdBle)
            {
                check(!link.open, "restore called before BLE link closed");
            }
            for (Link link : createdSpp)
            {
                check(!link.open, "restore called before SPP link closed");
            }
            if (restoreFailure)
            {
                throw new IOException("原APP连接已断开");
            }
        }

        @Override
        public boolean cancelled()
        {
            return cancelled;
        }

        @Override
        public long nowMs()
        {
            return now;
        }

        @Override
        public void pause(int milliseconds)
        {
            now += milliseconds;
            ++pauses;
            if (cancelAfterPause > 0 && pauses >= cancelAfterPause)
            {
                cancelled = true;
            }
            if (cancelAtPause > 0 && pauses == cancelAtPause)
            {
                cancelled = true;
            }
        }

        @Override
        public void stage(String text)
        {
            stages.add(text);
        }
    }

    private static OtaProtocol.Progress progress()
    {
        return (sent, total, stage) ->
        {
        };
    }

    private static void expectFailure(OtaUpgrade.Actions actions, byte[] data, String message)
    {
        try
        {
            OtaUpgrade.run(actions, data, progress());
            throw new AssertionError(message);
        }
        catch (AssertionError failed)
        {
            throw failed;
        }
        catch (Exception expected)
        {
        }
    }

    private static void checkBodyOnlyOnSpp(Actions actions, Link spp, String message)
    {
        for (Link link : actions.createdBle)
        {
            check(link.sendStartWrites == 0 && link.payloadBytes == 0, message + "：BLE不得承载正文");
        }
        check(spp.sendStartWrites == 1, message + "：SPP必须发送一次正文start");
    }

    private static void normalThenRecovery() throws Exception
    {
        Link oldApp = new Link(false, TIMEOUT, "OTA,REBOOTING,SPP");
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(oldApp, recovery);
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result == null, "successful restore must return null");
        check(actions.createdBle.size() == 1 && actions.createdSpp.size() == 1, "normal APP must open BLE then SPP");
        check(actions.createRecoveryAfterBleCloseViolation == 0, "BLE must close before SPP recovery");
        check(oldApp.startWrites == 1, "OTA,START must be sent exactly once on BLE");
        checkBodyOnlyOnSpp(actions, recovery, "normal APP");
        check(oldApp.infoWrites == 1 && recovery.infoWrites == 2, "probe before OTA and before send");
        check(oldApp.closes == 1 && recovery.closes == 1, "both links must close");
        check(actions.releases == 1 && actions.restores == 1, "release and restore must each run once");
        check(recovery.lines.isEmpty(), "all scripted recovery replies must be consumed");
        check(actions.restoreAtMs >= 1000, "restore must wait out the 500ms APP jump window");
        check(actions.stages.contains("等待手表启动APP"), "waiting stage must be reported before restore");
    }

    private static void alreadyRecovery() throws Exception
    {
        Link control = new Link(false, OtaProtocol.LAYOUT);
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(control, recovery);
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result == null, "already recovery must succeed");
        check(actions.createdBle.size() == 1 && actions.createdSpp.size() == 1, "already recovery still needs SPP");
        check(actions.createRecoveryAfterBleCloseViolation == 0, "already recovery must close BLE before SPP");
        check(control.startWrites == 0, "already recovery must not send OTA,START");
        checkBodyOnlyOnSpp(actions, recovery, "already recovery");
        check(actions.restores == 1, "restore must run after ACK");
        check(control.closes == 1 && recovery.closes == 1, "both links must close before restore");
        check(actions.restoreAtMs == 1000, "already recovery must skip reset wait but keep APP wait");
    }

    private static void lostAckButLayoutProof() throws Exception
    {
        Link oldApp = new Link(false, TIMEOUT);
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(oldApp, recovery);
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result == null, "layout proof after lost ACK must succeed");
        check(actions.createdBle.size() == 1 && actions.createdSpp.size() == 1, "lost ACK must fall through to SPP");
        check(actions.createRecoveryAfterBleCloseViolation == 0, "lost ACK must close BLE before SPP");
        check(oldApp.startWrites == 1, "OTA,START must not repeat on reconnect");
        checkBodyOnlyOnSpp(actions, recovery, "lost ACK");
        check(recovery.infoWrites == 2, "SPP reconnect must probe strictly");
        check(actions.restores == 1, "restore must run after confirmed firmware");
    }

    private static void failedBleOpenFallsBack() throws Exception
    {
        Link deadBle = new Link(false);
        deadBle.failOpen = true;
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(deadBle, recovery);
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result == null, "failed BLE open must fall through to SPP");
        check(actions.createdBle.size() == 1 && actions.createdSpp.size() == 1, "BLE failure must try SPP once");
        check(deadBle.startWrites == 0, "failed BLE open must not send OTA,START");
        checkBodyOnlyOnSpp(actions, recovery, "BLE open failure");
        check(deadBle.closes == 0, "unopened BLE link has nothing to close");
        check(actions.stages.contains("等待手表重启并重连升级通道"), "fallback must keep reset wait");
    }

    private static void writeFailureThenRecovery() throws Exception
    {
        Link oldApp = new Link(false, TIMEOUT);
        oldApp.failStartWrite = true;
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(oldApp, recovery);
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result == null, "write failure during request may still recover");
        check(actions.createdBle.size() == 1 && actions.createdSpp.size() == 1, "write failure must fall through to SPP");
        check(oldApp.startWrites == 0, "no OTA,START retry after write failure");
        checkBodyOnlyOnSpp(actions, recovery, "START write failure");
    }

    private static void explicitRejections() throws Exception
    {
        String[] lines = {"OTA,EEPROM_ERROR", "OTA,UNAVAILABLE", "OTA,LAYOUT,08008000,00058000"};
        for (String line : lines)
        {
            Link rejected = new Link(false, line);
            Actions actions = new Actions(rejected);
            expectFailure(actions, image(101), "device rejection must fail: " + line);
            check(actions.releases == 1, "release must run before probe");
            check(actions.createdBle.size() == 1 && actions.createdSpp.isEmpty(), "rejection must not reach SPP");
            check(rejected.startWrites == 0, "rejection must not send OTA,START");
            check(rejected.sendStartWrites == 0 && rejected.payloadBytes == 0, "rejection must not send firmware");
            check(actions.restores == 0, "rejection must not restore");
            check(rejected.closes == 1, "rejected link must close");
        }
    }

    private static void invalidImageBeforeRelease() throws Exception
    {
        Actions actions = new Actions();
        expectFailure(actions, new byte[8], "invalid image must fail before release");
        check(actions.releases == 0 && actions.createdBle.isEmpty() && actions.createdSpp.isEmpty() && actions.restores == 0,
            "invalid image must not touch the old connection");
        Actions canceled = new Actions();
        canceled.cancelled = true;
        expectFailure(canceled, image(101), "cancel before release must fail");
        check(canceled.releases == 0 && canceled.createdBle.isEmpty(), "cancel must stop before release");
    }

    private static void canceledReconnect() throws Exception
    {
        Link oldApp = new Link(false, TIMEOUT, "OTA,REBOOTING,SPP");
        Actions actions = new Actions(oldApp);
        actions.cancelAfterPause = 1;
        expectFailure(actions, image(101), "cancel during reconnect must fail");
        check(actions.createdBle.size() == 1 && actions.createdSpp.isEmpty(), "cancel during reconnect must stop before SPP");
        check(actions.restores == 0, "cancel before ACK must not restore");
        check(oldApp.closes == 1, "old link must close before waiting");
        check(oldApp.sendStartWrites == 0 && oldApp.payloadBytes == 0, "cancel before ACK must not send firmware");
    }

    private static void canceledBeforeSpp() throws Exception
    {
        Link control = new Link(false, OtaProtocol.LAYOUT);
        Link recovery = new Link(true, OtaProtocol.LAYOUT);
        recovery.cancelOnLayout = true;
        Actions actions = new Actions(control, recovery);
        expectFailure(actions, image(101), "cancel after SPP recovery must fail");
        check(actions.createRecoveryAfterBleCloseViolation == 0, "cancel path must still close BLE before SPP");
        check(recovery.infoWrites == 1, "probe may run once before cancellation");
        check(recovery.startWrites == 0 && recovery.sendStartWrites == 0 && recovery.payloadBytes == 0,
            "cancel must prevent firmware body");
        check(actions.restores == 0, "cancel before ACK must not restore");
        check(control.closes == 1 && recovery.closes == 1, "canceled links must close");
    }

    private static void failedTransferNoRetry() throws Exception
    {
        Link control = new Link(false, OtaProtocol.LAYOUT);
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:1");
        Actions actions = new Actions(control, recovery);
        expectFailure(actions, image(101), "wrong ACK must fail");
        check(actions.createdSpp.size() == 1, "transfer failure must not reconnect");
        check(recovery.sendStartWrites == 1, "firmware start must not be resent");
        check(actions.restores == 0, "failed firmware must not restore");
        check(recovery.closes == 1, "failed link must close");
    }

    private static void restoreFailureKeepsFirmwareSuccess() throws Exception
    {
        Link control = new Link(false, OtaProtocol.LAYOUT);
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(control, recovery);
        actions.restoreFailure = true;
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result != null, "restore failure must return a message");
        check(result.contains("升级已完成") && result.contains("恢复"), "message must report firmware success and restore failure");
        check(actions.restores == 1, "restore must be attempted once");
        check(recovery.closes == 1, "OTA link must close before restore");
        check(recovery.lines.isEmpty(), "firmware ACK must be consumed before restore");
        check(actions.restoreAtMs >= 1000, "restore attempt must wait out the APP jump window");
    }

    private static void canceledDuringAppStartWait() throws Exception
    {
        Link control = new Link(false, OtaProtocol.LAYOUT);
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(control, recovery);
        // 第1次pause是ACK后的等待分片；取消后应保留固件成功并跳过恢复。
        actions.cancelAtPause = 1;
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result != null, "cancel during APP start wait must return a warning");
        check(result.contains("升级已完成") && result.contains("取消"), "warning must preserve firmware success");
        check(actions.restores == 0, "cancel during APP start wait must skip restore");
        check(recovery.closes == 1, "OTA link must close before the APP start wait");
        check(actions.now < 1000, "cancel must stop the wait promptly");
    }

    private static void cancelDuringProbeTimeoutBlocksStart() throws Exception
    {
        Link oldApp = new Link(false);
        oldApp.cancelOnTimeout = true;
        Actions actions = new Actions(oldApp);
        expectFailure(actions, image(101), "cancel during probe timeout must fail");
        check(oldApp.startWrites == 0, "no OTA,START after cancel during probe");
        check(actions.createdBle.size() == 1 && actions.createdSpp.isEmpty(), "cancel during probe must not reach SPP");
        check(actions.restores == 0, "cancel before ACK must not restore");
        check(oldApp.sendStartWrites == 0 && oldApp.payloadBytes == 0, "cancel during probe must not send firmware");
        check(oldApp.closes == 1, "canceled link must close");
    }

    private static void normalAppProbeSurvivesEco() throws Exception
    {
        Link oldApp = new Link(false, TIMEOUT);
        oldApp.ecoCutoff = true;
        Link recovery = new Link(true, OtaProtocol.LAYOUT, OtaProtocol.LAYOUT, READY, "ok,len:101");
        Actions actions = new Actions(oldApp, recovery);
        String result = OtaUpgrade.run(actions, image(101), progress());
        check(result == null, "normal APP must finish OTA before ECO cuts BLE");
        // 初次探测必须短于1秒ECO窗口，否则START还没发出就被断电。
        check(oldApp.ecoCutoffs == 0, "initial BLE probe must stay below the 1s ECO quiet window");
        check(oldApp.startWrites == 1, "OTA,START must be sent exactly once on BLE");
        check(actions.createdBle.size() == 1 && actions.createdSpp.size() == 1, "ECO case must still use SPP body");
        check(actions.createRecoveryAfterBleCloseViolation == 0, "ECO case must close BLE before SPP");
        checkBodyOnlyOnSpp(actions, recovery, "normal APP under ECO");
        check(actions.restores == 1, "ECO case must restore after ACK");
    }

    public static void main(String[] args) throws Exception
    {
        normalThenRecovery();
        alreadyRecovery();
        lostAckButLayoutProof();
        failedBleOpenFallsBack();
        writeFailureThenRecovery();
        explicitRejections();
        invalidImageBeforeRelease();
        canceledReconnect();
        canceledBeforeSpp();
        failedTransferNoRetry();
        restoreFailureKeepsFirmwareSuccess();
        canceledDuringAppStartWait();
        cancelDuringProbeTimeoutBlocksStart();
        normalAppProbeSurvivesEco();
        System.out.println("PASS: BLE control then SPP body, close-before-SPP, lost ACK, BLE open fallback, strict rejection, "
            + "cancel before SPP, no resend, restore failure, close-before-restore, APP start wait, probe-timeout cancel "
            + "and ECO-safe initial BLE probe");
    }
}
