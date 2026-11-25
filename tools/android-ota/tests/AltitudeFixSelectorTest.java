package dev.mangta.wave;

public final class AltitudeFixSelectorTest
{
    private static void check(boolean condition)
    {
        if (!condition)
        {
            throw new AssertionError();
        }
    }

    private static String gga(double altitudeM)
    {
        String body = "GNGGA,123519,3114.000,N,12128.000,E,1,10,0.9," + altitudeM + ",M,11.2,M,,";
        int checksum = 0;
        for (int i = 0; i < body.length(); ++i)
        {
            checksum ^= body.charAt(i);
        }
        return "$" + body + String.format("*%02X", checksum) + "\r\n";
    }

    private static String ggaNoGeoid()
    {
        String body = "GNGGA,123519,3114.000,N,12128.000,E,1,10,0.9,112.0,M,,M,,";
        int checksum = 0;
        for (int i = 0; i < body.length(); ++i)
        {
            checksum ^= body.charAt(i);
        }
        return "$" + body + String.format("*%02X", checksum) + "\r\n";
    }

    private static AltitudeFixSelector.Sample<String> sample(String id, long fixElapsedMs, long receivedElapsedMs,
        long fixTimeMs, float hAcc, boolean hasVAcc, float vAcc, boolean hasSpeed, float speed)
    {
        return new AltitudeFixSelector.Sample<>(id, fixElapsedMs, receivedElapsedMs, fixTimeMs, false, true,
            true, hAcc, hasVAcc, vAcc, hasSpeed, speed);
    }

    private static void stationaryWithoutSpeedSucceedsImmediately()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        AltitudeFixSelector.Selected<String> selected = selector.onLocation(
            sample("fast-no-speed", 9900L, 10000L, 1700000010000L, 6.0f, true, 9.0f, false, Float.NaN), 10000L);
        check(selected != null);
        check("fast-no-speed".equals(selected.sample.payload));
        check(AltitudeFixSelector.formatSuccess(112, selected.sample, 9.4f).contains("垂直±9m"));
    }

    private static void movingFixRejectedWithClearReason()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        AltitudeFixSelector.Selected<String> selected = selector.onLocation(
            sample("moving", 9900L, 10000L, 1700000010000L, 6.0f, true, 8.0f, true, 3.2f), 10000L);
        check(selected == null);
        check(selector.onTimeout(40000L) == null);
        check(selector.timeoutMessage().contains("检测到移动(3.2m/s)"));
    }

    private static void candidateConvergesAndCommitsBestFreshFix()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        // 首次进入候选区（H8m V18m）：不立刻发，给收敛窗口。
        check(selector.onLocation(sample("c1", 9900L, 10000L, 1700000010000L, 8.0f, true, 18.0f, false, Float.NaN), 10000L) == null);
        check(selector.statusLabel().equals("收敛中 H8m V18m"));
        check(selector.remainingConvergeMs(10000L) == 6000L);

        // 期间出现更好的候选（V14m）。
        check(selector.onLocation(sample("c2", 12900L, 13000L, 1700000013000L, 7.0f, true, 14.0f, true, 0.2f), 13000L) == null);
        // 后续出现略差样本（V19m）不应覆盖更优的 c2。
        check(selector.onLocation(sample("c3", 14900L, 15000L, 1700000015000L, 7.0f, true, 19.0f, true, 0.1f), 15000L) == null);
        // 到 6 秒收敛定时器触发时，c2 仍新鲜（16000 - 12900 = 3100 <= 8000），应返回 c2。
        AltitudeFixSelector.Selected<String> ready = selector.onConvergeTimer(16000L);
        check(ready != null);
        check("c2".equals(ready.sample.payload));
    }

    private static void fastFixInterruptsConvergeImmediately()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        check(selector.onLocation(sample("c1", 9900L, 10000L, 1700000010000L, 9.0f, true, 19.0f, false, Float.NaN), 10000L) == null);
        AltitudeFixSelector.Selected<String> fast = selector.onLocation(
            sample("fast", 11900L, 12000L, 1700000012000L, 5.0f, true, 11.5f, false, Float.NaN), 12000L);
        check(fast != null);
        check("fast".equals(fast.sample.payload));
    }

    private static void missingVerticalAccuracyUsesConvergeLane()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        check(selector.onLocation(sample("no-v1", 9900L, 10000L, 1700000010000L, 12.0f, false, Float.NaN, false, Float.NaN), 10000L) == null);
        check(selector.statusLabel().equals("收敛中 H12m V--"));
        AltitudeFixSelector.Selected<String> ready = selector.onConvergeTimer(16000L);
        check(ready != null);
        check("no-v1".equals(ready.sample.payload));
        check(AltitudeFixSelector.formatSuccess(88, ready.sample, Float.NaN).contains("水平±12m"));
    }

    private static void nmeaPairsByMonotonicTimeDespiteLeapSecondTimestampOffset()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(true);
        // Location先到，waiting GGA。
        check(selector.onLocation(
            sample("api31", 9900L, 10000L, 1700000010000L, 6.0f, true, 10.0f, false, Float.NaN), 10000L) == null);
        check(selector.statusLabel().equals("等GGA H6m V10m"));

        // GGA后到50ms，但厂商timestamp比UTC快18秒（GPS时间），仍必须靠单调接收时间成功配对！
        AltitudeFixSelector.Selected<String> paired = selector.onNmea(gga(115.6), 1700000028000L, 10050L);
        check(paired != null);
        check("api31".equals(paired.sample.payload));
        check(paired.nmeaAltitudeM == 115.6);
    }

    private static void nmeaArrivingBeforeLocationAlsoPairs()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(true);
        // NMEA先到且timestamp为0。
        check(selector.onNmea(gga(42.5), 0L, 10000L) == null);
        AltitudeFixSelector.Selected<String> paired = selector.onLocation(
            sample("after-nmea", 10150L, 10200L, 1700000010200L, 5.0f, true, 9.0f, false, Float.NaN), 10200L);
        check(paired != null);
        check("after-nmea".equals(paired.sample.payload));
        check(paired.nmeaAltitudeM == 42.5);
    }

    private static void diagnosticsDistinguishRootCauses()
    {
        // 1. 完全无星
        AltitudeFixSelector<String> noFix = new AltitudeFixSelector<>(false);
        check(noFix.timeoutMessage().contains("未收到GPS卫星定位"));

        // 2. 精度超限，报告观测到的最佳水平/垂直精度
        AltitudeFixSelector<String> lowAcc = new AltitudeFixSelector<>(false);
        check(lowAcc.onLocation(sample("bad1", 9900L, 10000L, 1700000010000L, 28.0f, true, 42.0f, false, Float.NaN), 10000L) == null);
        check(lowAcc.onLocation(sample("bad2", 10900L, 11000L, 1700000011000L, 16.0f, true, 31.0f, false, Float.NaN), 11000L) == null);
        String msg = lowAcc.timeoutMessage();
        check(msg.contains("水平16m") && msg.contains("垂直31m"));

        // 3. Android 8-13 精度达标但没收到 GGA
        AltitudeFixSelector<String> noGga = new AltitudeFixSelector<>(true);
        check(noGga.onLocation(sample("ok", 9900L, 10000L, 1700000010000L, 6.0f, true, 10.0f, false, Float.NaN), 10000L) == null);
        check(noGga.timeoutMessage().contains("未输出NMEA GGA"));

        // 4. Android 8-13 收到缺 geoid 的 GGA
        AltitudeFixSelector<String> noGeoid = new AltitudeFixSelector<>(true);
        check(noGeoid.onLocation(sample("ok", 9900L, 10000L, 1700000010000L, 6.0f, true, 10.0f, false, Float.NaN), 10000L) == null);
        check(noGeoid.onNmea(ggaNoGeoid(), 0L, 10050L) == null);
        check(noGeoid.timeoutMessage().contains("缺少大地水准面数据"));
    }

    private static void staleCandidateIsNotReturned()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        // 收到时已经 2.5s 老，6 秒后变成 8.5s 老（超过 8s 新鲜上限），不得提交。
        check(selector.onLocation(sample("old", 7500L, 10000L, 1700000007500L, 8.0f, true, 18.0f, false, Float.NaN), 10000L) == null);
        check(selector.onConvergeTimer(16000L) == null);
        check(selector.onTimeout(16000L) == null);
    }

    private static void movementRestartsConvergence()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
        check(selector.onLocation(sample("before", 10000L, 10000L, 0L, 8.0f, true, 14.0f, true, 0.0f), 10000L) == null);
        for (long now = 11000L; now <= 15000L; now += 1000L)
        {
            check(selector.onLocation(sample("moving", now, now, 0L, 8.0f, true, 18.0f, true, 4.0f), now) == null);
        }
        check(selector.onLocation(sample("delayed-before", 14900L, 15500L, 0L, 8.0f, true, 9.0f, true, 0.0f), 15500L) == null);
        check(selector.onConvergeTimer(16000L) == null);
        check(selector.onTimeout(16000L) == null);
        check(selector.remainingConvergeMs(16000L) == -1L);
        check(selector.onLocation(sample("after", 17000L, 17000L, 0L, 8.0f, true, 19.0f, true, 0.0f), 17000L) == null);
        check(selector.remainingConvergeMs(17000L) == 6000L);
        check("after".equals(selector.onConvergeTimer(23000L).sample.payload));
    }

    private static void expiredBestFallsBackToFreshPairedCandidate()
    {
        AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(true);
        check(selector.onNmea(gga(111.0), 0L, 10000L) == null);
        check(selector.onLocation(sample("old-best", 7500L, 10000L, 0L, 8.0f, true, 14.0f, false, Float.NaN), 10000L) == null);
        check(selector.onNmea(gga(122.0), 0L, 14500L) == null);
        check(selector.onLocation(sample("fresh", 14500L, 14500L, 0L, 8.0f, true, 18.0f, false, Float.NaN), 14500L) == null);
        AltitudeFixSelector.Selected<String> ready = selector.onConvergeTimer(16000L);
        check(ready != null && "fresh".equals(ready.sample.payload) && ready.nmeaAltitudeM == 122.0);
        check(selector.onTimeout(16000L) == ready);
    }

    private static void progressivelyWorseAccuracyDoesNotReplaceBest()
    {
        for (boolean hasVAcc : new boolean[] {true, false})
        {
            AltitudeFixSelector<String> selector = new AltitudeFixSelector<>(false);
            for (int i = 0; i < 6; ++i)
            {
                long now = 10000L + 1000L * i;
                check(selector.onLocation(sample("fix-" + i, now, now, 0L, 8.0f + i, hasVAcc, 14.0f + i, false, Float.NaN), now) == null);
            }
            check("fix-0".equals(selector.onConvergeTimer(16000L).sample.payload));
        }
    }

    public static void main(String[] args)
    {
        stationaryWithoutSpeedSucceedsImmediately();
        movingFixRejectedWithClearReason();
        candidateConvergesAndCommitsBestFreshFix();
        fastFixInterruptsConvergeImmediately();
        missingVerticalAccuracyUsesConvergeLane();
        nmeaPairsByMonotonicTimeDespiteLeapSecondTimestampOffset();
        nmeaArrivingBeforeLocationAlsoPairs();
        diagnosticsDistinguishRootCauses();
        staleCandidateIsNotReturned();
        movementRestartsConvergence();
        expiredBestFallsBackToFreshPairedCandidate();
        progressivelyWorseAccuracyDoesNotReplaceBest();
        System.out.println("AltitudeFixSelectorTest: OK");
    }
}
