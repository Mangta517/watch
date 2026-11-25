package dev.mangta.wave;

import java.util.ArrayList;

/** 筛选并择优手机GNSS定位与NMEA GGA海拔，避免静止无speed或厂商NMEA时间戳差异导致误拒。 */
final class AltitudeFixSelector<T>
{
    static final float FAST_HORIZONTAL_ACCURACY_M = 15.0f;
    static final float FAST_VERTICAL_ACCURACY_M = 12.0f;
    static final float MAX_HORIZONTAL_ACCURACY_M = 20.0f;
    static final float MAX_VERTICAL_ACCURACY_M = 25.0f;
    static final float NO_VACCURACY_MAX_HORIZONTAL_M = 15.0f;
    static final float MAX_SPEED_MPS = 2.0f;
    static final float MAX_MSL_ACCURACY_M = 28.0f;
    static final long MIN_LIVE_FIX_AGE_MS = -500L;
    static final long MAX_LIVE_FIX_AGE_MS = 3000L;
    static final long MAX_CANDIDATE_AGE_MS = 8000L;
    static final long CONVERGE_WAIT_MS = 6000L;
    static final long GGA_PAIR_WINDOW_MS = 2500L;
    static final long MAX_GGA_AGE_MS = 3000L;

    static final class Sample<T>
    {
        final T payload;
        final long fixElapsedMs;
        final long receivedElapsedMs;
        final long fixTimeMs;
        final boolean mock;
        final boolean hasAltitude;
        final boolean hasAccuracy;
        final float accuracyM;
        final boolean hasVerticalAccuracy;
        final float verticalAccuracyM;
        final boolean hasSpeed;
        final float speedMps;

        Sample(T payload, long fixElapsedMs, long receivedElapsedMs, long fixTimeMs, boolean mock, boolean hasAltitude,
            boolean hasAccuracy, float accuracyM, boolean hasVerticalAccuracy, float verticalAccuracyM,
            boolean hasSpeed, float speedMps)
        {
            this.payload = payload;
            this.fixElapsedMs = fixElapsedMs;
            this.receivedElapsedMs = receivedElapsedMs;
            this.fixTimeMs = fixTimeMs;
            this.mock = mock;
            this.hasAltitude = hasAltitude;
            this.hasAccuracy = hasAccuracy;
            this.accuracyM = accuracyM;
            this.hasVerticalAccuracy = hasVerticalAccuracy;
            this.verticalAccuracyM = verticalAccuracyM;
            this.hasSpeed = hasSpeed;
            this.speedMps = speedMps;
        }

        boolean validVerticalAccuracy()
        {
            return hasVerticalAccuracy && Float.isFinite(verticalAccuracyM) && verticalAccuracyM > 0.0f;
        }
    }

    static final class Selected<T>
    {
        final Sample<T> sample;
        final double nmeaAltitudeM;

        Selected(Sample<T> sample, double nmeaAltitudeM)
        {
            this.sample = sample;
            this.nmeaAltitudeM = nmeaAltitudeM;
        }
    }

    private final boolean requireNmea;
    private int fixCount;
    private Sample<T> lastSample;
    private Sample<T> pendingEligible;
    private final ArrayList<Selected<T>> candidates = new ArrayList<>();
    private long candidateStartElapsedMs = -1L;
    private long lastMovingFixElapsedMs = Long.MIN_VALUE;

    private double latestGgaAltitudeM = Double.NaN;
    private long latestGgaReceivedElapsedMs = Long.MIN_VALUE;
    private long latestGgaTimestampMs;
    private int ggaCount;
    private boolean ggaOkSeen;
    private boolean ggaNoGeoidSeen;

    private boolean mockSeen;
    private boolean noAltitudeSeen;
    private boolean speedTooHighSeen;
    private float lastHighSpeedMps;
    private boolean eligibleEverSeen;
    private boolean eligibleWaitingGga;
    private float bestObservedHAcc = Float.POSITIVE_INFINITY;
    private float bestObservedVAcc = Float.POSITIVE_INFINITY;

    AltitudeFixSelector(boolean requireNmea)
    {
        this.requireNmea = requireNmea;
    }

    Selected<T> onLocation(Sample<T> sample, long nowElapsedMs)
    {
        ++fixCount;
        lastSample = sample;
        expireStaleCandidates(nowElapsedMs);

        long ageMs = nowElapsedMs - sample.fixElapsedMs;
        if (ageMs < MIN_LIVE_FIX_AGE_MS || ageMs > MAX_LIVE_FIX_AGE_MS)
        {
            pendingEligible = null;
            return null;
        }
        if (sample.mock)
        {
            mockSeen = true;
            pendingEligible = null;
            return null;
        }
        boolean validHorizontalAccuracy = sample.hasAccuracy && Float.isFinite(sample.accuracyM) && sample.accuracyM > 0.0f;
        if (sample.hasAltitude && validHorizontalAccuracy)
        {
            if (sample.accuracyM < bestObservedHAcc)
            {
                bestObservedHAcc = sample.accuracyM;
            }
            if (sample.validVerticalAccuracy() && sample.verticalAccuracyM < bestObservedVAcc)
            {
                bestObservedVAcc = sample.verticalAccuracyM;
            }
        }

        if (sample.hasSpeed && (!Float.isFinite(sample.speedMps) || sample.speedMps > MAX_SPEED_MPS))
        {
            speedTooHighSeen = true;
            lastHighSpeedMps = sample.speedMps;
            lastMovingFixElapsedMs = Math.max(lastMovingFixElapsedMs, sample.fixElapsedMs);
            candidates.clear();
            candidateStartElapsedMs = -1L;
            pendingEligible = null;
            eligibleWaitingGga = false;
            latestGgaAltitudeM = Double.NaN;
            return null;
        }
        if (!sample.hasAltitude)
        {
            noAltitudeSeen = true;
        }
        if (!sample.hasAltitude || !validHorizontalAccuracy || sample.fixElapsedMs <= lastMovingFixElapsedMs ||
            !isCandidateEligible(sample))
        {
            pendingEligible = null;
            return null;
        }

        eligibleEverSeen = true;
        pendingEligible = sample;
        return evaluate(nowElapsedMs);
    }

    Selected<T> onNmea(String sentence, long timestampMs, long nowElapsedMs)
    {
        int status = GgaAltitude.diagnose(sentence);
        if (status == GgaAltitude.STATUS_NOT_GGA)
        {
            return null;
        }
        ++ggaCount;
        if (status == GgaAltitude.STATUS_NO_GEOID)
        {
            ggaNoGeoidSeen = true;
            return null;
        }
        if (status != GgaAltitude.STATUS_OK)
        {
            return null;
        }
        double altitudeM = GgaAltitude.parse(sentence);
        if (Double.isNaN(altitudeM))
        {
            return null;
        }
        ggaOkSeen = true;
        latestGgaAltitudeM = altitudeM;
        latestGgaReceivedElapsedMs = nowElapsedMs;
        latestGgaTimestampMs = timestampMs;
        return evaluate(nowElapsedMs);
    }

    Selected<T> onConvergeTimer(long nowElapsedMs)
    {
        if (remainingConvergeMs(nowElapsedMs) == 0L)
        {
            return bestCandidate();
        }
        return null;
    }

    Selected<T> onTimeout(long nowElapsedMs)
    {
        expireStaleCandidates(nowElapsedMs);
        return bestCandidate();
    }

    long remainingConvergeMs(long nowElapsedMs)
    {
        expireStaleCandidates(nowElapsedMs);
        if (candidates.isEmpty())
        {
            return -1L;
        }
        return Math.max(0L, CONVERGE_WAIT_MS - (nowElapsedMs - candidateStartElapsedMs));
    }

    String statusLabel()
    {
        if (fixCount == 0 || lastSample == null)
        {
            return "正在搜星，请站定...";
        }
        String accuracy = formatLiveAccuracy(lastSample);
        if (!candidates.isEmpty())
        {
            return "收敛中 " + accuracy;
        }
        if (requireNmea && eligibleWaitingGga)
        {
            return "等GGA " + accuracy;
        }
        return "定位中 " + accuracy;
    }

    String timeoutMessage()
    {
        if (fixCount == 0)
        {
            return "未收到GPS卫星定位，请到室外开阔处站定重试";
        }
        if (mockSeen && bestObservedHAcc == Float.POSITIVE_INFINITY)
        {
            return "检测到模拟定位，请关闭模拟位置后重试";
        }
        if (requireNmea && eligibleEverSeen)
        {
            if (ggaCount == 0)
            {
                return "定位精度已达标但手机未输出NMEA GGA，可在手表手动校准";
            }
            if (!ggaOkSeen && ggaNoGeoidSeen)
            {
                return "NMEA GGA缺少大地水准面数据，可在手表手动校准";
            }
            if (!ggaOkSeen)
            {
                return "NMEA GGA未锁定有效海拔，请到开阔处重试";
            }
            return "定位与NMEA GGA时间未对齐，请站定后重试";
        }
        if (speedTooHighSeen && bestObservedHAcc <= MAX_HORIZONTAL_ACCURACY_M &&
            (bestObservedVAcc <= MAX_VERTICAL_ACCURACY_M || bestObservedVAcc == Float.POSITIVE_INFINITY))
        {
            return "检测到移动(" + formatOneDecimal(lastHighSpeedMps) + "m/s)，请站定后重试";
        }
        if (bestObservedHAcc < Float.POSITIVE_INFINITY)
        {
            if (bestObservedVAcc < Float.POSITIVE_INFINITY)
            {
                return "定位精度不足(最佳 水平" + Math.round(bestObservedHAcc) + "m/垂直" +
                    Math.round(bestObservedVAcc) + "m)，请到开阔处重试";
            }
            return "定位精度不足(最佳 水平" + Math.round(bestObservedHAcc) + "m/无垂直精度)，请到开阔处重试";
        }
        if (noAltitudeSeen)
        {
            return "GPS未输出海拔数据，请到开阔处重试";
        }
        return "定位精度不足，请到开阔处站定后重试";
    }

    static String formatSuccess(long roundedMeters, Sample<?> sample, float mslAccuracyM)
    {
        if (Float.isFinite(mslAccuracyM) && mslAccuracyM > 0.0f)
        {
            return "已发送 " + roundedMeters + " m（垂直±" + Math.round(mslAccuracyM) + "m），请在手表气压页确认";
        }
        if (sample != null && sample.validVerticalAccuracy())
        {
            return "已发送 " + roundedMeters + " m（垂直±" + Math.round(sample.verticalAccuracyM) + "m），请在手表气压页确认";
        }
        if (sample != null && sample.hasAccuracy && Float.isFinite(sample.accuracyM) && sample.accuracyM > 0.0f)
        {
            return "已发送 " + roundedMeters + " m（水平±" + Math.round(sample.accuracyM) + "m），请在手表气压页确认";
        }
        return "已发送 " + roundedMeters + " m，请在手表气压页确认";
    }

    private Selected<T> evaluate(long nowElapsedMs)
    {
        expireStaleCandidates(nowElapsedMs);
        if (pendingEligible != null)
        {
            long ageMs = nowElapsedMs - pendingEligible.fixElapsedMs;
            if (ageMs < MIN_LIVE_FIX_AGE_MS || ageMs > MAX_LIVE_FIX_AGE_MS)
            {
                pendingEligible = null;
            }
            else
            {
                double ggaAlt = matchGga(pendingEligible, nowElapsedMs);
                if (requireNmea && Double.isNaN(ggaAlt))
                {
                    eligibleWaitingGga = true;
                }
                else
                {
                    eligibleWaitingGga = false;
                    Selected<T> paired = new Selected<>(pendingEligible, ggaAlt);
                    if (isFastFix(pendingEligible))
                    {
                        return paired;
                    }
                    if (candidates.isEmpty())
                    {
                        candidateStartElapsedMs = nowElapsedMs;
                    }
                    int lastIndex = candidates.size() - 1;
                    if (lastIndex < 0 || candidates.get(lastIndex).sample != pendingEligible)
                    {
                        candidates.add(paired);
                    }
                    else if (!Double.isNaN(ggaAlt))
                    {
                        candidates.set(lastIndex, paired);
                    }
                }
            }
        }
        return onConvergeTimer(nowElapsedMs);
    }

    private void expireStaleCandidates(long nowElapsedMs)
    {
        candidates.removeIf(candidate -> !isCandidateFresh(candidate.sample, nowElapsedMs));
        if (candidates.isEmpty())
        {
            candidateStartElapsedMs = -1L;
        }
    }

    private Selected<T> bestCandidate()
    {
        Selected<T> best = null;
        for (Selected<T> candidate : candidates)
        {
            if (best == null || isBetterCandidate(candidate.sample, best.sample))
            {
                best = candidate;
            }
        }
        return best;
    }

    private boolean isCandidateFresh(Sample<T> sample, long nowElapsedMs)
    {
        long ageMs = nowElapsedMs - sample.fixElapsedMs;
        return ageMs >= MIN_LIVE_FIX_AGE_MS && ageMs <= MAX_CANDIDATE_AGE_MS;
    }

    private double matchGga(Sample<T> sample, long nowElapsedMs)
    {
        if (Double.isNaN(latestGgaAltitudeM) || latestGgaReceivedElapsedMs == Long.MIN_VALUE)
        {
            return Double.NaN;
        }
        long ggaAgeMs = nowElapsedMs - latestGgaReceivedElapsedMs;
        if (ggaAgeMs < MIN_LIVE_FIX_AGE_MS || ggaAgeMs > MAX_GGA_AGE_MS)
        {
            return Double.NaN;
        }
        if (Math.abs(sample.receivedElapsedMs - latestGgaReceivedElapsedMs) <= GGA_PAIR_WINDOW_MS)
        {
            return latestGgaAltitudeM;
        }
        if (latestGgaTimestampMs > 0L && Math.abs(sample.fixTimeMs - latestGgaTimestampMs) <= GGA_PAIR_WINDOW_MS)
        {
            return latestGgaAltitudeM;
        }
        return Double.NaN;
    }

    private static boolean isFastFix(Sample<?> sample)
    {
        return sample.accuracyM <= FAST_HORIZONTAL_ACCURACY_M &&
            sample.validVerticalAccuracy() &&
            sample.verticalAccuracyM <= FAST_VERTICAL_ACCURACY_M;
    }

    private static boolean isCandidateEligible(Sample<?> sample)
    {
        if (sample.validVerticalAccuracy())
        {
            return sample.accuracyM <= MAX_HORIZONTAL_ACCURACY_M &&
                sample.verticalAccuracyM <= MAX_VERTICAL_ACCURACY_M;
        }
        return sample.accuracyM <= NO_VACCURACY_MAX_HORIZONTAL_M;
    }

    private static boolean isBetterCandidate(Sample<?> incoming, Sample<?> current)
    {
        boolean incomingV = incoming.validVerticalAccuracy();
        boolean currentV = current.validVerticalAccuracy();
        if (incomingV != currentV)
        {
            return incomingV;
        }
        if (incomingV)
        {
            return incoming.verticalAccuracyM < current.verticalAccuracyM ||
                (incoming.verticalAccuracyM == current.verticalAccuracyM && incoming.fixElapsedMs > current.fixElapsedMs);
        }
        return incoming.accuracyM < current.accuracyM ||
            (incoming.accuracyM == current.accuracyM && incoming.fixElapsedMs > current.fixElapsedMs);
    }

    private static String formatLiveAccuracy(Sample<?> sample)
    {
        String horizontal = sample.hasAccuracy && Float.isFinite(sample.accuracyM) && sample.accuracyM > 0.0f
            ? "H" + Math.round(sample.accuracyM) + "m" : "H--";
        String vertical = sample.validVerticalAccuracy()
            ? "V" + Math.round(sample.verticalAccuracyM) + "m" : "V--";
        return horizontal + " " + vertical;
    }

    private static String formatOneDecimal(float value)
    {
        if (!Float.isFinite(value))
        {
            return "--";
        }
        long tenths = Math.round(value * 10.0f);
        return (tenths / 10L) + "." + Math.abs(tenths % 10L);
    }
}
