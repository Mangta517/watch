package dev.mangta.wave;

public final class GgaAltitudeTest
{
    private static String sentence(String body)
    {
        int checksum = 0;
        for (int i = 0; i < body.length(); ++i)
        {
            checksum ^= body.charAt(i);
        }
        return "$" + body + String.format("*%02X", checksum) + "\r\n";
    }

    private static void check(boolean condition)
    {
        if (!condition)
        {
            throw new AssertionError();
        }
    }

    public static void main(String[] args)
    {
        String fix = "GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-50.4,M,46.9,M,,";
        check(GgaAltitude.parse(sentence(fix)) == -50.4);
        check(GgaAltitude.diagnose(sentence(fix)) == GgaAltitude.STATUS_OK);
        check(GgaAltitude.parse(sentence(fix.replace(",M,", ",m,"))) == -50.4);
        check(Double.isNaN(GgaAltitude.parse(sentence(fix.replace(",1,08,", ",0,08,")))));
        check(GgaAltitude.diagnose(sentence(fix.replace(",1,08,", ",0,08,"))) == GgaAltitude.STATUS_NO_FIX);
        check(Double.isNaN(GgaAltitude.parse(sentence(fix.replace(",1,08,", ",6,08,")))));
        check(Double.isNaN(GgaAltitude.parse(sentence(fix.replace(",46.9,", ",,")))));
        check(GgaAltitude.diagnose(sentence(fix.replace(",46.9,", ",,"))) == GgaAltitude.STATUS_NO_GEOID);
        check(Double.isNaN(GgaAltitude.parse(sentence(fix.replace(",-50.4,M,", ",NaN,M,")))));
        check(GgaAltitude.diagnose(sentence(fix.replace(",-50.4,M,", ",NaN,M,"))) == GgaAltitude.STATUS_INVALID_VALUE);
        check(Double.isNaN(GgaAltitude.parse(sentence(fix.replace(",-50.4,M,", ",9001,M,")))));
        check(Double.isNaN(GgaAltitude.parse(sentence(fix).replace(",-50.4,", ",-51.4,"))));
        check(GgaAltitude.diagnose(sentence(fix).replace(",-50.4,", ",-51.4,")) == GgaAltitude.STATUS_BAD_CHECKSUM);
        check(GgaAltitude.diagnose("$GNRMC,123519,A,4807.038,N,01131.000,E,0.0,0.0,230394,,,A*7F") == GgaAltitude.STATUS_NOT_GGA);
        System.out.println("GgaAltitudeTest: OK");
    }
}
