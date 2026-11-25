package dev.mangta.wave;

/** Android 8–13从接收机的GGA读取正高，椭球高不参与BLE校准。 */
final class GgaAltitude
{
    static final int STATUS_NOT_GGA = 0;
    static final int STATUS_BAD_CHECKSUM = 1;
    static final int STATUS_NO_FIX = 2;
    static final int STATUS_NO_GEOID = 3;
    static final int STATUS_INVALID_VALUE = 4;
    static final int STATUS_OK = 5;

    static int diagnose(String sentence)
    {
        if (sentence == null)
        {
            return STATUS_NOT_GGA;
        }
        String line = sentence.trim();
        int star = line.indexOf('*');
        if (star < 7 || line.length() != star + 3 || line.charAt(0) != '$' || !line.startsWith("GGA,", 3))
        {
            return STATUS_NOT_GGA;
        }
        int checksum = 0;
        for (int i = 1; i < star; ++i)
        {
            checksum ^= line.charAt(i);
        }
        int high = Character.digit(line.charAt(star + 1), 16);
        int low = Character.digit(line.charAt(star + 2), 16);
        if (high < 0 || low < 0 || checksum != high * 16 + low)
        {
            return STATUS_BAD_CHECKSUM;
        }
        String[] fields = line.substring(0, star).split(",", -1);
        if (fields.length < 13 || fields[6].length() != 1 || "1245".indexOf(fields[6].charAt(0)) < 0)
        {
            return STATUS_NO_FIX;
        }
        if (!fields[10].equalsIgnoreCase("M") || fields[11].isEmpty() || !fields[12].equalsIgnoreCase("M"))
        {
            return STATUS_NO_GEOID;
        }
        return Double.isNaN(parse_fields(fields)) ? STATUS_INVALID_VALUE : STATUS_OK;
    }

    static double parse(String sentence)
    {
        if (sentence == null)
        {
            return Double.NaN;
        }
        String line = sentence.trim();
        int star = line.indexOf('*');
        if (star < 7 || line.length() != star + 3 || line.charAt(0) != '$' || !line.startsWith("GGA,", 3))
        {
            return Double.NaN;
        }
        int checksum = 0;
        for (int i = 1; i < star; ++i)
        {
            checksum ^= line.charAt(i);
        }
        int high = Character.digit(line.charAt(star + 1), 16);
        int low = Character.digit(line.charAt(star + 2), 16);
        if (high < 0 || low < 0 || checksum != high * 16 + low)
        {
            return Double.NaN;
        }
        String[] fields = line.substring(0, star).split(",", -1);
        if (fields.length < 13 || fields[6].length() != 1 || "1245".indexOf(fields[6].charAt(0)) < 0 ||
            !fields[10].equalsIgnoreCase("M") || fields[11].isEmpty() || !fields[12].equalsIgnoreCase("M"))
        {
            return Double.NaN;
        }
        return parse_fields(fields);
    }

    private static double parse_fields(String[] fields)
    {
        try
        {
            double altitude_m = Double.parseDouble(fields[9]);
            double geoid_offset_m = Double.parseDouble(fields[11]);
            if (altitude_m >= -1000 && altitude_m <= 9000 && !Double.isNaN(geoid_offset_m) && !Double.isInfinite(geoid_offset_m))
            {
                return altitude_m;
            }
        }
        catch (NumberFormatException error)
        {
            return Double.NaN;
        }
        return Double.NaN;
    }
}
