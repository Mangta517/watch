package dev.mangta.wave;

/** Android 8–13从接收机的GGA读取正高，椭球高不参与BLE校准。 */
final class GgaAltitude
{
    static double parse(String sentence)
    {
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
            !fields[10].equals("M") || fields[11].isEmpty() || !fields[12].equals("M"))
        {
            return Double.NaN;
        }
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
