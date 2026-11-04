package android.os;

public final class Looper
{
    private static final Looper MAIN = new Looper();

    public static Looper getMainLooper()
    {
        return MAIN;
    }

    public static Looper myLooper()
    {
        if ("main".equals(Thread.currentThread().getName()))
        {
            return MAIN;
        }
        return null;
    }
}
