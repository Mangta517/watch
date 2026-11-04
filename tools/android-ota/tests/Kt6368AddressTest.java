package dev.mangta.wave;

/**
 * 用手册第9-10页的实例验证KT6368A SPP地址换算。
 * 换算只改Android显示的首字节（模256），AT串与Android显示的字节序相反是原因。
 */
public final class Kt6368AddressTest
{
    private static void check(boolean value)
    {
        if (!value)
        {
            throw new AssertionError();
        }
    }

    private static void equal(String expected, String actual)
    {
        if (!expected.equals(actual))
        {
            throw new AssertionError("期望 " + expected + "，实际 " + actual);
        }
    }

    private static void rejects(String value)
    {
        try
        {
            Kt6368Address.spp(value);
        }
        catch (IllegalArgumentException expected)
        {
            return;
        }
        throw new AssertionError("应拒绝地址: " + value);
    }

    public static void main(String[] args)
    {
        // 手册第9-10页：TN+220CB1C8A22C -> BLE 2C:A2:C8:B1:0C:22，TS+220CB1C8A22D -> SPP 2D:A2:C8:B1:0C:22。
        String ble = "2C:A2:C8:B1:0C:22";
        String spp = Kt6368Address.spp(ble);
        equal("2D:A2:C8:B1:0C:22", spp);
        check(!spp.equals(ble));
        check(!spp.equals("2C:A2:C8:B1:0C:23"));
        check(spp.endsWith(":22"));

        // 输入大小写混用，输出仍为规范大写。
        equal("2D:A2:C8:B1:0C:22", Kt6368Address.spp("2c:a2:C8:b1:0c:22"));
        equal("2D:A2:C8:B1:0C:22", Kt6368Address.spp("2C:a2:c8:B1:0c:22"));

        // 只有首字节加一（模256），其余五字节原样保留。
        equal("01:11:22:33:44:55", Kt6368Address.spp("00:11:22:33:44:55"));
        equal("80:00:00:00:00:00", Kt6368Address.spp("7F:00:00:00:00:00"));
        equal("00:00:00:00:00:FF", Kt6368Address.spp("FF:00:00:00:00:FF"));
        equal("FF:FF:FF:FF:FF:00", Kt6368Address.spp("FE:FF:FF:FF:FF:00"));

        // 长度、分隔符、十六进制和空值都必须拒绝。
        rejects(null);
        rejects("");
        rejects("2C:A2:C8:B1:0C:2");
        rejects("2C:A2:C8:B1:0C:22:33");
        rejects("2CA2C8B10C22");
        rejects("2C-A2-C8-B1-0C-22");
        rejects("2C:A2:C8:B1:0C:2G");
        rejects("2C:A2:C8:B1:0C:+2");
        rejects("2C:A2:C8:B1:0C::22");
        rejects(" 2C:A2:C8:B1:0C:2");
        rejects("2C:A2:C8:B1:0C:2 ");

        System.out.println("Kt6368AddressTest: 全部通过");
    }
}
