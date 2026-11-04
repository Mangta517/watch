package dev.mangta.wave;

/**
 * KT6368A BLE地址与SPP地址换算。
 * 依据《KT6368A_芯片使用手册_V2.0.pdf》第9-10页：SPP地址是BLE地址首字节加1（模256），
 * 其余五字节保持不变。手册示例TN+220CB1C8A22C对应BLE Android 2C:A2:C8:B1:0C:22，
 * TS+220CB1C8A22D对应SPP Android 2D:A2:C8:B1:0C:22。
 * AT串里的地址字节序与Android显示相反（AT串高位在前，Android显示低位在前），
 * 所以AT串末字节对应Android首字节；换算改的是Android首字节，不是末字节，也不是同一地址。
 * 本类只做Android侧BLE MAC到SPP MAC的换算，不能把SPP地址当BLE地址使用。
 */
final class Kt6368Address
{
    private Kt6368Address()
    {
    }

    /**
     * 把Android显示的BLE地址换成同一模块的SPP地址。
     * 输入必须是六字节冒号分隔的ASCII十六进制（大小写均可），输出为规范大写。
     * 输入格式非法时抛出IllegalArgumentException。
     */
    static String spp(String bleAddress)
    {
        if (bleAddress == null)
        {
            throw new IllegalArgumentException("BLE地址不能为空");
        }
        if (bleAddress.length() != 17)
        {
            throw new IllegalArgumentException("BLE地址必须是六字节冒号分隔的17字符: " + bleAddress);
        }
        for (int index = 0; index < 17; ++index)
        {
            char current = bleAddress.charAt(index);
            if (index % 3 == 2)
            {
                if (current != ':')
                {
                    throw new IllegalArgumentException("BLE地址分隔符必须是冒号: " + bleAddress);
                }
            }
            else
            {
                boolean hex = (current >= '0' && current <= '9') || (current >= 'a' && current <= 'f')
                    || (current >= 'A' && current <= 'F');
                if (!hex)
                {
                    throw new IllegalArgumentException("BLE地址必须是十六进制字节: " + bleAddress);
                }
            }
        }
        char[] digits = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
        StringBuilder result = new StringBuilder(17);
        for (int index = 0; index < 6; ++index)
        {
            int base = index * 3;
            int value = (Character.digit(bleAddress.charAt(base), 16) << 4) | Character.digit(bleAddress.charAt(base + 1), 16);
            if (index == 0)
            {
                value = (value + 1) & 0xff;
            }
            if (index > 0)
            {
                result.append(':');
            }
            // 固定ASCII大写，避免String.format跟随系统区域设置输出非ASCII数字。
            result.append(digits[(value >> 4) & 0xf]);
            result.append(digits[value & 0xf]);
        }
        return result.toString();
    }
}
