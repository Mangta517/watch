/**
  ******************************************************************************
  * @file    touch.h
  * @brief   CST816 电容触摸驱动 (软件模拟 I2C, 从机地址 0x15/0x14 自动探测)
  *          移植自 D:/3251/f411RET6/Watch 工程 interface/touch.h (源头 FryPi
  *          OV_Watch BSP/TOUCH/CST816.c, GPL-3.0)。
  *          本工程 CubeMX 未配置硬件 I2C (PB7 已被 LCD_RST 占用), 故采用
  *          软件 I2C: SCL(TP_SCL)=PB6  SDA(TP_SDA)=PB4  RST(TP_RST)=PA15
  *          (实测本板 TP 走线与早期引脚表相反, 已在 CubeMX 对调标签)。
  *          本板无 INT 引脚 → 驱动上电关闭芯片自动休眠, 纯轮询;
  *          连续失败200次后逐拍推进复位和地址重探测，实际耗时随任务调度变化。
  ******************************************************************************
***/
#ifndef __TOUCH_H
#define __TOUCH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

#define TOUCH_READ_ERROR 0xFF /* 读取失败、恢复中或扫描不可用，不代表手指已松开。 */

    /* 返回读到的 ChipID (CST816S≈0xB5, CST816D≈0xB7); 总线异常返回 0 */
    uint8_t Touch_Init(void);

    /* touch.c定义；初始化、ChipID读取和恢复流程更新，调试器/主机验证只读。 */
    extern volatile uint8_t g_tp_chip_id; /* 0xB5/0xB7=I2C通，0=未读到有效ID。 */

    uint8_t Touch_GetChipID(void);

    /* 返回手指数：0=已松手，TOUCH_READ_ERROR=读取不可用；有触点时才更新x/y屏幕坐标。 */
    uint8_t Touch_GetPoint(uint16_t *x, uint16_t *y);

    void Touch_Reset(void);
    void Touch_Sleep(void);  /* 本板无TP_INT，休眠后由PA4触发复位唤醒。 */
    void Touch_Wakeup(void); /* 复位唤醒 */

#ifdef __cplusplus
}
#endif

#endif /* __TOUCH_H */
