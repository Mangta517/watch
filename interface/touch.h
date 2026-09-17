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
  *          连续失败约 1s 自动做复位重探测自愈 (无永久失效死角)。
  ******************************************************************************
***/
#ifndef __TOUCH_H
#define __TOUCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 返回读到的 ChipID (CST816S≈0xB5, CST816D≈0xB7); 总线异常返回 0 */
uint8_t Touch_Init(void);

extern volatile uint8_t g_tp_chip_id;   /* 最近一次 ChipID 读数 (0xB5/0xB7=I2C通, 0=总线死) */

uint8_t Touch_GetChipID(void);

/* 读取一个触点: 返回手指数(0=无触摸/读失败), x/y 为屏幕坐标(与 LCD 对齐) */
uint8_t Touch_GetPoint(uint16_t *x, uint16_t *y);

void    Touch_Reset(void);
void    Touch_Sleep(void);                       /* 进低功耗(触摸自唤醒) */
void    Touch_Wakeup(void);                      /* 复位唤醒 */

#ifdef __cplusplus
}
#endif

#endif /* __TOUCH_H */
