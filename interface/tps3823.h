/**
 * @file tps3823.h
 * @brief TPS3823-33 + BL1551B 外部看门狗：PB2 喂狗，PB1 低电平接通复位。
 */
#ifndef TPS3823_H
#define TPS3823_H

/* MX_GPIO_Init 后调用；接口由同一任务串行使用，禁止在 ISR 中调用。 */
void tps3823_init(void);
/* 首次启用/重新启用等待至少 300ms 释放已有复位；运行中重复调用不重新计时。 */
void tps3823_enable(void);
/* 屏蔽外部复位并将 WDI 置高阻，停止芯片内部看门狗计时。 */
void tps3823_disable(void);
/* 每次翻转一次 WDI；禁用时无动作。调用间隔须明显小于最短超时 900ms。 */
void tps3823_feed(void);

#endif
