/**
 * @file tps3823.h
 * @brief TPS3823-33 + BL1551B 外部看门狗：PB2 喂狗，PB1 低电平接通复位。
 */
#ifndef TPS3823_H
#define TPS3823_H

/* MX_GPIO_Init 后调用；接口由同一任务串行使用，禁止在 ISR 中调用。 */
void tps3823_init(void);
/* 首次启用/重新启用等待正常复位延时；不能解除非A版已锁存复位，重复调用不重新计时。 */
void tps3823_enable(void);
/* 屏蔽外部复位并将 WDI 置高阻，停止芯片内部看门狗计时。 */
void tps3823_disable(void);
/* 每次输出高脉冲后回到低电平，确保一次有效下降沿；禁用时无动作。 */
void tps3823_feed(void);

#endif
