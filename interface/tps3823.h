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
/* 仅在真实长STOP前调用：已启用时把WDI切回无上下拉输入，停止超时检测；
 * 不动PB1/WDG_EN，欠压复位通路保持接通；不等待、不调用enable/disable。 */
void tps3823_stop_pause(void);
/* STOP醒来、恢复PLL前调用：仅当暂停由本驱动实际执行且仍处启用状态时，
 * 恢复WDI推挽低并补一次喂狗脉冲；PB1/WDG_EN保持现状。 */
void tps3823_stop_resume(void);

#endif
