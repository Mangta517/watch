#ifndef APP_TOUCH_H
#define APP_TOUCH_H

#include <stdint.h>

/* 应用层触摸缓存：touch任务独占CST816软件I2C并压入事件，LVGL任务只消费缓存。
 * 周期/超龄/退避单位均为毫秒；坐标单位为屏幕像素。
 * 亮屏活跃采集周期为10ms，待机为20ms。 */
#define APP_TOUCH_PERIOD_MS   10
#define APP_TOUCH_STACK_WORDS 256

/* LVGL读取到的单条事件：
   x/y 为屏幕像素坐标，无触点时沿用上一坐标，便于LVGL在松开帧继续给出位置；
   pressed=1 表示该事件为按下态；more=1 表示队列里还有未消费事件，LVGL应在同一拍继续读；
   canceled=1 表示上一段手势已作废（读失败、超龄或队列溢出），只通报一次后即清除。 */
typedef struct
{
    uint16_t x, y;
    uint8_t pressed, more, canceled;
} app_touch_struct;

/* 仅touch任务调用：独占触摸总线，初始化/恢复不进入LVGL上下文。
   初始化失败不返回错误，由驱动在后续轮询中逐步恢复。 */
void app_touch_init(void);
/* 仅touch任务调用：亮屏采集并压入事件队列；熄屏发起休眠或按退避重试。 */
void app_touch_service(void);
/* 仅LVGL任务调用：短临界区读事件，不访问I2C或等待触摸任务。
   队列为空时沿用上次坐标；more=1表示还有事件，应在同一拍继续读。 */
void app_touch_read(app_touch_struct *sample);
/* 由LVGL任务在亮灭屏切换时调用：清空队列、使过期触点采样结果失效并作废当前手势，
   enabled=0表示进入熄屏。唤醒阻塞中的touch任务由app_task_set_power_state负责。 */
void app_touch_set_enabled(uint8_t enabled);
/* 仅touch任务调用：确认休眠命令写入成功，随后可阻塞等待亮屏通知。 */
uint8_t app_touch_is_sleeping(void);
/* 仅touch任务调用：熄屏休眠失败退避期内返回距下次重试的剩余毫秒（最少1），
   非退避（亮屏、已真实休眠、首次尝试前或窗口已到期）返回0，供调用方限时等待亮屏通知。 */
uint32_t app_touch_sleep_retry_ms(void);

#endif
