#ifndef APP_TOUCH_H
#define APP_TOUCH_H

#include <stdint.h>

#define APP_TOUCH_PERIOD_MS   5
#define APP_TOUCH_STACK_WORDS 256

typedef struct
{
    uint16_t x, y;
    uint8_t pressed, more, canceled;
} app_touch_struct;

/* 仅touch任务调用：独占触摸总线，初始化/恢复不进入LVGL上下文。 */
void app_touch_init(void);
void app_touch_service(void);
/* 仅LVGL任务调用：短临界区读事件，不访问I2C或等待触摸任务。 */
void app_touch_read(app_touch_struct *sample);
void app_touch_set_enabled(uint8_t enabled);
/* 仅touch任务调用：确认休眠命令写入成功，随后可阻塞等待亮屏通知。 */
uint8_t app_touch_is_sleeping(void);

#endif
