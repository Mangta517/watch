#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../application/app_touch.c"

static TickType_t tick;
static uint16_t point_x, point_y;
static uint8_t pressed, switch_screen;
static unsigned int critical, reads;
static unsigned int sleeps, wakes;
static uint8_t chip_sleeping;

void host_touch_enter_critical(void)
{
    assert(critical++ == 0);
}
void host_touch_exit_critical(void)
{
    assert(critical-- == 1);
}
TickType_t xTaskGetTickCount(void)
{
    return tick;
}
uint8_t Touch_Init(void)
{
    assert(critical == 0);
    return 0;
}
uint8_t Touch_GetPoint(uint16_t *x, uint16_t *y)
{
    assert(critical == 0 && chip_sleeping == 0);
    ++reads;
    if (switch_screen != 0)
    {
        switch_screen = 0;
        app_touch_set_enabled(0);
        app_touch_set_enabled(1);
    }
    *x = point_x;
    *y = point_y;
    return pressed;
}
void Touch_Sleep(void)
{
    assert(critical == 0 && chip_sleeping == 0);
    chip_sleeping = 1;
    ++sleeps;
}
void Touch_Wakeup(void)
{
    assert(critical == 0 && chip_sleeping != 0);
    chip_sleeping = 0;
    ++wakes;
}
static void poll(uint8_t down, uint16_t x, uint16_t y)
{
    pressed = down;
    point_x = x;
    point_y = y;
    ++tick;
    app_touch_service();
}
static app_touch_struct read_event(uint8_t down, uint8_t more)
{
    app_touch_struct sample;
    unsigned int before = reads;
    app_touch_read(&sample);
    assert(sample.pressed == down && sample.more == more && reads == before);
    assert(critical == 0);
    return sample;
}
int main(void)
{
    app_touch_struct sample;
    unsigned int i, before;
    app_touch_init(); /* 初始化失败也允许原驱动后续恢复。 */
    read_event(0, 0);
    poll(1, 20, 30);
    poll(1, 60, 35);
    poll(0, 0, 0);
    sample = read_event(1, 1);
    assert(sample.x == 20 && sample.y == 30);
    sample = read_event(1, 1);
    assert(sample.x == 60 && sample.y == 35);
    sample = read_event(0, 0);
    assert(sample.x == 60 && sample.y == 35);
    /* 短点击的按下/释放均早于下一次LVGL读取，仍按顺序输出。 */
    poll(1, 100, 120);
    poll(0, 0, 0);
    read_event(1, 1);
    read_event(0, 0);
    poll(1, 100, 120);
    read_event(1, 0);
    tick += 99;
    read_event(1, 0);
    ++tick;
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    poll(1, 100, 120);
    read_event(0, 0); /* 过期后必须先看到松手，不能恢复成旧长按。 */
    poll(0, 0, 0);
    poll(1, 120, 130);
    read_event(1, 0);
    app_touch_set_enabled(0);
    before = reads;
    poll(1, 150, 160);
    assert(reads == before);
    assert(app_touch_is_sleeping() != 0 && sleeps == 1U);
    poll(1, 150, 160);
    assert(reads == before && sleeps == 1U); /* 熄屏只发一次休眠命令。 */
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    app_touch_set_enabled(1);
    poll(1, 150, 160);
    assert(app_touch_is_sleeping() == 0 && wakes == 1U);
    read_event(0, 0);
    poll(0, 0, 0);
    poll(1, 160, 170);
    read_event(1, 0);
    /* 读取过程中发生熄屏再亮屏，旧事务结果不得进入新事件流。 */
    switch_screen = 1;
    poll(1, 180, 190);
    read_event(0, 0);
    poll(0, 0, 0);
    for (i = 0; i < APP_TOUCH_QUEUE_SIZE + 1U; ++i)
    {
        poll((i % 2U) == 0, 10, 20);
    }
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    poll(0, 0, 0);
    poll(1, 10, 20);
    poll(0, 0, 0);
    tick += 100;
    poll(0, 0, 0); /* 生产仍正常，但积压的旧点击也不能重放。 */
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    poll(0, 0, 0);
    tick = UINT32_MAX - 50U;
    poll(1, 30, 40);
    read_event(1, 0);
    tick += 99;
    read_event(1, 0);
    ++tick;
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    /* 总线错误取消已按下的手势；恢复中和仍按住都不能解除等待松手。 */
    poll(0, 0, 0);
    poll(1, 30, 40);
    read_event(1, 0);
    poll(TOUCH_READ_ERROR, 0, 0);
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    for (i = 0; i < 4; ++i)
    {
        poll(TOUCH_READ_ERROR, 0, 0);
        sample = read_event(0, 0);
        assert(sample.canceled != 0);
        poll(1, 35, 45);
        read_event(0, 0);
    }
    poll(0, 0, 0);
    read_event(0, 0);
    poll(1, 50, 60);
    poll(0, 0, 0);
    sample = read_event(1, 1);
    assert(sample.x == 50 && sample.y == 60 && sample.canceled == 0);
    sample = read_event(0, 0);
    assert(sample.canceled == 0);
    /* LVGL尚未读到的按下也必须一并丢弃，不能在错误后重放。 */
    poll(1, 70, 80);
    poll(TOUCH_READ_ERROR, 0, 0);
    sample = read_event(0, 0);
    assert(sample.canceled != 0);
    puts("PASS: ordered press/move/release, short clicks, stale release, overflow/error cancellation, screen transitions, in-flight reads and tick wrap; consumer never reads I2C.");
    return 0;
}
