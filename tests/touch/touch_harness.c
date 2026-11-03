#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../application/app_touch.c"

static TickType_t tick;
static uint16_t point_x, point_y;
static uint8_t pressed, switch_screen, switch_during_sleep;
static unsigned int critical, reads;
static unsigned int sleeps, wakes;
static uint8_t chip_sleeping, sleep_result;

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
uint8_t Touch_Sleep(void)
{
    assert(critical == 0 && chip_sleeping == 0);
    ++sleeps;
    if (switch_during_sleep != 0)
    {
        switch_during_sleep = 0;
        app_touch_set_enabled(1); /* 休眠事务期间快速亮屏再熄屏，事务结果对新会话已失效。 */
        app_touch_set_enabled(0);
    }
    if (sleep_result != 0)
    {
        return 1;
    }
    chip_sleeping = 1;
    return 0;
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
    unsigned int i, before, base_sleeps, base_wakes;
    unsigned int window = APP_TOUCH_SLEEP_RETRY_MS;
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
    /* 瞬态NACK不能假报休眠：首次熄屏立即尝试，失败后一个窗口内不再重复发起事务。 */
    app_touch_set_enabled(0);
    sleep_result = 1;
    before = reads;
    base_sleeps = sleeps;
    base_wakes = wakes;
    assert(app_touch_sleep_retry_ms() == 0U); /* 尚未发起尝试，不属于退避期。 */
    app_touch_service();
    assert(sleeps == base_sleeps + 1 && app_touch_is_sleeping() == 0 && chip_sleeping == 0);
    assert(app_touch_sleep_retry_ms() == (uint32_t)window); /* 刚失败，剩余整个窗口。 */
    for (i = 1; i < window; ++i)
    {
        ++tick;
        assert(app_touch_sleep_retry_ms() == (uint32_t)(window - i));
        app_touch_service(); /* 退避期内既不读坐标，也不重发休眠命令。 */
        assert(sleeps == base_sleeps + 1 && reads == before && app_touch_is_sleeping() == 0);
    }
    ++tick; /* 窗口到期，重试一次。 */
    assert(app_touch_sleep_retry_ms() == 0U);
    app_touch_service();
    assert(sleeps == base_sleeps + 2 && app_touch_is_sleeping() == 0 && reads == before);
    assert(app_touch_sleep_retry_ms() == (uint32_t)window);
    /* 重试仍失败则继续按窗口退避；成功后只发一次命令。 */
    sleep_result = 0;
    for (i = 1; i < window; ++i)
    {
        ++tick;
        assert(app_touch_sleep_retry_ms() == (uint32_t)(window - i));
        app_touch_service();
        assert(sleeps == base_sleeps + 2 && app_touch_is_sleeping() == 0);
    }
    ++tick;
    app_touch_service();
    assert(sleeps == base_sleeps + 3 && app_touch_is_sleeping() != 0 && chip_sleeping != 0);
    assert(app_touch_sleep_retry_ms() == 0U); /* 真实休眠成功后不再退避。 */
    ++tick;
    app_touch_service();
    ++tick;
    app_touch_service();
    assert(sleeps == base_sleeps + 3 && reads == before); /* 成功后不重发命令，也不读坐标。 */
    /* 亮屏时按真实休眠状态执行一次唤醒，随即恢复正常读取节奏。 */
    app_touch_set_enabled(1);
    read_event(0, 0); /* 先消费切换时置起的取消标记。 */
    poll(0, 0, 0);
    assert(wakes == base_wakes + 1 && app_touch_is_sleeping() == 0 && reads == before + 1);
    assert(app_touch_sleep_retry_ms() == 0U); /* 亮屏期间立刻正常读取。 */
    poll(1, 150, 160);
    sample = read_event(1, 0);
    assert(sample.x == 150 && sample.y == 160 && sample.canceled == 0);
    poll(0, 0, 0);
    read_event(0, 0);
    /* 重试未成功就亮屏：直接恢复采样，不凭空执行休眠唤醒。 */
    app_touch_set_enabled(0);
    sleep_result = 1;
    base_sleeps = sleeps;
    base_wakes = wakes;
    app_touch_service();
    assert(sleeps == base_sleeps + 1 && app_touch_is_sleeping() == 0);
    assert(app_touch_sleep_retry_ms() == (uint32_t)window);
    before = reads;
    app_touch_set_enabled(1);
    assert(app_touch_sleep_retry_ms() == 0U); /* 亮屏立刻退出退避。 */
    poll(0, 0, 0);
    assert(reads == before + 1 && sleeps == base_sleeps + 1 && wakes == base_wakes);
    /* 熄屏→亮屏→熄屏快速切换：新会话立即重试，不沿用旧窗口的剩余延迟。 */
    app_touch_set_enabled(0);
    assert(app_touch_sleep_retry_ms() == 0U);
    base_sleeps = sleeps;
    app_touch_service();
    assert(sleeps == base_sleeps + 1 && app_touch_is_sleeping() == 0);
    assert(app_touch_sleep_retry_ms() == (uint32_t)window);
    app_touch_set_enabled(1);
    app_touch_set_enabled(0);
    assert(app_touch_sleep_retry_ms() == 0U);
    app_touch_service();
    assert(sleeps == base_sleeps + 2 && app_touch_is_sleeping() == 0);
    /* 事务期间快速切屏：失效的失败结果不写入新会话，不留残留延迟。 */
    app_touch_set_enabled(0);
    switch_during_sleep = 1;
    base_sleeps = sleeps;
    app_touch_service();
    assert(sleeps == base_sleeps + 1 && app_touch_is_sleeping() == 0);
    assert(app_touch_sleep_retry_ms() == 0U);
    app_touch_service();
    assert(sleeps == base_sleeps + 2); /* 立即重试，而非等待旧的500ms窗口。 */
    /* 事务期间快速切屏但休眠真实成功：仍记录已休眠，亮屏时按Touch_Wakeup语义恢复。 */
    app_touch_set_enabled(0);
    switch_during_sleep = 1;
    sleep_result = 0;
    base_sleeps = sleeps;
    base_wakes = wakes;
    app_touch_service();
    assert(sleeps == base_sleeps + 1 && app_touch_is_sleeping() != 0 && chip_sleeping != 0);
    assert(app_touch_sleep_retry_ms() == 0U);
    app_touch_set_enabled(1);
    before = reads;
    poll(0, 0, 0);
    assert(wakes == base_wakes + 1 && reads == before + 1 && app_touch_is_sleeping() == 0);
    /* 退避跨tick回绕：无符号差值仍然正确，不提前也不重复发起事务。 */
    app_touch_set_enabled(0);
    sleep_result = 1;
    base_sleeps = sleeps;
    before = reads;
    tick = UINT32_MAX - 100U;
    app_touch_service();
    assert(sleeps == base_sleeps + 1 && app_touch_is_sleeping() == 0);
    assert(app_touch_sleep_retry_ms() == (uint32_t)window);
    for (i = 1; i < window; ++i)
    {
        ++tick; /* 第101拍跨过 UINT32_MAX 回绕到 0。 */
        assert(app_touch_sleep_retry_ms() == (uint32_t)(window - i));
        app_touch_service();
        assert(sleeps == base_sleeps + 1 && reads == before && app_touch_is_sleeping() == 0);
    }
    ++tick;
    assert(app_touch_sleep_retry_ms() == 0U);
    app_touch_service();
    assert(sleeps == base_sleeps + 2 && app_touch_is_sleeping() == 0);
    ++tick;
    app_touch_service();
    assert(sleeps == base_sleeps + 2); /* 回绕后新窗口内仍不重复事务。 */
    /* 亮屏后恢复正常手势链路。 */
    app_touch_set_enabled(1);
    read_event(0, 0);
    poll(1, 210, 220);
    poll(0, 0, 0);
    poll(1, 210, 220);
    sample = read_event(1, 0);
    assert(sample.x == 210 && sample.y == 220 && sample.canceled == 0);
    poll(0, 0, 0);
    sample = read_event(0, 0);
    assert(sample.canceled == 0);
    assert(app_touch_sleep_retry_ms() == 0U);
    puts("PASS: sleep NACK backs off one window (wrap-safe), retries once per window, sends the command only until it really succeeds, never fakes sleep, and screen-on leaves the backoff at once.");
    puts("PASS: ordered press/move/release, short clicks, stale release, overflow/error cancellation, screen transitions, in-flight reads and tick wrap; consumer never reads I2C.");
    return 0;
}
