#include "app_touch.h"
#include "touch.h"
#include "FreeRTOS.h"
#include "task.h"

#define APP_TOUCH_QUEUE_SIZE 32
#define APP_TOUCH_MAX_AGE_MS 100
#define APP_TOUCH_SLEEP_RETRY_MS 500 /* 熄屏休眠写失败后的最小重试间隔，单位ms。 */

typedef struct
{
    TickType_t tick;
    uint16_t x, y;
    uint8_t pressed;
} app_touch_event_struct;

/* 两任务只在短临界区交换事件；I2C始终在临界区外。 */
static app_touch_event_struct s_events[APP_TOUCH_QUEUE_SIZE];
static app_touch_event_struct s_latest, s_output;
static uint32_t s_generation;
static uint8_t s_head, s_count, s_valid, s_wait_release, s_canceled;
static uint8_t s_enabled = 1;
static uint8_t s_sleeping;
/* 熄屏休眠失败退避：touch任务发起事务时记录时刻，set_enabled两侧切换都清零，避免旧窗口延迟新会话。 */
static uint8_t s_sleep_backoff;
static TickType_t s_sleep_fail_tick;

void app_touch_init(void)
{
    (void)Touch_Init(); /* 失败时由原驱动在后续轮询中逐步恢复。 */
}

void app_touch_set_enabled(uint8_t enabled)
{
    taskENTER_CRITICAL();
    s_enabled = enabled != 0;
    ++s_generation; /* 丢弃熄屏/亮屏切换期间尚未完成的I2C结果。 */
    s_head = s_count = s_valid = 0;
    s_output.pressed = 0;
    s_wait_release = 1;
    s_canceled = 1;
    s_sleep_backoff = 0; /* 亮屏立刻退出退避；再次熄屏也要立即尝试，不沿用旧会话的剩余窗口。 */
    taskEXIT_CRITICAL();
}

void app_touch_service(void)
{
    app_touch_event_struct sample = {0};
    uint32_t generation;
    TickType_t now, fail_tick;
    uint8_t enabled, sleeping, backoff, tail, read_result, sleep_result;

    taskENTER_CRITICAL();
    enabled = s_enabled;
    generation = s_generation;
    sleeping = s_sleeping;
    backoff = s_sleep_backoff;
    fail_tick = s_sleep_fail_tick;
    taskEXIT_CRITICAL();
    if (enabled == 0)
    {
        now = xTaskGetTickCount();
        /* 无符号差值统计自上次发起休眠事务以来的时间，tick回绕时仍然正确。 */
        if (sleeping == 0 && (backoff == 0 || (TickType_t)(now - fail_tick) >= pdMS_TO_TICKS(APP_TOUCH_SLEEP_RETRY_MS)))
        {
            /* 首次熄屏立即尝试；失败后每500ms最多发起一次事务，不再每20ms无限重试。 */
            sleep_result = Touch_Sleep();
            taskENTER_CRITICAL();
            if (sleep_result == 0)
            {
                s_sleeping = 1; /* 只有真实写入成功才认定已休眠，亮屏时据此执行Touch_Wakeup。 */
                s_sleep_backoff = 0;
            }
            else if (generation == s_generation)
            {
                /* 失败保持未休眠，从本次发起时刻起退避一个窗口；事务期间发生过熄屏/亮屏切换时结果作废，不写入新会话造成残留延迟。 */
                s_sleep_backoff = 1;
                s_sleep_fail_tick = now;
            }
            taskEXIT_CRITICAL();
        }
        return;
    }
    if (sleeping != 0)
    {
        Touch_Wakeup(); /* 本板没有TP_INT，按PA4后由本任务复位、探测并恢复扫描。 */
        s_sleeping = 0;
    }

    read_result = Touch_GetPoint(&sample.x, &sample.y);
    sample.pressed = read_result != 0;
    sample.tick = xTaskGetTickCount();
    taskENTER_CRITICAL();
    if (generation == s_generation && s_enabled != 0)
    {
        if (read_result == TOUCH_READ_ERROR)
        {
            /* 读失败取消当前手势；恢复后必须实际松手，不能合成一次点击。 */
            s_head = s_count = s_valid = 0;
            s_output.pressed = 0;
            s_wait_release = 1;
            s_canceled = 1;
            taskEXIT_CRITICAL();
            return;
        }
        if (sample.pressed == 0)
        {
            sample.x = s_latest.x;
            sample.y = s_latest.y;
        }
        if (s_wait_release != 0)
        {
            s_wait_release = sample.pressed;
        }
        else if (s_valid == 0 || sample.pressed != s_latest.pressed ||
                 sample.x != s_latest.x || sample.y != s_latest.y)
        {
            tail = (s_head + s_count) % APP_TOUCH_QUEUE_SIZE;
            if (s_count < APP_TOUCH_QUEUE_SIZE)
            {
                s_events[tail] = sample;
                ++s_count;
            }
            else
            {
                /* 消费停滞时取消整段手势，不能覆盖释放边沿后留下长按。 */
                s_head = s_count = 0;
                s_output.pressed = 0;
                s_wait_release = sample.pressed;
                s_canceled = 1;
            }
        }
        s_latest = sample;
        s_valid = 1;
    }
    taskEXIT_CRITICAL();
}

uint8_t app_touch_is_sleeping(void)
{
    return s_sleeping;
}

uint32_t app_touch_sleep_retry_ms(void)
{
    TickType_t now, elapsed, fail_tick;
    uint32_t remaining_ticks, remaining_ms;
    uint8_t enabled, sleeping, backoff;

    taskENTER_CRITICAL();
    enabled = s_enabled;
    sleeping = s_sleeping;
    backoff = s_sleep_backoff;
    fail_tick = s_sleep_fail_tick;
    taskEXIT_CRITICAL();
    if (enabled != 0 || sleeping != 0 || backoff == 0)
    {
        return 0; /* 亮屏、已真实休眠或未进入退避时不需要限时等待。 */
    }

    now = xTaskGetTickCount();
    elapsed = (TickType_t)(now - fail_tick);
    if (elapsed >= pdMS_TO_TICKS(APP_TOUCH_SLEEP_RETRY_MS))
    {
        return 0; /* 退避已到期，下一次服务调用立即重试。 */
    }
    remaining_ticks = (uint32_t)(pdMS_TO_TICKS(APP_TOUCH_SLEEP_RETRY_MS) - elapsed);
    /* tick向上取整换算ms，等待时长不短于剩余窗口；调用方仍靠亮屏通知提前打断退避。 */
    remaining_ms = (remaining_ticks * 1000 + configTICK_RATE_HZ - 1) / configTICK_RATE_HZ;
    return remaining_ms;
}

void app_touch_read(app_touch_struct *sample)
{
    TickType_t now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    if (s_valid != 0 && ((TickType_t)(now - s_latest.tick) >= pdMS_TO_TICKS(APP_TOUCH_MAX_AGE_MS) ||
                         (s_count != 0 && (TickType_t)(now - s_events[s_head].tick) >= pdMS_TO_TICKS(APP_TOUCH_MAX_AGE_MS))))
    {
        s_head = s_count = s_valid = 0;
        s_output.pressed = 0;
        s_wait_release = 1;
        s_canceled = 1;
    }
    if (s_count != 0)
    {
        s_output = s_events[s_head];
        s_head = (s_head + 1) % APP_TOUCH_QUEUE_SIZE;
        --s_count;
    }
    sample->x = s_output.x;
    sample->y = s_output.y;
    sample->pressed = s_output.pressed;
    sample->more = s_count != 0;
    sample->canceled = s_canceled;
    s_canceled = 0;
    taskEXIT_CRITICAL();
}
