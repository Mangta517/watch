#include "app_touch.h"
#include "touch.h"
#include "FreeRTOS.h"
#include "task.h"

#define APP_TOUCH_QUEUE_SIZE 32
#define APP_TOUCH_MAX_AGE_MS 100

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
    taskEXIT_CRITICAL();
}

void app_touch_service(void)
{
    app_touch_event_struct sample = {0};
    uint32_t generation;
    uint8_t enabled, tail, read_result;

    taskENTER_CRITICAL();
    enabled = s_enabled;
    generation = s_generation;
    taskEXIT_CRITICAL();
    if (enabled == 0)
    {
        if (s_sleeping == 0)
        {
            Touch_Sleep();
            s_sleeping = 1;
        }
        return;
    }
    if (s_sleeping != 0)
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
