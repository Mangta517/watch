/**
  ******************************************************************************
  * @file    app_touch.c
  * @brief   应用层-触摸事件缓存 (CST816 软件I2C轮询 → LVGL输入)
  *          数据归属: touch任务独占I2C，按APP_TOUCH_PERIOD_MS采集并把坐标压成事件队列；
  *          LVGL任务只通过app_touch_read在短临界区取事件，不访问I2C、不等待touch任务。
  *          队列/最新帧/取消标志都在taskENTER_CRITICAL临界区内交换，I2C事务始终在临界区外。
  *          超龄判据: 最新帧或队首事件未被消费达到APP_TOUCH_MAX_AGE_MS(100ms)即整段作废。
  *          熄屏: 先成功写入器件休眠才置s_sleeping，之后touch任务阻塞等待亮屏通知；
  *          写失败按APP_TOUCH_SLEEP_RETRY_MS(500ms)退避重试，不每拍空转。
  ******************************************************************************
***/
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
    /* 初始化触摸器件：复位、探测 CST816 地址并关闭自动休眠；返回的 ChipID 不在此保留。 */
    (void)Touch_Init(); /* 失败时由原驱动在后续轮询中逐步恢复。 */
}

void app_touch_set_enabled(uint8_t enabled)
{
    /* 进入短临界区，把下面这些共享状态作为一组整体切换，避免被服务/读取路径看到中间态。 */
    taskENTER_CRITICAL();
    /* 参数归一化为 0/1：非 0 即亮屏使能，0 表示熄屏停采。 */
    s_enabled = enabled != 0;
    ++s_generation; /* 使亮灭屏切换前发起的过期触点采样结果失效。 */
    /* 清空事件队列的读写下标、计数与有效标志，旧事件不再参与消费。 */
    s_head = s_count = s_valid = 0;
    /* 对外输出帧的按下态清零，避免读取方沿用上一段手势的按压状态。 */
    s_output.pressed = 0;
    /* 置“等待松手”标志：切换后必须实际看到一次松开，才重新接受新手势。 */
    s_wait_release = 1;
    /* 置取消标志，让 LVGL 下一次读取得知上一段手势已作废（读取后即清除）。 */
    s_canceled = 1;
    s_sleep_backoff = 0; /* 亮屏立刻退出退避；再次熄屏也要立即尝试，不沿用旧会话的剩余窗口。 */
    /* 退出临界区，恢复中断与任务切换。 */
    taskEXIT_CRITICAL();
}

/* 由touch任务每轮调用：熄屏时只负责发起休眠和退避重试，不读触点；
 * 亮屏时把器件读数压成事件队列，坐标/状态没有变化的一拍不入队，避免LVGL空转。 */
void app_touch_service(void)
{
    /* 本轮采样结果：tick/x/y/pressed，先整体清零再按实际读数填写。 */
    app_touch_event_struct sample = {0};
    /* 临界区取出的会话代号快照，用于判断本轮 I2C/休眠结果是否已过期。 */
    uint32_t generation;
    /* now 为本轮时刻，fail_tick 为上次发起休眠事务的时刻。 */
    TickType_t now, fail_tick;
    /* 前3项为使能/已休眠/退避的共享状态快照；tail为本地队列尾下标，后2项为后续读/休眠写返回值。 */
    uint8_t enabled, sleeping, backoff, tail, read_result, sleep_result;

    /* 进入短临界区，一次性拷出全部共享状态，避免逐项读取时被其他任务插入修改。 */
    taskENTER_CRITICAL();
    enabled = s_enabled;
    generation = s_generation;
    sleeping = s_sleeping;
    backoff = s_sleep_backoff;
    fail_tick = s_sleep_fail_tick;
    taskEXIT_CRITICAL();
    /* 熄屏分支：不读触点，只负责把器件置入休眠并按退避窗口重试。 */
    if (enabled == 0)
    {
        /* 读取当前 tick，用于计算与上次失败时刻的间隔。 */
        now = xTaskGetTickCount();
        /* 无符号差值统计自上次发起休眠事务以来的时间，tick回绕时仍然正确。 */
        if (sleeping == 0 && (backoff == 0 || (TickType_t)(now - fail_tick) >= pdMS_TO_TICKS(APP_TOUCH_SLEEP_RETRY_MS)))
        {
            /* 首次熄屏立即尝试；失败后每500ms最多发起一次事务，不再每20ms无限重试。 */
            /* 向器件写休眠命令；返回 0 表示写入成功，1 表示失败。 */
            sleep_result = Touch_Sleep();
            /* 重新进入临界区，把本次事务结果写回共享状态。 */
            taskENTER_CRITICAL();
            if (sleep_result == 0)
            {
                s_sleeping = 1; /* 只有真实写入成功才认定已休眠，亮屏时据此执行Touch_Wakeup。 */
                /* 写成功即退出退避，下次熄屏重新从“立即尝试”开始。 */
                s_sleep_backoff = 0;
            }
            else if (generation == s_generation)
            {
                /* 失败保持未休眠，从本次发起时刻起退避一个窗口；事务期间发生过熄屏/亮屏切换时结果作废，不写入新会话造成残留延迟。 */
                /* 置退避标志并记录发起时刻，之后按 500ms 窗口节流重试。 */
                s_sleep_backoff = 1;
                s_sleep_fail_tick = now;
            }
            /* 释放临界区，允许其他任务访问触摸共享状态。 */
            taskEXIT_CRITICAL();
        }
        /* 熄屏路径到此结束，本轮不读取触点，直接返回。 */
        return;
    }
    /* 亮屏且此前已休眠：先唤醒器件，再恢复扫描。 */
    if (sleeping != 0)
    {
        Touch_Wakeup(); /* 本板没有TP_INT，按PA4后由本任务复位、探测并恢复扫描。 */
        /* 复位唤醒已执行，清除休眠标志，后续轮次按亮屏路径采集触点。 */
        s_sleeping = 0;
    }

    /* 读取一次触点：返回值 0 表示已松手，非 0 表示有触点，TOUCH_READ_ERROR 表示读取不可用。 */
    read_result = Touch_GetPoint(&sample.x, &sample.y);
    /* 把返回值折算成按下标志：任何非 0（含有触点与错误值）都先记为按下。 */
    sample.pressed = read_result != 0;
    /* 记录本次采样的 tick，供消费端做 100ms 超龄判定。 */
    sample.tick = xTaskGetTickCount();
    /* 进入短临界区，把采样结果与共享队列/最新帧一起更新。 */
    taskENTER_CRITICAL();
    /* 只在本轮期间未发生亮灭屏切换、且仍处亮屏时提交结果，否则整拍丢弃。 */
    if (generation == s_generation && s_enabled != 0)
    {
        if (read_result == TOUCH_READ_ERROR)
        {
            /* 读失败取消当前手势；恢复后必须实际松手，不能合成一次点击。 */
            /* 清空队列、有效标志与对外按下态，作废已缓存的整段手势。 */
            s_head = s_count = s_valid = 0;
            s_output.pressed = 0;
            /* 置等待松手，恢复后先看到一次松开才重新接受事件。 */
            s_wait_release = 1;
            /* 置取消标志，通知 LVGL 本段手势已作废（读取后清除）。 */
            s_canceled = 1;
            /* 先释放临界区再返回，保证不带着临界区退出函数。 */
            taskEXIT_CRITICAL();
            return;
        }
        /* 已松手时不更新坐标：沿用最新帧的坐标，供 LVGL 在松开帧继续给出位置。 */
        if (sample.pressed == 0)
        {
            sample.x = s_latest.x;
            sample.y = s_latest.y;
        }
        /* 等待松手阶段：只跟踪松开边沿，不向队列压入事件。 */
        if (s_wait_release != 0)
        {
            /* 把当前按下态写入等待标志：读到 0 即认为已松手，后续恢复入队。 */
            s_wait_release = sample.pressed;
        }
        /* 正常阶段：无有效帧，或按下态/坐标相对最新帧有变化时才产生新事件。 */
        else if (s_valid == 0 || sample.pressed != s_latest.pressed ||
                 sample.x != s_latest.x || sample.y != s_latest.y)
        {
            /* 环形队列尾下标按容量取模推进。 */
            tail = (s_head + s_count) % APP_TOUCH_QUEUE_SIZE;
            /* 队列未满：把本拍采样写入队尾并累加计数。 */
            if (s_count < APP_TOUCH_QUEUE_SIZE)
            {
                s_events[tail] = sample;
                ++s_count;
            }
            else
            {
                /* 消费停滞时取消整段手势，不能覆盖释放边沿后留下长按。 */
                /* 丢弃整个队列并从队首重新开始，避免覆盖未消费的旧事件。 */
                s_head = s_count = 0;
                s_output.pressed = 0;
                /* 以当前按下态重置等待松手标志，防止残留长按。 */
                s_wait_release = sample.pressed;
                /* 置取消标志，让消费端丢弃本段手势。 */
                s_canceled = 1;
            }
        }
        /* 无论是否入队，都把本拍记为最新帧，供下一拍做变化比较与超龄判定。 */
        s_latest = sample;
        /* 标记当前快照有效，超龄判定据此启用。 */
        s_valid = 1;
    }
    /* 释放临界区，结束本轮状态交换。 */
    taskEXIT_CRITICAL();
}

uint8_t app_touch_is_sleeping(void)
{
    /* 直接返回休眠标志：1 表示休眠命令已真实写入成功，0 表示未休眠。 */
    return s_sleeping;
}

uint32_t app_touch_sleep_retry_ms(void)
{
    /* now 为当前 tick，elapsed 为距上次失败发起时刻的 tick 数，fail_tick 为该时刻快照。 */
    TickType_t now, elapsed, fail_tick;
    /* remaining_ticks/remaining_ms 分别是退避剩余量的 tick 数与换算后的毫秒数。 */
    uint32_t remaining_ticks, remaining_ms;
    /* 临界区快照：亮灭屏使能、已休眠标志与退避标志。 */
    uint8_t enabled, sleeping, backoff;

    /* 进入短临界区，一次性取出判定所需的共享状态。 */
    taskENTER_CRITICAL();
    enabled = s_enabled;
    sleeping = s_sleeping;
    backoff = s_sleep_backoff;
    fail_tick = s_sleep_fail_tick;
    taskEXIT_CRITICAL();
    /* 亮屏、已真实休眠或未进入退避，都不需要限时等待，直接返回 0。 */
    if (enabled != 0 || sleeping != 0 || backoff == 0)
    {
        return 0; /* 亮屏、已真实休眠或未进入退避时不需要限时等待。 */
    }

    /* 读取当前 tick 并计算距上次失败发起时刻经过的 tick 数（无符号差可跨回绕）。 */
    now = xTaskGetTickCount();
    elapsed = (TickType_t)(now - fail_tick);
    /* 退避窗口已走完：下一次服务调用会立即重试，无需再等待。 */
    if (elapsed >= pdMS_TO_TICKS(APP_TOUCH_SLEEP_RETRY_MS))
    {
        return 0; /* 退避已到期，下一次服务调用立即重试。 */
    }
    /* 计算剩余 tick 数：把 500ms 窗口换算成 tick 后减去已过去的部分。 */
    remaining_ticks = (uint32_t)(pdMS_TO_TICKS(APP_TOUCH_SLEEP_RETRY_MS) - elapsed);
    /* tick向上取整换算ms，等待时长不短于剩余窗口；调用方仍靠亮屏通知提前打断退避。 */
    remaining_ms = (remaining_ticks * 1000 + configTICK_RATE_HZ - 1) / configTICK_RATE_HZ;
    /* 返回剩余毫秒数，供调用方设置限时等待。 */
    return remaining_ms;
}

/* 由LVGL输入回调调用，取出一条事件：最新帧或队首事件未被消费的时间达到100ms时整段作废，
 * 防止熄屏停采或任务卡顿后补发一整套旧手势；取消标志读完即清，不重复通报。 */
void app_touch_read(app_touch_struct *sample)
{
    /* 进入函数时先取当前 tick，后面用它判断最新帧与队首事件是否超龄。 */
    TickType_t now = xTaskGetTickCount();

    /* 进入短临界区，保证超龄判定、出队与字段拷贝对同一份状态完成。 */
    taskENTER_CRITICAL();
    /* 已有有效帧且（最新帧超龄，或队首事件超龄）时，整段手势作废。 */
    if (s_valid != 0 && ((TickType_t)(now - s_latest.tick) >= pdMS_TO_TICKS(APP_TOUCH_MAX_AGE_MS) ||
                         (s_count != 0 && (TickType_t)(now - s_events[s_head].tick) >= pdMS_TO_TICKS(APP_TOUCH_MAX_AGE_MS))))
    {
        /* 清空队列、有效标志与对外按下态，防止补发整套旧手势。 */
        s_head = s_count = s_valid = 0;
        s_output.pressed = 0;
        /* 要求重新实际松手后才恢复接受事件。 */
        s_wait_release = 1;
        /* 置取消标志，本次读取会把它通报给 LVGL。 */
        s_canceled = 1;
    }
    /* 队列非空时取出队首事件，并推进队首下标、减少计数。 */
    if (s_count != 0)
    {
        s_output = s_events[s_head];
        s_head = (s_head + 1) % APP_TOUCH_QUEUE_SIZE;
        --s_count;
    }
    /* 把当前输出帧的坐标与按下态写入调用方缓冲区。 */
    sample->x = s_output.x;
    sample->y = s_output.y;
    sample->pressed = s_output.pressed;
    /* 队列还有剩余事件时置 more=1，提示 LVGL 同一拍继续读取。 */
    sample->more = s_count != 0;
    /* 把当前挂起的取消标志通报给调用方，随后清零。 */
    sample->canceled = s_canceled;
    s_canceled = 0;
    /* 释放临界区，拷贝与状态更新在锁内完成，调用方拿到一致快照。 */
    taskEXIT_CRITICAL();
}
