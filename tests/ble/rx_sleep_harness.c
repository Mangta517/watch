/* 真实UART接收与显示任务等待路径；按115200/8N1模拟熄屏字节流。 */
#include <assert.h>
#include <stdio.h>
void Error_Handler(void);
#include "../../application/app_task.c"
#include "../../interface/kt6368a.c"

UART_HandleTypeDef huart1 = {USART1, {115200}, 0};
static uint32_t tick, notifications, irq_count, wait_count, delay_count;
static uint32_t wire_remaining, wire_sent, wire_fraction, consumed;
static uint8_t *rx_target;
static uint8_t wait_race, wake_key_active;

static void inject_bytes(uint32_t count)
{
    while (count-- != 0)
    {
        *rx_target = (uint8_t)wire_sent++;
        HAL_UART_RxCpltCallback(&huart1);
    }
}

static void advance_ms(uint32_t ms)
{
    while (ms-- != 0)
    {
        uint32_t count;
        ++tick;
        wire_fraction += 11520; /* 1起始位、8数据位、1停止位：每秒11520字节。 */
        count = wire_fraction / 1000;
        wire_fraction %= 1000;
        if (count > wire_remaining)
        {
            count = wire_remaining;
        }
        wire_remaining -= count;
        inject_bytes(count);
    }
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

void __DMB(void)
{
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && size == 1);
    rx_target = data;
    return HAL_OK;
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    assert(task == &notifications);
    ++notifications;
    ++irq_count;
    *woken = pdTRUE;
}

void startup_yield_from_isr(BaseType_t woken)
{
    assert(woken == pdTRUE);
}

uint8_t App_UI_IsWakeKeyActive(void)
{
    return wake_key_active;
}

void vTaskDelay(TickType_t ticks)
{
    ++delay_count;
    advance_ms(ticks);
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait)
{
    uint32_t result;
    assert(clear == pdTRUE && wait == 100);
    ++wait_count;
    if (wait_race == 1)
    {
        wait_race = 0;
        inject_bytes(128); /* 检查空队列之后、真正阻塞之前到达。 */
    }
    while (notifications == 0 && wait != 0)
    {
        advance_ms(1);
        --wait;
        if (wait_race == 2 && wait == 0)
        {
            wait_race = 0;
            inject_bytes(128); /* 100ms到期与RX水位同时到达。 */
        }
    }
    result = notifications;
    notifications = 0;
    return result;
}

static void consume_budget(void)
{
    uint8_t data[256]; /* 与UI每轮接收预算一致，不假设一轮可以清空环形缓冲。 */
    uint16_t count = kt6368a_read(data, sizeof(data));
    uint16_t i;
    for (i = 0; i < count; ++i)
    {
        assert(data[i] == (uint8_t)consumed++);
    }
}

static void reset_receiver(void)
{
    memset(&ble, 0, sizeof(ble));
    tick = notifications = irq_count = wait_count = delay_count = 0;
    wire_remaining = wire_sent = wire_fraction = consumed = 0;
    wait_race = wake_key_active = 0;
    ble.powered = 1;
    s_lvgl_task_handle = &notifications;
    s_power_state = APP_POWER_IDLE | APP_POWER_SCREEN_OFF;
    s_start.phase = KT_START_OFF;
    kt6368a_set_rx_callback(app_bluetooth_rx_from_isr);
    assert(HAL_UART_Receive_IT(&huart1, &ble.rx_byte, 1) == HAL_OK);
}

int main(void)
{
    uint32_t before, waits, delays;
    reset_receiver();
    app_lvgl_wait(20, 1);
    assert(tick == 100 && wait_count == 1 && irq_count == 0);

    before = tick;
    wait_race = 1;
    app_lvgl_wait(20, 1);
    assert(tick == before && notifications == 0 && kt6368a_rx_pending() != 0);
    consume_budget();
    before = tick;
    wait_race = 2;
    app_lvgl_wait(20, 1);
    assert(tick - before == 100 && notifications == 0);
    consume_budget();

    inject_bytes(700);
    waits = wait_count;
    delays = delay_count;
    before = tick;
    while (kt6368a_rx_pending() != 0)
    {
        app_lvgl_wait(20, 1);
        consume_budget();
    }
    assert(wait_count == waits && delay_count == delays + 3 && tick - before == 3);
    app_lvgl_wait(20, 1); /* 回收已到达的水位通知。 */
    before = tick;
    app_lvgl_wait(20, 1);
    assert(tick - before == 100 && kt6368a_get_rx_dropped() == 0);

    reset_receiver();
    wire_remaining = 50000;
    while (wire_remaining != 0 || kt6368a_rx_pending() != 0)
    {
        app_lvgl_wait(20, 1);
        consume_budget();
        assert(kt6368a_get_rx_dropped() == 0);
        assert(tick < 5000);
    }
    assert(consumed == 50000 && wire_sent == consumed);
    assert(irq_count > 300 && irq_count < 500); /* 批量通知，不能逐字节唤醒。 */
    assert(delay_count == 0);

    reset_receiver();
    s_power_state = 0;
    inject_bytes(128);
    assert(irq_count == 0); /* 亮屏仍使用原周期，水位不额外通知。 */
    consume_budget();
    s_power_state = APP_POWER_IDLE | APP_POWER_SCREEN_OFF;
    wake_key_active = 1;
    before = tick;
    app_lvgl_wait(20, 1);
    assert(tick - before == 20); /* PA4消抖保持原轮询周期。 */
    puts("PASS: real UART RX and LVGL sleep wait preserve 50kB at 115200, drain backlog, batch notifications, retain early/deadline wakes and idle 100ms waits.");
    return 0;
}
