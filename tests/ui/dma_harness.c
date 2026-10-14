/* 真实LVGL双缓冲移植层；模拟DMA跨两拍完成，验证等待时sensor可以运行。 */
#include <assert.h>
#include <stdio.h>
#include "../../Middlewares/LVGL/lvgl/examples/porting/lv_port_disp_template.c"

static LCD_CallbackFunc_t dma_done;
static int scheduler_state = taskSCHEDULER_RUNNING;
static unsigned dma_pending, dma_ticks, flush_calls, sensor_runs, lcd_inits;
static TickType_t host_tick;
static unsigned notifications, dma_wakes, yields;

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return (TaskHandle_t)&host_tick;
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    assert(task == (TaskHandle_t)&host_tick);
    ++notifications;
    ++dma_wakes;
    *woken = pdTRUE;
}

void host_dma_yield(BaseType_t woken)
{
    assert(woken == pdTRUE);
    ++yields;
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait)
{
    assert(clear == pdTRUE);
    if (notifications == 0 && wait != 0)
    {
        vTaskDelay(wait);
    }
    if (notifications != 0)
    {
        notifications = 0;
        return 1;
    }
    return 0;
}

TickType_t xTaskGetTickCount(void)
{
    return host_tick;
}

void LCD_Init(void)
{
    ++lcd_inits;
}

void LCD_Set_Flush_Complete_Callback(LCD_CallbackFunc_t cb)
{
    dma_done = cb;
}

void LCD_Color_Fill_DMA(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const uint16_t *pixels)
{
    assert(!dma_pending && dma_done != NULL && pixels != NULL);
    assert(x1 <= x2 && y1 <= y2 && x2 < LCD_WIDTH && y2 < LCD_HEIGHT);
    assert((unsigned)(x2 - x1 + 1) * (y2 - y1 + 1) <= LVGL_BUF_PX);
    dma_pending = 1;
    dma_ticks = 2;
    ++flush_calls;
}

int xTaskGetSchedulerState(void)
{
    return scheduler_state;
}

void vTaskDelay(TickType_t ticks)
{
    assert(scheduler_state == taskSCHEDULER_RUNNING && ticks == 1U && dma_pending);
    ++sensor_runs;
    ++host_tick;
    if (--dma_ticks == 0)
    {
        dma_pending = 0;
        dma_done();
    }
}

int main(void)
{
    lv_disp_t *display;
    unsigned before;
    lv_init();
    lv_port_disp_init();
    display = lv_disp_get_default();
    assert(lcd_inits == 1 && display != NULL && display->driver->wait_cb == disp_wait);
    assert(display->driver->draw_buf->buf1 != NULL && display->driver->draw_buf->buf2 != NULL);
    lv_refr_now(display);
    assert(flush_calls >= 10 && sensor_runs >= 18);
    while (dma_pending)
    {
        disp_wait(display->driver);
    }
    assert(s_flush_drv == NULL && display->driver->draw_buf->flushing == 0);
    assert(dma_wakes == flush_calls && yields == dma_wakes);
    before = sensor_runs;
    (void)ulTaskNotifyTake(pdTRUE, 0);
    notifications = 1;
    disp_wait(display->driver); /* 完成通知已到时立即返回，不再等待一个tick。 */
    assert(sensor_runs == before && notifications == 0);
    before = sensor_runs;
    scheduler_state = taskSCHEDULER_NOT_STARTED;
    disp_wait(display->driver);
    scheduler_state = taskSCHEDULER_SUSPENDED;
    disp_wait(display->driver);
    assert(sensor_runs == before);
    scheduler_state = taskSCHEDULER_RUNNING;
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(display);
    assert(sensor_runs > before);
    while (dma_pending)
    {
        disp_wait(display->driver);
    }
    assert(s_flush_drv == NULL && display->driver->draw_buf->flushing == 0);
    puts("PASS: real LVGL DMA wait yields to sensor, ISR notification wakes promptly, buffers remain single-flight.");
    return 0;
}
