/* 复用HAL桩，执行真实LCD驱动与LVGL首帧，丢弃第一笔DMA的所有回调。 */
#define main driver_suite_main
#define vTaskDelay driver_task_delay
#include "../sensors/driver_harness.c"
#undef vTaskDelay
#undef main
#include <string.h>

static uint32_t timeout_host_ipsr;
static unsigned int timeout_notifications;
static unsigned int startup_progress_calls;
static uint32_t startup_render_ms;

uint32_t __get_IPSR(void)
{
    return timeout_host_ipsr;
}

void vTaskDelay(TickType_t ticks);
#include "../../Middlewares/LVGL/lvgl/examples/porting/lv_port_disp_template.c"

TickType_t xTaskGetTickCount(void)
{
    return elapsed_ms;
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return &elapsed_ms;
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    assert(task == &elapsed_ms && timeout_host_ipsr != 0);
    ++timeout_notifications;
    *woken = pdTRUE;
}

void host_dma_yield(BaseType_t woken)
{
    assert(woken == pdTRUE);
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait)
{
    uint32_t count;
    assert(clear == pdTRUE);
    if (timeout_notifications == 0 && wait != 0)
    {
        vTaskDelay(wait);
    }
    count = timeout_notifications;
    timeout_notifications = 0;
    return count;
}

void vTaskDelay(TickType_t ticks)
{
    assert(primask == 0 && scheduler_state == taskSCHEDULER_RUNNING);
    elapsed_ms += ticks;
    if (dma_starts > 1U && (host_dma_stream.CR & DMA_SxCR_EN) != 0)
    {
        host_dma_stream.CR = 0;
        timeout_host_ipsr = 1;
        HAL_SPI_TxCpltCallback(&hspi1);
        timeout_host_ipsr = 0;
    }
}

static void startup_progress(void)
{
    assert(timeout_host_ipsr == 0 && s_flush_drv == NULL);
    elapsed_ms += startup_render_ms;
    ++startup_progress_calls;
}

int main(int argc, char **argv)
{
    lv_disp_t *display;
    uint32_t started;
    unsigned int progress_calls;
    scheduler_state = taskSCHEDULER_RUNNING;
    lv_init();
    lv_port_disp_init();
    display = lv_disp_get_default();
    if (argc > 1 && (strcmp(argv[1], "wrap") == 0 || strcmp(argv[1], "slow_wrap") == 0))
    {
        elapsed_ms = UINT32_MAX - 50U;
    }
    if (argc > 1 && (strcmp(argv[1], "slow") == 0 || strcmp(argv[1], "slow_wrap") == 0))
    {
        startup_render_ms = 150U;
    }
    lv_port_disp_set_progress_callback(startup_progress);
    started = elapsed_ms;
    lv_refr_now(display);
    lv_port_disp_set_progress_callback(NULL);
    progress_calls = startup_progress_calls;
    while (display->driver->draw_buf->flushing != 0)
    {
        disp_wait(display->driver);
    }
    assert(startup_progress_calls >= 10U && startup_progress_calls == dma_starts);
    assert((uint32_t)(elapsed_ms - started) >= 100U + startup_render_ms * startup_progress_calls);
    assert((uint32_t)(elapsed_ms - started) < 300U + startup_render_ms * startup_progress_calls);
    assert(dma_stops == 1U && spi_aborts == 1U && s_flush_drv == NULL && s_redraw_pending != 0);
    HAL_SPI_TxCpltCallback(&hspi1);
    lv_port_disp_service();
    lv_refr_now(display);
    while (display->driver->draw_buf->flushing != 0)
    {
        disp_wait(display->driver);
    }
    assert(dma_stops == 1U && s_flush_drv == NULL && s_redraw_pending == 0);
    assert(startup_progress_calls == progress_calls);
    puts("PASS: startup progress callback runs only for new task-side chunks and is removed before normal redraw.");
    puts("PASS: real LVGL first frame survives a missing DMA callback, releases safely and redraws, including tick wrap.");
    return 0;
}
