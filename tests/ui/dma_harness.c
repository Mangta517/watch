/* 真实 LCD + LVGL 显示移植层；只替换 HAL/RTOS，注入 DMA 中途错误。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../Middlewares/LVGL/lvgl/examples/porting/lv_port_disp_template.c"
#include "spi.h"
#include "tim.h"

GPIO_TypeDef host_gpio_a, host_gpio_b;
SPI_TypeDef host_spi;
static DMA_HandleTypeDef dma = {HAL_DMA_STATE_READY};
SPI_HandleTypeDef hspi1 = {.Instance = &host_spi, .hdmatx = &dma};
TIM_HandleTypeDef htim3;
static TickType_t tick;
static uint32_t mask;
static unsigned active, notifications, launches, aborts, clears, wakeups;
static unsigned mode, pending_ticks, error_sent;
static unsigned dma_flags, nvic_pending;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi);

uint32_t __get_PRIMASK(void)
{
    return mask;
}
void __disable_irq(void)
{
    mask = 1;
}
void __set_PRIMASK(uint32_t value)
{
    mask = value;
}
uint32_t HAL_GetTick(void)
{
    return tick;
}
TickType_t xTaskGetTickCount(void)
{
    return tick;
}
int xTaskGetSchedulerState(void)
{
    return taskSCHEDULER_RUNNING;
}
TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return &tick;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    assert(task == &tick);
    notifications++;
    wakeups++;
    *woken = pdTRUE;
}
void host_dma_yield(BaseType_t woken)
{
    assert(woken == pdTRUE);
}
void vTaskDelay(TickType_t ticks)
{
    assert(mask == 0);
    tick += ticks;
    if (active && pending_ticks != 0)
    {
        pending_ticks--;
        if (pending_ticks == 0 && mode == 0)
        {
            active = 0;
            dma.State = HAL_DMA_STATE_READY;
            host_spi.CR2 &= ~SPI_CR2_TXDMAEN;
            HAL_SPI_TxCpltCallback(&hspi1);
            HAL_SPI_TxCpltCallback(&hspi1); /* 重复回调不重复发布完成。 */
        }
        else if (pending_ticks == 0 && (mode == 1 || mode == 4) && !error_sent)
        {
            error_sent = 1;
            host_spi.CR2 &= ~SPI_CR2_TXDMAEN;
            if (mode == 4)
            {
                active = 0; /* 真实 DMA TE 路径在回调前已停止流并置 READY。 */
                dma.State = HAL_DMA_STATE_READY;
            }
            HAL_SPI_ErrorCallback(&hspi1);
            HAL_SPI_TxCpltCallback(&hspi1); /* 错误后的旧完成事件不能提前释放。 */
        }
    }
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
void HAL_Delay(uint32_t ms)
{
    tick += ms;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    (void)pin;
    (void)state;
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq == DMA2_Stream2_IRQn && priority == 12 && subpriority == 0);
}
void HAL_NVIC_ClearPendingIRQ(int irq)
{
    assert(irq == DMA2_Stream2_IRQn && active == 0);
    clears++;
    nvic_pending = 0; /* NVIC 清 pending 不能清 DMA 外设标志。 */
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi1 && data && size && timeout && !active);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size)
{
    assert(spi == &hspi1 && data && size && !active);
    assert(dma_flags == 0 && nvic_pending == 0);
    launches++;
    error_sent = 0;
    if (mode == 3)
    {
        return HAL_ERROR;
    }
    active = 1;
    host_spi.CR2 |= SPI_CR2_TXDMAEN;
    dma.State = HAL_DMA_STATE_BUSY;
    pending_ticks = 2;
    return HAL_OK;
}
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *handle)
{
    return handle->State;
}
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *handle)
{
    assert(mask == 0 && handle == &dma && active);
    active = 0;
    handle->State = HAL_DMA_STATE_READY;
    pending_ticks = 0;
    dma_flags = 0;
    aborts++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi)
{
    assert(mask == 0 && spi == &hspi1);
    if ((host_spi.CR2 & SPI_CR2_TXDMAEN) != 0)
    {
        /* 与真实 HAL 一致：若先取消 DMA，这里会拒绝二次取消。 */
        assert(dma.State == HAL_DMA_STATE_BUSY);
        assert(HAL_DMA_Abort(&dma) == HAL_OK);
        host_spi.CR2 &= ~SPI_CR2_TXDMAEN;
    }
    HAL_SPI_TxCpltCallback(spi); /* 取消期间到达的旧回调必须被忽略。 */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(timer == &htim3 && channel == TIM_CHANNEL_3);
    return HAL_OK;
}
void Error_Handler(void)
{
    abort();
}
static void drain(lv_disp_t *display)
{
    unsigned waits = 0;
    while (s_flush_drv != NULL)
    {
        assert(++waits <= 110);
        disp_wait(display->driver);
        if (display->driver->draw_buf->flushing == 0)
        {
            assert(!active);
        }
    }
}
static void transfer(lv_disp_t *display, unsigned failure_mode)
{
    lv_area_t area = {0, 0, 3, 3};
    mode = failure_mode;
    display->driver->draw_buf->flushing = 1;
    disp_flush(display->driver, &area, display->driver->draw_buf->buf1);
    if (failure_mode == 2)
    {
        dma_flags = 1;
        nvic_pending = 1;
    }
    drain(display);
    assert(display->driver->draw_buf->flushing == 0);
    assert(lcd_flush_service() == LCD_FLUSH_WAIT);
    HAL_SPI_TxCpltCallback(&hspi1); /* 空闲时迟到的事件也不能产生新完成。 */
    assert(lcd_flush_service() == LCD_FLUSH_WAIT);
}
int main(void)
{
    lv_init();
    lv_port_disp_init();
    lv_disp_t *display = lv_disp_get_default();
    assert(display && display->driver->draw_buf->buf1 && display->driver->draw_buf->buf2);
    lv_refr_now(display);
    drain(display);
    assert(launches >= 10 && wakeups == launches);
    transfer(display, 1);
    assert(aborts == 1 && clears == 1 && s_redraw_required);
    transfer(display, 0);
    transfer(display, 4);
    assert(aborts == 1 && clears == 2);
    transfer(display, 2);
    assert(aborts == 2 && clears == 3);
    tick = UINT32_MAX - 50;
    transfer(display, 2); /* 截止时间跨 tick 回绕。 */
    assert(aborts == 3 && clears == 4);
    transfer(display, 3);
    assert(clears == 5);
    transfer(display, 0);
    lv_port_disp_service();
    assert(s_redraw_required == 0);
    lv_refr_now(display);
    drain(display);
    puts("PASS: real LCD+LVGL async buffers, DMA error/timeout/wrap/launch failure, quiescing, duplicate/late events and next-frame recovery.");
    return 0;
}
