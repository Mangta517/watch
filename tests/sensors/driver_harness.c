/* Compile real LCD/touch drivers; script SDA reads and record reset/clock edges. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include "lcd.h"
#include "touch.h"
#include "spi.h"
#include "tim.h"
#include "task.h"

#ifdef _WIN32
#undef assert
#define assert(condition)                                                                                                                  \
    do                                                                                                                                     \
    {                                                                                                                                      \
        if (!(condition))                                                                                                                  \
        {                                                                                                                                  \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition);                                                                   \
            exit(1);                                                                                                                       \
        }                                                                                                                                  \
    } while (0)
#endif

GPIO_TypeDef host_gpio_a, host_gpio_b;
SPI_TypeDef host_spi;
static DMA_Stream_TypeDef host_dma_stream;
static DMA_HandleTypeDef host_dma = {&host_dma_stream};
SPI_HandleTypeDef hspi1 = {.Instance = &host_spi, .hdmatx = &host_dma};
static TIM_TypeDef host_tim3;
TIM_HandleTypeDef htim3 = {.Instance = &host_tim3};
static uint32_t lcd_clock_mask = 7;
static int scheduler_state;
static uint32_t primask, elapsed_ms, hal_waits, task_waits;
static uint32_t tp_reset_start, tp_ready_at, lcd_reset_start;
static unsigned tp_resets, scl_edges, released_edges, edges_at_reset, released_at_reset;
static GPIO_PinState scl_state = GPIO_PIN_SET;
static uint32_t sda_mode;
static GPIO_PinState sda_latch;
static unsigned handoff_start_reply, handoff_bytes, handoff_clocks, handoff_checks;
static uint8_t handoff_active;
static GPIO_PinState replies[128];
static unsigned reply_count, reply_pos;
static jmp_buf error_jump;
static uint8_t error_expected, spi_capture, recovery_sample;
static int spi_fail_after = -1;
static HAL_StatusTypeDef spi_error;
static unsigned spi_payload_calls, spi_payload_bytes;
static uint16_t spi_expected_color = 0x1234;
static int spi_command_fail_after = -1;
static GPIO_PinState lcd_dc_state = GPIO_PIN_SET;
static uint8_t spi_lcd_command, spi_address_capture;
static uint8_t spi_address_bytes[11];
static uint16_t spi_address_sizes[5];
static GPIO_PinState spi_address_dc[5];
static unsigned spi_calls, spi_address_calls, spi_address_length;
static HAL_StatusTypeDef dma_start_result = HAL_OK;
static unsigned dma_starts, dma_stops, spi_aborts, flush_completions;
static uint8_t dma_stuck;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi);

void host_lcd_clock_set(uint32_t clock, uint8_t enabled)
{
    if (enabled != 0)
    {
        lcd_clock_mask |= clock;
    }
    else
    {
        assert(host_dma_stream.CR == 0 && (host_spi.SR & SPI_SR_BSY) == 0);
        assert(htim3.compare == 0 && (host_tim3.CR1 & TIM_CR1_CEN) == 0);
        lcd_clock_mask &= ~clock;
    }
}

uint32_t __get_PRIMASK(void)
{
    return primask;
}
void __disable_irq(void)
{
    primask = 1;
}
void __set_PRIMASK(uint32_t mask)
{
    primask = mask;
}
int xTaskGetSchedulerState(void)
{
    return scheduler_state;
}
void HAL_Delay(uint32_t ms)
{
    assert(!primask && scheduler_state != taskSCHEDULER_RUNNING);
    ++hal_waits;
    elapsed_ms += ms;
}
void vTaskDelay(TickType_t ticks)
{
    assert(!primask && scheduler_state == taskSCHEDULER_RUNNING && ticks > 0);
    ++task_waits;
    elapsed_ms += ticks; /* Same 1kHz tick as the firmware. */
}
void Error_Handler(void)
{
    if (error_expected)
    {
        longjmp(error_jump, 1);
    }
    abort();
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    assert(primask && port == TP_SDA_GPIO_Port && config->Pin == TP_SDA_Pin);
    if (handoff_active != 0 && config->Mode == GPIO_MODE_OUTPUT_PP)
    {
        assert(handoff_clocks % 9 == 8 && sda_latch == GPIO_PIN_RESET);
    }
    sda_mode = config->Mode;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == TP_SCL_GPIO_Port && pin == TP_SCL_Pin)
    {
        if (state == GPIO_PIN_SET && scl_state == GPIO_PIN_RESET)
        {
            if (handoff_active != 0)
            {
                if (handoff_clocks % 9 < 8 || handoff_clocks + 1 == handoff_bytes * 9)
                {
                    assert(sda_mode == GPIO_MODE_INPUT); /* 数据位和末字节 NACK 必须释放。 */
                }
                else
                {
                    assert(sda_mode == GPIO_MODE_OUTPUT_PP && sda_latch == GPIO_PIN_RESET);
                }
                ++handoff_clocks;
            }
            ++scl_edges;
            if (sda_mode == GPIO_MODE_INPUT)
            {
                ++released_edges;
            }
        }
        scl_state = state;
        if (handoff_active != 0 && state == GPIO_PIN_RESET && handoff_clocks == handoff_bytes * 9)
        {
            handoff_active = 0;
            ++handoff_checks;
        }
    }
    else if (port == TP_SDA_GPIO_Port && pin == TP_SDA_Pin)
    {
        if (handoff_active != 0 && sda_mode == GPIO_MODE_OUTPUT_PP)
        {
            assert(state == GPIO_PIN_RESET); /* 连两拍间的短暂推挽高电平也不可接受。 */
        }
        sda_latch = state;
    }
    else if (port == TP_RST_GPIO_Port && pin == TP_RST_Pin)
    {
        if (state == GPIO_PIN_RESET)
        {
            ++tp_resets;
            tp_reset_start = elapsed_ms;
            edges_at_reset = scl_edges;
            released_at_reset = released_edges;
        }
        else
        {
            assert(elapsed_ms - tp_reset_start >= 10);
            tp_ready_at = elapsed_ms + 300;
        }
    }
    else if (port == LCD_RST_GPIO_Port && pin == LCD_RST_Pin)
    {
        if (state == GPIO_PIN_RESET)
        {
            lcd_reset_start = elapsed_ms;
        }
        else
        {
            assert(elapsed_ms - lcd_reset_start >= 100);
        }
    }
    else if (port == LCD_DC_GPIO_Port && pin == LCD_DC_Pin)
    {
        lcd_dc_state = state;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == TP_SCL_GPIO_Port && pin == TP_SCL_Pin)
    {
        recovery_sample = 1;
        return scl_state;
    }
    if (recovery_sample != 0)
    {
        recovery_sample = 0;
        return GPIO_PIN_SET; /* 恢复后采样线路空闲，不是下一事务的ACK。 */
    }
    assert(!primask && port == TP_SDA_GPIO_Port && pin == TP_SDA_Pin);
    assert(sda_mode == GPIO_MODE_INPUT && elapsed_ms >= tp_ready_at);
    if (reply_pos < reply_count)
    {
        GPIO_PinState reply = replies[reply_pos++];
        if (handoff_start_reply != 0 && reply_pos == handoff_start_reply)
        {
            handoff_start_reply = 0;
            handoff_clocks = 0;
            handoff_active = 1; /* 读地址 ACK 后，从机开始持有 SDA。 */
        }
        return reply;
    }
    return GPIO_PIN_SET; /* Unconnected device NACKs. */
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    assert((irq == DMA2_Stream2_IRQn || irq == SPI1_IRQn) && priority == 12 && subpriority == 0);
}
void HAL_NVIC_EnableIRQ(int irq)
{
    assert(irq == SPI1_IRQn);
}
uint32_t HAL_GetTick(void)
{
    return elapsed_ms;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi1 && data && size && timeout == 50);
    assert((lcd_clock_mask & HOST_LCD_CLOCK_SPI) != 0);
    ++spi_calls;
    if (lcd_dc_state == GPIO_PIN_RESET)
    {
        assert(size == 1);
        spi_lcd_command = data[0];
    }
    if (spi_address_capture != 0)
    {
        unsigned i;
        assert(spi_address_calls < 5 && spi_address_length + size <= sizeof(spi_address_bytes));
        spi_address_sizes[spi_address_calls] = size;
        spi_address_dc[spi_address_calls++] = lcd_dc_state;
        for (i = 0; i < size; ++i)
        {
            spi_address_bytes[spi_address_length++] = data[i];
        }
    }
    if (spi_command_fail_after >= 0)
    {
        if (spi_command_fail_after-- == 0)
        {
            return spi_error;
        }
    }
    if (spi_capture && size > 1 && spi_lcd_command != 0x2A && spi_lcd_command != 0x2B)
    {
        unsigned i;
        assert(lcd_dc_state == GPIO_PIN_SET && size % 2 == 0);
        for (i = 0; i < size; i += 2)
        {
            assert(data[i] == (uint8_t)(spi_expected_color >> 8));
            assert(data[i + 1] == (uint8_t)spi_expected_color);
        }
        ++spi_payload_calls;
        spi_payload_bytes += size;
        if (spi_fail_after == 0)
        {
            return spi_error;
        }
        if (spi_fail_after > 0)
        {
            --spi_fail_after;
        }
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size)
{
    assert(spi == &hspi1 && data && size);
    assert((lcd_clock_mask & (HOST_LCD_CLOCK_SPI | HOST_LCD_CLOCK_DMA)) == (HOST_LCD_CLOCK_SPI | HOST_LCD_CLOCK_DMA));
    ++dma_starts;
    host_dma_stream.CR = DMA_SxCR_EN;
    return dma_start_result;
}
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *spi)
{
    assert(spi == &hspi1);
    ++dma_stops;
    if (dma_stuck != 0)
    {
        return HAL_TIMEOUT;
    }
    host_dma_stream.CR = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi)
{
    assert(spi == &hspi1 && host_dma_stream.CR == 0);
    ++spi_aborts;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(timer == &htim3 && channel == TIM_CHANNEL_3);
    assert((lcd_clock_mask & HOST_LCD_CLOCK_TIM) != 0);
    timer->Instance->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

static void queue_bit(GPIO_PinState bit)
{
    assert(reply_count < sizeof(replies) / sizeof(replies[0]));
    replies[reply_count++] = bit;
}
static void queue_read(uint8_t data)
{
    unsigned i;
    for (i = 0; i < 3; ++i)
    {
        queue_bit(GPIO_PIN_RESET); /* Address/register/read-address ACKs. */
    }
    for (i = 0; i < 8; ++i)
    {
        if ((data & (0x80U >> i)) != 0)
        {
            queue_bit(GPIO_PIN_SET);
        }
        else
        {
            queue_bit(GPIO_PIN_RESET);
        }
    }
}
static void queue_write(void)
{
    unsigned i;
    for (i = 0; i < 3; ++i)
    {
        queue_bit(GPIO_PIN_RESET);
    }
}
static void reset_observations(int state)
{
    assert(reply_pos == reply_count && !primask);
    scheduler_state = state;
    hal_waits = task_waits = 0;
    reply_count = reply_pos = 0;
    scl_edges = released_edges = tp_resets = 0;
    edges_at_reset = released_at_reset = 0;
}
static void init_delay_tests(int state)
{
    uint32_t start;
    reset_observations(state);
    start = elapsed_ms;
    LCD_Init();
    assert(elapsed_ms - start >= 320);
    assert(hal_waits + task_waits == 3);
    assert(htim3.arr == LCD_BL_ARR && htim3.compare == LCD_BL_ARR * 80 / 100);
    LCD_ST7789_SleepIn();
    assert(lcd_clock_mask == 0 && htim3.compare == 0 && (host_tim3.CR1 & TIM_CR1_CEN) == 0);
    LCD_ST7789_SleepOut();
    assert(lcd_clock_mask == 7 && htim3.compare == 0 && (host_tim3.CR1 & TIM_CR1_CEN) != 0);
    LCD_Set_Light(80);
    assert(htim3.compare == LCD_BL_ARR * 80 / 100);
    assert(hal_waits + task_waits == 5);
    assert(elapsed_ms - start >= 520);

    reset_observations(state);
    queue_read(0xB5);
    queue_write();
    start = elapsed_ms;
    assert(Touch_Init() == 0xB5 && g_tp_chip_id == 0xB5);
    assert(tp_resets == 1 && elapsed_ms - start >= 312);
    assert(reply_pos == reply_count && hal_waits + task_waits == 3);
    if (state == taskSCHEDULER_RUNNING)
    {
        assert(hal_waits == 0 && task_waits == 3);
    }
    else
    {
        assert(task_waits == 0 && hal_waits == 3);
    }
}
static void touch_recovery_tests(void)
{
    uint16_t x = 123, y = 456;
    uint32_t start;
    unsigned i, before_edges, before_released;
    reset_observations(taskSCHEDULER_RUNNING);
    queue_bit(GPIO_PIN_SET); /* First candidate absent; second candidate responds. */
    queue_read(0xB7);
    queue_write();
    start = elapsed_ms;
    Touch_Wakeup();
    assert(tp_resets == 1 && g_tp_chip_id == 0xB7 && elapsed_ms - start >= 410);
    assert(reply_pos == reply_count && hal_waits == 0 && task_waits == 3);
    /* Nine real rising edges before reset, with SDA released throughout. */
    assert(edges_at_reset == 10 && released_at_reset == 9);

    reset_observations(taskSCHEDULER_RUNNING);
    queue_read(0); /* A successful no-finger sample clears the failure streak. */
    assert(Touch_GetPoint(&x, &y) == 0 && x == 123 && y == 456);
    for (i = 0; i < 199; ++i)
    {
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    }
    assert(tp_resets == 0 && task_waits == 0);
    before_edges = scl_edges;
    before_released = released_edges;
    reply_count = reply_pos = 0;
    queue_bit(GPIO_PIN_SET); /* 200th read fails, then recovery succeeds. */
    queue_read(0xB5);
    queue_write();
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    assert(tp_resets == 1 && g_tp_chip_id == 0 && task_waits == 0 && hal_waits == 0);
    start = elapsed_ms;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && elapsed_ms == start);
    elapsed_ms += 9;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    ++elapsed_ms;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    elapsed_ms += 299;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && g_tp_chip_id == 0);
    ++elapsed_ms;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && g_tp_chip_id == 0xB5);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    assert(task_waits == 0 && hal_waits == 0 && elapsed_ms - start == 310);
    assert(edges_at_reset - before_edges == 20 && released_at_reset - before_released == 10);
    assert(reply_pos == reply_count);
    reply_count = reply_pos = 0;
    queue_read(1);
    queue_read(0); /* Coordinate read ACKs and XH, followed by XL/YH/YL. */
    for (i = 0; i < 24; ++i)
    {
        queue_bit(GPIO_PIN_RESET);
    }
    assert(Touch_GetPoint(&x, &y) == 1 && x == 239 && y == 279);
    assert(reply_pos == reply_count && tp_resets == 1);

    reset_observations(taskSCHEDULER_RUNNING);
    start = elapsed_ms;
    assert(Touch_Init() == 0); /* Both candidate addresses remain absent. */
    assert(tp_resets == 1 && elapsed_ms - start >= 1312);
    assert(hal_waits == 0 && task_waits == 13);
    reset_observations(taskSCHEDULER_RUNNING);
    start = elapsed_ms;
    Touch_Wakeup();
    assert(g_tp_chip_id == 0 && tp_resets == 1 && elapsed_ms - start >= 510);
    assert(hal_waits == 0 && task_waits == 4);
    queue_write();
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && reply_pos == reply_count);
}

static void lcd_address_tests(void)
{
    static const uint8_t expected_bytes[] = {0x2A, 0, 7, 0, 239, 0x2B, 0, 255, 1, 43, 0x2C};
    static const uint16_t expected_sizes[] = {1, 4, 1, 4, 1};
    static const GPIO_PinState expected_dc[] = {GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET};
    static const HAL_StatusTypeDef errors[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    static const uint16_t pixels[4] = {0};
    unsigned i, error, failure, before;

    spi_address_capture = 1;
    LCD_Address_Set(7, 235, 239, 279);
    assert(spi_address_calls == 5 && spi_address_length == sizeof(expected_bytes) && lcd_dc_state == GPIO_PIN_SET);
    for (i = 0; i < sizeof(expected_bytes); ++i)
    {
        assert(spi_address_bytes[i] == expected_bytes[i]);
    }
    for (i = 0; i < 5; ++i)
    {
        assert(spi_address_sizes[i] == expected_sizes[i] && spi_address_dc[i] == expected_dc[i]);
    }
    /* 任一命令/参数分组失败，阻塞刷屏不能继续发地址或像素。 */
    for (error = 0; error < sizeof(errors) / sizeof(errors[0]); ++error)
    {
        for (failure = 0; failure < 5; ++failure)
        {
            spi_address_calls = spi_address_length = 0;
            spi_command_fail_after = (int)failure;
            spi_error = errors[error];
            before = spi_calls;
            error_expected = 1;
            if (setjmp(error_jump) == 0)
            {
                LCD_Color_Fill(7, 235, 8, 236, pixels);
                assert(0 && "LCD_Color_Fill must stop on address failure");
            }
            error_expected = 0;
            assert(spi_calls == before + failure + 1 && spi_address_calls == failure + 1);
            assert(lcd_dc_state == GPIO_PIN_SET);
            spi_command_fail_after = -1;
        }
    }
    spi_address_capture = 0;
    puts("PASS: LCD address uses five groups with unchanged bytes/DC; every group stops on ERROR/BUSY/TIMEOUT at 50ms.");
}

static void lcd_fill_tests(void)
{
    static const uint16_t widths[] = {1, 63, 64, 65, 128, 130};
    static const HAL_StatusTypeDef errors[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    unsigned i, error, failure;

    spi_capture = 1;
    for (i = 0; i < sizeof(widths) / sizeof(widths[0]); ++i)
    {
        spi_payload_calls = spi_payload_bytes = 0;
        LCD_Fill(0, 0, widths[i] - 1, 0, 0x1234);
        assert(spi_payload_bytes == widths[i] * 2);
        assert(spi_payload_calls == (widths[i] + 63U) / 64);
    }
    /* RGB565原生色值按线上字节验证，同时覆盖整块和尾块。 */
    {
        static const uint16_t colors[] = {LCD_RED, LCD_GREEN, LCD_BLUE, LCD_BLACK, LCD_WHITE};
        for (i = 0; i < sizeof(colors) / sizeof(colors[0]); ++i)
        {
            spi_expected_color = colors[i];
            LCD_DrawPoint(0, 0, colors[i]);
            LCD_Fill(0, 0, 64, 0, colors[i]);
        }
        spi_expected_color = 0x1234;
    }
    /* Fail the first/second full chunk or the tail, and require the stop path. */
    for (error = 0; error < sizeof(errors) / sizeof(errors[0]); ++error)
    {
        for (failure = 0; failure < 3; ++failure)
        {
            spi_payload_calls = spi_payload_bytes = 0;
            spi_fail_after = failure;
            spi_error = errors[error];
            error_expected = 1;
            if (setjmp(error_jump) == 0)
            {
                LCD_Fill(0, 0, 129, 0, 0x1234);
                assert(0 && "LCD_Fill must enter Error_Handler on SPI failure");
            }
            error_expected = 0;
            assert(spi_payload_calls == failure + 1);
            spi_fail_after = -1;
        }
    }
    spi_capture = 0;
    puts("PASS: LCD_Fill chunk/tail bytes and colors, ERROR/BUSY/TIMEOUT stop without later payloads");
}

static void flush_done(void)
{
    assert(host_dma_stream.CR == 0);
    ++flush_completions;
}

static void lcd_dma_error_tests(void)
{
    static uint16_t pixels[LCD_WIDTH * LCD_HEIGHT];
    static const HAL_StatusTypeDef errors[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    unsigned i, error, before, starts, calls;
    LCD_Set_Flush_Complete_Callback(flush_done);
    for (i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i)
    {
        dma_start_result = errors[i];
        before = flush_completions;
        LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
        assert(flush_completions == before);
        assert(LCD_Flush_Service() == 1 && flush_completions == before + 1);
        HAL_SPI_TxCpltCallback(&hspi1);
        assert(LCD_Flush_Service() == 0 && flush_completions == before + 1);
    }
    dma_start_result = HAL_OK;
    for (error = 0; error < sizeof(errors) / sizeof(errors[0]); ++error)
    {
        for (i = 0; i < 5; ++i)
        {
            spi_command_fail_after = (int)i;
            spi_error = errors[error];
            starts = dma_starts;
            calls = spi_calls;
            LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
            assert(spi_calls == calls + i + 1 && dma_starts == starts && LCD_Flush_Service() == 1);
        }
    }
    spi_command_fail_after = -1;
    LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
    before = flush_completions;
    starts = spi_aborts;
    HAL_SPI_ErrorCallback(&hspi1);
    assert(spi_aborts == starts && flush_completions == before);
    dma_stuck = 1;
    assert(LCD_Flush_Service() == 0 && flush_completions == before);
    dma_stuck = 0;
    assert(LCD_Flush_Service() == 1 && flush_completions == before + 1);
    HAL_SPI_ErrorCallback(&hspi1);
    HAL_SPI_TxCpltCallback(&hspi1);
    assert(LCD_Flush_Service() == 0 && flush_completions == before + 1);
    LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
    host_dma_stream.CR = 0;
    HAL_SPI_TxCpltCallback(&hspi1);
    assert(flush_completions == before + 2 && LCD_Flush_Service() == 0);
    starts = dma_starts;
    LCD_Color_Fill_DMA(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, pixels);
    assert(dma_starts == starts && flush_completions == before + 3);
    for (i = 0; i < sizeof(pixels) / sizeof(pixels[0]); ++i)
    {
        pixels[i] = 0x1234;
    }
    spi_expected_color = 0x3412; /* 块缓冲已由调用方准备好线上字节序。 */
    spi_capture = 1;
    spi_fail_after = 0;
    spi_error = HAL_TIMEOUT;
    LCD_Color_Fill_DMA(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, pixels);
    assert(flush_completions == before + 3);
    assert(LCD_Flush_Service() == 1 && flush_completions == before + 4);
    spi_capture = 0;
    spi_fail_after = -1;
    assert(dma_stops == spi_aborts + 1);
    puts("PASS: LCD address/start/transfer errors stop DMA before releasing buffers, ignore late callbacks and allow the next frame.");
}

static void lcd_dma_timeout_tests(void)
{
    static uint16_t pixels[4];
    unsigned int before, stops, wrap;
    for (wrap = 0; wrap < 2; ++wrap)
    {
        elapsed_ms = 1000;
        if (wrap != 0)
        {
            elapsed_ms = UINT32_MAX - 50;
        }
        before = flush_completions;
        stops = dma_stops;
        LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
        elapsed_ms += 99;
        assert(LCD_Flush_Service() == 0 && dma_stops == stops);
        ++elapsed_ms;
        dma_stuck = 1;
        assert(LCD_Flush_Service() == 0 && flush_completions == before);
        dma_stuck = 0;
        assert(LCD_Flush_Service() == 1 && flush_completions == before + 1);
        HAL_SPI_TxCpltCallback(&hspi1);
        assert(flush_completions == before + 1 && LCD_Flush_Service() == 0);
        LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
        host_dma_stream.CR = 0;
        HAL_SPI_TxCpltCallback(&hspi1);
        elapsed_ms += 1000;
        assert(LCD_Flush_Service() == 0 && flush_completions == before + 2);
    }
    assert(primask == 0);
    puts("PASS: lost DMA callbacks time out at 100ms, wait for buffer safety, complete once and handle tick wrap.");
}

static void touch_coordinate_failure_tests(void)
{
    uint16_t x = 123, y = 456;
    uint32_t i, start;

    reset_observations(taskSCHEDULER_RUNNING);
    queue_read(0);
    assert(Touch_GetPoint(&x, &y) == 0);
    reply_count = reply_pos = 0;
    queue_read(0xFF); /* 扫描不可用不能充当真实松手，也不能覆盖旧坐标。 */
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && x == 123 && y == 456);
    for (i = 0; i < 198; ++i)
    {
        reply_count = reply_pos = 0;
        queue_read(1);
        queue_bit(GPIO_PIN_SET); /* 手指数可读，坐标事务NACK。 */
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && x == 123U && y == 456U);
    }
    assert(tp_resets == 0);
    reply_count = reply_pos = 0;
    queue_read(0); /* 无手指是成功采样，应清除连续失败计数。 */
    assert(Touch_GetPoint(&x, &y) == 0 && tp_resets == 0);
    for (i = 0; i < 200; ++i)
    {
        reply_count = reply_pos = 0;
        queue_read(1);
        queue_bit(GPIO_PIN_SET);
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && x == 123U && y == 456U);
        if (i < 199)
        {
            assert(tp_resets == 0);
        }
        elapsed_ms += 5;
    }
    assert(tp_resets == 1 && task_waits == 0 && hal_waits == 0);
    reply_count = reply_pos = 0;
    queue_read(0xB5);
    queue_write();
    start = elapsed_ms;
    elapsed_ms += 10;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    elapsed_ms += 300;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && g_tp_chip_id == 0xB5);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    assert(elapsed_ms - start == 310 && task_waits == 0 && hal_waits == 0);
    assert(reply_pos == reply_count);
    reply_count = reply_pos = 0;
    queue_read(1);
    queue_read(0);
    for (i = 0; i < 24; ++i)
    {
        queue_bit(GPIO_PIN_RESET);
    }
    assert(Touch_GetPoint(&x, &y) == 1 && x == 239U && y == 279U);
    assert(reply_pos == reply_count && tp_resets == 1);
    puts("PASS: coordinate-only NACKs trigger bounded deferred recovery; no-finger success resets the streak.");
}

static void touch_sda_handoff_tests(void)
{
    static const uint8_t values[] = {0x00, 0x80, 0x01, 0xFF};
    uint16_t x = 123, y = 456;
    unsigned i, before = handoff_checks;

    reset_observations(taskSCHEDULER_RUNNING);
    for (i = 0; i < sizeof(values); ++i)
    {
        reply_count = reply_pos = 0;
        queue_read(values[i]);
        handoff_start_reply = 3;
        handoff_bytes = 1;
        assert(Touch_GetChipID() == values[i] && reply_pos == reply_count);
        assert(handoff_active == 0 && handoff_checks == before + i + 1);
    }
    reply_count = reply_pos = 0;
    queue_read(1);
    queue_read(0x80);
    for (i = 0; i < 24; ++i)
    {
        queue_bit(GPIO_PIN_RESET);
    }
    handoff_start_reply = 14;
    handoff_bytes = 4;
    assert(Touch_GetPoint(&x, &y) == 1 && x == 239 && y == 279);
    assert(reply_pos == reply_count && handoff_checks == before + 5 && handoff_active == 0);
    puts("PASS: SDA stays released from read-address ACK through data; ACK takes over low and final NACK releases SDA.");
}

static void touch_invalid_scan_tests(void)
{
    uint16_t x = 123, y = 456;
    uint32_t start;
    unsigned i;

    reset_observations(taskSCHEDULER_RUNNING);
    queue_read(0);
    assert(Touch_GetPoint(&x, &y) == 0);
    start = elapsed_ms;
    for (i = 0; i < 2000; ++i)
    {
        reply_count = reply_pos = 0;
        queue_read(0xFF);
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && x == 123 && y == 456);
        if (i < 199)
        {
            assert(tp_resets == 0);
        }
        else
        {
            assert(tp_resets != 0);
        }
        elapsed_ms += 5;
    }
    assert(elapsed_ms - start == 10000 && task_waits == 0 && hal_waits == 0);
    reply_count = reply_pos = 0;
    puts("PASS: 2000 invalid scans over 10 seconds trigger deferred recovery at the 200th failure, without publishing a release.");
}

static void touch_config_failure_tests(void)
{
    uint16_t x = 123, y = 456;
    unsigned i, before;

    reset_observations(taskSCHEDULER_RUNNING);
    queue_read(0xB5);
    queue_write();
    assert(Touch_Init() == 0xB5); /* 清除上一场景的恢复状态。 */
    reply_count = reply_pos = 0;
    queue_read(0xB5);
    queue_bit(GPIO_PIN_SET); /* ChipID 正常，但关闭自动休眠写入 NACK。 */
    assert(Touch_Init() == 0 && g_tp_chip_id == 0xB5 && reply_pos == reply_count);
    hal_waits = task_waits = 0;
    before = tp_resets;
    for (i = 0; i < 198; ++i)
    {
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && x == 123 && y == 456);
    }
    assert(tp_resets == before);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && tp_resets == before + 1);
    elapsed_ms += 10;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    elapsed_ms += 300;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    reply_count = reply_pos = 0;
    queue_read(0xB5);
    queue_bit(GPIO_PIN_SET); /* 异步恢复中的配置失败也必须继续返回错误。 */
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && g_tp_chip_id == 0xB5);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && reply_pos == reply_count);
    reply_count = reply_pos = 0;
    queue_write();
    queue_read(0);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && reply_pos == 3);
    assert(Touch_GetPoint(&x, &y) == 0 && reply_pos == reply_count);
    assert(task_waits == 0 && hal_waits == 0);

    reply_count = reply_pos = 0;
    queue_read(0xB5);
    queue_bit(GPIO_PIN_SET);
    Touch_Wakeup();
    assert(reply_pos == reply_count);
    hal_waits = task_waits = 0;
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && x == 123 && y == 456);
    reply_count = reply_pos = 0;
    queue_write();
    queue_read(0);
    assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && reply_pos == 3);
    assert(Touch_GetPoint(&x, &y) == 0 && reply_pos == reply_count);
    assert(task_waits == 0 && hal_waits == 0);
    puts("PASS: init, wake and deferred recovery retain config failures; persistent config NACKs reset the chip and success resumes sampling.");
}

static void touch_async_absent_tests(void)
{
    uint16_t x = 123, y = 456;
    uint32_t before;
    unsigned i;
    reset_observations(taskSCHEDULER_RUNNING);
    elapsed_ms = UINT32_MAX - 5;
    for (i = 0; i < 200; ++i)
    {
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR);
    }
    assert(tp_resets == 1 && task_waits == 0 && hal_waits == 0);
    for (i = 0; i < 120; ++i)
    {
        elapsed_ms += 5;
        before = elapsed_ms;
        assert(Touch_GetPoint(&x, &y) == TOUCH_READ_ERROR && elapsed_ms == before);
    }
    assert(g_tp_chip_id == 0 && x == 123 && y == 456);
    assert(tp_resets == 1 && task_waits == 0 && hal_waits == 0);
    puts("PASS: absent touch recovers across tick wrap without sleeping in input callbacks or publishing stale coordinates.");
}

int main(void)
{
    lcd_address_tests();
    lcd_fill_tests();
    lcd_dma_error_tests();
    lcd_dma_timeout_tests();
    init_delay_tests(taskSCHEDULER_NOT_STARTED);
    init_delay_tests(taskSCHEDULER_RUNNING);
    init_delay_tests(taskSCHEDULER_SUSPENDED);
    touch_recovery_tests();
    touch_coordinate_failure_tests();
    touch_sda_handoff_tests();
    touch_invalid_scan_tests();
    touch_config_failure_tests();
    touch_async_absent_tests();
    puts("PASS: LCD sleep gates idle SPI/DMA/PWM clocks and wake restores clocks before commands, retaining a dark backlight until brightness restore.");
    puts("PASS: real LCD/touch init and sleep delays, reset/probe recovery, failure threshold and coordinates");
    return 0;
}

void host_i2c_nop(void)
{
}
