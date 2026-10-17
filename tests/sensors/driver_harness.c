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
TIM_HandleTypeDef htim3;
static int scheduler_state;
static uint32_t primask, elapsed_ms, hal_waits, task_waits;
static uint32_t tp_reset_start, tp_ready_at, lcd_reset_start;
static unsigned tp_resets, scl_edges, released_edges, edges_at_reset, released_at_reset;
static GPIO_PinState scl_state = GPIO_PIN_SET;
static uint32_t sda_mode;
static GPIO_PinState replies[128];
static unsigned reply_count, reply_pos;
static jmp_buf error_jump;
static uint8_t error_expected, spi_capture, recovery_sample;
static int spi_fail_after = -1;
static HAL_StatusTypeDef spi_error;
static unsigned spi_payload_calls, spi_payload_bytes;
static int spi_command_fail_after = -1;
static HAL_StatusTypeDef dma_start_result = HAL_OK;
static unsigned dma_starts, dma_stops, spi_aborts, flush_completions;
static uint8_t dma_stuck;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi);

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
    sda_mode = config->Mode;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (port == TP_SCL_GPIO_Port && pin == TP_SCL_Pin)
    {
        if (state == GPIO_PIN_SET && scl_state == GPIO_PIN_RESET)
        {
            ++scl_edges;
            if (sda_mode == GPIO_MODE_INPUT)
            {
                ++released_edges;
            }
        }
        scl_state = state;
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
        return replies[reply_pos++];
    }
    return GPIO_PIN_SET; /* Unconnected device NACKs. */
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq == DMA2_Stream2_IRQn && priority == 12 && subpriority == 0);
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi1 && data && size && timeout);
    if (size == 1 && spi_command_fail_after >= 0)
    {
        if (spi_command_fail_after-- == 0)
        {
            return HAL_ERROR;
        }
    }
    if (spi_capture && size > 1)
    {
        unsigned i;
        const uint16_t *pixels = (const uint16_t *)data;
        assert(size % 2 == 0);
        for (i = 0; i < size / 2U; ++i)
        {
            assert(pixels[i] == 0x1234);
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
        queue_bit((data & (0x80U >> i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    LCD_ST7789_SleepOut();
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
        assert(Touch_GetPoint(&x, &y) == 0);
    }
    assert(tp_resets == 0 && task_waits == 0);
    before_edges = scl_edges;
    before_released = released_edges;
    reply_count = reply_pos = 0;
    queue_bit(GPIO_PIN_SET); /* 200th read fails, then recovery succeeds. */
    queue_read(0xB5);
    queue_write();
    assert(Touch_GetPoint(&x, &y) == 0);
    assert(tp_resets == 1 && g_tp_chip_id == 0xB5 && task_waits == 2 && hal_waits == 0);
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
        assert(spi_payload_bytes == widths[i] * 2U);
        assert(spi_payload_calls == (widths[i] + 63U) / 64U);
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
    unsigned i, before, starts;
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
    for (i = 0; i < 11; ++i)
    {
        spi_command_fail_after = (int)i;
        starts = dma_starts;
        LCD_Color_Fill_DMA(0, 0, 1, 1, pixels);
        assert(dma_starts == starts && LCD_Flush_Service() == 1);
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

int main(void)
{
    lcd_fill_tests();
    lcd_dma_error_tests();
    init_delay_tests(taskSCHEDULER_NOT_STARTED);
    init_delay_tests(taskSCHEDULER_RUNNING);
    init_delay_tests(taskSCHEDULER_SUSPENDED);
    touch_recovery_tests();
    puts("PASS: real LCD/touch init and sleep delays, reset/probe recovery, failure threshold and coordinates");
    return 0;
}

void host_i2c_nop(void)
{
}
