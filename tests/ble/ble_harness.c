#include "kt6368a.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#if defined(F411_BOOTLOADER)
#define TEST_EXPECTED_SPP "01"
#define TEST_INITIAL_SPP  "00"
#else
#define TEST_EXPECTED_SPP "00"
#define TEST_INITIAL_SPP  "01"
#endif

UART_HandleTypeDef huart1 = {USART1, {115200}};
static uint32_t tick, mask, delay_calls;
static uint8_t *rx_target;
static int rx_armed, powered, connected, boots, writes, silent, reject_setting;
static int fail_tx, fail_rx;
static unsigned query_count, drop_queries;
static uint8_t query_uart_error;
static uint8_t *tx_target;
static uint16_t tx_size;
static uint32_t tx_aborts;
static HAL_StatusTypeDef async_start_result = HAL_OK;
static uint32_t uart_gpio_mode;
static char saved_name[31] = "F411-Watch-SPP", current_name[31];
static char saved_ble_name[31] = "KT6368A-BLE", current_ble_name[31];
static char saved_ble[3] = "00", saved_spp[3] = TEST_INITIAL_SPP;
static char current_ble[3], current_spp[3];
static char pending[256];
static size_t pending_pos, pending_size;
static const char *custom_reply;

static void inject(uint8_t byte)
{
    assert(rx_armed && rx_target != NULL);
    rx_armed = 0;
    *rx_target = byte;
    HAL_UART_RxCpltCallback(&huart1);
}

static void advance(void)
{
    tick++;
    if (powered && rx_armed && pending_pos < pending_size)
    {
        inject((uint8_t)pending[pending_pos++]);
    }
}

static void reply(const char *text)
{
    assert(strlen(text) < sizeof(pending));
    strcpy(pending, text);
    pending_pos = 0;
    pending_size = strlen(text);
}

uint32_t HAL_GetTick(void)
{
    advance();
    return tick;
}
void HAL_Delay(uint32_t ms)
{
    ++delay_calls;
    while (ms--)
    {
        advance();
    }
}
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
void __DMB(void)
{
}
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOA && gpio->Pin == (BLE_TX_Pin | BLE_RX_Pin));
    uart_gpio_mode = gpio->Mode;
}
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState value)
{
    assert(port == BLE_EN_GPIO_Port && pin == BLE_EN_Pin);
    if (value == GPIO_PIN_SET && !powered)
    {
        boots++;
        strcpy(current_name, saved_name);
        strcpy(current_ble_name, saved_ble_name);
        strcpy(current_ble, saved_ble);
        strcpy(current_spp, saved_spp);
        reply("AT+VER2.0\r\nTD+boot-info\r\n");
    }
    powered = (value == GPIO_PIN_SET);
    if (!powered)
    {
        pending_size = pending_pos = 0;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    assert(0 && "BLE driver must not read the heart LED or any unconnected status GPIO");
    return GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && size == 1);
    if (fail_rx)
    {
        return HAL_ERROR;
    }
    if (rx_armed)
    {
        return HAL_BUSY;
    }
    rx_target = data;
    rx_armed = 1;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(uart == &huart1);
    rx_armed = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    assert(uart == &huart1 && powered && tx_target == NULL);
    if (async_start_result != HAL_OK)
    {
        return async_start_result;
    }
    tx_target = data;
    tx_size = size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    assert(uart == &huart1);
    tx_target = NULL;
    ++tx_aborts;
    return HAL_OK;
}

static void async_tx_tests(void)
{
    uint8_t frame[] = "WDG1 test";
    HAL_StatusTypeDef result;
    uint32_t before = tick;
    uint32_t aborts = tx_aborts;
    assert(kt6368a_write_async(NULL, 1, 10) == HAL_ERROR);
    assert(kt6368a_write_async(frame, 0, 10) == HAL_ERROR);
    assert(kt6368a_write_async(frame, 513, 10) == HAL_ERROR);
    assert(kt6368a_write_async(frame, sizeof(frame), 0) == HAL_ERROR);
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_OK);
    assert(tick - before < 5 && tx_target == frame && tx_size == sizeof(frame));
    assert(kt6368a_tx_is_busy() && !kt6368a_take_tx_result(&result));
    assert(kt6368a_write(frame, sizeof(frame), 10) == HAL_BUSY);
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_BUSY);
    inject(0xA5); /* TX期间原RX仍持续入队。 */
    HAL_UART_ErrorCallback(&huart1);
    assert(kt6368a_tx_is_busy() && rx_armed);
    tx_target = NULL;
    HAL_UART_TxCpltCallback(&huart1);
    assert(kt6368a_tx_is_busy()); /* 完成结果归原发起者，不允许新异步写入抢占。 */
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_BUSY);
    assert(kt6368a_take_tx_result(&result) && result == HAL_OK);
    assert(!kt6368a_tx_is_busy());
    assert(!kt6368a_take_tx_result(&result));
    for (unsigned i = HAL_BUSY; i <= HAL_TIMEOUT; ++i)
    {
        async_start_result = (HAL_StatusTypeDef)i;
        assert(kt6368a_write_async(frame, sizeof(frame), 10) == (HAL_StatusTypeDef)i);
        assert(!kt6368a_tx_is_busy() && !kt6368a_take_tx_result(&result));
    }
    async_start_result = HAL_ERROR;
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_ERROR);
    async_start_result = HAL_OK;
    tick = UINT32_MAX - 5U;
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_OK);
    for (unsigned i = 0; i < 10; ++i)
    {
        advance();
    }
    assert(kt6368a_take_tx_result(&result) && result == HAL_TIMEOUT);
    assert(tx_target == NULL && tx_aborts == aborts + 1U && rx_armed && mask == 0);
    HAL_UART_TxCpltCallback(&huart1);
    assert(!kt6368a_take_tx_result(&result));
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_OK);
    kt6368a_power_off();
    assert(tx_target == NULL && !rx_armed && tx_aborts == aborts + 2U);
    assert(kt6368a_take_tx_result(&result) && result == HAL_ERROR);
    assert(kt6368a_write_async(frame, sizeof(frame), 10) == HAL_BUSY);
    assert(kt6368a_power_on() == HAL_OK);
    kt6368a_flush_rx();
    puts("PASS: async TX returns promptly, preserves RX, serializes writers, completes once, aborts on timeout/wrap and power off.");
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size, uint32_t timeout)
{
    char command[64], response[64];
    (void)timeout;
    assert(uart == &huart1 && powered && uart_gpio_mode == GPIO_MODE_AF_PP);
    if (fail_tx)
    {
        return HAL_ERROR;
    }
    if (connected)
    {
        return HAL_OK;
    }
    if (size >= sizeof(command))
    {
        return HAL_ERROR;
    }
    memcpy(command, data, size);
    command[size] = '\0';
    if (size >= 6U && strncmp(command, "AT+T", 4) == 0)
    {
        ++query_count;
        if (drop_queries != 0)
        {
            --drop_queries;
            return HAL_OK;
        }
        if (query_uart_error != 0)
        {
            query_uart_error = 0;
            HAL_UART_ErrorCallback(&huart1);
        }
    }
    if (silent)
    {
        return HAL_OK;
    }
    if (custom_reply != NULL)
    {
        reply(custom_reply);
        return HAL_OK;
    }
    if (strcmp(command, "AT+TM\r\n") == 0)
    {
        snprintf(response, sizeof(response), "OK\r\nTM+%s\r\n", current_ble_name);
        reply(response);
    }
    else if (strcmp(command, "AT+TD\r\n") == 0)
    {
        snprintf(response, sizeof(response), "OK\r\nTD+%s\r\n", current_name);
        reply(response);
    }
    else if (strcmp(command, "AT+T4\r\n") == 0)
    {
        snprintf(response, sizeof(response), "T4+%s\r\n", current_ble);
        reply(response);
    }
    else if (strcmp(command, "AT+T5\r\n") == 0)
    {
        snprintf(response, sizeof(response), "T5+%s\r\n", current_spp);
        reply(response);
    }
    else if (strncmp(command, "AT+BM", 5) == 0 || strncmp(command, "AT+BD", 5) == 0)
    {
        writes++;
        if (!reject_setting)
        {
            char *value = command[4] == 'D' ? saved_name : saved_ble_name;
            memcpy(value, command + 5, size - 7U);
            value[size - 7U] = '\0';
        }
        reply(reject_setting ? "ER+4\r\n" : "OK\r\n");
    }
    else if (strncmp(command, "AT+B4", 5) == 0 || strncmp(command, "AT+B5", 5) == 0)
    {
        writes++;
        if (!reject_setting)
        {
            char *value = command[4] == '4' ? saved_ble : saved_spp;
            memcpy(value, command + 5, 2);
        }
        reply(reject_setting ? "ER+4\r\n" : "OK\r\n");
    }
    else
    {
        reply("ER+2\r\n");
    }
    return HAL_OK;
}

static HAL_StatusTypeDef run_staged_startup(void)
{
    uint32_t delays = delay_calls;
    HAL_StatusTypeDef status = HAL_BUSY;
    kt6368a_init_begin();
    assert(kt6368a_init_is_pending() && kt6368a_get_init_status() == HAL_BUSY);
    assert(kt6368a_write((const uint8_t *)"WDG1", 4, 10) == HAL_BUSY);
    for (unsigned i = 0; i < 1200 && kt6368a_init_is_pending(); ++i)
    {
        uint32_t before = tick;
        status = kt6368a_init_service();
        assert((uint32_t)(tick - before) < 10U);
        for (unsigned j = 0; j < 5; ++j)
        {
            advance();
        }
    }
    assert(!kt6368a_init_is_pending() && delay_calls == delays);
    assert(status == kt6368a_get_init_status());
    return status;
}
static void staged_startup_tests(void)
{
    int old_writes, old_boots;
    silent = fail_tx = fail_rx = reject_setting = connected = 0;
    custom_reply = NULL;
    strcpy(saved_ble_name, "KT6368A-BLE");
    strcpy(saved_ble, "00");
    strcpy(saved_spp, TEST_INITIAL_SPP);
    old_writes = writes;
    old_boots = boots;
    tick = UINT32_MAX - 500U;
    assert(run_staged_startup() == HAL_OK);
    assert(writes == old_writes + 3 && boots == old_boots + 2);
    assert(strcmp(current_ble_name, KT6368A_BLE_NAME) == 0);
    assert(strcmp(current_spp, TEST_EXPECTED_SPP) == 0 && strcmp(current_ble, "01") == 0);
    old_writes = writes;
    old_boots = boots;
    assert(run_staged_startup() == HAL_OK);
    assert(writes == old_writes && boots == old_boots + 1);
    query_count = 0;
    drop_queries = 1;
    old_boots = boots;
    assert(run_staged_startup() == HAL_OK && query_count == 4U && boots == old_boots + 1);
    query_count = 0;
    query_uart_error = 1;
    assert(run_staged_startup() == HAL_OK && query_count == 4U);
    silent = 1;
    query_count = 0;
    assert(run_staged_startup() == HAL_TIMEOUT && query_count == 3U);
    silent = 0;
    fail_tx = 1;
    assert(run_staged_startup() == HAL_ERROR);
    fail_tx = 0;
    fail_rx = 1;
    assert(run_staged_startup() == HAL_ERROR);
    fail_rx = 0;
    reject_setting = 1;
    strcpy(saved_spp, TEST_INITIAL_SPP);
    assert(run_staged_startup() == HAL_ERROR);
    reject_setting = 0;
    assert(run_staged_startup() == HAL_OK);
    puts("PASS: staged Bluetooth boot/config/reboot, pending TX gate, wrap, no HAL_Delay, failures and recovery");
}

int main(void)
{
    uint8_t buffer[1024];
    char response[64], long_reply[90];
    int before, i;
    uint32_t errors;

    assert(kt6368a_get_init_status() == HAL_ERROR);
    /* 从相反SPP配置迁移到当前APP/升级模式，重新上电后生效，SPP名称保持不变。 */
    assert(kt6368a_init() == HAL_OK);
    assert(kt6368a_get_init_status() == HAL_OK);
    assert(strcmp(current_ble_name, KT6368A_BLE_NAME) == 0);
    assert(strcmp(current_name, "F411-Watch-SPP") == 0);
    assert(strcmp(current_spp, TEST_EXPECTED_SPP) == 0 && strcmp(current_ble, "01") == 0);
    assert(boots == 2 && writes == 3 && !kt6368a_is_connected());
    before = writes;
    assert(kt6368a_init() == HAL_OK);
    assert(writes == before); /* 已保存的配置不重复写Flash */

    /* 模块拒绝切换SPP时必须报告失败，后续正常设置可恢复。 */
    strcpy(saved_spp, TEST_INITIAL_SPP);
    strcpy(current_spp, TEST_INITIAL_SPP);
    reject_setting = 1;
    assert(kt6368a_configure(KT6368A_BLE_NAME) == HAL_ERROR);
    assert(strcmp(current_spp, TEST_INITIAL_SPP) == 0);
    reject_setting = 0;
    assert(kt6368a_configure(KT6368A_BLE_NAME) == HAL_OK);
    assert(strcmp(current_spp, TEST_EXPECTED_SPP) == 0 && strcmp(current_ble, "01") == 0);

    /* 实际ISR接收路径：字节顺序、环形回绕、满缓冲丢弃计数。 */
    for (i = 0; i < 900; ++i)
    {
        inject((uint8_t)i);
    }
    assert(kt6368a_read(buffer, 800) == 800);
    for (i = 0; i < 800; ++i)
    {
        assert(buffer[i] == (uint8_t)i);
    }
    for (i = 900; i < 1500; ++i)
    {
        inject((uint8_t)i);
    }
    assert(kt6368a_read(buffer, sizeof(buffer)) == 700);
    for (i = 0; i < 700; ++i)
    {
        assert(buffer[i] == (uint8_t)(i + 800));
    }
    for (i = 0; i < 1033; ++i)
    {
        inject((uint8_t)i);
    }
    assert(kt6368a_get_rx_dropped() == 10);
    assert(kt6368a_read(buffer, sizeof(buffer)) == 1023);
    for (i = 0; i < 1023; ++i)
    {
        assert(buffer[i] == (uint8_t)i);
    }

    errors = kt6368a_get_rx_errors();
    HAL_UART_ErrorCallback(&huart1);
    assert(kt6368a_get_rx_errors() == errors + 1 && rx_armed);
    inject(0xA5);
    assert(kt6368a_read(buffer, sizeof(buffer)) == 1 && buffer[0] == 0xA5);

    connected = 1;
    assert(!kt6368a_is_connected()); /* 手机连接也不能通过本板 GPIO 确认 */
    assert(kt6368a_write((const uint8_t *)"hello", 5, 100) == HAL_OK);
    assert(kt6368a_write(buffer, 513, 100) == HAL_ERROR);
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), 100) == HAL_TIMEOUT);
    connected = 0;
    /* 无状态脚时不按连接状态拦截，UART 成功不保证手机收到。 */
    assert(kt6368a_write(buffer, 1, 100) == HAL_OK);
    async_tx_tests();
    assert(kt6368a_at_command("AT+TM", "TM+", response, sizeof(response), 100) == HAL_ERROR);
    assert(kt6368a_configure("bad\r\nname") == HAL_ERROR);

    custom_reply = "OK\r\nTM+fragmented\r\n";
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), 100) == HAL_OK);
    assert(strcmp(response, "TM+fragmented") == 0);
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, 4, 100) == HAL_ERROR);
    custom_reply = "ER+3\r\n";
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), 100) == HAL_ERROR);
    memset(long_reply, 'x', sizeof(long_reply));
    long_reply[87] = '\r';
    long_reply[88] = '\n';
    long_reply[89] = '\0';
    custom_reply = long_reply;
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), 200) == HAL_ERROR);
    custom_reply = NULL;
    silent = 1;
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), 50) == HAL_TIMEOUT);
    silent = 0;
    fail_tx = 1;
    assert(kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), 50) == HAL_ERROR);
    fail_tx = 0;

    reject_setting = 1;
    assert(kt6368a_configure("Rejected-Name") == HAL_ERROR);
    reject_setting = 0;
    kt6368a_power_off();
    assert(!powered && !rx_armed && uart_gpio_mode == GPIO_MODE_INPUT);
    assert(kt6368a_write(buffer, 1, 100) == HAL_BUSY);
    assert(!kt6368a_is_connected() && kt6368a_read(buffer, sizeof(buffer)) == 0);
    fail_rx = 1;
    assert(kt6368a_power_on() == HAL_ERROR && !powered);
    fail_rx = 0;
    tick = UINT32_MAX - 600U;
    assert(kt6368a_power_on() == HAL_OK); /* HAL tick回绕不影响启动判断 */
    assert(!kt6368a_is_connected());
    connected = 1;
    assert(!kt6368a_is_connected()); /* 手机连接也不能通过本板 GPIO 确认 */
    connected = 0;
    silent = 1;
    assert(kt6368a_init() == HAL_TIMEOUT);
    assert(kt6368a_get_init_status() == HAL_TIMEOUT);
    puts("KT6368A: config persistence, RX wrap/overflow, UART error recovery, AT framing/timeouts, power gating and absence of status GPIO checks passed.");
    staged_startup_tests();
    return 0;
}
