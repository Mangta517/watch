#include "kt6368a.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1 = {USART1, {115200}};
static uint32_t tick, mask;
static uint8_t *rx_target;
static int rx_armed, powered, connected, boots, writes, silent, reject_setting;
static int fail_tx, fail_rx;
static uint32_t uart_gpio_mode;
static char saved_name[31] = "F411-Watch-SPP", current_name[31];
static char saved_ble_name[31] = "KT6368A-BLE", current_ble_name[31];
static char saved_ble[3] = "00", saved_spp[3] = "01";
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
    assert(size < sizeof(command));
    memcpy(command, data, size);
    command[size] = '\0';
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

int main(void)
{
    uint8_t buffer[1024];
    char response[64], long_reply[90];
    int before, i;
    uint32_t errors;

    assert(kt6368a_get_init_status() == HAL_ERROR);
    /* 从旧SPP开启配置迁移到仅BLE，重新上电后生效，SPP名称保持不变。 */
    assert(kt6368a_init() == HAL_OK);
    assert(kt6368a_get_init_status() == HAL_OK);
    assert(strcmp(current_ble_name, KT6368A_BLE_NAME) == 0);
    assert(strcmp(current_name, "F411-Watch-SPP") == 0);
    assert(strcmp(current_spp, "00") == 0 && strcmp(current_ble, "01") == 0);
    assert(boots == 2 && writes == 3 && !kt6368a_is_connected());
    before = writes;
    assert(kt6368a_init() == HAL_OK);
    assert(writes == before); /* 已保存的配置不重复写Flash */

    /* 模块拒绝关闭SPP时必须报告失败，后续正常设置可恢复。 */
    strcpy(saved_spp, "01");
    strcpy(current_spp, "01");
    reject_setting = 1;
    assert(kt6368a_configure(KT6368A_BLE_NAME) == HAL_ERROR);
    assert(strcmp(current_spp, "01") == 0);
    reject_setting = 0;
    assert(kt6368a_configure(KT6368A_BLE_NAME) == HAL_OK);
    assert(strcmp(current_spp, "00") == 0 && strcmp(current_ble, "01") == 0);

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
    return 0;
}
