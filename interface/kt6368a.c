/**
 ******************************************************************************
 * @file    kt6368a.c
 * @brief   KT6368A驱动。USART1 ISR只入队/重启接收，不调用FreeRTOS API。
 ******************************************************************************
 */
#include "kt6368a.h"
#include "usart.h"
#include <string.h>

typedef struct
{
    uint8_t rx_byte;
    uint8_t rx_buffer[KT6368A_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_dropped;
    volatile uint32_t rx_errors;
    volatile uint8_t powered;
    uint32_t power_on_tick;
} kt6368a_t;

static kt6368a_t ble;
static HAL_StatusTypeDef init_status = HAL_ERROR;

uint16_t kt6368a_read(uint8_t *data, uint16_t size)
{
    uint16_t count = 0;
    if (data == NULL)
    {
        return 0;
    }
    while ((count < size) && (ble.rx_tail != ble.rx_head))
    {
        __DMB();
        data[count++] = ble.rx_buffer[ble.rx_tail];
        __DMB();
        ble.rx_tail = (uint16_t)((ble.rx_tail + 1U) % KT6368A_RX_BUFFER_SIZE);
    }
    return count;
}

void kt6368a_flush_rx(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ble.rx_tail = ble.rx_head;
    __set_PRIMASK(mask);
}

uint32_t kt6368a_get_rx_dropped(void)
{
    return ble.rx_dropped;
}
uint32_t kt6368a_get_rx_errors(void)
{
    return ble.rx_errors;
}
HAL_StatusTypeDef kt6368a_get_init_status(void)
{
    return init_status;
}

uint8_t kt6368a_is_connected(void)
{
    /* KT_LED 未接到 STM32，保留接口：0 表示无法确认连接，不能用心率 PB15 推断。 */
    return 0;
}

void kt6368a_power_off(void)
{
    GPIO_InitTypeDef gpio = {0};
    ble.powered = 0;
    (void)HAL_UART_AbortReceive(&huart1);
    /* BL1551B关断时将蓝牙电源接地；UART闲置高电平不能继续驱动模块。 */
    gpio.Pin = BLE_TX_Pin | BLE_RX_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);
    kt6368a_flush_rx();
}

HAL_StatusTypeDef kt6368a_power_on(void)
{
    GPIO_InitTypeDef gpio = {0};
    HAL_StatusTypeDef status;
    if (ble.powered)
    {
        return HAL_OK;
    }
    /* init调用位置保证UART已初始化；保留CubeMX生成的115200/8N1配置。 */
    if ((huart1.Instance != USART1) || (huart1.Init.BaudRate != 115200U))
    {
        return HAL_ERROR;
    }
    HAL_Delay(20); /* 重新上电前给电源电容放电时间 */
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_SET);
    ble.power_on_tick = HAL_GetTick();
    gpio.Pin = BLE_TX_Pin | BLE_RX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    kt6368a_flush_rx();
    ble.powered = 1;
    status = HAL_UART_Receive_IT(&huart1, &ble.rx_byte, 1);
    if (status != HAL_OK)
    {
        kt6368a_power_off();
        return status;
    }
    HAL_Delay(KT6368A_BOOT_DELAY_MS);
    return HAL_OK;
}

HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > KT6368A_TX_MAX_SIZE))
    {
        return HAL_ERROR;
    }
    if (!ble.powered || ((uint32_t)(HAL_GetTick() - ble.power_on_tick) < KT6368A_BOOT_DELAY_MS))
    {
        return HAL_BUSY;
    }
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, size, timeout_ms);
}

HAL_StatusTypeDef kt6368a_at_command(const char *command, const char *expected_prefix, char *response, uint16_t response_size,
                                     uint32_t timeout_ms)
{
    char line[64];
    uint16_t length = 0;
    uint8_t byte, too_long = 0;
    uint32_t start, errors, dropped;
    size_t command_size, prefix_size;
    HAL_StatusTypeDef status;

    if ((command == NULL) || (expected_prefix == NULL) || (response == NULL) || (response_size == 0U) || (timeout_ms == 0U))
    {
        return HAL_ERROR;
    }
    response[0] = '\0';
    command_size = strlen(command);
    prefix_size = strlen(expected_prefix);
    if ((command_size < 7U) || (command_size > 63U) || (prefix_size == 0U) || (strncmp(command, "AT+", 3) != 0) ||
        (strcmp(command + command_size - 2U, "\r\n") != 0))
    {
        return HAL_ERROR;
    }
    /* 防止一次调用注入多条AT命令。 */
    if ((memchr(command, '\r', command_size - 2U) != NULL) || (memchr(command, '\n', command_size - 2U) != NULL))
    {
        return HAL_ERROR;
    }
    if (!ble.powered)
    {
        return HAL_BUSY;
    }
    kt6368a_flush_rx();
    errors = ble.rx_errors;
    dropped = ble.rx_dropped;
    start = HAL_GetTick();
    status = HAL_UART_Transmit(&huart1, (uint8_t *)command, (uint16_t)command_size, timeout_ms);
    if (status != HAL_OK)
    {
        return status;
    }

    while ((uint32_t)(HAL_GetTick() - start) < timeout_ms)
    {
        if ((ble.rx_errors != errors) || (ble.rx_dropped != dropped))
        {
            return HAL_ERROR;
        }
        if (kt6368a_read(&byte, 1) == 0U)
        {
            continue;
        }
        if (byte == '\n')
        {
            if (too_long)
            {
                return HAL_ERROR;
            }
            line[length] = '\0';
            if (strncmp(line, "ER+", 3) == 0)
            {
                return HAL_ERROR;
            }
            if (strncmp(line, expected_prefix, prefix_size) == 0)
            {
                if (length >= response_size)
                {
                    return HAL_ERROR;
                }
                memcpy(response, line, length + 1U);
                return HAL_OK;
            }
            length = 0;
        }
        else if (byte != '\r')
        {
            if (length < sizeof(line) - 1U)
            {
                line[length++] = (char)byte;
            }
            else
            {
                too_long = 1;
            }
        }
    }
    return HAL_TIMEOUT;
}

/* 先查后改；设置响应可能被CR00关闭，因此依据后续查询和重新上电验证结果。 */
static HAL_StatusTypeDef kt6368a_set_if_needed(const char *query, const char *prefix, const char *value, const char *setting,
                                               uint8_t *changed)
{
    char response[64];
    HAL_StatusTypeDef status;
    status = kt6368a_at_command(query, prefix, response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    if (strcmp(response + strlen(prefix), value) == 0)
    {
        return HAL_OK;
    }
    status = HAL_UART_Transmit(&huart1, (uint8_t *)setting, (uint16_t)strlen(setting), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    HAL_Delay(100);
    *changed = 1;
    return HAL_OK;
}

HAL_StatusTypeDef kt6368a_configure(const char *name)
{
    char setting[40], response[64];
    size_t name_size;
    uint8_t changed = 0;
    HAL_StatusTypeDef status;
    if (name == NULL)
    {
        return HAL_ERROR;
    }
    name_size = strlen(name);
    if ((name_size == 0U) || (name_size > 30U) || (strchr(name, '\r') != NULL) || (strchr(name, '\n') != NULL))
    {
        return HAL_ERROR;
    }
    if (!ble.powered)
    {
        return HAL_BUSY;
    }

    /* 先查BLE名称，模块无响应时返回；保留SPP名称、UART波特率和MAC。 */
    memcpy(setting, "AT+BM", 5);
    memcpy(setting + 5, name, name_size);
    memcpy(setting + 5 + name_size, "\r\n", 3);
    status = kt6368a_set_if_needed("AT+TM\r\n", "TM+", name, setting, &changed);
    if (status != HAL_OK)
    {
        return status;
    }
    status = kt6368a_set_if_needed("AT+T4\r\n", "T4+", "01", "AT+B401\r\n", &changed);
    if (status != HAL_OK)
    {
        return status;
    }
    status = kt6368a_set_if_needed("AT+T5\r\n", "T5+", "01", "AT+B501\r\n", &changed);
    if (status != HAL_OK)
    {
        return status;
    }
    if (!changed)
    {
        return HAL_OK; /* 三项查询均匹配，无需重复验证 */
    }
    kt6368a_power_off();
    status = kt6368a_power_on();
    if (status != HAL_OK)
    {
        return status;
    }
    status = kt6368a_at_command("AT+TM\r\n", "TM+", response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    if (strcmp(response + 3, name) != 0)
    {
        return HAL_ERROR;
    }
    status = kt6368a_at_command("AT+T4\r\n", "T4+", response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    if (strcmp(response, "T4+01") != 0)
    {
        return HAL_ERROR;
    }
    status = kt6368a_at_command("AT+T5\r\n", "T5+", response, sizeof(response), KT6368A_AT_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }
    return (strcmp(response, "T5+01") == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef kt6368a_init(void)
{
    init_status = HAL_ERROR;
    kt6368a_power_off();
    ble.rx_dropped = 0;
    ble.rx_errors = 0;
    init_status = kt6368a_power_on();
    if (init_status == HAL_OK)
    {
        init_status = kt6368a_configure(KT6368A_BLE_NAME);
    }
    kt6368a_flush_rx();
    return init_status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;
    if ((huart != &huart1) || !ble.powered)
    {
        return;
    }
    next = (uint16_t)((ble.rx_head + 1U) % KT6368A_RX_BUFFER_SIZE);
    if (next == ble.rx_tail)
    {
        ble.rx_dropped++;
    }
    else
    {
        ble.rx_buffer[ble.rx_head] = ble.rx_byte;
        __DMB();
        ble.rx_head = next;
    }
    if (HAL_UART_Receive_IT(huart, &ble.rx_byte, 1) != HAL_OK)
    {
        ble.rx_errors++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != &huart1) || !ble.powered)
    {
        return;
    }
    ble.rx_errors++;
    /* 本工程没有UART RX DMA；AbortReceive仅关闭接收中断，不在ISR等待DMA。 */
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(huart, &ble.rx_byte, 1) != HAL_OK)
    {
        ble.rx_errors++;
    }
}
