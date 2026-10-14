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

enum
{
    KT_START_OFF,
    KT_START_DISCHARGE,
    KT_START_BOOT,
    KT_START_QUERY,
    KT_START_REPLY,
    KT_START_SAVE
};

typedef struct
{
    uint32_t phase_ms;
    uint32_t rx_errors;
    uint32_t rx_dropped;
    char line[64];
    uint8_t length;
    uint8_t overflow;
    uint8_t phase;
    uint8_t query;
    uint8_t changed;
    uint8_t verifying;
} kt6368a_start_struct;

static kt6368a_start_struct s_start;
/* 0空闲、1发送中、2结果待取；ISR只发布完成结果，不调用RTOS接口。 */
static volatile uint8_t s_tx_state;
static volatile HAL_StatusTypeDef s_tx_result;
static uint32_t s_tx_started_ms, s_tx_timeout_ms;


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
    uint32_t mask = __get_PRIMASK();
    ble.powered = 0;
    __disable_irq();
    if (s_tx_state == 1U)
    {
        (void)HAL_UART_AbortTransmit(&huart1);
        s_tx_result = HAL_ERROR;
        s_tx_state = 2U;
    }
    __set_PRIMASK(mask);
    (void)HAL_UART_AbortReceive(&huart1);
    /* BL1551B关断时将蓝牙电源接地；UART闲置高电平不能继续驱动模块。 */
    gpio.Pin = BLE_TX_Pin | BLE_RX_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(BLE_EN_GPIO_Port, BLE_EN_Pin, GPIO_PIN_RESET);
    kt6368a_flush_rx();
}

static HAL_StatusTypeDef kt6368a_power_enable(void)
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
    return HAL_OK;
}

HAL_StatusTypeDef kt6368a_power_on(void)
{
    HAL_StatusTypeDef status;
    if (ble.powered != 0)
    {
        return HAL_OK;
    }
    HAL_Delay(20);
    status = kt6368a_power_enable();
    if (status == HAL_OK)
    {
        HAL_Delay(KT6368A_BOOT_DELAY_MS);
    }
    return status;
}

HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    if ((data == NULL) || (size == 0U) || (size > KT6368A_TX_MAX_SIZE))
    {
        return HAL_ERROR;
    }
    if (s_tx_state == 1U || s_start.phase != KT_START_OFF || !ble.powered ||
        ((uint32_t)(HAL_GetTick() - ble.power_on_tick) < KT6368A_BOOT_DELAY_MS))
    {
        return HAL_BUSY;
    }
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, size, timeout_ms);
}

HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    if (data == NULL || size == 0U || size > KT6368A_TX_MAX_SIZE || timeout_ms == 0U)
    {
        return HAL_ERROR;
    }
    if (s_tx_state != 0U || s_start.phase != KT_START_OFF || !ble.powered ||
        (uint32_t)(HAL_GetTick() - ble.power_on_tick) < KT6368A_BOOT_DELAY_MS)
    {
        return HAL_BUSY;
    }
    s_tx_started_ms = HAL_GetTick();
    s_tx_timeout_ms = timeout_ms;
    s_tx_state = 1U;
    status = HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, size);
    if (status != HAL_OK)
    {
        s_tx_state = 0U;
    }
    return status;
}

uint8_t kt6368a_tx_is_busy(void)
{
    return s_tx_state != 0U; /* 完成结果仍由发起者收取，其他异步写入继续等待。 */
}

uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    uint32_t now = HAL_GetTick();
    uint32_t mask = __get_PRIMASK();
    uint8_t ready = 0;
    __disable_irq();
    /* 只中止TX，保持原RX接收；保护完成中断与超时判定的交接。 */
    if (s_tx_state == 1U && (uint32_t)(now - s_tx_started_ms) >= s_tx_timeout_ms)
    {
        (void)HAL_UART_AbortTransmit(&huart1);
        s_tx_result = HAL_TIMEOUT;
        s_tx_state = 2U;
    }
    if (s_tx_state == 2U)
    {
        *result = s_tx_result;
        s_tx_state = 0U;
        ready = 1;
    }
    __set_PRIMASK(mask);
    return ready;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1 && s_tx_state == 1U)
    {
        s_tx_result = HAL_OK;
        s_tx_state = 2U;
    }
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
    if (!ble.powered || s_tx_state == 1U)
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
    if (strcmp(response, "T5+01") == 0)
    {
        return HAL_OK;
    }
    return HAL_ERROR;
}

/* 初始化期间AT独占接收队列，UI通过is_pending暂停业务收发。 */
void kt6368a_init_begin(void)
{
    kt6368a_power_off();
    ble.rx_dropped = 0;
    ble.rx_errors = 0;
    memset(&s_start, 0, sizeof(s_start));
    s_start.phase = KT_START_DISCHARGE;
    s_start.phase_ms = HAL_GetTick();
    init_status = HAL_BUSY;
}

uint8_t kt6368a_init_is_pending(void)
{
    return s_start.phase != KT_START_OFF;
}

static HAL_StatusTypeDef kt6368a_start_finish(HAL_StatusTypeDef status)
{
    init_status = status;
    s_start.phase = KT_START_OFF;
    kt6368a_flush_rx();
    return status;
}

static void kt6368a_start_next(void)
{
    ++s_start.query;
    if (s_start.query < 3U)
    {
        s_start.phase = KT_START_QUERY;
    }
    else if (s_start.changed != 0 && s_start.verifying == 0)
    {
        /* 参数保存后保留原重新上电验证，不重复写已匹配参数。 */
        kt6368a_power_off();
        s_start.verifying = 1;
        s_start.query = 0;
        s_start.phase = KT_START_DISCHARGE;
        s_start.phase_ms = HAL_GetTick();
    }
    else
    {
        (void)kt6368a_start_finish(HAL_OK);
    }
}

HAL_StatusTypeDef kt6368a_init_service(void)
{
    static const char *const queries[] = {"AT+TM\r\n", "AT+T4\r\n", "AT+T5\r\n"};
    static const char *const prefixes[] = {"TM+", "T4+", "T5+"};
    uint32_t now = HAL_GetTick();
    uint16_t budget = 64;
    uint8_t byte;
    char setting[40];
    const char *target;
    HAL_StatusTypeDef status;
    if (s_start.phase == KT_START_OFF)
    {
        return init_status;
    }
    if (s_start.phase == KT_START_DISCHARGE)
    {
        if ((uint32_t)(now - s_start.phase_ms) >= 20U)
        {
            status = kt6368a_power_enable();
            if (status != HAL_OK)
            {
                return kt6368a_start_finish(status);
            }
            s_start.phase = KT_START_BOOT;
        }
        return HAL_BUSY;
    }
    if (s_start.phase == KT_START_BOOT)
    {
        if ((uint32_t)(now - ble.power_on_tick) >= KT6368A_BOOT_DELAY_MS)
        {
            s_start.phase = KT_START_QUERY;
        }
        return HAL_BUSY;
    }
    if (s_start.phase == KT_START_SAVE)
    {
        if ((uint32_t)(now - s_start.phase_ms) >= 100U)
        {
            kt6368a_start_next();
        }
        return init_status;
    }
    if (s_start.phase == KT_START_QUERY)
    {
        kt6368a_flush_rx();
        s_start.length = s_start.overflow = 0;
        s_start.rx_errors = ble.rx_errors;
        s_start.rx_dropped = ble.rx_dropped;
        s_start.phase_ms = now;
        status = HAL_UART_Transmit(&huart1, (uint8_t *)queries[s_start.query], (uint16_t)strlen(queries[s_start.query]), 10U);
        if (status != HAL_OK)
        {
            return kt6368a_start_finish(status);
        }
        s_start.phase = KT_START_REPLY;
        return HAL_BUSY;
    }
    if (ble.rx_errors != s_start.rx_errors || ble.rx_dropped != s_start.rx_dropped)
    {
        return kt6368a_start_finish(HAL_ERROR);
    }
    if ((uint32_t)(now - s_start.phase_ms) >= KT6368A_AT_TIMEOUT_MS)
    {
        return kt6368a_start_finish(HAL_TIMEOUT);
    }
    while (budget-- != 0 && kt6368a_read(&byte, 1) != 0)
    {
        if (byte == '\r')
        {
            continue;
        }
        if (byte != '\n')
        {
            if (s_start.length < sizeof(s_start.line) - 1U)
            {
                s_start.line[s_start.length++] = (char)byte;
            }
            else
            {
                s_start.overflow = 1;
            }
            continue;
        }
        if (s_start.overflow != 0)
        {
            return kt6368a_start_finish(HAL_ERROR);
        }
        s_start.line[s_start.length] = '\0';
        s_start.length = 0;
        if (strncmp(s_start.line, "ER+", 3) == 0)
        {
            return kt6368a_start_finish(HAL_ERROR);
        }
        if (strncmp(s_start.line, prefixes[s_start.query], 3) != 0)
        {
            continue;
        }
        target = "01";
        if (s_start.query == 0)
        {
            target = KT6368A_BLE_NAME;
        }
        if (strcmp(s_start.line + 3, target) == 0)
        {
            kt6368a_start_next();
            return init_status;
        }
        if (s_start.verifying != 0)
        {
            return kt6368a_start_finish(HAL_ERROR);
        }
        if (s_start.query == 0)
        {
            memcpy(setting, "AT+BM", 5);
            memcpy(setting + 5, target, strlen(target));
            memcpy(setting + 5 + strlen(target), "\r\n", 3);
        }
        else if (s_start.query == 1)
        {
            strcpy(setting, "AT+B401\r\n");
        }
        else
        {
            strcpy(setting, "AT+B501\r\n");
        }
        status = HAL_UART_Transmit(&huart1, (uint8_t *)setting, (uint16_t)strlen(setting), 10U);
        if (status != HAL_OK)
        {
            return kt6368a_start_finish(status);
        }
        s_start.changed = 1;
        s_start.phase = KT_START_SAVE;
        s_start.phase_ms = HAL_GetTick();
        return HAL_BUSY;
    }
    return HAL_BUSY;
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
