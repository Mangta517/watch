/* UI集成与独立BLE服务测试共用的UART到达时间、错误边界及异步发送桩。
 * 调用者提供host_tick和HAL_GetTick；不包含app_ui.c或app_ble_service.c。 */
#ifndef BLE_SERVICE_HOST_UART_H
#define BLE_SERVICE_HOST_UART_H
#include "kt6368a.h"
#include "app_uptime.h"

static uint8_t host_bt_pending, host_rtc_pending;
static uint8_t host_bt_powered = 1;
static unsigned host_bt_enable_calls, host_bt_disable_calls;
void kt6368a_set_enabled(uint8_t enabled)
{
    host_bt_powered = enabled;
    if (enabled != 0)
    {
        ++host_bt_enable_calls;
    }
    else
    {
        ++host_bt_disable_calls;
    }
    kt6368a_flush_rx();
}
static HAL_StatusTypeDef host_bt_init_status = HAL_OK;
static uint8_t host_bt_rx[1024];
static uint32_t host_bt_rx_ms[1024];
static uint8_t host_bt_rx_flags[1024];
static uint16_t host_bt_rx_size;
static uint16_t host_bt_rx_pos;
static uint32_t host_bt_dropped, host_bt_errors;
static uint32_t host_bt_activity_ms;
static uint8_t host_bt_activity_valid, host_bt_loss_pending;
static uint8_t host_bt_inject_on_empty;
extern unsigned host_diag_rx_resets;
static HAL_StatusTypeDef host_bt_tx_result = HAL_OK;
static uint8_t host_bt_tx_busy;
static uint8_t host_async_active;
static uint8_t host_async_wait;
static HAL_StatusTypeDef host_async_result = HAL_OK;
static const uint8_t *host_async_data;
static uint8_t host_bt_tx_data[256];
static uint16_t host_bt_tx_size;
static uint32_t host_bt_tx_calls;
static uint8_t host_previous_valid;
static uint32_t host_previous_seconds;

uint8_t app_uptime_get_previous(uint32_t *seconds)
{
    if (host_previous_valid == 0)
    {
        return 0;
    }
    *seconds = host_previous_seconds;
    return 1;
}
uint8_t kt6368a_tx_is_busy(void)
{
    return host_bt_tx_busy;
}

uint8_t kt6368a_rx_pending(void)
{
    return host_bt_rx_pos != host_bt_rx_size;
}

HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    uint16_t i;
    assert(data != NULL && size < sizeof(host_bt_tx_data));
    assert(timeout_ms == 20 || timeout_ms == 100);
    ++host_bt_tx_calls;
    if (host_bt_tx_result != HAL_OK)
    {
        return host_bt_tx_result;
    }
    for (i = 0; i < size; ++i)
    {
        host_bt_tx_data[i] = data[i];
    }
    host_bt_tx_data[size] = '\0';
    host_bt_tx_size = size;
    return HAL_OK;
}

uint8_t kt6368a_is_connected(void)
{
    return 0; /* 本板没有连接状态输入 */
}

HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef result = kt6368a_write(data, size, timeout_ms);
    if (result == HAL_OK)
    {
        host_async_active = host_bt_tx_busy = 1;
        host_async_data = data;
    }
    return result;
}

uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result)
{
    if (host_async_active == 0 || host_async_wait != 0)
    {
        return 0;
    }
    assert(memcmp(host_async_data, host_bt_tx_data, host_bt_tx_size) == 0);
    host_async_active = host_bt_tx_busy = 0;
    *result = host_async_result;
    return 1;
}
uint8_t kt6368a_init_is_pending(void)
{
    return host_bt_pending;
}
HAL_StatusTypeDef kt6368a_get_init_status(void)
{
    return host_bt_init_status;
}
uint16_t kt6368a_read(uint8_t *data, uint16_t size)
{
    uint16_t i;
    uint16_t count = host_bt_rx_size - host_bt_rx_pos;
    if (count > size)
    {
        count = size;
    }
    for (i = 0; i < count; ++i)
    {
        data[i] = host_bt_rx[host_bt_rx_pos + i];
    }
    host_bt_rx_pos += count;
    return count;
}
static void host_bluetooth_feed(const uint8_t *data, uint16_t size);
uint16_t kt6368a_read_timed(uint8_t *data, uint16_t size, kt6368a_rx_info_struct *info)
{
    uint16_t count = 0;
    info->received_ms = host_bt_activity_ms;
    info->sampled_ms = HAL_GetTick();
    info->flags = 0;
    if (host_bt_rx_pos == host_bt_rx_size)
    {
        if (host_bt_loss_pending != 0)
        {
            info->flags = KT6368A_RX_LOSS;
        }
        if (host_bt_inject_on_empty != 0)
        {
            host_bt_inject_on_empty = 0;
            host_bluetooth_feed((const uint8_t *)"4:56\n", 5);
            ++host_tick;
        }
        return 0;
    }
    info->received_ms = host_bt_rx_ms[host_bt_rx_pos];
    info->flags = host_bt_rx_flags[host_bt_rx_pos];
    while (count < size && host_bt_rx_pos < host_bt_rx_size)
    {
        if (count != 0 && (host_bt_rx_ms[host_bt_rx_pos] != info->received_ms || host_bt_rx_flags[host_bt_rx_pos] != 0))
        {
            break;
        }
        data[count++] = host_bt_rx[host_bt_rx_pos++];
    }
    return count;
}
uint32_t kt6368a_get_rx_dropped(void)
{
    return host_bt_dropped;
}
uint32_t kt6368a_get_rx_errors(void)
{
    return host_bt_errors;
}
void kt6368a_flush_rx(void)
{
    host_bt_rx_pos = host_bt_rx_size;
    host_bt_activity_valid = 0;
    host_bt_loss_pending = 0;
}
static void host_bluetooth_feed(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    if (host_bt_rx_pos == host_bt_rx_size)
    {
        host_bt_rx_size = host_bt_rx_pos = 0;
    }
    assert(host_bt_rx_size + size <= sizeof(host_bt_rx));
    for (i = 0; i < size; ++i)
    {
        uint16_t index = host_bt_rx_size++;
        host_bt_rx[index] = data[i];
        host_bt_rx_ms[index] = HAL_GetTick();
        host_bt_rx_flags[index] = 0;
        if (host_bt_activity_valid != 0 && (uint32_t)(HAL_GetTick() - host_bt_activity_ms) >= KT6368A_RX_IDLE_MS)
        {
            host_bt_rx_flags[index] |= KT6368A_RX_GAP;
        }
        if (host_bt_loss_pending != 0)
        {
            host_bt_rx_flags[index] |= KT6368A_RX_LOSS;
            host_bt_loss_pending = 0;
        }
        host_bt_activity_ms = HAL_GetTick();
        host_bt_activity_valid = 1;
    }
}
static void host_bluetooth_fault(uint8_t dropped)
{
    if (dropped != 0)
    {
        ++host_bt_dropped;
    }
    else
    {
        ++host_bt_errors;
    }
    host_bt_loss_pending = 1;
    host_bt_activity_ms = HAL_GetTick();
    host_bt_activity_valid = 1;
}

#endif
