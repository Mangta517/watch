/**
 ******************************************************************************
 * @file    kt6368a.h
 * @brief   KT6368A 蓝牙串口驱动：USART1 + BL1551B 电源开关。
 *          PA9(TX)接模块7脚(RX)，PA10(RX)接模块8脚(TX)。
 *          PA8高电平供电；PB15（HEART_LED，旧名 BLE_LED）实际为心率 LED_EN，与蓝牙 KT_LED 不相连。
 ******************************************************************************
 */
#ifndef __KT6368A_H__
#define __KT6368A_H__

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define KT6368A_BLE_NAME       "F411-Watch"
#define KT6368A_RX_BUFFER_SIZE 1024U /* 环形缓冲有效容量1023字节 */
#define KT6368A_TX_MAX_SIZE    512U
#define KT6368A_BOOT_DELAY_MS  1200U /* 蓝牙模块上电等待时间 */
#define KT6368A_AT_TIMEOUT_MS  500U

    /* 调用前须完成MX_GPIO_Init/MX_USART1_UART_Init。
 * 兼容旧调用方的阻塞入口；手表启动使用下方begin/service。配置BLE名称、开启BLE并查询验证。
 * 普通/升级APP关闭SPP；仅F411_BOOTLOADER升级模式开启SPP，返回APP后重新关闭。
 * 参数已匹配时不写配置；修改后重新上电。失败不阻止其他手表功能启动。
 * 本型号不配置CL/UT低功耗功能。返回HAL_OK/ERROR/TIMEOUT/BUSY。 */
    HAL_StatusTypeDef kt6368a_init(void);
    /* LVGL循环分步启动：20ms放电/1200ms上电/AT查询/参数保存均不长时间等待。
     * pending期间由本状态机独占UART RX，业务write返回BUSY。 */
    void kt6368a_init_begin(void);
    /* LVGL任务的PA4消抖切换：0断电/停UART/释放PA9/PA10，1重新分步启动。 */
    void kt6368a_set_enabled(uint8_t enabled);
    HAL_StatusTypeDef kt6368a_init_service(void);
    uint8_t kt6368a_init_is_pending(void);
    HAL_StatusTypeDef kt6368a_get_init_status(void);

    /* 电源切换/配置/收发接口须由同一个调用上下文串行使用，禁止在ISR调用。
 * power_on含HAL_Delay；power_off关闭USART1时钟/中断，UART引脚设为模拟无上下拉，避免反向供电。
 * 上电不改参数；configure开启BLE并设置BLE名称，按APP/Bootloader选择关闭/开启SPP。
 * 仅在手机未连接时调用；本板无法读取连接状态。
 * name为1~30字节且不含CR/LF。 */
    HAL_StatusTypeDef kt6368a_power_on(void);
    void kt6368a_power_off(void);
    HAL_StatusTypeDef kt6368a_configure(const char *name);
    /* 本板未接蓝牙状态脚，保留接口固定返回0（无法确认），不代表手机一定未连接。 */
    uint8_t kt6368a_is_connected(void);

    /* 手机向FFF1执行Write Without Response；接收手表数据须订阅FFF1 Notify。
 * write在模块供电并完成启动等待后直接发送；返回值仅表示UART发送结果，不代表手机已收到。
 * 单次最多512字节；大数据需应用层安排发送节奏，不能无限连续灌满模块缓存。
 * read为非阻塞字节流，业务消息的分包/拼包由应用层处理。 */
    HAL_StatusTypeDef kt6368a_write(const uint8_t *data, uint16_t size, uint32_t timeout_ms);
    /* 异步入口不复制数据；调用方保持缓冲不变直到take_tx_result取得完成/失败结果。
     * HAL_OK只表示启动成功；同一上下文轮询结果，超时中止TX且不影响RX。 */
    HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms);
    /* 发送中或结果待原发起者收取时为1。 */
    uint8_t kt6368a_tx_is_busy(void);
    uint8_t kt6368a_take_tx_result(HAL_StatusTypeDef *result);
    uint16_t kt6368a_read(uint8_t *data, uint16_t size);
    void kt6368a_flush_rx(void);
    uint32_t kt6368a_get_rx_dropped(void);
    uint32_t kt6368a_get_rx_errors(void);

    /* AT命令须以AT+开头、实际CRLF结尾，只能在未连接时调用。
 * expected_prefix如"TM+"、"T4+"、"OK"；response保存匹配的整行，不含CRLF。
 * 发送前清空旧接收数据；ER+返回HAL_ERROR，无匹配响应返回HAL_TIMEOUT。
 * 此接口含等待，只在调度器前配置阶段使用。 */
    HAL_StatusTypeDef kt6368a_at_command(const char *command, const char *expected_prefix, char *response, uint16_t response_size,
                                         uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
