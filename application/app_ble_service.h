/**
 * @file app_ble_service.h
 * @brief 显示任务独占的蓝牙业务服务：共享接收、发送协调、日志与ECO供电。
 *
 * 与驱动初始化、诊断发送在同一显示任务串行调用，不在ISR或sensor任务调用。
 * 不创建任务或动态分配；各协议模块继续保有自己的解析器和异步TX结果。
 */
#ifndef APP_BLE_SERVICE_H
#define APP_BLE_SERVICE_H

#include <stdint.h>
#include "app_clock.h"

typedef enum
{
    APP_BLE_INIT_OK,
    APP_BLE_INIT_BUSY,
    APP_BLE_INIT_TIMEOUT,
    APP_BLE_INIT_ERROR
} app_ble_init_result_enum;

typedef enum
{
    APP_BLE_TX_READY,
    APP_BLE_TX_UART_OK,
    APP_BLE_TX_BUSY,
    APP_BLE_TX_TIMEOUT,
    APP_BLE_TX_ERROR,
    APP_BLE_TX_UPTIME_OK,
    APP_BLE_TX_UPTIME_ERROR,
    APP_BLE_TX_HEART_OK,
    APP_BLE_TX_HEART_ERROR
} app_ble_tx_state_enum;

typedef struct
{
    const char *device_name;
    const char *log; /* 借用服务缓冲；仅同一显示任务立即读取，下一次修改日志前有效。 */
    uint32_t log_revision;
    uint32_t rx_bytes;
    uint16_t log_length;
    uint8_t enabled; /* 策略维护的供电状态，不表示手机已连接。 */
    uint8_t eco;
    app_ble_init_result_enum init_result;
    app_ble_tx_state_enum tx_state;
    app_clock_result clock_result;
} app_ble_snapshot_struct;

/* 每轮先接收并维护天气温度有效期，再推进管理服务、UPTIME和可选心率波形。
 * 返回1仅表示原管理服务需要延长UI活动时间，由UI调用lv_disp_trig_activity。
 * 不在此更新供电，调用方处理完按键/抬腕后再执行update_power。 */
uint8_t app_ble_service(uint8_t screen_on, uint8_t sport_mode);
/* restart_quiet=1用于LCD电源切换成功后，重新开始1000ms安静期。
 * SPORT立即断电；ECO须等待初始化、管理服务、TX、RX空闲后再等待安静期。 */
void app_ble_update_power(uint8_t screen_on, uint8_t sport_mode, uint8_t restart_quiet);
/* 仅RAM保存，开机默认ECO；设置时重启安静期，随后由调用方更新供电。 */
void app_ble_set_eco(uint8_t enabled);
void app_ble_get_snapshot(app_ble_snapshot_struct *snapshot);
/* 只清显示日志与RX显示计数，不清UART队列或协议残帧。 */
void app_ble_clear_log(void);
/* 仅APP_DEBUG_TELEMETRY开启时同步发送测试行；结果仅表示UART发送结果。 */
void app_ble_send_test(void);

#endif
