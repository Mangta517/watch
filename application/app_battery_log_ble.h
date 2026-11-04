/**
 * @file app_battery_log_ble.h
 * @brief BATLOG文本协议的蓝牙导出通道：STATUS/READ/CLEAR/STOP，供上位机拉取电压日志与复位现场。
 *        由 LVGL 任务的蓝牙接收入口转交数据，只使用自己的异步发送在途，不消费其他模块的发送结果。
 */
#ifndef APP_BATTERY_LOG_BLE_H
#define APP_BATTERY_LOG_BLE_H
#include <stdint.h>

/* 累积一行命令(≤23字符)；只有CR/LF才成形，控制字符或超长使该帧作废。
 * 超过1秒没有新字节则先丢弃残帧，now_ms为调用方的LVGL毫秒tick。 */
void app_battery_log_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
/* 断开或错误后丢弃残帧与待处理命令；在途TX结果仍保留到自行回收。 */
void app_battery_log_ble_reset(void);
/* 每轮推进协议状态机：取走自己的TX结果、按预算导出记录、必要时让出UART。
 * ready=0表示调用方判定蓝牙尚未就绪（如断电、运动模式或初始化在途）；返回1表示正在处理日志命令/发送，原心率波形须等待；断电后仍回收自己的在途结果。 */
uint8_t app_battery_log_ble_service(uint8_t ready, uint32_t now_ms);

#endif
