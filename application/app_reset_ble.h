#ifndef APP_RESET_BLE_H
#define APP_RESET_BLE_H

#include <stdint.h>

/* 复位/运行状态的文本查询协议(RESET,READ / STATE,READ / HEART,READ / RESET,HISTORY)。
 * 由LVGL蓝牙上下文串行调用，命令必须以CR/LF结束；响应一拍最多一行、行间至少100ms，
 * 每次都是异步启动并由本模块自己收回TX结果。 */

/* 由LVGL蓝牙上下文串行调用；RESET/STATE/HEART,READ及RESET,HISTORY必须以CR/LF结束。 */
void app_reset_ble_reset(void);
/* 按字节喂入串口数据；now_ms 为LVGL毫秒时基，距上一字节达到1秒则丢弃半行重新开始。 */
void app_reset_ble_receive(const uint8_t *data, uint16_t size, uint32_t now_ms);
/* ready=0（蓝牙断电/运动模式/初始化在途）时丢弃未完成的报告；now_ms 为LVGL毫秒时基。
   返回1表示报告仍在排队或本次异步TX仍占用发送服务（并不表示发送成功）；
   断电后也先回收自己的异步TX结果。 */
uint8_t app_reset_ble_service(uint8_t ready, uint32_t now_ms);

#endif
