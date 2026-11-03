/** 本次开机时间记录；计时/EEPROM由sensor任务独占，不累计往次开机时间。 */
#ifndef APP_UPTIME_H
#define APP_UPTIME_H

#include <stdint.h>

void app_uptime_init(void);
/* 仅sensor读取本次开机秒数，包含Tickless补时及tick回绕。 */
uint32_t app_uptime_get_seconds(void);
/* 每拍更新计时；storage_available为0时推迟新写入，已开始的写入继续收尾。
 * 返回1表示占用EEPROM，其他校准保存应等到返回0后再开始。 */
uint8_t app_uptime_service(uint8_t storage_available);
/* 返回1表示存在有效记录；seconds为最近一次保存的开机秒数，重启后可读取上次记录。 */
uint8_t app_uptime_get_saved(uint32_t *seconds);
/* 可由LVGL调用：启动时恢复的旧记录只发布一次，不随本次10分钟保存更新。 */
uint8_t app_uptime_get_previous(uint32_t *seconds);

#endif
