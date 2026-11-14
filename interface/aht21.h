/**
 * @file aht21.h
 * @brief AHT21 温湿度驱动；与现有传感器共用 PB14/PB13，由 sensor 任务独占访问。
 */
#ifndef __AHT21_H__
#define __AHT21_H__

#include "climate_types.h"

#define AHT21_ADDR          0x38
#define AHT21_CONVERSION_MS 80

/* 总线初始化后调用；仅启动阶段等待 40ms，必要时初始化再等待 10ms。 */
aht21_result aht21_init(void);
/* sensor任务运行期恢复：每次step至多一笔事务；BUSY=推进中，其他结果=结束。 */
void aht21_recover_begin(uint32_t now_ms);
aht21_result aht21_recover_step(uint32_t now_ms);
/* 仅发 AC 33 00，不等待转换；总线在 STOP 后可供其他传感器使用。 */
aht21_result aht21_trigger(void);
/* 触发至少 80ms 后调用；校验七字节帧，失败时不覆盖输出。 */
aht21_result aht21_read(aht21_struct *sample);

#endif
