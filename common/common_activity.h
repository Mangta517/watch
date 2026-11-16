#ifndef COMMON_ACTIVITY_H
#define COMMON_ACTIVITY_H

#include <stdint.h>

/* 计步/抬腕公共层，只消费sensor任务喂入的新鲜加速度(g)，不访问硬件。
 * 状态由调用方持有，标称200ms消费一帧，允许60ms调度/振荡器容差。断采超过600ms时重建连续动作，
 * 累计步数与已发布抬腕序号保留。 */
#define COMMON_ACTIVITY_MAX_GAP_MS 600

typedef struct
{
    uint32_t steps;
    uint32_t wrist_sequence;
    uint32_t wrist_ms;
    uint32_t sample_ms;
    uint32_t peak_ms;
    uint32_t last_step_ms;
    float gravity_g;
    uint8_t have_sample, peak_pending, walking_run;
    uint8_t down_frames, view_frames, wrist_armed;
} common_activity_struct;

/* 状态从零初始化；加速度单位g。仅消费新帧，100Hz来源须先低通并按200ms送入，不产生硬件访问。 */
void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms);

#endif
