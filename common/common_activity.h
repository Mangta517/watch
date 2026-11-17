#ifndef COMMON_ACTIVITY_H
#define COMMON_ACTIVITY_H

#include <stdint.h>

/* 计步/抬腕公共层，只消费sensor任务喂入的新鲜加速度(g)，不访问硬件。
 * 状态由调用方持有，标称25ms消费一帧，允许20/30ms交替输入。断采超过600ms时重建连续动作，
 * 累计步数与已发布抬腕序号保留。强信号连续3步、弱信号5步确认并回补。 */
#define COMMON_ACTIVITY_MAX_GAP_MS 600

typedef struct
{
    uint32_t steps;
    uint32_t wrist_sequence;
    uint32_t wrist_ms;
    uint32_t sample_ms;
    uint32_t peak_ms;
    uint32_t last_step_ms;
    float mag_prev1, mag_prev2, fast_mag, peak_g, trough_g;
    float motion_min[3], motion_max[3], direction[3];
    uint32_t quiet_since_ms, direction_ms, turn_ms, step_interval_ms;
    uint32_t down_since_ms, view_since_ms;
    uint8_t have_sample, peak_pending, walking_run;
    uint8_t still, turn_active, walking_confirmed, confirmation_steps;
    uint8_t down_active, view_active, wrist_armed;
} common_activity_struct;

/* 状态从零初始化；加速度单位g。仅消费新帧，100Hz来源须先低通并按平均25ms送入。 */
void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms);

#endif
