#ifndef COMMON_ACTIVITY_H
#define COMMON_ACTIVITY_H

#include <stdint.h>

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

/* 状态从零初始化；加速度单位g。仅消费新帧，最多5Hz，不产生硬件访问。 */
void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms);

#endif
