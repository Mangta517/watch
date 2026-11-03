#ifndef COMMON_ACTIVITY_H
#define COMMON_ACTIVITY_H

#include <stdint.h>

/* 相邻两帧新鲜加速度允许的最大间隔(ms)：超过即视为断采，计步/抬腕/静止引用全部作废。
 * 后台5Hz取200ms、静止慢待机取400ms，留出调度抖动后仍不超过该界限。 */
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
    uint32_t still_since_ms;      /* 加速度持续稳定计时起点(ms)，差值判定回绕安全。 */
    float last_ax, last_ay, last_az; /* 上一新鲜帧各轴分量(g)，用于轴差静止判定。 */
    float last_magnitude;         /* 上一新鲜帧合成幅值(g)。 */
    uint8_t have_sample, peak_pending, walking_run;
    uint8_t down_frames, view_frames, wrist_armed;
    uint8_t have_still_ref;       /* 已有可比较的上一新鲜帧，静止计时有效。 */
    uint8_t stationary;           /* 连续约10秒加速度稳定=1，供后台放宽读取节拍。 */
} common_activity_struct;

/* 状态从零初始化；加速度单位g。仅消费新帧，最多5Hz，不产生硬件访问。 */
void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms);

/* 读失败或无新帧时取消静止：清零静止标记并作废稳定计时引用，
 * 保留已确认的步数与抬腕状态；重新静止必须由后续新鲜帧再连续稳定约10秒。 */
void common_activity_stationary_cancel(common_activity_struct *state);

#endif
