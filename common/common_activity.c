#include "common_activity.h"
#include <math.h>

/* 静止判定只用新鲜加速度，不使用已关闭的陀螺仪；阈值对应日常佩戴的微小抖动。 */
#define COMMON_ACTIVITY_STILL_ENTER_MS 10000 /* 连续稳定该时长后进入静止。 */
#define COMMON_ACTIVITY_STILL_AXIS_DELTA_G 0.05f /* 单轴相邻帧变化超过该值(g)视为活动。 */
#define COMMON_ACTIVITY_STILL_MAG_DELTA_G 0.05f  /* 合成幅值相邻帧变化超过该值(g)视为活动。 */

void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms)
{
    float magnitude, dynamic, axis_delta;
    uint32_t gap, interval;
    uint8_t still;
    if (state->have_sample != 0)
    {
        gap = sample_ms - state->sample_ms;
        if (gap < 200)
        {
            return;
        }
        if (gap > COMMON_ACTIVITY_MAX_GAP_MS)
        {
            state->have_sample = 0;
            state->peak_pending = state->walking_run = 0;
            state->down_frames = state->view_frames = state->wrist_armed = 0;
            state->have_still_ref = 0;
            state->stationary = 0;
        }
    }
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    state->sample_ms = sample_ms;
    if (!(magnitude >= 0.5f && magnitude <= 2.0f))
    {
        state->have_sample = 0;
        state->peak_pending = state->walking_run = 0;
        state->down_frames = state->view_frames = state->wrist_armed = 0;
        state->have_still_ref = 0;
        state->stationary = 0;
        return;
    }
    if (state->have_sample == 0)
    {
        state->gravity_g = magnitude;
        state->have_sample = 1;
        state->have_still_ref = 0;
        state->stationary = 0;
        state->last_ax = ax;
        state->last_ay = ay;
        state->last_az = az;
        state->last_magnitude = magnitude;
        return;
    }
    /* 用新鲜加速度帧的轴差和幅值差判定静止；任一项超阈值即重新计时并清零静止。 */
    still = 1;
    if (state->have_still_ref != 0)
    {
        axis_delta = fabsf(ax - state->last_ax);
        if (fabsf(ay - state->last_ay) > axis_delta)
        {
            axis_delta = fabsf(ay - state->last_ay);
        }
        if (fabsf(az - state->last_az) > axis_delta)
        {
            axis_delta = fabsf(az - state->last_az);
        }
        if (axis_delta >= COMMON_ACTIVITY_STILL_AXIS_DELTA_G ||
            fabsf(magnitude - state->last_magnitude) >= COMMON_ACTIVITY_STILL_MAG_DELTA_G)
        {
            still = 0;
        }
    }
    else
    {
        state->still_since_ms = sample_ms;
    }
    state->last_ax = ax;
    state->last_ay = ay;
    state->last_az = az;
    state->last_magnitude = magnitude;
    state->have_still_ref = 1;
    if (still == 0)
    {
        state->stationary = 0;
        state->still_since_ms = sample_ms;
    }
    else if ((uint32_t)(sample_ms - state->still_since_ms) >= COMMON_ACTIVITY_STILL_ENTER_MS)
    {
        state->stationary = 1;
    }
    dynamic = magnitude - state->gravity_g;
    state->gravity_g += dynamic * 0.125f;
    /* 正峰后必须出现负谷；连续3个有效步态后回补，过滤单次抬手冲击。
     * shortcut: 5Hz针对日常步行，快速跑步可能混叠漏计，需提高采样率后再扩展。 */
    if (state->peak_pending != 0 && (uint32_t)(sample_ms - state->peak_ms) > 1000)
    {
        state->peak_pending = 0;
        state->walking_run = 0;
    }
    if (state->peak_pending == 0 && dynamic >= 0.12f)
    {
        state->peak_pending = 1;
        state->peak_ms = sample_ms;
    }
    else if (state->peak_pending != 0 && dynamic <= -0.04f)
    {
        state->peak_pending = 0;
        interval = state->peak_ms - state->last_step_ms;
        if (state->walking_run == 0 || interval > 2000)
        {
            state->walking_run = 1;
            state->last_step_ms = state->peak_ms;
        }
        else if (interval >= 300)
        {
            state->last_step_ms = state->peak_ms;
            if (state->walking_run < 3)
            {
                ++state->walking_run;
                if (state->walking_run == 3)
                {
                    state->steps += 3;
                }
            }
            else
            {
                ++state->steps;
            }
        }
    }
    /* 当前板正面朝上平放为负Z：先放下约400ms，再朝向观看区约400ms。
     * 用重力方向和幅值限制辅助抬腕，不把持续平放或剧烈甩动当成重复抬腕。 */
    if (magnitude >= 0.8f && magnitude <= 1.2f)
    {
        if (az > -0.35f * magnitude)
        {
            state->view_frames = 0;
            if (state->down_frames < 3)
            {
                ++state->down_frames;
            }
            if (state->down_frames == 3)
            {
                state->wrist_armed = 1;
            }
        }
        else if (state->wrist_armed != 0 && az < -0.75f * magnitude && fabsf(ax) < 0.65f * magnitude)
        {
            state->down_frames = 0;
            if (++state->view_frames >= 3)
            {
                ++state->wrist_sequence;
                state->wrist_ms = sample_ms;
                state->wrist_armed = 0;
                state->view_frames = 0;
            }
        }
        else
        {
            state->down_frames = state->view_frames = 0;
        }
    }
    else
    {
        state->down_frames = state->view_frames = 0;
    }
}

void common_activity_stationary_cancel(common_activity_struct *state)
{
    /* 读失败或无新帧无法确认这一拍仍稳定：静止标记与稳定计时引用一起作废，
     * 步数、抬腕和重力基线保持不变，下一新鲜帧重新起算连续10秒。 */
    state->stationary = 0;
    state->have_still_ref = 0;
}
