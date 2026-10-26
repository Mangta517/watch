#include "common_activity.h"
#include <math.h>

void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms)
{
    float magnitude, dynamic;
    uint32_t gap, interval;
    if (state->have_sample != 0)
    {
        gap = sample_ms - state->sample_ms;
        if (gap < 200)
        {
            return;
        }
        if (gap > 600)
        {
            state->have_sample = 0;
            state->peak_pending = state->walking_run = 0;
            state->down_frames = state->view_frames = state->wrist_armed = 0;
        }
    }
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    state->sample_ms = sample_ms;
    if (!(magnitude >= 0.5f && magnitude <= 2.0f))
    {
        state->have_sample = 0;
        state->peak_pending = state->walking_run = 0;
        state->down_frames = state->view_frames = state->wrist_armed = 0;
        return;
    }
    if (state->have_sample == 0)
    {
        state->gravity_g = magnitude;
        state->have_sample = 1;
        return;
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
