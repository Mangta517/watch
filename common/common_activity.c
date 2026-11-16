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
        if (gap > COMMON_ACTIVITY_MAX_GAP_MS)
        {
            /* 断采后重建重力基线与连续动作，保留累计步数和抬腕序号。 */
            state->have_sample = 0;
            state->peak_pending = state->walking_run = 0;
            state->down_frames = state->view_frames = state->wrist_armed = 0;
        }
    }
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    state->sample_ms = sample_ms;
    if (!(magnitude >= 0.5f && magnitude <= 2.0f))
    {
        /* 幅值超出0.5~2.0g或NaN，丢弃本帧并清除连续动作。 */
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
    /* 动态分量 = 当前幅值 - 重力基线，用于步态峰值/谷值判定。 */
    dynamic = magnitude - state->gravity_g;
    /* 一阶低通更新重力基线(系数0.125)，缓慢跟随佩戴姿态变化。 */
    state->gravity_g += dynamic * 0.125f;
    /* 正峰后必须出现负谷；连续3个有效步态后回补，过滤单次抬手冲击。
     * shortcut: 5Hz针对日常步行，快速跑步可能混叠漏计，需提高采样率后再扩展。 */
    if (state->peak_pending != 0 && (uint32_t)(sample_ms - state->peak_ms) > 1000)
    {
        /* 峰值挂起超过1s仍未见负谷：视为无效步态，清挂起与连续计数。 */
        state->peak_pending = 0;
        state->walking_run = 0;
    }
    if (state->peak_pending == 0 && dynamic >= 0.12f)
    {
        /* 动态分量达到正峰门限：记录峰值时刻，等待随后的负谷确认。 */
        state->peak_pending = 1;
        state->peak_ms = sample_ms;
    }
    else if (state->peak_pending != 0 && dynamic <= -0.04f)
    {
        /* 负谷出现：一次完整步态成立，计算本次与上一步的间隔。 */
        state->peak_pending = 0;
        interval = state->peak_ms - state->last_step_ms;
        if (state->walking_run == 0 || interval > 2000)
        {
            /* 首个步态或间隔超过2s：重新开始连续步态计数，本次不计步。 */
            state->walking_run = 1;
            state->last_step_ms = state->peak_ms;
        }
        else if (interval >= 300)
        {
            /* 步态间隔不小于300ms：更新上一步时刻，并按连续状态计步。 */
            state->last_step_ms = state->peak_ms;
            if (state->walking_run < 3)
            {
                /* 前两步先累计连续数；第3步成立时一次性回补前3步。 */
                ++state->walking_run;
                if (state->walking_run == 3)
                {
                    state->steps += 3;
                }
            }
            else
            {
                /* 已确认连续步行：之后每个有效步态直接加1步。 */
                ++state->steps;
            }
        }
    }
    /* 当前板正面朝上平放为负Z：先放下约400ms，再朝向观看区约400ms。
     * 用重力方向和幅值限制辅助抬腕，不把持续平放或剧烈甩动当成重复抬腕。 */
    if (magnitude >= 0.8f && magnitude <= 1.2f)
    {
        /* 幅值接近1g：排除明显甩动或冲击后，再按重力方向判断手部朝向。 */
        if (az > -0.35f * magnitude)
        {
            /* az大于-0.35倍合成幅值，视为放下候选，累计放下帧并清观看帧。 */
            state->view_frames = 0;
            if (state->down_frames < 3)
            {
                ++state->down_frames;
            }
            if (state->down_frames == 3)
            {
                /* 连续3帧放下后武装一次抬腕检测，等待随后出现观看姿态。 */
                state->wrist_armed = 1;
            }
        }
        else if (state->wrist_armed != 0 && az < -0.75f * magnitude && fabsf(ax) < 0.65f * magnitude)
        {
            /* 已武装且az小于-0.75倍合成幅值、|ax|小于0.65倍合成幅值：累计观看候选帧。 */
            state->down_frames = 0;
            if (++state->view_frames >= 3)
            {
                /* 连续3帧观看姿态：确认一次抬腕，记录序号与时刻并解除武装。 */
                ++state->wrist_sequence;
                state->wrist_ms = sample_ms;
                state->wrist_armed = 0;
                state->view_frames = 0;
            }
        }
        else
        {
            /* 姿态不满足放下或观看条件：清空两侧帧计数，避免跨姿态拼接。 */
            state->down_frames = state->view_frames = 0;
        }
    }
    else
    {
        /* 幅值不在0.8~1.2g：视为剧烈运动或异常帧，清空抬腕帧计数。 */
        state->down_frames = state->view_frames = 0;
    }
}
