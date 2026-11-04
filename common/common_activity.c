#include "common_activity.h"
#include <math.h>

/* 静止判定只用新鲜加速度，不使用已关闭的陀螺仪；阈值对应日常佩戴的微小抖动。 */
#define COMMON_ACTIVITY_STILL_ENTER_MS 10000 /* 连续稳定该时长后进入静止。 */
#define COMMON_ACTIVITY_STILL_AXIS_DELTA_G 0.05f /* 单轴相邻帧变化超过该值(g)视为活动。 */
#define COMMON_ACTIVITY_STILL_MAG_DELTA_G 0.05f  /* 合成幅值相邻帧变化超过该值(g)视为活动。 */

void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms)
{
    /* magnitude: 三轴合成加速度幅值(g)；dynamic: 相对重力基线的动态分量(g)；
     * axis_delta: 相邻两帧三轴变化量中的最大值(g)。 */
    float magnitude, dynamic, axis_delta;
    /* gap: 与上一有效帧的时间差(ms)；interval: 相邻两次步态峰值的间隔(ms)；
     * max_gap_ms: 本帧允许的最大间隔，已静止用1200ms，非静止仍用600ms。 */
    uint32_t gap, interval, max_gap_ms;
    /* still: 本帧是否仍满足静止判据，1=稳定，0=检测到活动。 */
    uint8_t still;
    if (state->have_sample != 0)
    {
        /* 计算本帧与上一有效帧的时间差，用于过滤过密帧并识别超长断采。 */
        gap = sample_ms - state->sample_ms;
        if (gap < 200)
        {
            return; /* 距上一有效帧不足200ms，本次不推进任何状态。 */
        }
        if (state->stationary != 0)
        {
            /* 已进入静止：允许慢待机1000ms读取间隔加抖动，不因此取消静止。 */
            max_gap_ms = COMMON_ACTIVITY_STATIONARY_MAX_GAP_MS;
        }
        else
        {
            /* 尚未静止：仍需200/400ms快速连续性，间隔收紧到600ms。 */
            max_gap_ms = COMMON_ACTIVITY_MAX_GAP_MS;
        }
        if (gap > max_gap_ms)
        {
            /* 断采超限：旧的运动连续性已不可信，清掉连续判定与静止计时，仅保留步数与抬腕累计。 */
            state->have_sample = 0;
            state->peak_pending = state->walking_run = 0;
            state->down_frames = state->view_frames = state->wrist_armed = 0;
            state->have_still_ref = 0;
            state->stationary = 0;
        }
    }
    /* 由三轴加速度(g)求合成幅值，用于判断重力范围与动态分量。 */
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    /* 记录本帧时刻, 作为下一帧计算间隔与各计时的基准。 */
    state->sample_ms = sample_ms;
    if (!(magnitude >= 0.5f && magnitude <= 2.0f))
    {
        /* 幅值超出0.5~2.0g(含NaN)：视为不可用帧，重置连续性并丢弃本帧，不更新重力基线。 */
        state->have_sample = 0;
        state->peak_pending = state->walking_run = 0;
        state->down_frames = state->view_frames = state->wrist_armed = 0;
        state->have_still_ref = 0;
        state->stationary = 0;
        return;
    }
    if (state->have_sample == 0)
    {
        /* 首个有效帧：只建立重力基线与差分参考，不产生步数或抬腕判定。 */
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
    still = 1; /* 先假定本帧静止，再由轴差/幅值差否决。 */
    if (state->have_still_ref != 0)
    {
        /* 取三轴相邻帧变化量的最大值，避免单轴抖动被其他轴平均掉。 */
        axis_delta = fabsf(ax - state->last_ax);
        if (fabsf(ay - state->last_ay) > axis_delta)
        {
            axis_delta = fabsf(ay - state->last_ay);
        }
        if (fabsf(az - state->last_az) > axis_delta)
        {
            axis_delta = fabsf(az - state->last_az);
        }
        /* 任一轴变化或合成幅值变化达到阈值，即判为活动而非静止。 */
        if (axis_delta >= COMMON_ACTIVITY_STILL_AXIS_DELTA_G ||
            fabsf(magnitude - state->last_magnitude) >= COMMON_ACTIVITY_STILL_MAG_DELTA_G)
        {
            still = 0;
        }
    }
    else
    {
        /* 尚无差分参考：以本帧作为静止计时的起点。 */
        state->still_since_ms = sample_ms;
    }
    /* 保存本帧数据，供下一帧做轴差/幅值差比较。 */
    state->last_ax = ax;
    state->last_ay = ay;
    state->last_az = az;
    state->last_magnitude = magnitude;
    state->have_still_ref = 1;
    if (still == 0)
    {
        /* 有活动：撤销静止标记，并把稳定起点推到本帧重新累计。 */
        state->stationary = 0;
        state->still_since_ms = sample_ms;
    }
    else if ((uint32_t)(sample_ms - state->still_since_ms) >= COMMON_ACTIVITY_STILL_ENTER_MS)
    {
        /* 连续稳定达到进入时长，才置静止标记。 */
        state->stationary = 1;
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

void common_activity_stationary_cancel(common_activity_struct *state)
{
    /* 读失败或无新帧无法确认这一拍仍稳定：静止标记与稳定计时引用一起作废，
     * 步数、抬腕和重力基线保持不变，下一新鲜帧重新起算连续10秒。 */
    state->stationary = 0;   /* 当前静止结论作废。 */
    state->have_still_ref = 0; /* 差分参考失效，下一帧重新建立静止起点。 */
}
