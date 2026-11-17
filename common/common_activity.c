#include "common_activity.h"
#include <math.h>

static void activity_reset_gait(common_activity_struct *state)
{
    state->peak_pending = 0;
    state->walking_run = 0;
    state->walking_confirmed = 0;
    state->step_interval_ms = 0;
    state->confirmation_steps = 3;
}

void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms)
{
    float magnitude, filtered, dt, drop, swing, threshold, fall_threshold, direction_dot = 0.0f;
    float accel[3] = {ax, ay, az};
    uint32_t gap = 25, interval, quiet_hold_ms;
    uint8_t axis, moving = 0, step_blocked;
    if (state->have_sample != 0)
    {
        gap = sample_ms - state->sample_ms;
        if (gap < 15)
        {
            return;
        }
        if (gap > COMMON_ACTIVITY_MAX_GAP_MS)
        {
            /* 断采后重建滤波与连续动作，保留累计步数和抬腕序号。 */
            state->have_sample = 0;
            activity_reset_gait(state);
            state->down_active = state->view_active = state->wrist_armed = 0;
        }
    }
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    state->sample_ms = sample_ms;
    if (!(magnitude >= 0.35f && magnitude <= 3.8f))
    {
        /* 单次冲击不拆散已确认的步态；正常帧仍按实际间隔处理峰值超时。 */
        state->down_active = state->view_active = state->wrist_armed = 0;
        return;
    }
    if (state->have_sample == 0)
    {
        if (magnitude < 0.8f || magnitude > 1.2f)
        {
            return;
        }
        state->mag_prev1 = state->mag_prev2 = state->fast_mag = magnitude;
        state->peak_g = state->trough_g = magnitude;
        for (axis = 0; axis < 3; ++axis)
        {
            state->motion_min[axis] = state->motion_max[axis] = accel[axis];
            state->direction[axis] = accel[axis] / magnitude;
        }
        state->quiet_since_ms = state->direction_ms = sample_ms;
        state->still = state->turn_active = 0;
        activity_reset_gait(state);
        state->have_sample = 1;
        return;
    }
    /* 三轴各自的峰谷差检测静止，避免缓慢转腕在模长不变时被当成静止。 */
    for (axis = 0; axis < 3; ++axis)
    {
        state->motion_min[axis] = fminf(state->motion_min[axis], accel[axis]);
        state->motion_max[axis] = fmaxf(state->motion_max[axis], accel[axis]);
        if (state->motion_max[axis] - state->motion_min[axis] > 0.045f)
        {
            moving = 1;
        }
        direction_dot += state->direction[axis] * accel[axis] / magnitude;
    }
    if (moving != 0)
    {
        state->quiet_since_ms = sample_ms;
        for (axis = 0; axis < 3; ++axis)
        {
            state->motion_min[axis] = state->motion_max[axis] = accel[axis];
        }
    }
    state->still = (uint32_t)(sample_ms - state->quiet_since_ms) >= 450;
    /* 方向参考独立于模长滤波，约150ms内转动超过37度时取消未确认步态。 */
    if (state->walking_confirmed == 0 && direction_dot < 0.80f)
    {
        activity_reset_gait(state);
        state->turn_active = 1;
        state->turn_ms = sample_ms;
    }
    if ((uint32_t)(sample_ms - state->direction_ms) >= 150)
    {
        for (axis = 0; axis < 3; ++axis)
        {
            state->direction[axis] = accel[axis] / magnitude;
        }
        state->direction_ms = sample_ms;
    }
    if (state->turn_active != 0 && (uint32_t)(sample_ms - state->turn_ms) >= 250)
    {
        state->turn_active = 0;
    }
    quiet_hold_ms = 1500;
    if (state->step_interval_ms != 0)
    {
        quiet_hold_ms = state->step_interval_ms + 200;
        if (quiet_hold_ms > 1500)
        {
            quiet_hold_ms = 1500;
        }
    }
    /* 窄脉冲之间也可能平静450ms，按已测步频留出200ms余量再清连续状态。 */
    if (state->walking_run != 0 &&
        ((uint32_t)(sample_ms - state->last_step_ms) > 1500 ||
         (state->still != 0 && (uint32_t)(sample_ms - state->last_step_ms) >= quiet_hold_ms)))
    {
        activity_reset_gait(state);
    }
    step_blocked = state->still != 0 || state->turn_active != 0;
    /* 40Hz下三点二项式FIR压低近奈奎斯特振动，随后按实际毫秒间隔平滑。 */
    filtered = 0.25f * (magnitude + 2.0f * state->mag_prev1 + state->mag_prev2);
    state->mag_prev2 = state->mag_prev1;
    state->mag_prev1 = magnitude;
    dt = (float)gap;
    if (dt > 200.0f)
    {
        dt = 200.0f;
    }
    state->fast_mag += dt / (12.0f + dt) * (filtered - state->fast_mag);
    if (step_blocked != 0)
    {
        state->peak_pending = 0;
        state->trough_g = state->fast_mag;
    }
    /* 正面朝上时提高峰谷门限；摆臂与较强落地信号仍可通过。 */
    threshold = 0.080f;
    if (az < -0.70f * magnitude)
    {
        threshold = 0.120f;
    }
    if (state->peak_pending != 0 && (uint32_t)(sample_ms - state->peak_ms) > 1000)
    {
        state->peak_pending = 0;
        state->trough_g = state->fast_mag;
    }
    if (state->peak_pending == 0)
    {
        if (state->fast_mag < state->trough_g)
        {
            state->trough_g = state->fast_mag;
        }
        if (step_blocked == 0 && state->fast_mag - state->trough_g >= threshold)
        {
            state->peak_pending = 1;
            state->peak_g = state->fast_mag;
            state->peak_ms = sample_ms;
        }
    }
    else if (state->fast_mag > state->peak_g)
    {
        state->peak_g = state->fast_mag;
        state->peak_ms = sample_ms;
    }
    drop = state->peak_g - state->fast_mag;
    swing = state->peak_g - state->trough_g;
    fall_threshold = fmaxf(0.060f, 0.55f * swing);
    if (state->walking_confirmed != 0)
    {
        /* 至少回落半个峰谷差，防止停走时的半个上升波在回到静止值后多算一步。 */
        fall_threshold = fmaxf(0.045f, 0.50f * swing);
    }
    /* 已确认步行时提前落峰输出；起步仍等待较完整的峰谷以排除零散动作。 */
    if (state->peak_pending != 0 && drop >= fall_threshold && swing >= threshold)
    {
        state->peak_pending = 0;
        state->trough_g = state->fast_mag;
        interval = state->peak_ms - state->last_step_ms;
        if (state->walking_run == 0 || interval > 1500 ||
            (interval >= 250 && state->step_interval_ms != 0 &&
             ((float)interval < 0.65f * (float)state->step_interval_ms ||
              (float)interval > 1.55f * (float)state->step_interval_ms)))
        {
            /* 断档或节律突变后重新确认，累计步数保留。 */
            activity_reset_gait(state);
            state->walking_run = 1;
            state->last_step_ms = state->peak_ms;
            if (swing < 0.14f)
            {
                state->confirmation_steps = 5;
            }
        }
        else if (interval >= 250)
        {
            state->last_step_ms = state->peak_ms;
            if (state->step_interval_ms == 0)
            {
                state->step_interval_ms = interval;
            }
            else
            {
                state->step_interval_ms = (3 * state->step_interval_ms + interval) / 4;
            }
            if (state->walking_confirmed == 0)
            {
                if (swing < 0.14f)
                {
                    state->confirmation_steps = 5;
                }
                ++state->walking_run;
                if (state->walking_run >= state->confirmation_steps)
                {
                    state->steps += state->walking_run;
                    state->walking_confirmed = 1;
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
            state->view_active = 0;
            if (state->down_active == 0)
            {
                state->down_active = 1;
                state->down_since_ms = sample_ms;
            }
            if ((uint32_t)(sample_ms - state->down_since_ms) >= 400)
            {
                state->wrist_armed = 1;
            }
        }
        else if (state->wrist_armed != 0 && az < -0.75f * magnitude && fabsf(ax) < 0.65f * magnitude)
        {
            state->down_active = 0;
            if (state->view_active == 0)
            {
                state->view_active = 1;
                state->view_since_ms = sample_ms;
            }
            if ((uint32_t)(sample_ms - state->view_since_ms) >= 400)
            {
                ++state->wrist_sequence;
                state->wrist_ms = sample_ms;
                state->wrist_armed = 0;
                state->view_active = 0;
            }
        }
        else
        {
            state->down_active = state->view_active = 0;
        }
    }
    else
    {
        state->down_active = state->view_active = 0;
    }
}
