#include "common_activity.h"
#include <math.h>

#define ACTIVITY_MIN_STEP_MS 250
#define ACTIVITY_MAX_STEP_MS 1750

static void activity_reset_gait(common_activity_struct *state)
{
    state->peak_pending = 0;
    state->walking_run = 0;
    state->walking_confirmed = 0;
    state->step_interval_ms = 0;
    state->previous_interval_ms = 0;
    state->recent_swing_g = 0.0f;
    state->confirmation_steps = 3;
}

/* 短长两拍可交替，但相邻两步的总时长仍应符合当前步频。 */
static uint8_t activity_cadence_valid(const common_activity_struct *state, uint32_t interval)
{
    float mean = (float)state->step_interval_ms;
    uint32_t previous = state->previous_interval_ms;
    uint32_t pair = interval + previous;
    if (mean == 0.0f || ((float)interval >= 0.65f * mean && (float)interval <= 1.55f * mean))
    {
        return 1;
    }
    return (float)interval >= 0.35f * (float)pair && (float)previous >= 0.35f * (float)pair &&
           (state->walking_run == 2 || ((float)pair >= 1.56f * mean && (float)pair <= 2.44f * mean));
}

uint8_t common_activity_is_mot_ready(const common_activity_struct *state, uint32_t now_ms, uint32_t hold_ms)
{
    return state->have_sample != 0 && state->still != 0 && state->walking_run == 0 && state->view_active == 0 &&
           (uint32_t)(now_ms - state->sample_ms) <= COMMON_ACTIVITY_MAX_GAP_MS &&
           (uint32_t)(state->sample_ms - state->quiet_since_ms) >= hold_ms;
}

void common_activity_resume_from_mot(common_activity_struct *state, uint32_t now_ms)
{
    if (state->have_sample != 0 && state->still != 0)
    {
        /* 从休眠前的静止基线恢复，首个落地峰仍有相对谷值。 */
        state->sample_ms = now_ms - 25;
        state->quiet_since_ms = now_ms;
        state->mag_prev1 = state->mag_prev2 = state->fast_mag = state->baseline_g;
        state->peak_g = state->trough_g = state->baseline_g;
        state->turn_active = 0;
    }
    else
    {
        state->have_sample = 0;
    }
    state->still = 0;
    state->down_active = 0;
    state->view_active = 0;
    state->wrist_mot_window = state->wrist_armed;
    state->mot_resume_ms = now_ms;
    activity_reset_gait(state);
}

void common_activity_update(common_activity_struct *state, float ax, float ay, float az, uint32_t sample_ms)
{
    float magnitude, step_mag, filtered, dt, drop, swing, threshold, fall_threshold, direction_dot = 0.0f;
    float accel[3] = {ax, ay, az};
    uint32_t gap = 25, interval, quiet_hold_ms;
    uint8_t axis, moving = 0, step_blocked, impact, missed_step;
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
            state->wrist_mot_window = 0;
        }
    }
    if (state->wrist_mot_window != 0 && (uint32_t)(sample_ms - state->mot_resume_ms) >= 3000)
    {
        state->wrist_armed = 0;
        state->view_active = 0;
        state->wrist_mot_window = 0;
    }
    magnitude = sqrtf(ax * ax + ay * ay + az * az);
    state->sample_ms = sample_ms;
    step_mag = magnitude;
    impact = !(magnitude >= 0.35f && magnitude <= 3.8f);
    if (impact != 0)
    {
        /* 冲击打断静止和抬腕，计步仍保留落地峰与腾空谷。 */
        state->quiet_since_ms = sample_ms;
        state->still = 0;
        state->down_active = state->view_active = state->wrist_armed = 0;
        state->wrist_mot_window = 0;
        if (!(magnitude > 0.0f && isfinite(magnitude)))
        {
            return;
        }
        step_mag = fminf(3.8f, fmaxf(0.35f, magnitude));
    }
    if (state->have_sample == 0)
    {
        if (magnitude < 0.65f || magnitude > 1.45f)
        {
            return;
        }
        state->mag_prev1 = state->mag_prev2 = state->fast_mag = magnitude;
        state->peak_g = state->trough_g = state->baseline_g = magnitude;
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
    for (axis = 0; impact == 0 && axis < 3; ++axis)
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
    /* 约150ms内快速转腕时，暂停接收新的候选峰。 */
    if (impact == 0 && state->walking_confirmed == 0 && direction_dot < 0.80f)
    {
        state->turn_active = 1;
        state->turn_ms = sample_ms;
    }
    if (impact == 0 && (uint32_t)(sample_ms - state->direction_ms) >= 150)
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
    quiet_hold_ms = ACTIVITY_MAX_STEP_MS;
    if (state->step_interval_ms != 0)
    {
        quiet_hold_ms = state->step_interval_ms + 200;
        if (quiet_hold_ms > ACTIVITY_MAX_STEP_MS)
        {
            quiet_hold_ms = ACTIVITY_MAX_STEP_MS;
        }
    }
    /* 窄脉冲之间也可能平静450ms，按已测步频留出200ms余量再清连续状态。 */
    if (state->walking_run != 0 &&
        ((uint32_t)(sample_ms - state->last_step_ms) > ACTIVITY_MAX_STEP_MS + 150 ||
         (state->still != 0 && (uint32_t)(sample_ms - state->last_step_ms) >= quiet_hold_ms)))
    {
        activity_reset_gait(state);
    }
    step_blocked = state->still != 0 || state->turn_active != 0;
    /* 40Hz下三点二项式FIR压低近奈奎斯特振动，随后按实际毫秒间隔平滑。 */
    filtered = 0.25f * (step_mag + 2.0f * state->mag_prev1 + state->mag_prev2);
    state->mag_prev2 = state->mag_prev1;
    state->mag_prev1 = step_mag;
    dt = (float)gap;
    if (dt > 200.0f)
    {
        dt = 200.0f;
    }
    state->fast_mag += dt / (12.0f + dt) * (filtered - state->fast_mag);
    if (state->still != 0)
    {
        state->baseline_g = state->fast_mag;
    }
    else
    {
        state->baseline_g += dt / (450.0f + dt) * (state->fast_mag - state->baseline_g);
    }
    if (step_blocked != 0)
    {
        state->peak_pending = 0;
        state->trough_g = state->fast_mag;
    }
    /* 有近期强步时接收较弱的对侧波峰。 */
    threshold = 0.080f;
    if (state->recent_swing_g >= 0.11f)
    {
        threshold = 0.055f;
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
    fall_threshold = fmaxf(fminf(0.060f, threshold), 0.55f * swing);
    if (state->walking_confirmed != 0)
    {
        /* 至少回落半个峰谷差，防止停走时的半个上升波在回到静止值后多算一步。 */
        fall_threshold = fmaxf(0.045f, 0.52f * swing);
    }
    /* 表面接近水平时，弱峰前后至少出现一次卸载低谷，以排除单向桌面轻敲。 */
    if (state->peak_pending != 0 && drop >= fall_threshold && swing >= threshold &&
        (az >= -0.92f * magnitude || swing >= 0.120f ||
         state->baseline_g - fminf(state->trough_g, state->fast_mag) >= 0.040f))
    {
        state->peak_pending = 0;
        state->trough_g = state->fast_mag;
        interval = state->peak_ms - state->last_step_ms;
        /* 有持续运动且跨了两拍时，保留步态；只累计检测到的实际峰谷。 */
        missed_step = state->walking_confirmed != 0 &&
                      (float)interval >= 1.70f * (float)state->step_interval_ms &&
                      (float)interval <= 2.30f * (float)state->step_interval_ms;
        if (state->walking_run == 0 || interval > ACTIVITY_MAX_STEP_MS ||
            (interval >= ACTIVITY_MIN_STEP_MS && missed_step == 0 && activity_cadence_valid(state, interval) == 0))
        {
            /* 断档或节律突变后重新确认，累计步数保留。 */
            activity_reset_gait(state);
            state->walking_run = 1;
            state->last_step_ms = state->peak_ms;
            state->recent_swing_g = swing;
        }
        else if (interval >= ACTIVITY_MIN_STEP_MS)
        {
            state->last_step_ms = state->peak_ms;
            state->recent_swing_g = fmaxf(swing, 0.75f * state->recent_swing_g);
            if (missed_step == 0)
            {
                state->previous_interval_ms = interval;
                if (state->step_interval_ms == 0)
                {
                    state->step_interval_ms = interval;
                }
                else
                {
                    state->step_interval_ms = (3 * state->step_interval_ms + interval) / 4;
                }
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
    if (impact == 0 && magnitude >= 0.8f && magnitude <= 1.2f)
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
                state->wrist_mot_window = 0; /* 已再次观察到完整垂腕动作。 */
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
                state->wrist_mot_window = 0;
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
