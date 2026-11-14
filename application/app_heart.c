/**
 * @file app_heart.c
 * @brief 去背景、三轴运动参考滤波、局部波峰确认、多门限及RR方差选优；有效范围40～200BPM。
 */
#include "app_heart.h"
#include "app_diag.h"
#include "em7028.h"
#include "imu_types.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <math.h>
#include <string.h>

#define APP_HEART_POLL_MS          30 /* 测量时sensor每10ms服务，普通/运动模式均约30ms读取。 */
#define APP_HEART_WARMUP_MS        1000
#define APP_HEART_MAX_GAP_MS       120 /* 长时间缺采样后重新建立滤波与周期 */
#define APP_HEART_STALE_MS         250
#define APP_HEART_MIN_BEAT_MS      300  /* 200 BPM */
#define APP_HEART_MAX_BEAT_MS      1500 /* 40 BPM */
#define APP_HEART_BEAT_TIMEOUT     2500
#define APP_HEART_MIN_SIGNAL       8.0f /* 峰谷反向变化的最小幅度；单位 ADC 计数 */
#define APP_HEART_STABLE_BEATS     3
#define APP_HEART_RR_TOLERANCE     0.35f /* 单路原始峰间隔容差；最终平滑周期仍检查25%连续性 */
#define APP_HEART_MIN_BPM          40
#define APP_HEART_MAX_BPM          200
#define APP_HEART_GATE_COUNT       5
#define APP_HEART_PEAK_WINDOW      7
#define APP_HEART_SAMPLE_COUNT     32
#define APP_HEART_JUMP_COUNTS      1024 /* 突变门限下限；还须超过上一原始值25%，实板诊断参数 */
#define APP_HEART_MOTION_TAPS      4
#define APP_HEART_MOTION_AGE_MS    50
#define APP_HEART_MOTION_SETTLE_MS 1500

/* HeartPy fit_peaks 的多门限/RR方差选优思路，独立实现为固定内存的C实时版本。 */
static const float heart_gate_factors[APP_HEART_GATE_COUNT] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f};

typedef struct
{
    float cycle_peak;
    float trough;
    float swing;
    float interval_ms;
    float rr[APP_HEART_STABLE_BEATS];
    uint32_t beat_tick;
    uint32_t candidate_tick;
    uint32_t peak_tick;
    uint16_t last_interval_ms;
    uint8_t armed;
    uint8_t peak_confirmed;
    uint8_t have_beat;
    uint8_t stable_beats;
    uint8_t rr_next;
} heart_peak_struct;

typedef struct
{
    float history[3][APP_HEART_MOTION_TAPS];
    float weights[3][APP_HEART_MOTION_TAPS];
    float power;
    float optical_power;
    float residual_power;
    uint32_t started_ms;
    uint8_t active;
} heart_motion_struct;

typedef struct
{
    float dc;
    float optical;
    float filtered;
    float amplitude;
    uint32_t warmup_tick;
    uint32_t output_beat_tick;
    float output_interval_ms;
    uint16_t previous_raw;
    uint8_t seeded;
    float window[APP_HEART_PEAK_WINDOW];
    uint32_t window_ticks[APP_HEART_PEAK_WINDOW];
    uint8_t window_next;
    uint8_t window_count;
    heart_peak_struct peaks[APP_HEART_GATE_COUNT];
    heart_motion_struct motion;
} heart_detector_struct;

/* 与IMU/heart服务同属sensor任务；只保留最新有效帧，不增加总线事务。 */
static struct
{
    float baseline[3];
    float filtered[3];
    uint32_t sample_ms;
    uint8_t seeded;
} motion_input;

static SemaphoreHandle_t heart_mutex;
static app_heart_struct heart_data;
static heart_detector_struct detector;
static uint8_t requested; /* UI最近一次手动启动/停止请求，1为要求测量。 */
static uint8_t running; /* 器件当前是否处于已启动测量状态。 */
static volatile uint8_t s_stop_pending; /* 上一拍停测写失败，1表示下轮service需重试待机。 */
#define APP_HEART_SLEEP_PERIOD_MS  600000
#define APP_HEART_SLEEP_TIMEOUT_MS 10000
static uint8_t s_sleep_mode, s_sleep_active; /* 熄屏模式；后台自动测量窗口是否进行中。 */
static uint8_t s_sport_mode, s_sport_failed; /* 运动模式；运动模式下启动失败锁存。 */
static uint8_t s_auto_enabled = 1; /* 后台自动心率开关；开机默认开启每10分钟测量，不写EEPROM。 */
static uint32_t s_sleep_cycle_ms, s_sleep_started_ms; /* 后台周期起点与本轮窗口起点，单位ms。 */
static uint32_t poll_tick; /* 上一次进入PPG读取的tick，用于30ms节流。 */
static uint32_t good_tick; /* 最近一次成功读到原始值的tick。 */
static app_heart_sample_struct heart_samples[APP_HEART_SAMPLE_COUNT]; /* 待导出原始波形环形队列。 */
static uint16_t sample_head; /* 环形队列队头下标。 */
static uint16_t sample_count; /* 队列中有效样本条数。 */
static uint16_t sample_sequence; /* 样本序号，随每次成功读数递增。 */
static uint32_t sample_start_tick; /* 本轮测量起点，用于样本相对时间。 */

/* 写快照锁；last_bpm是本次开机最近有效值，无新有效BPM时沿用旧值，供熄屏桌面显示。 */
static void heart_publish(const app_heart_struct *data)
{
    uint16_t last_bpm; /* 本次要写入快照的最近有效心率，先在此暂存。 */
    /* 取快照锁，独占heart_data的读改写。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    /* 先取旧快照里的最近有效值作为默认沿用值。 */
    last_bpm = heart_data.last_bpm;
    if (data->last_bpm != 0)
    {
        /* 调用方已带非0的最近有效值(如窗口收尾)，用它覆盖默认值。 */
        last_bpm = data->last_bpm;
    }
    /* 用本次结果整体覆盖共享快照。 */
    heart_data = *data;
    if (data->state == APP_HEART_VALID)
    {
        /* 本帧状态有效时，用当前bpm刷新最近有效值。 */
        last_bpm = data->bpm;
    }
    /* 把选定的最近有效值写回快照，供熄屏桌面沿用。 */
    heart_data.last_bpm = last_bpm;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
}

static void heart_fail(app_heart_struct *data)
{
    /* 先把当前读/启动失败的故障快照拷入结果结构。 */
    em7028_get_fault(&data->fault); /* 先保留读/启动失败，不能被 stop 的结果覆盖 */
    /* 把故障种类、寄存器、值上报诊断模块。 */
    app_diag_heart_fault((uint8_t)data->fault.kind, data->fault.reg, data->fault.value);
    /* 尝试让器件进待机；失败则留下标志，下轮service重试关闭。 */
    s_stop_pending = em7028_stop() != EM7028_OK; /* 先关外部照明，待机写失败保留关闭请求。 */
    /* 标记测量已停止。 */
    running = 0;
    /* 清空对外BPM，并把状态置为错误。 */
    data->bpm = 0;
    data->state = APP_HEART_ERROR;
    /* 取快照锁后同步内部状态。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    if (s_sleep_mode == 0 && s_sport_mode == 0)
    {
        /* 普通亮屏模式下，故障同时撤销UI的手动启动请求。 */
        requested = 0;
    }
    /* 结束后台自动窗口标记。 */
    s_sleep_active = 0;
    s_sport_failed = s_sport_mode; /* 运动期间故障停测，重新切换模式后才重试。 */
    /* 丢弃尚未导出的原始样本队列。 */
    sample_head = sample_count = 0;
    /* 用带错误状态的本次结果覆盖共享快照。 */
    heart_data = *data;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
}

static void heart_reset_peaks(uint32_t now)
{
    unsigned i;
    /* 把五组门限的峰谷/周期状态全部清零。 */
    memset(detector.peaks, 0, sizeof(detector.peaks));
    for (i = 0; i < APP_HEART_GATE_COUNT; ++i)
    {
        /* 每组以当前滤波值作为新的谷与周期峰起点，避免首拍出现虚假大摆幅。 */
        detector.peaks[i].trough = detector.peaks[i].cycle_peak = detector.filtered;
        /* 峰值时刻对齐到本次采样时刻。 */
        detector.peaks[i].peak_tick = now;
    }
}

/* 仅sensor任务在取得新MPU帧后调用；加速度换算成g后维护基线与高通/低通，
 * 供PPG运动参考。超过50ms未更新或首次输入时重建基线，不读I2C、不跨任务访问。 */
void app_heart_motion_update(int16_t accel_x, int16_t accel_y, int16_t accel_z, uint32_t sample_ms)
{
    const int16_t accel[3] = {accel_x, accel_y, accel_z}; /* 三轴原始加速度打包，便于按轴循环。 */
    uint32_t elapsed = sample_ms - motion_input.sample_ms; /* 距上一帧的毫秒间隔，用于滤波系数换算。 */
    uint8_t axis;
    float value;

    if (motion_input.seeded != 0 && elapsed == 0)
    {
        /* 同一时间戳重复调用：没有新的时间信息，直接丢弃本次输入。 */
        return;
    }
    for (axis = 0; axis < 3; ++axis)
    {
        /* 原始LSB按每g的LSB数换算成g。 */
        value = (float)accel[axis] / MPU6050_ACCEL_LSB_PER_G;
        if (motion_input.seeded == 0 || elapsed > APP_HEART_MOTION_AGE_MS)
        {
            /* 首次输入或断档超过50ms：以当前值重建基线，滤波输出清零。 */
            motion_input.baseline[axis] = value;
            motion_input.filtered[axis] = 0.0f;
        }
        else
        {
            /* 基线按约1.5秒时间常数缓慢跟踪重力方向与姿态。 */
            motion_input.baseline[axis] += (float)elapsed / (1500.0f + (float)elapsed) * (value - motion_input.baseline[axis]);
            /* 高通后的运动分量再按约70ms时间常数低通，作为NLMS参考输入。 */
            motion_input.filtered[axis] += (float)elapsed / (70.0f + (float)elapsed) *
                                           (value - motion_input.baseline[axis] - motion_input.filtered[axis]);
        }
    }
    /* 记录本帧时间戳，供下次计算间隔与新鲜度判断。 */
    motion_input.sample_ms = sample_ms;
    /* 标记已有可用基线。 */
    motion_input.seeded = 1;
}

/* 三轴四抽头NLMS：用运动参考拟合PPG中的相关成分，残差仍交给原五门限选峰。
 * 系数/门限为本工程适配值；与步频重合且不可分离时不输出猜测BPM。 */
static uint8_t heart_remove_motion(uint32_t now, uint32_t elapsed)
{
    heart_motion_struct *motion = &detector.motion; /* 本检测器的运动消除状态。 */
    float energy = 0.0f, prediction = 0.0f, residual, step;
    /* 按采样间隔换算功率跟踪的一阶平滑系数。 */
    float smoothing = (float)elapsed / (300.0f + (float)elapsed);
    uint8_t axis, tap;

    if (motion_input.seeded == 0 || (uint32_t)(now - motion_input.sample_ms) > APP_HEART_MOTION_AGE_MS)
    {
        /* 没有IMU参考或参考已过期：清空运动状态，滤波值退回原始PPG。 */
        memset(motion, 0, sizeof(*motion));
        detector.filtered = detector.optical;
        return 0;
    }
    for (axis = 0; axis < 3; ++axis)
    {
        /* 每轴历史抽头整体后移一格，空出最新位置。 */
        for (tap = APP_HEART_MOTION_TAPS - 1; tap > 0; --tap)
        {
            motion->history[axis][tap] = motion->history[axis][tap - 1];
        }
        /* 最新抽头填入当前运动参考值。 */
        motion->history[axis][0] = motion_input.filtered[axis];
        /* 累加三轴当前功率，用于判断是否存在足够运动。 */
        energy += motion_input.filtered[axis] * motion_input.filtered[axis];
    }
    /* 平滑更新运动功率估计。 */
    motion->power += smoothing * (energy - motion->power);
    /* 约0.03g以下保留原信号，避免把静止时的加速度噪声作为消除参考。 */
    if (motion->power < 0.0009f)
    {
        /* 运动不足：清空拟合系数并停用自适应。 */
        memset(motion->weights, 0, sizeof(motion->weights));
        motion->active = 0;
        detector.filtered = detector.optical;
        return 0;
    }
    if (motion->active == 0)
    {
        /* 首次进入有效运动段：记录起点并清零功率统计。 */
        motion->started_ms = now;
        motion->optical_power = motion->residual_power = 0.0f;
        motion->active = 1;
    }
    energy = 0.001f; /* 归一化分母下限，单位g平方。 */
    for (axis = 0; axis < 3; ++axis)
    {
        for (tap = 0; tap < APP_HEART_MOTION_TAPS; ++tap)
        {
            /* 用当前权重对历史参考做线性预测，作为PPG中的运动分量。 */
            prediction += motion->weights[axis][tap] * motion->history[axis][tap];
            /* 累加参考能量，作为NLMS步长归一化分母。 */
            energy += motion->history[axis][tap] * motion->history[axis][tap];
        }
    }
    /* 残差是原始PPG减去运动预测后的部分。 */
    residual = detector.optical - prediction;
    /* NLMS步长：固定0.08增益按残差与能量归一化。 */
    step = 0.08f * residual / energy;
    for (axis = 0; axis < 3; ++axis)
    {
        for (tap = 0; tap < APP_HEART_MOTION_TAPS; ++tap)
        {
            /* 沿参考方向更新权重，使下一拍预测更接近运动分量。 */
            motion->weights[axis][tap] += step * motion->history[axis][tap];
        }
    }
    /* 输出残差作为后续选峰的滤波信号。 */
    detector.filtered = residual;
    /* 平滑统计原始光量与残差功率。 */
    motion->optical_power += smoothing * (detector.optical * detector.optical - motion->optical_power);
    motion->residual_power += smoothing * (residual * residual - motion->residual_power);
    if ((uint32_t)(now - motion->started_ms) < APP_HEART_MOTION_SETTLE_MS ||
        motion->residual_power < motion->optical_power * 0.01f)
    {
        return 1; /* 尚未收敛或几乎全为运动相关波形，不能宣称测得有效脉搏。 */
    }
    return 0;
}

/* 环形窗口按时间顺序比较；右侧严格降低，平台峰取最后一个等高点。 */
static uint8_t heart_find_local_peak(uint32_t now, uint32_t *peak_tick)
{
    unsigned i, center, index, start;
    unsigned window = APP_HEART_PEAK_WINDOW; /* 比较窗口长度，默认用全部7点。 */
    float value;
    /* 把当前滤波值和时间戳写入环形窗口的下一空位。 */
    detector.window[detector.window_next] = detector.filtered;
    detector.window_ticks[detector.window_next] = now;
    /* 写指针前进一格并回绕。 */
    detector.window_next = (uint8_t)((detector.window_next + 1) % APP_HEART_PEAK_WINDOW);
    if (detector.window_count < APP_HEART_PEAK_WINDOW)
    {
        /* 未填满时累加有效点计数。 */
        ++detector.window_count;
    }
    if (detector.window_count < APP_HEART_PEAK_WINDOW)
    {
        /* 窗口未满，无法判定局部峰。 */
        return 0;
    }
    /* 最近六个间隔平均至少39ms时用五点，保持约160ms跨度；单帧抖动不切换窗口。 */
    if ((uint32_t)(now - detector.window_ticks[detector.window_next]) >= 39 * (APP_HEART_PEAK_WINDOW - 1))
    {
        /* 采样偏慢时缩短为5点，仍覆盖约160ms。 */
        window = 5;
    }
    /* 计算窗口起点：从最旧位置回退window个点。 */
    start = (detector.window_next + APP_HEART_PEAK_WINDOW - window) % APP_HEART_PEAK_WINDOW;
    /* 窗口正中点作为候选峰。 */
    center = (start + window / 2) % APP_HEART_PEAK_WINDOW;
    /* 取候选峰幅值。 */
    value = detector.window[center];
    for (i = 0; i < window; ++i)
    {
        /* 按时间顺序遍历窗口内各点。 */
        index = (start + i) % APP_HEART_PEAK_WINDOW;
        if ((i < window / 2 && detector.window[index] > value) ||
            (i > window / 2 && detector.window[index] >= value))
        {
            /* 左侧出现更高点或右侧出现等高/更高点，说明候选峰不是平台末端的局部极大。 */
            return 0;
        }
    }
    /* 通过检查，输出中心点的时间戳。 */
    *peak_tick = detector.window_ticks[center];
    return 1;
}

/* 每个门限独立记录峰谷和周期；波峰还须通过局部极大值检查。 */
static void heart_track_peak(heart_peak_struct *peak, float factor, uint32_t now, uint8_t local_peak, uint32_t local_peak_tick)
{
    float threshold = peak->swing * factor; /* 本门限的判定幅度：近期摆幅乘该路门限系数。 */
    float swing;
    uint32_t interval;
    uint32_t candidate_interval;
    if (peak->have_beat != 0 && (uint32_t)(now - peak->beat_tick) > APP_HEART_BEAT_TIMEOUT)
    {
        /* 超过2.5秒没有下一拍：整组状态清零，重建谷与周期起点。 */
        memset(peak, 0, sizeof(*peak));
        peak->trough = peak->cycle_peak = detector.filtered;
        peak->peak_tick = now;
        return;
    }
    /* 超过预计周期仍未确认下一拍时，逐步降低旧幅度门限，避免较小波峰一直无法更新门限。 */
    if (peak->stable_beats != 0 && (float)(uint32_t)(now - peak->beat_tick) > peak->interval_ms)
    {
        /* 按“预计周期/实际等待时长”比例衰减门限。 */
        threshold *= peak->interval_ms / (float)(uint32_t)(now - peak->beat_tick);
    }
    if (threshold < APP_HEART_MIN_SIGNAL)
    {
        /* 门限不低于8个ADC计数的最小信号幅度。 */
        threshold = APP_HEART_MIN_SIGNAL;
    }
    if (peak->armed == 0)
    {
        /* 未武装阶段只跟踪谷值。 */
        if (detector.filtered < peak->trough)
        {
            /* 刷新本组最低点。 */
            peak->trough = detector.filtered;
        }
        if (detector.filtered - peak->trough >= threshold)
        {
            /* 从谷底回升达到门限：武装并记下本周期峰起点。 */
            peak->armed = 1;
            peak->cycle_peak = detector.filtered;
            peak->peak_tick = now;
            /* 新峰尚未被局部极大值检查确认。 */
            peak->peak_confirmed = 0;
        }
        return;
    }
    if (detector.filtered >= peak->cycle_peak)
    {
        /* 武装后刷新周期内最高点及其时刻。 */
        peak->cycle_peak = detector.filtered;
        peak->peak_tick = now;
        /* 峰抬高后需重新等待局部峰确认。 */
        peak->peak_confirmed = 0;
    }
    if (local_peak != 0 && local_peak_tick == peak->peak_tick)
    {
        /* 环形窗口确认该时刻确为局部极大，标记本峰已确认。 */
        peak->peak_confirmed = 1;
    }
    if (peak->peak_confirmed == 0 || peak->cycle_peak - detector.filtered < threshold)
    {
        /* 未确认或回落不足门限：本拍尚未成立，继续等待。 */
        return;
    }
    /* 记录本周期峰谷摆幅。 */
    swing = peak->cycle_peak - peak->trough;
    /* 结束本周期武装，下一谷从当前值重新开始。 */
    peak->armed = 0;
    peak->trough = detector.filtered;
    if (swing < APP_HEART_MIN_SIGNAL * 2.0f)
    {
        /* 摆幅不足16计数：视为无效拍，清掉连续拍统计但保留门限起点。 */
        peak->have_beat = 0;
        peak->stable_beats = 0;
        peak->swing = 0.0f;
        return;
    }
    if (peak->swing == 0.0f)
    {
        /* 首个有效摆幅直接作为基准。 */
        peak->swing = swing;
    }
    else
    {
        /* 后续按0.75旧值+0.25新值平滑门限基准幅度。 */
        peak->swing = peak->swing * 0.75f + swing * 0.25f;
    }
    /* 本拍与上一确认拍的间隔，单位ms。 */
    interval = (uint32_t)(peak->peak_tick - peak->beat_tick);
    /* 本拍与上一候选拍的间隔，用于剔除过密次峰。 */
    candidate_interval = (uint32_t)(peak->peak_tick - peak->candidate_tick);
    /* 候选时刻推进到本拍。 */
    peak->candidate_tick = peak->peak_tick;
    if (peak->have_beat == 0)
    {
        /* 首拍只建立基准，不计算心率。 */
        peak->beat_tick = peak->peak_tick;
        peak->have_beat = 1;
        return;
    }
    /* 短次峰保留有效心跳；检查相邻候选间隔，避免把持续高频峰隔拍拼成心率。 */
    if (candidate_interval + APP_HEART_POLL_MS < APP_HEART_MIN_BEAT_MS)
    {
        /* 相邻候选间隔加上30ms采样容差后仍不足300ms：本拍不计为心跳；尚未建立连续序列时顺带更新诊断周期。 */
        if (peak->stable_beats == 0)
        {
            peak->last_interval_ms = (uint16_t)candidate_interval;
        }
        return;
    }
    /* 确认本拍为新的心跳基准时刻。 */
    peak->beat_tick = peak->peak_tick;
    if (interval > UINT16_MAX)
    {
        /* 超长间隔饱和到uint16上限，避免截断。 */
        peak->last_interval_ms = UINT16_MAX;
    }
    else
    {
        /* 常规间隔直接保存为诊断周期。 */
        peak->last_interval_ms = (uint16_t)interval;
    }
    /* 允许一帧采样量化误差，最终输出仍必须在40～200BPM内。 */
    if (interval + APP_HEART_POLL_MS < APP_HEART_MIN_BEAT_MS || interval > APP_HEART_MAX_BEAT_MS + APP_HEART_POLL_MS)
    {
        /* 考虑30ms采样容差后仍超出允许间隔范围：连续拍计数清零，等待重新建立节律。 */
        peak->stable_beats = 0;
        return;
    }
    if (peak->stable_beats == 0 || fabsf((float)interval - peak->interval_ms) > peak->interval_ms * APP_HEART_RR_TOLERANCE)
    {
        /* 首拍或间隔跳变超过35%：以本拍重新起算连续序列。 */
        peak->interval_ms = (float)interval;
        peak->rr[0] = (float)interval;
        peak->stable_beats = 1;
        peak->rr_next = 1;
        return;
    }
    /* 间隔连续：平滑更新周期，并把本拍间隔存入RR环形缓冲。 */
    peak->interval_ms = peak->interval_ms * 0.75f + (float)interval * 0.25f;
    peak->rr[peak->rr_next] = (float)interval;
    /* RR写指针前进一格并回绕。 */
    peak->rr_next = (uint8_t)((peak->rr_next + 1) % APP_HEART_STABLE_BEATS);
    if (peak->stable_beats < APP_HEART_STABLE_BEATS)
    {
        /* 连续拍计数累加，满3拍后可供选优。 */
        ++peak->stable_beats;
    }
}

/* 在合法且与近期有效周期连续的候选中按RR方差选优。
 * 五组共享滤波，三个连续相近间隔后才输出；旧周期超过2.5秒未更新时允许重新选定。 */
static void heart_process(uint16_t raw, uint32_t now, uint32_t elapsed, app_heart_struct *data)
{
    heart_peak_struct *best = NULL; /* 当前RR方差最小的候选门限。 */
    float best_variance = 0.0f; /* 对应候选的RR方差。 */
    float mean, variance, difference; /* RR均值、方差与单点偏差。 */
    uint16_t jump_limit, raw_difference; /* 本帧突变门限与相邻原始值绝对差，单位ADC计数。 */
    uint16_t bpm; /* 候选门限换算出的心率。 */
    unsigned i, j; /* i遍历五组门限，j遍历RR缓冲。 */
    uint8_t active = 0; /* 任一门限已有心跳时为1。 */
    uint8_t local_peak; /* 本帧是否确认为局部峰。 */
    uint32_t local_peak_tick = 0; /* 局部峰对应时刻，单位ms。 */
    /* 先把本帧原始值与采样时刻写入对外结果。 */
    data->raw = raw;
    data->sample_ms = now;
    if (raw > detector.previous_raw)
    {
        /* 原始值上升：差值取新减旧。 */
        raw_difference = raw - detector.previous_raw;
    }
    else
    {
        /* 原始值下降或持平：差值取旧减新，得到绝对变化量。 */
        raw_difference = detector.previous_raw - raw;
    }
    /* 突变门限取上一原始值的四分之一。 */
    jump_limit = detector.previous_raw / 4;
    if (jump_limit < APP_HEART_JUMP_COUNTS)
    {
        /* 门限下限固定为1024计数：低于此值一律按1024处理。 */
        jump_limit = APP_HEART_JUMP_COUNTS;
    }
    /* 极值/大幅阶跃重建基线；上一帧滤波偏移也覆盖多帧累计的接触变化。原始值仍照常导出。 */
    if (raw == 0 || raw == UINT16_MAX ||
        (detector.seeded != 0 &&
         (raw_difference > jump_limit ||
          (fabsf(detector.optical) > APP_HEART_JUMP_COUNTS && fabsf(detector.optical) > detector.dc * 0.25f))))
    {
        /* 判定为接触跳变/饱和：整个检测器状态(含滤波、窗口、五组峰、运动拟合)清零。 */
        memset(&detector, 0, sizeof(detector));
    }
    if (detector.seeded == 0)
    {
        /* 首次或重建后：以当前原始值作为背景基线，并记录预热起点。 */
        detector.dc = (float)raw;
        detector.warmup_tick = now;
        detector.seeded = 1;
    }
    /* 保存本帧原始值，供下帧突变判断。 */
    detector.previous_raw = raw;
    if ((uint32_t)(now - detector.warmup_tick) < APP_HEART_WARMUP_MS)
    {
        /* 启动/重建期间跟随当前背景，避免首帧未稳定让1.5秒背景滤波长时间追赶。 */
        detector.dc = (float)raw;
        /* 光量、滤波值与幅度统计保持为0。 */
        detector.optical = detector.filtered = detector.amplitude = 0.0f;
        /* 预热时填满零基线窗口，首个真实波峰不会因窗口未满而漏确认。 */
        (void)heart_find_local_peak(now, &local_peak_tick);
        /* 五组峰状态以当前零点重建。 */
        heart_reset_peaks(now);
        /* 对外信号幅度、周期、BPM均清零。 */
        data->signal_x10 = 0;
        data->interval_ms = 0;
        data->bpm = 0;
        if (raw == 0 || raw == UINT16_MAX)
        {
            /* 原始值处于极值：判定无有效信号。 */
            data->state = APP_HEART_NO_SIGNAL;
        }
        else
        {
            /* 原始值正常：仍在预热测量中。 */
            data->state = APP_HEART_MEASURING;
        }
        return;
    }
    /* 背景基线按约1.5秒时间常数跟踪原始值。 */
    detector.dc += (float)elapsed / (1500.0f + (float)elapsed) * ((float)raw - detector.dc);
    /* 去背景后的交流分量按约70ms时间常数低通。 */
    detector.optical += (float)elapsed / (70.0f + (float)elapsed) * ((float)raw - detector.dc - detector.optical);
    if (heart_remove_motion(now, elapsed) != 0)
    {
        /* 运动滤波尚在稳定期或原信号几乎全被运动参考拟合：本轮不判峰，清窗口与输出周期。 */
        heart_reset_peaks(now);
        detector.window_count = detector.window_next = 0;
        detector.output_interval_ms = 0.0f;
        detector.amplitude = 0.0f;
        data->bpm = data->interval_ms = 0;
        data->signal_x10 = 0;
        data->state = APP_HEART_MEASURING;
        return;
    }
    /* 滤波信号绝对值按约300ms时间常数平滑，得到显示用信号幅度。 */
    detector.amplitude += (float)elapsed / (300.0f + (float)elapsed) * (fabsf(detector.filtered) - detector.amplitude);
    /* 幅度乘10后四舍五入为整数，单位0.1个ADC计数。 */
    data->signal_x10 = (uint32_t)(detector.amplitude * 10.0f + 0.5f);
    /* 把当前采样写入窗口，再判断窗口中心历史点是否为局部峰，并取回其时刻。 */
    local_peak = heart_find_local_peak(now, &local_peak_tick);
    for (i = 0; i < APP_HEART_GATE_COUNT; ++i)
    {
        /* 逐个门限更新本帧峰谷与周期。 */
        heart_peak_struct *peak = &detector.peaks[i];
        heart_track_peak(peak, heart_gate_factors[i], now, local_peak, local_peak_tick);
        /* 汇总是否有任一门限已建立心跳。 */
        active |= peak->have_beat;
        if (peak->stable_beats < APP_HEART_STABLE_BEATS)
        {
            /* 连续相近间隔不足3拍：该门限尚不可输出。 */
            continue;
        }
        /* 周期换算为BPM并四舍五入。 */
        bpm = (uint16_t)(60000.0f / peak->interval_ms + 0.5f);
        if (bpm < APP_HEART_MIN_BPM || bpm > APP_HEART_MAX_BPM)
        {
            /* 超出40～200BPM范围：跳过该候选。 */
            continue;
        }
        /* 最近有效周期仍新鲜时，只在原25%周期容差内选优；失去连续性就重新测量。 */
        if (detector.output_interval_ms != 0.0f && (uint32_t)(now - detector.output_beat_tick) <= APP_HEART_BEAT_TIMEOUT &&
            fabsf(peak->interval_ms - detector.output_interval_ms) > detector.output_interval_ms * 0.25f)
        {
            /* 与近期输出周期偏差超过25%：视为不连续，跳过。 */
            continue;
        }
        mean = 0.0f;
        for (j = 0; j < APP_HEART_STABLE_BEATS; ++j)
        {
            /* 累加3个RR间隔求平均。 */
            mean += peak->rr[j] / (float)APP_HEART_STABLE_BEATS;
        }
        variance = 0.0f;
        for (j = 0; j < APP_HEART_STABLE_BEATS; ++j)
        {
            /* 累加各RR与均值的偏差平方，得到总体方差。 */
            difference = peak->rr[j] - mean;
            variance += difference * difference / (float)APP_HEART_STABLE_BEATS;
        }
        /* 方差相等时选较高门限，减少重复识别小次峰。 */
        if (best == NULL || variance <= best_variance)
        {
            /* 记录当前最优候选及其方差。 */
            best = peak;
            best_variance = variance;
        }
    }
    if (best != NULL)
    {
        /* 选中候选：记录其平滑周期与拍时刻，供下一帧连续性检查。 */
        detector.output_interval_ms = best->interval_ms;
        detector.output_beat_tick = best->beat_tick;
        /* 由选中周期换算对外BPM。 */
        data->bpm = (uint16_t)(60000.0f / best->interval_ms + 0.5f);
        /* 对外周期取该门限最近一次相邻间隔，并把状态置为有效。 */
        data->interval_ms = best->last_interval_ms;
        data->state = APP_HEART_VALID;
        return;
    }
    /* 无候选可输出：BPM清零，周期仍取中间30%门限做诊断。 */
    data->bpm = 0;
    data->interval_ms = detector.peaks[2].last_interval_ms; /* 未选优时保留原30%门限的周期诊断 */
    if (active != 0 || (uint32_t)(now - detector.warmup_tick) <= APP_HEART_BEAT_TIMEOUT)
    {
        /* 仍有心跳活动或从预热起点起尚未超过2.5秒：继续测量。 */
        data->state = APP_HEART_MEASURING;
    }
    else
    {
        /* 长时间无任何心跳：判定无信号。 */
        data->state = APP_HEART_NO_SIGNAL;
    }
}

void app_heart_init_hw(void)
{
    app_diag_sensor_init(APP_DIAG_HEART, em7028_init());
}

/* 创建快照锁并保留启动时的器件故障；器件待机配置由sensor启动流程完成。 */
void app_heart_init(void)
{
    /* 发布快照锁前保存启动故障，UI此时仍按未初始化处理。 */
    em7028_get_fault(&heart_data.fault);
    if (heart_data.fault.kind != EM7028_FAULT_NONE)
    {
        /* 启动时已有器件故障：对外状态直接置错误并上报诊断。 */
        heart_data.state = APP_HEART_ERROR;
        app_diag_heart_fault((uint8_t)heart_data.fault.kind, heart_data.fault.reg, heart_data.fault.value);
    }
    /* 创建快照互斥锁，供后续跨任务读写heart_data。 */
    heart_mutex = xSemaphoreCreateMutex();
}

/* 亮/熄屏切换都把自动周期起点对齐到此刻，并清空样本队列，不把亮屏手动波形留到熄屏后导出。 */
void app_heart_set_sleep_mode(uint8_t sleeping)
{
    if (heart_mutex == NULL)
    {
        /* 锁尚未创建：此时无共享状态可写，直接返回。 */
        return;
    }
    /* 取快照锁，独占模式与队列状态。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    /* 记录当前亮/熄屏模式。 */
    s_sleep_mode = sleeping;
    /* 任何亮熄屏切换都结束在途后台窗口。 */
    s_sleep_active = 0;
    s_sleep_cycle_ms = HAL_GetTick(); /* 熄屏时把周期起点对齐到本次熄屏，配合启用起点满足“启用/熄屏起等10分钟”。 */
    /* 丢弃旧样本，避免亮屏手动波形串到熄屏导出。 */
    sample_head = sample_count = 0;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
}

/* 后台自动心率开关；只提交请求，启用时把周期起点对齐到此刻，实际停测由sensor下一轮完成。 */
void app_heart_set_auto_enabled(uint8_t enabled)
{
    if (heart_mutex == NULL)
    {
        /* 锁未创建时无法提交请求，直接返回。 */
        return;
    }
    /* 取快照锁后更新自动开关。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    /* 归一化为0/1保存。 */
    s_auto_enabled = enabled != 0;
    if (s_auto_enabled != 0)
    {
        s_sleep_cycle_ms = HAL_GetTick(); /* 启用后从此刻起算10分钟，不在每轮service重置。 */
    }
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
    /* 只提交请求：关闭时在途后台测量由 sensor 任务在下一轮 service 停止并保留有效值。 */
}

/* 锁内读取后台自动心率开关；锁未建时按关闭处理。 */
uint8_t app_heart_get_auto_enabled(void)
{
    uint8_t enabled;
    if (heart_mutex == NULL)
    {
        /* 锁未建时没有可读的共享开关，按关闭返回。 */
        return 0;
    }
    /* 取快照锁后读取开关副本。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    enabled = s_auto_enabled;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
    return enabled;
}

/* 供sensor判断本轮是否该提前收尾：在途测量/停测、后台窗口进行中或已到期都返回1；
 * 熄屏自动关闭或尚未到期返回0。锁内取模式与周期起点，避免读到半更新状态。 */
uint8_t app_heart_sleep_pending(void)
{
    uint8_t sleep_mode; /* 当前亮/熄屏模式副本。 */
    uint8_t sleep_active; /* 后台窗口是否进行中。 */
    uint8_t auto_enabled; /* 后台自动开关副本。 */
    uint32_t cycle_ms; /* 后台周期起点副本，单位ms。 */

    /* 与UI写入AUTO及周期起点使用同一把锁；在途采样/关闭仍须由sensor完成。 */
    if (heart_mutex == NULL)
    {
        /* 锁未建：只能看无锁的在途标志，有测量或待停就返回1。 */
        return running != 0 || s_stop_pending != 0;
    }
    /* 锁内一次性取模式、窗口、自动开关和周期起点副本。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    sleep_mode = s_sleep_mode;
    sleep_active = s_sleep_active;
    auto_enabled = s_auto_enabled;
    cycle_ms = s_sleep_cycle_ms;
    /* 释放快照锁，后续判断不再持锁。 */
    xSemaphoreGive(heart_mutex);
    if (running != 0 || s_stop_pending != 0 || sleep_active != 0)
    {
        /* 在途测量、待停或后台窗口进行中：sensor本轮需继续处理。 */
        return 1;
    }
    if (auto_enabled == 0 || sleep_mode == 0)
    {
        /* 自动关闭或亮屏：没有待收尾的后台测量。 */
        return 0;
    }
    if ((uint32_t)(HAL_GetTick() - cycle_ms) >= APP_HEART_SLEEP_PERIOD_MS)
    {
        /* 距周期起点已满10分钟：后台窗口到期，需要开始测量。 */
        return 1;
    }
    return 0;
}

/* 运动模式强制连续测量并覆盖手动请求；样本队列同样清空(运动期间不导出原始波形)。 */
void app_heart_set_sport_mode(uint8_t enabled)
{
    if (heart_mutex == NULL)
    {
        /* 锁未创建时无法写入模式，直接返回。 */
        return;
    }
    /* 取快照锁后切换运动模式。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    s_sport_mode = enabled;
    /* 每次切换都清除上次运动失败锁存，允许重新尝试启动。 */
    s_sport_failed = 0;
    /* 丢弃旧样本，运动期间不导出原始波形。 */
    sample_head = sample_count = 0;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
}

/* 后台自动窗口收尾：有效结果转成last_bpm，照明/HRS当拍就停；停测写失败留标志由下轮重试。 */
static void heart_finish_sleep(app_heart_struct *data)
{
    em7028_result result;
    if (data->state == APP_HEART_VALID)
    {
        /* 窗口结束前有有效结果：转存为最近有效BPM，供熄屏桌面沿用。 */
        data->last_bpm = data->bpm;
    }
    result = em7028_stop(); /* 同一拍关闭照明和HRS，不等下次采样。 */
    /* 待机写失败则保留标志，下轮service重试。 */
    s_stop_pending = result != EM7028_OK;
    /* 标记测量已结束。 */
    running = 0;
    /* 锁内清除后台窗口进行标记。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    s_sleep_active = 0;
    xSemaphoreGive(heart_mutex);
    if (result != EM7028_OK)
    {
        /* 停测失败：BPM清零、状态置错误，并刷新故障快照。 */
        data->bpm = 0;
        data->state = APP_HEART_ERROR;
        em7028_get_fault(&data->fault);
    }
    else if (data->state != APP_HEART_VALID)
    {
        /* 停测成功但窗口内无有效结果：对外按无信号收尾。 */
        data->bpm = 0;
        data->state = APP_HEART_NO_SIGNAL;
    }
    /* 把收尾结果发布到共享快照。 */
    heart_publish(data);
}

/* UI只提交手动请求(START/STOP)，实际开关器件由sensor下一轮service完成。 */
void app_heart_set_enabled(uint8_t enabled)
{
    if (heart_mutex == NULL)
    {
        /* 锁未创建时无法提交请求，直接返回。 */
        return;
    }
    /* 取快照锁后记录UI的启动/停止意图。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    requested = enabled != 0;
    /* 新请求从空样本队列开始。 */
    sample_head = sample_count = 0;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
}

/* 展示口径在这里统一推导，UI不必自己组合这些标志：
 * enabled按运动/熄屏+自动开关覆盖；有请求但sensor尚未启动时先显示MEASURING；
 * 亮屏且未在测量归OFF；VALID超过APP_HEART_STALE_MS未刷新结果降级为NO_SIGNAL。 */
void app_heart_get_snapshot(app_heart_struct *heart)
{
    uint8_t sleeping; /* 1表示熄屏且非运动模式，决定是否套用后台显示口径。 */
    if (heart == NULL)
    {
        /* 无输出缓冲：不写任何数据直接返回。 */
        return;
    }
    if (heart_mutex == NULL)
    {
        /* 锁未建：返回全零快照并把状态置为错误。 */
        memset(heart, 0, sizeof(*heart));
        heart->state = APP_HEART_ERROR;
        return;
    }
    /* 锁内拷贝共享快照与模式标志。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    *heart = heart_data;
    /* 默认启用标志取UI手动请求。 */
    heart->enabled = requested;
    if (s_sport_mode != 0)
    {
        /* 运动模式覆盖手动请求：未锁存失败即视为启用。 */
        heart->enabled = s_sport_failed == 0;
    }
    else if (s_sleep_mode != 0)
    {
        if (s_auto_enabled != 0)
        {
            /* 熄屏且后台自动开启：只在窗口进行中显示启用。 */
            heart->enabled = s_sleep_active;
        }
        else
        {
            heart->enabled = 0; /* 后台关闭时不显示继续采样，即便在途窗口尚未由 sensor 停止。 */
        }
    }
    /* 只有熄屏且非运动模式才按“睡眠”口径处理显示。 */
    sleeping = s_sleep_mode != 0 && s_sport_mode == 0;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
    if (heart->enabled != 0 && s_stop_pending == 0 && (heart->state == APP_HEART_OFF || heart->state == APP_HEART_ERROR))
    {
        heart->state = APP_HEART_MEASURING; /* 启动请求可能正在等待 sensor 任务处理 */
    }
    if (sleeping == 0 && heart->enabled == 0 && heart->state != APP_HEART_ERROR)
    {
        /* 亮屏且未启用：对外显示关闭并清空BPM。 */
        heart->bpm = 0;
        heart->state = APP_HEART_OFF;
    }
    if ((sleeping == 0 || heart->enabled != 0) && heart->state == APP_HEART_VALID &&
        (uint32_t)(HAL_GetTick() - heart->sample_ms) > APP_HEART_STALE_MS)
    {
        /* 有效值超过250ms未刷新：降级为无信号并清空BPM。 */
        heart->bpm = 0;
        heart->state = APP_HEART_NO_SIGNAL;
    }
}

/* 取走即出队(旧样本不重复发送)；size为调用方缓冲容量，返回实际取出的条数。 */
uint16_t app_heart_read_samples(app_heart_sample_struct *samples, uint16_t size)
{
    uint16_t count = 0; /* 已取出的条数。 */
    if (samples == NULL || heart_mutex == NULL)
    {
        /* 无缓冲或锁未建：无法安全取样本。 */
        return 0;
    }
    /* 锁内独占队列读写。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    while (count < size && sample_count != 0)
    {
        /* 从队头取一条样本到调用方缓冲。 */
        samples[count++] = heart_samples[sample_head];
        /* 队头前进一格并回绕。 */
        sample_head = (uint16_t)((sample_head + 1) % APP_HEART_SAMPLE_COUNT);
        /* 有效样本数减一。 */
        --sample_count;
    }
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
    return count;
}

/* sensor每轮调用，顺序固定：补做未完成的待机→按运动/熄屏/请求定使能→后台窗口超时收尾→
 * 停测或启动→按APP_HEART_POLL_MS下限读PPG并估算。返回1表示本轮访问了PPG总线。 */
uint8_t app_heart_service(void)
{
    app_heart_struct data;
    app_heart_sample_struct sample;
    em7028_result result;
    uint16_t raw;
    uint32_t now = HAL_GetTick(); /* 本轮统一使用的时间基准，单位ms。 */
    uint32_t elapsed;
    uint8_t enabled;
    uint32_t read_started;
    if (heart_mutex == NULL)
    {
        /* 锁未建：不访问器件，直接报告本轮未读总线。 */
        return 0;
    }
    if (s_stop_pending != 0)
    {
        /* 旧测量先完成待机，再处理新START或自动窗口；保留最初的故障快照。 */
        s_stop_pending = em7028_stop() != EM7028_OK;
        return 1;
    }
    /* 锁内推导本轮是否应测量，并取一份共享快照副本。 */
    xSemaphoreTake(heart_mutex, portMAX_DELAY);
    enabled = requested;
    if (s_sport_mode != 0)
    {
        /* 运动模式强制连续测量，失败锁存时暂停。 */
        enabled = s_sport_failed == 0;
    }
    else if (s_sleep_mode != 0)
    {
        if (s_auto_enabled != 0)
        {
            if (s_sleep_active == 0 && (uint32_t)(now - s_sleep_cycle_ms) >= APP_HEART_SLEEP_PERIOD_MS)
            {
                /* 熄屏且自动开启、距周期起点满10分钟：开启一轮后台窗口并记录起点。 */
                s_sleep_cycle_ms = s_sleep_started_ms = now;
                s_sleep_active = 1;
            }
        }
        else
        {
            s_sleep_active = 0; /* 后台关闭：结束在途窗口，下面 enabled==0 流程负责停测关LED并保留 last_bpm。 */
        }
        /* 熄屏时启用状态只由后台窗口决定。 */
        enabled = s_sleep_active;
    }
    /* 拷贝共享快照作为本轮处理的基础。 */
    data = heart_data;
    /* 释放快照锁。 */
    xSemaphoreGive(heart_mutex);
    if (s_sleep_active != 0 && (uint32_t)(now - s_sleep_started_ms) >= APP_HEART_SLEEP_TIMEOUT_MS)
    {
        /* 后台窗口已满10秒：无论是否得到有效值都在本拍收尾停测。 */
        heart_finish_sleep(&data);
        return 1;
    }
    if (enabled == 0)
    {
        if (running != 0)
        {
            /* 由启用转为停用且仍在运行：本拍停测并关闭照明/HRS。 */
            result = em7028_stop();
            s_stop_pending = result != EM7028_OK;
            running = 0;
            data.bpm = 0;
            if (result == EM7028_OK)
            {
                /* 停测成功：对外状态置关闭。 */
                data.state = APP_HEART_OFF;
            }
            else
            {
                /* 停测失败：对外状态置错误。 */
                data.state = APP_HEART_ERROR;
            }
            if (result != EM7028_OK)
            {
                /* 刷新失败对应的器件故障快照。 */
                em7028_get_fault(&data.fault);
            }
            /* 发布停测后的结果。 */
            heart_publish(&data);
            return 1;
        }
        /* 本来就没在测量：本轮无总线动作。 */
        return 0;
    }
    if (running == 0)
    {
        /* 需要测量但尚未启动：本拍启动器件。 */
        result = em7028_start();
        /* 记录启动结果供诊断。 */
        app_diag_sensor_init(APP_DIAG_HEART, (uint32_t)result);
        if (result != EM7028_OK)
        {
            /* 启动失败：走统一故障收尾。 */
            heart_fail(&data);
            return 1;
        }
        /* 启动成功：清空检测器全部滤波/峰状态。 */
        memset(&detector, 0, sizeof(detector));
        /* 清空对外结果，状态置测量中。 */
        memset(&data, 0, sizeof(data));
        data.state = APP_HEART_MEASURING;
        /* 采样节拍、有效读数时刻与样本时间起点都对齐到本次启动。 */
        poll_tick = good_tick = now;
        sample_start_tick = now;
        sample_sequence = 0;
        /* 标记测量已运行。 */
        running = 1;
        heart_publish(&data);
        return 1;
    }
    /* 已运行：按30ms下限节流读取。 */
    elapsed = (uint32_t)(now - poll_tick);
    if (elapsed < APP_HEART_POLL_MS)
    {
        /* 未到下一轮读取时刻。 */
        return 0;
    }
    poll_tick = now;
    if (elapsed > APP_HEART_MAX_GAP_MS)
    {
        /* 距上次轮询超过120ms：采样节拍断档，清检测器并重新测量。 */
        memset(&detector, 0, sizeof(detector));
        data.bpm = 0;
        data.state = APP_HEART_MEASURING;
    }
    /* 记录读前时钟计数，用于诊断本次总线耗时。 */
    read_started = app_diag_clock_count();
    result = em7028_read_raw(&raw);
    /* 上报本次读结果与耗时。 */
    app_diag_sensor_record(APP_DIAG_HEART, (uint8_t)result, (uint32_t)result, read_started);
    /* 短暂 NACK/跨更新只丢弃本帧；连续 250ms 无有效读数才关闭测量。 */
    if (result != EM7028_OK && (uint32_t)(now - good_tick) > APP_HEART_STALE_MS)
    {
        /* 超过250ms没有有效读数：按故障停测。 */
        heart_fail(&data);
        return 1;
    }
    if (result == EM7028_OK)
    {
        /* 有效读数：计算距上一有效读数的间隔。 */
        elapsed = (uint32_t)(now - good_tick);
        if (elapsed > APP_HEART_MAX_GAP_MS)
        {
            /* 有效读数间隔也超过120ms：滤波与周期需要重建。 */
            memset(&detector, 0, sizeof(detector));
            data.bpm = 0;
            data.state = APP_HEART_MEASURING;
        }
        /* 更新最近有效读数时刻。 */
        good_tick = now;
        /* 把原始值送入检测流程，结果写回data。 */
        heart_process(raw, now, elapsed, &data);
        /* 组装一条原始波形样本：时间戳、原始值、滤波值、背景基线。 */
        sample.elapsed_ms = (uint32_t)(now - sample_start_tick);
        sample.raw = raw;
        sample.filtered = detector.filtered;
        sample.dc = detector.dc;
        /* 带上本帧心率、周期与状态。 */
        sample.bpm = data.bpm;
        sample.interval_ms = data.interval_ms;
        sample.state = data.state;
        /* 样本序号递增，供接收端检查连续性。 */
        sample.sequence = sample_sequence++;
        /* 仅成功读数入队；与UI共用快照锁，UART发送时不占此锁。 */
        xSemaphoreTake(heart_mutex, portMAX_DELAY);
        if (requested != 0 && s_sleep_mode == 0 && s_sport_mode == 0)
        {
            /* 只在亮屏手动测量时保留波形，熄屏/运动不导出。 */
            if (sample_count == APP_HEART_SAMPLE_COUNT)
            {
                /* 队列已满：丢弃最旧一条并计数溢出，再写入新样本。 */
                app_diag_heart_queue(1);
                sample_head = (uint16_t)((sample_head + 1) % APP_HEART_SAMPLE_COUNT);
                --sample_count;
            }
            /* 把样本追加到队尾。 */
            heart_samples[(sample_head + sample_count) % APP_HEART_SAMPLE_COUNT] = sample;
            ++sample_count;
        }
        xSemaphoreGive(heart_mutex);
    }
    if (s_sleep_active != 0 && data.state == APP_HEART_VALID)
    {
        /* 后台窗口已得到有效结果：提前收尾停测。 */
        heart_finish_sleep(&data);
        return 1;
    }
    /* 发布本轮测量结果。 */
    heart_publish(&data);
    return 1;
}
