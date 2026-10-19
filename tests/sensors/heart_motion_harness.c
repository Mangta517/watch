/* 复用真实服务桩，构造同步PPG/三轴加速度；不代表实板运动心率准确率。 */
#define main heart_existing_tests_main
#include "heart_harness.c"
#undef main
#include "mpu6050.h"
#include <stdlib.h>

/* 不弹出Windows CRT对话框，让失败用例直接返回给运行脚本。 */
#undef assert
#define assert(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
            exit(1); \
        } \
    } while (0)

static void motion_wave(uint32_t duration_ms, float pulse_bpm, float motion_bpm, float pulse_amplitude,
                        float motion_amplitude, uint8_t axis, uint8_t supply_motion)
{
    app_heart_struct heart;
    float phase, motion, pulse;
    int16_t accel[3];
    uint32_t elapsed;
    uint32_t step = 10U;
    uint32_t valid = 0, correct = 0;

    for (elapsed = 0; elapsed < duration_ms; elapsed += step)
    {
        step = 10U;
        if (supply_motion == 2U && elapsed % 70U == 0U)
        {
            step = 20U;
        }
        tick += step;
        phase = (float)(elapsed + step) * 6.2831853f / 60000.0f;
        motion = sinf(phase * motion_bpm);
        pulse = sinf(phase * pulse_bpm + 0.7f);
        accel[0] = accel[1] = 0;
        accel[2] = (int16_t)MPU6050_ACCEL_LSB_PER_G;
        accel[axis] += (int16_t)(0.3f * MPU6050_ACCEL_LSB_PER_G * motion);
        if (supply_motion != 0)
        {
            app_heart_motion_update(accel[0], accel[1], accel[2], tick);
        }
        raw = (uint16_t)(30000.0f + pulse_amplitude * pulse + motion_amplitude * sinf(phase * motion_bpm + 0.45f));
        app_heart_service();
        app_heart_get_snapshot(&heart);
        if (elapsed >= 12000U && heart.sample_ms == tick)
        {
            if (heart.state == APP_HEART_VALID)
            {
                ++valid;
                if (fabsf((float)heart.bpm - pulse_bpm) <= 4.0f)
                {
                    ++correct;
                }
            }
        }
    }
    app_heart_get_snapshot(&heart);
    printf("pulse=%.0f motion=%.0f axis=%u reference=%u: state=%u BPM=%u, correct/valid=%lu/%lu\n",
           (double)pulse_bpm, (double)motion_bpm, axis, supply_motion, (unsigned)heart.state, heart.bpm,
           (unsigned long)correct, (unsigned long)valid);
    if (pulse_amplitude == 0.0f || (supply_motion != 0 && pulse_bpm == motion_bpm && motion_bpm != 0.0f))
    {
        assert(valid == 0U && heart.bpm == 0);
    }
    else
    {
        assert(valid > 100U && correct * 100U >= valid * 95U);
        assert(heart.state == APP_HEART_VALID && fabsf((float)heart.bpm - pulse_bpm) <= 4.0f);
    }
}

int main(void)
{
    uint8_t axis;
    app_heart_init();
    for (axis = 0; axis < 3U; ++axis)
    {
        begin();
        motion_wave(24000U, 90.0f, 150.0f, 120.0f, 400.0f, axis, 1);
        begin();
        motion_wave(18000U, 0.0f, 120.0f, 0.0f, 400.0f, axis, 1);
    }
    tick = UINT32_MAX - 2000U;
    begin();
    motion_wave(24000U, 60.0f, 150.0f, 120.0f, 400.0f, 0, 1);
    begin();
    motion_wave(24000U, 120.0f, 180.0f, 120.0f, 400.0f, 1, 2);
    begin();
    motion_wave(24000U, 180.0f, 120.0f, 120.0f, 400.0f, 2, 1);
    begin();
    motion_wave(18000U, 90.0f, 90.0f, 120.0f, 400.0f, 0, 1);
    begin();
    motion_wave(18000U, 90.0f, 150.0f, 120.0f, 0.0f, 1, 1);
    /* IMU断流后原PPG仍可测量；不能把最后一份运动样本反复当作新数据。 */
    motion_wave(18000U, 90.0f, 0.0f, 120.0f, 0.0f, 0, 0);
    begin();
    motion_wave(18000U, 90.0f, 0.0f, 120.0f, 0.0f, 0, 1);
    puts("PASS: synchronized motion rejection, mixed pulse recovery, three axes, stale IMU fallback, static reference and tick wrap.");
    return 0;
}
