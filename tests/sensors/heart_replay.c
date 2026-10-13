/* 回放实测原始值和板端读取时间；包含真实服务以比较滤波和选优过程，不访问硬件。 */
#define main heart_synthetic_main
#include "heart_harness.c"
#undef main
#include "../../application/app_heart.c"
#include <stdlib.h>

int main(int argc, char **argv)
{
    FILE *input;
    FILE *output;
    char line[2048];
    unsigned index, rx_ms;
    float recorded_raw, recorded_filtered, recorded_dc, recorded_bpm, recorded_interval, seconds, sequence, state;
    app_heart_sample_struct sample;
    unsigned rows = 0;
    if (argc != 3)
    {
        return 2;
    }
    input = fopen(argv[1], "r");
    output = fopen(argv[2], "w");
    assert(input != NULL && output != NULL);
    assert(fgets(line, sizeof(line), input) != NULL);
    app_heart_init();
    tick = 0;
    begin();
    fprintf(output, "time_ms,raw,old_filtered,new_filtered,old_bpm,new_bpm,old_state,new_state,new_dc,new_interval_ms\n");
    while (fgets(line, sizeof(line), input) != NULL)
    {
        assert(sscanf(line, "%u,%u,%f,%f,%f,%f,%f,%f,%f,%f", &index, &rx_ms, &recorded_raw, &recorded_filtered,
                      &recorded_dc, &recorded_bpm, &recorded_interval, &seconds, &sequence, &state) == 10);
        tick = (uint32_t)(seconds * 1000.0f + 0.5f);
        raw = (uint16_t)recorded_raw;
        app_heart_service();
        assert(app_heart_read_samples(&sample, 1) == 1);
        assert(sample.raw == raw && sample.sequence == (uint16_t)sequence && sample.elapsed_ms == tick);
        fprintf(output, "%lu,%u,%.6f,%.6f,%.0f,%u,%u,%u,%.6f,%u\n", (unsigned long)tick, raw,
                (double)recorded_filtered, (double)sample.filtered, (double)recorded_bpm, sample.bpm, (unsigned)state,
                (unsigned)sample.state, (double)sample.dc, sample.interval_ms);
        if (sample.state == APP_HEART_VALID)
        {
            assert(sample.bpm >= 40 && sample.bpm <= 200);
        }
        ++rows;
    }
    assert(fclose(input) == 0 && fclose(output) == 0 && rows != 0);
    printf("PASS: replayed %u recorded samples through the real service.\n", rows);
    return 0;
}
