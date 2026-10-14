#ifndef DIAG_HOST_LVGL_H
#define DIAG_HOST_LVGL_H
#include <stdint.h>
typedef struct
{
    uint32_t total_size, free_size, max_used;
} lv_mem_monitor_t;
void lv_mem_monitor(lv_mem_monitor_t *memory);
#endif
