/**
  ******************************************************************************
  * @file    app_diag.c
  * @brief   应用层-运行诊断与二进制遥测 (WDG1 帧)
  *          数据归属: 任务周期/工作时长由各任务在循环首尾自行上报，传感器结果由驱动
  *          调用点上报，启动阶段时间戳由 app_task 与 sensor 任务写入，喂狗与复位统计在
  *          app_reset; 本层配置并读取 TIM2、经 UART 分页发送，不主动采集任何传感器。
  *          计时: TIM2 自由运行、100us 一格 (PCLK1 倍频后 10kHz 分频)，比 1ms 的 HAL
  *          tick 细，用于任务工作时长与单次 I2C 事务耗时; STOP 期间 TIM2 停走，由
  *          app_tickless 把停走时间补回来，故诊断占比在深睡后仍连续。
  *          发送: 默认关闭，手机串口发 DIAG,1 才开启; 每次调用最多发一行，整轮行号推进完后
  *          重新采样，关闭后仍负责收回自己在途的异步发送结果。
  ******************************************************************************
***/
#include "app_diag.h"
#include "app_touch.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "kt6368a.h"
#include "lvgl.h"
#include <string.h>

#define APP_DIAG_MAX_TASKS 8
#define APP_DIAG_FRAME_MS  100

typedef struct
{
    uint32_t started;
    uint32_t loops;
    uint32_t period_last;
    uint32_t period_min;
    uint32_t period_max;
    uint32_t work_last;
    uint32_t work_max;
    uint32_t overruns;
    uint32_t target_ms;
    uint8_t have_start;
} app_diag_task_struct;

typedef struct
{
    uint32_t ok;
    uint32_t not_ready;
    uint32_t errors;
    uint32_t result;
    uint32_t good_ms;
    uint32_t gap_last_ms;
    uint32_t gap_max_ms;
    uint32_t work_last_us;
    uint32_t work_max_us;
    uint32_t init_result;
    uint32_t recovery_attempts;
    uint8_t recovery_state;
} app_diag_sensor_struct;

static app_diag_task_struct s_tasks[APP_DIAG_TASK_COUNT];
static app_diag_sensor_struct s_sensors[APP_DIAG_SENSOR_COUNT];
static TaskStatus_t s_task_status[APP_DIAG_MAX_TASKS];
static uint32_t s_task_words[APP_DIAG_MAX_TASKS][14];
static uint8_t s_task_ids[APP_DIAG_MAX_TASKS];
static uint32_t s_sensor_words[APP_DIAG_SENSOR_COUNT][10];
static uint32_t s_system_words[16];
static uint32_t s_boot_values[APP_BOOT_FIELD_COUNT];
static uint32_t s_boot_words[APP_BOOT_FIELD_COUNT];
static uint32_t s_imu_save_values[4] = {0, 0, UINT32_MAX, UINT32_MAX};
static uint32_t s_imu_save_words[4];
static uint32_t s_mpu_cal_values[16];
static uint32_t s_mpu_cal_words[16];
static uint32_t s_previous_runtime[APP_DIAG_MAX_TASKS];
static uint32_t s_previous_total;
static uint32_t s_snapshot_ms;
static uint32_t s_last_send_ms;
static uint32_t s_queue_dropped, s_heart_tx_ok, s_heart_tx_failed;
static uint32_t s_tx_ok, s_tx_failed;
static uint32_t s_lvgl_peak_used;
static uint8_t s_tx_frame[78]; /* USART1中断发送期间保持有效。 */
static uint16_t s_sequence;
static uint8_t s_task_count, s_row, s_enabled = APP_DEBUG_TELEMETRY;
static uint8_t s_snapshot_ready;
static uint8_t s_tx_pending;

/* 把TIM2配成100us一格、ARR拉满的自由运行计数器（TIM2为32位，约5天才回绕，统计只用无符号差值），
 * 并把传感器/启动字段初值置为UINT32_MAX(未知)，避免开机早期把"没记录"读成"记录了0"。 */
void app_diag_clock_init(void)
{
    /* APB1当前频率(Hz)，作为TIM2计数时钟的基准。 */
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq();
    /* 下面两张表初始化时共用的循环下标。 */
    uint8_t i;
    /* PPRE1非0说明APB1做了分频，定时器时钟需要按F4规则补偿。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
    {
        /* APB1分频后，TIM2时钟为PCLK1的2倍。 */
        timer_hz *= 2;
    }
    /* 打开TIM2时钟，之后才能读写它的寄存器。 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* 先停计数器并清控制位，便于安全配置。 */
    TIM2->CR1 = 0;
    /* 分频到10kHz：一个计数代表100us。 */
    TIM2->PSC = timer_hz / 10000 - 1;
    /* 自动重装值拉满，让32位计数器自由运行到回绕。 */
    TIM2->ARR = UINT32_MAX;
    /* 产生一次更新事件，把PSC/ARR立即装入影子寄存器。 */
    TIM2->EGR = TIM_EGR_UG;
    /* 清更新标志，避免刚配置完就留下待处理事件。 */
    TIM2->SR = 0;
    /* 计数从0开始，后续统计只用无符号差值。 */
    TIM2->CNT = 0;
    /* 使能计数，TIM2开始以100us一格自由运行。 */
    TIM2->CR1 = TIM_CR1_CEN;
    /* 逐个传感器把初始值与最近结果置为未知。 */
    for (i = 0; i < APP_DIAG_SENSOR_COUNT; ++i)
    {
        /* 尚未初始化，init_result保持UINT32_MAX表示未知。 */
        s_sensors[i].init_result = UINT32_MAX;
        /* 尚无读取结果，result同样标为未知。 */
        s_sensors[i].result = UINT32_MAX;
    }
    /* 启动阶段字段先全部标为未知。 */
    for (i = 0; i < APP_BOOT_FIELD_COUNT; ++i)
    {
        /* UINT32_MAX表示该项没有记录。 */
        s_boot_values[i] = UINT32_MAX;
    }
    /* 开机尚未校准，校准状态按0上报。 */
    s_boot_values[APP_BOOT_CAL_STATE] = 0;
    /* 已累计校准帧数从0开始。 */
    s_boot_values[APP_BOOT_CAL_FRAMES] = 0;
    /* 满足静止条件的帧数也从0开始。 */
    s_boot_values[APP_BOOT_CAL_STABLE_FRAMES] = 0;
}

/* 启动阶段字段（类型4遥测）: LVGL与sensor任务都会写；每次调用只保护单字段更新，
 * 多次调用之间不是原子事务，快照可能跨在两次写入之间。 */
void app_diag_boot_set(uint8_t field, uint32_t value)
{
    /* 进入临界区，避免读到写了一半的单字段。 */
    taskENTER_CRITICAL();
    /* field由调用方按APP_BOOT_*枚举传入，只更新这一项。 */
    s_boot_values[field] = value;
    /* 退出临界区，恢复中断/调度。 */
    taskEXIT_CRITICAL();
}

uint32_t app_diag_clock_count(void)
{
    /* 直接返回TIM2计数值，单位100us。 */
    return TIM2->CNT;
}

/* 校准系数落盘进度（类型5遥测）: sensor任务在写EEPROM的各个阶段更新，UINT32_MAX表示该项未知。 */
void app_diag_imu_save_record(uint32_t stage, uint32_t error, uint32_t address, uint32_t elapsed_ms)
{
    /* 四个字段一起更新，防止快照取到新旧混合值。 */
    taskENTER_CRITICAL();
    /* 保存阶段编号，含义由app_imu_store的保存流程定义。 */
    s_imu_save_values[0] = stage;
    /* 保存该阶段的错误码，0通常表示无错误。 */
    s_imu_save_values[1] = error;
    /* 保存本次涉及的EEPROM地址，UINT32_MAX表示不适用。 */
    s_imu_save_values[2] = address;
    /* 保存从保存开始到当前的HAL毫秒耗时。 */
    s_imu_save_values[3] = elapsed_ms;
    /* 四项写完再退出临界区。 */
    taskEXIT_CRITICAL();
}

/* 零偏校准遥测（类型6）: 整份拷贝驱动维护的16个校准诊断字段（含义见 mpu6050.h 的 MPU_CAL_DIAG_*），
 * 只在sensor任务里被调用。 */
void app_diag_mpu_cal_record(const uint32_t *values)
{
    /* 拷贝与快照读取共用临界区，保证16个字来自同一份驱动状态。 */
    taskENTER_CRITICAL();
    /* 按数组大小整体复制，避免逐字段遗漏。 */
    memcpy(s_mpu_cal_values, values, sizeof(s_mpu_cal_values));
    /* 复制完成后再放开临界区。 */
    taskEXIT_CRITICAL();
}

/* 任务循环首尾配对调用。begin 用"本轮起点-上一轮起点"统计实际循环周期(us)，
 * 其中包含上一轮工作与其后的等待，不只是等待时长；首次调用只建立起点。
 * end 用"终点-起点"统计工作时长，超过目标周期计入overruns。
 * 两者都以100us计数乘100换算成微秒。 */
void app_diag_task_begin(uint8_t id)
{
    /* 取本任务在统计表里的条目。 */
    app_diag_task_struct *stats = &s_tasks[id];
    /* 记录本轮开始时刻，单位100us。 */
    uint32_t now = app_diag_clock_count();
    /* 用于暂存换算后的循环周期，单位us。 */
    uint32_t period;
    /* 周期与起点要成对更新，放进临界区。 */
    taskENTER_CRITICAL();
    /* 只有已经有过上一轮起点时才能算出周期。 */
    if (stats->have_start != 0)
    {
        /* 两次起点的100us计数差乘以100换算成us，无符号减法可跨回绕。 */
        period = (now - stats->started) * 100;
        /* 保存最近一次循环周期。 */
        stats->period_last = period;
        /* 首次统计时period_min还是0，直接采用本次；之后取更小值。 */
        if (stats->period_min == 0 || period < stats->period_min)
        {
            /* 更新历史最小周期。 */
            stats->period_min = period;
        }
        /* 比历史最大值还大则刷新最大值。 */
        if (period > stats->period_max)
        {
            /* 更新历史最大周期。 */
            stats->period_max = period;
        }
    }
    /* 本轮起点留作下一轮算周期的基准。 */
    stats->started = now;
    /* 标记已有起点，下一轮begin不再只建立基准。 */
    stats->have_start = 1;
    /* 结束这次成对更新。 */
    taskEXIT_CRITICAL();
}

void app_diag_task_end(uint8_t id, uint32_t target_ms)
{
    /* 取本任务在统计表里的条目。 */
    app_diag_task_struct *stats = &s_tasks[id];
    /* 从本轮begin到现在的100us计数差，换算成us即工作时长。 */
    uint32_t work = (app_diag_clock_count() - stats->started) * 100;
    /* 工作时长、最大值、超预算和循环次数一起更新。 */
    taskENTER_CRITICAL();
    /* 保存最近一次处理耗时，单位us。 */
    stats->work_last = work;
    /* 记录调用方给出的目标周期，单位ms，供遥测和超预算判断。 */
    stats->target_ms = target_ms;
    /* 刷新历史最大处理耗时。 */
    if (work > stats->work_max)
    {
        /* 本次更慢，更新最大值。 */
        stats->work_max = work;
    }
    /* 目标周期由ms换成us后再与工作耗时比较。 */
    if (work > target_ms * 1000)
    {
        /* 超出本轮预算，累计一次超预算。 */
        ++stats->overruns;
    }
    /* 一次完整的begin/end算一个循环。 */
    ++stats->loops;
    /* 更新完毕，退出临界区。 */
    taskEXIT_CRITICAL();
}

void app_diag_sensor_init(uint8_t id, uint32_t result)
{
    /* 初始化结果与快照读取互斥。 */
    taskENTER_CRITICAL();
    /* 原样保存驱动初始化返回值，由上层解释。 */
    s_sensors[id].init_result = result;
    /* 写完后退出临界区。 */
    taskEXIT_CRITICAL();
}

/* 恢复状态机的一拍上报：刚进入"恢复中"(2)算一次尝试；从恢复中离开时把init_result
 * 改写为离开状态，便于区分"曾经初始化失败"与"仍在失败冷却"。 */
void app_diag_sensor_recovery(uint8_t id, uint8_t state)
{
    /* 取该传感器的统计条目。 */
    app_diag_sensor_struct *stats = &s_sensors[id];
    /* 状态比较与计数更新放在同一临界区。 */
    taskENTER_CRITICAL();
    /* 本次进入恢复中而上一次不是恢复中，算一次新的恢复尝试。 */
    if (state == 2 && stats->recovery_state != 2)
    {
        /* 累计恢复尝试次数。 */
        ++stats->recovery_attempts;
    }
    /* 离开恢复中时，用离开状态覆盖初始化结果。 */
    if (stats->recovery_state == 2 && state != 2)
    {
        /* state可能是就绪0或失败冷却1，便于区分两种结局。 */
        stats->init_result = state;
    }
    /* 保存本次状态，供下一拍比较。 */
    stats->recovery_state = state;
    /* 退出临界区。 */
    taskEXIT_CRITICAL();
}

/* 传感器调用点每笔事务上报一次: outcome 区分成功/未就绪/失败，result 原样保留驱动返回值
 * (本层按outcome分类，不解析驱动返回值含义，当前所有成功码都是0)；同时记录事务耗时和两次成功之间的间隔。 */
void app_diag_sensor_record(uint8_t id, uint8_t outcome, uint32_t result, uint32_t started)
{
    /* 取该传感器的统计条目。 */
    app_diag_sensor_struct *stats = &s_sensors[id];
    /* 当前HAL毫秒时刻，用于计算两次成功之间的间隔。 */
    uint32_t now_ms = HAL_GetTick();
    /* 从事务起点到现在的100us计数差，换算成us。 */
    uint32_t work_us = (app_diag_clock_count() - started) * 100;
    /* 结果、耗时和三类计数一起更新。 */
    taskENTER_CRITICAL();
    /* 原样保存驱动返回值；本层按outcome分类，不解析返回值含义。 */
    stats->result = result;
    /* 保存本次事务耗时，单位us。 */
    stats->work_last_us = work_us;
    /* 刷新历史最大事务耗时。 */
    if (work_us > stats->work_max_us)
    {
        /* 本次更慢，更新最大值。 */
        stats->work_max_us = work_us;
    }
    /* 按调用方给出的结果分类统计。 */
    if (outcome == APP_DIAG_OK)
    {
        /* 之前已有成功记录时，才计算成功间隔。 */
        if (stats->ok != 0)
        {
            /* 当前时刻减去上次成功时刻，得到本次间隔。 */
            stats->gap_last_ms = now_ms - stats->good_ms;
            /* 比历史最大间隔还大则刷新。 */
            if (stats->gap_last_ms > stats->gap_max_ms)
            {
                /* 更新最大成功间隔。 */
                stats->gap_max_ms = stats->gap_last_ms;
            }
        }
        /* 记录本次成功时刻，作为下一次间隔的起点。 */
        stats->good_ms = now_ms;
        /* 累计成功次数。 */
        ++stats->ok;
    }
    /* 未就绪单独计数，不算通信失败。 */
    else if (outcome == APP_DIAG_NOT_READY)
    {
        /* 累计未就绪次数。 */
        ++stats->not_ready;
    }
    /* 其余outcome归为错误。 */
    else
    {
        /* 累计错误次数。 */
        ++stats->errors;
    }
    /* 退出临界区。 */
    taskEXIT_CRITICAL();
}

/* 心率样本队列溢出计数: sensor任务写入、LVGL任务读取，累加与读取都在临界区内。 */
void app_diag_heart_queue(uint8_t dropped)
{
    /* 累加与快照读取共用临界区。 */
    taskENTER_CRITICAL();
    /* 把本次丢弃的样本数累加到总计数。 */
    s_queue_dropped += dropped;
    /* 累加完成后退出临界区。 */
    taskEXIT_CRITICAL();
}

/* 波形发送帧数累计: 只在LVGL任务里被调用，与其他写入方无并发，因此不加临界区。 */
void app_diag_heart_tx(uint16_t count, uint8_t success)
{
    /* 非0表示这批样本发送成功。 */
    if (success != 0)
    {
        /* 按这批的样本数量累加成功计数。 */
        s_heart_tx_ok += count;
    }
    /* 否则计为失败。 */
    else
    {
        /* 按这批的样本数量累加失败计数。 */
        s_heart_tx_failed += count;
    }
}

void app_diag_heart_fault(uint8_t kind, uint8_t reg, uint8_t value)
{
    /* 结果字段与快照读取互斥。 */
    taskENTER_CRITICAL();
    /* 低字节固定为2(ERROR)，其余字节依次放故障类型、寄存器号和读回值。 */
    s_sensors[APP_DIAG_HEART].result = 2 | ((uint32_t)kind << 8) | ((uint32_t)reg << 16) | ((uint32_t)value << 24);
    /* 写完后退出临界区。 */
    taskEXIT_CRITICAL();
}

static uint8_t s_imu_cal_requested;
static char s_rx_command[16];
static uint8_t s_rx_length, s_rx_overflow;

/* 丢弃半行残留并清除溢出标记；蓝牙断电或检测到丢字节时由LVGL调用。 */
void app_diag_reset_receive(void)
{
    /* 长度清零等于丢弃尚未遇到行尾的半行内容。 */
    s_rx_length = 0;
    /* 清溢出标记，下一行可以重新参与匹配。 */
    s_rx_overflow = 0;
}

/* 取出并清零校准请求：返回1表示本轮需要重新做零偏校准（sensor任务上下文调用）。 */
uint8_t app_diag_take_imu_calibration(void)
{
    /* 保存取出前的请求标志，作为返回值。 */
    uint8_t requested;
    /* 读取与清零必须成对完成，否则同一请求可能被消费两次或丢失。 */
    taskENTER_CRITICAL();
    /* 先读走当前请求。 */
    requested = s_imu_cal_requested;
    /* 立即清零，本次调用之后不再重复触发。 */
    s_imu_cal_requested = 0;
    /* 退出临界区。 */
    taskEXIT_CRITICAL();
    /* 返回1表示需要校准，0表示本轮没有请求。 */
    return requested;
}

/* LVGL把串口字节按行喂进来，支持 DIAG,0 / DIAG,1 / IMU,CAL，行尾 CR 或 LF 都算结束：
 * DIAG,1 会重置轮内行号(s_row)并强制重新采样；IMU,CAL 只置请求标志，校准由sensor任务消费后执行。 */
void app_diag_receive(const uint8_t *data, uint16_t size)
{
    /* 逐个字节处理本次收到的数据。 */
    uint16_t i;
    /* 按接收顺序扫描，不假设整行一次到达。 */
    for (i = 0; i < size; ++i)
    {
        /* CR或LF都作为行结束符。 */
        if (data[i] == '\n' || data[i] == '\r')
        {
            /* 在已收字符后补字符串结束符，便于strcmp比较。 */
            s_rx_command[s_rx_length] = '\0';
            /* 未溢出且整行等于DIAG,0时关闭诊断发送。 */
            if (s_rx_overflow == 0 && strcmp(s_rx_command, "DIAG,0") == 0)
            {
                /* 停止后续WDG1帧发送，在途结果仍由service收取。 */
                s_enabled = 0;
            }
            /* 未溢出且整行等于DIAG,1时开启诊断发送。 */
            else if (s_rx_overflow == 0 && strcmp(s_rx_command, "DIAG,1") == 0)
            {
                /* 允许service发送WDG1帧。 */
                s_enabled = 1;
                /* 从本轮第一行重新开始发送。 */
                s_row = 0;
                /* 丢弃旧快照，下一帧前重新采样。 */
                s_snapshot_ready = 0;
            }
            /* 未溢出且整行等于IMU,CAL时只登记校准请求。 */
            else if (s_rx_overflow == 0 && strcmp(s_rx_command, "IMU,CAL") == 0)
            {
                /* 标志由sensor任务消费，这里不直接执行校准。 */
                taskENTER_CRITICAL();
                /* 置位请求，等待app_diag_take_imu_calibration取走。 */
                s_imu_cal_requested = 1;
                /* 退出临界区。 */
                taskEXIT_CRITICAL();
            }
            /* 无论命令是否匹配，行尾都丢弃缓存内容并清溢出标记。 */
            app_diag_reset_receive();
        }
        /* 非行尾且缓存还有空位时继续收字符。 */
        else if (s_rx_length < sizeof(s_rx_command) - 1)
        {
            /* 存入一个字节并后移长度，最后一位留给'\0'。 */
            s_rx_command[s_rx_length++] = (char)data[i];
        }
        /* 缓存已满仍没遇到行尾。 */
        else
        {
            /* 超过15字符记溢出，整行到行尾一起丢弃，不做部分匹配。 */
            s_rx_overflow = 1;
        }
    }
}

static uint8_t diag_task_id(const char *name)
{
    /* LVGL任务固定映射到ID0。 */
    if (strcmp(name, "lvgl") == 0)
    {
        /* 返回任务表中的第0项。 */
        return 0;
    }
    /* 传感器任务固定映射到ID1。 */
    if (strcmp(name, "sensor") == 0)
    {
        /* 返回任务表中的第1项。 */
        return 1;
    }
    /* 空闲任务使用枚举里的IDLE编号。 */
    if (strcmp(name, "IDLE") == 0)
    {
        /* 返回APP_DIAG_IDLE。 */
        return APP_DIAG_IDLE;
    }
    /* 触摸任务使用枚举里的TOUCH编号。 */
    if (strcmp(name, "touch") == 0)
    {
        /* 返回APP_DIAG_TOUCH。 */
        return APP_DIAG_TOUCH;
    }
    /* 名字不在已知列表内，返回越界值表示未识别。 */
    return APP_DIAG_TASK_COUNT;
}

/* 整轮遥测的唯一采样点：任务表来自 uxTaskGetSystemState（优先级/状态/栈高水位/运行时长），
 * 运行时长占比用与上次快照的增量除以总增量，因此读的是"这段时间内"的分布而非开机平均；
 * 名字匹配不到的任务仍保留内核给出的状态、优先级、栈高水位与CPU占比，只跳过业务循环统计
 * (loops/周期/工作时长等字段保持0)；它们的总栈大小字段为0，运行时长增量与其它未识别
 * 任务共用一个历史槽，占比只能作参考。传感器/系统/启动字段各自按段在临界区内拷贝，
 * 各段之间仍是分别采样，不是全局原子快照。 */
static void diag_snapshot(void)
{
    /* total为运行时间统计计数器当前总值，delta为与上次快照的增量；runtime为单任务增量；words指向当前任务行的字段区。 */
    uint32_t total, delta, runtime, *words;
    /* i遍历内核返回的任务/传感器，id为任务名映射出的业务ID。 */
    uint8_t i, id;
    /* 在临界区内取出的任务统计副本。 */
    app_diag_task_struct task_stats;
    /* 在临界区内取出的传感器统计副本。 */
    app_diag_sensor_struct sensor_stats;
    /* LVGL内存池监视结果。 */
    lv_mem_monitor_t memory;
    /* 向内核查询任务表，返回实际填入的任务数并给出总运行计数。 */
    s_task_count = (uint8_t)uxTaskGetSystemState(s_task_status, APP_DIAG_MAX_TASKS, &total);
    /* 本次与上次快照之间的总运行计数增量。 */
    delta = total - s_previous_total;
    /* 保存本次总值，供下一次计算增量。 */
    s_previous_total = total;
    /* 记录本组帧共用的快照时刻，单位HAL毫秒。 */
    s_snapshot_ms = HAL_GetTick();
    /* 逐个处理内核返回的任务。 */
    for (i = 0; i < s_task_count; ++i)
    {
        /* 按任务名映射业务ID，未知名字得到APP_DIAG_TASK_COUNT。 */
        id = diag_task_id(s_task_status[i].pcTaskName);
        /* 保存本行对应的业务ID，发送时写入帧的ID字节。 */
        s_task_ids[i] = id;
        /* 指向本任务固定的14字输出区。 */
        words = s_task_words[i];
        /* 先清空上一轮快照，未赋值字段保持0。 */
        memset(words, 0, sizeof(s_task_words[i]));
        /* 字段0为当前优先级。 */
        words[0] = s_task_status[i].uxCurrentPriority;
        /* 字段1为当前任务状态(就绪/阻塞等枚举值)。 */
        words[1] = s_task_status[i].eCurrentState;
        /* lvgl的栈总量按创建时的1024字换算成字节。 */
        if (id == 0)
        {
            /* 字数乘StackType_t字节数得到字节数。 */
            words[2] = 1024 * sizeof(StackType_t);
        }
        /* sensor任务的栈总量按512字换算。 */
        else if (id == 1)
        {
            /* 512字乘每字字节数。 */
            words[2] = 512 * sizeof(StackType_t);
        }
        /* IDLE按内核最小栈配置换算。 */
        else if (id == 2)
        {
            /* configMINIMAL_STACK_SIZE以字为单位，换算成字节。 */
            words[2] = configMINIMAL_STACK_SIZE * sizeof(StackType_t);
        }
        /* touch不在上述ID里，用名字再判断一次。 */
        else if (strcmp(s_task_status[i].pcTaskName, "touch") == 0)
        {
            /* 按触摸任务创建时的字数换算成字节。 */
            words[2] = APP_TOUCH_STACK_WORDS * sizeof(StackType_t);
        }
        /* 字段3为历史最小剩余栈，高水位以字计，换算成字节。 */
        words[3] = s_task_status[i].usStackHighWaterMark * sizeof(StackType_t);
        /* 本任务运行计数减去上次保存值，得到这段时间的增量。 */
        runtime = s_task_status[i].ulRunTimeCounter - s_previous_runtime[id];
        /* 保存本次运行计数，供下次求增量。 */
        s_previous_runtime[id] = s_task_status[i].ulRunTimeCounter;
        /* 总增量非0才能算百分比，避免除零。 */
        if (delta != 0)
        {
            /* 用64位中间量乘1000再除，结果是以千分之一为单位的占比。 */
            words[4] = (uint32_t)((uint64_t)runtime * 1000 / delta);
        }
        /* 已识别且不是IDLE的任务才有业务循环统计。 */
        if (id < APP_DIAG_TASK_COUNT && id != APP_DIAG_IDLE)
        {
            /* 临界区内整份复制任务统计，避免逐字段取到不一致值。 */
            taskENTER_CRITICAL();
            /* 取一致快照。 */
            task_stats = s_tasks[id];
            /* 复制完退出临界区。 */
            taskEXIT_CRITICAL();
            /* 字段5为累计循环次数。 */
            words[5] = task_stats.loops;
            /* 字段6为最近循环周期，单位us。 */
            words[6] = task_stats.period_last;
            /* 字段7为历史最小周期，单位us。 */
            words[7] = task_stats.period_min;
            /* 字段8为历史最大周期，单位us。 */
            words[8] = task_stats.period_max;
            /* 字段9为最近处理耗时，单位us。 */
            words[9] = task_stats.work_last;
            /* 字段10为历史最大处理耗时，单位us。 */
            words[10] = task_stats.work_max;
            /* 字段11为超出目标周期的累计次数。 */
            words[11] = task_stats.overruns;
            /* 字段12为调用方给出的目标周期，单位ms。 */
            words[12] = task_stats.target_ms;
            /* 字段13标记是否已建立过begin起点。 */
            words[13] = task_stats.have_start;
        }
    }
    /* 逐个传感器生成10字快照。 */
    for (i = 0; i < APP_DIAG_SENSOR_COUNT; ++i)
    {
        /* 临界区内整份复制，保证各计数来自同一时刻附近。 */
        taskENTER_CRITICAL();
        /* 取一致快照。 */
        sensor_stats = s_sensors[i];
        /* 复制完退出临界区。 */
        taskEXIT_CRITICAL();
        /* 指向该传感器的固定输出区。 */
        words = s_sensor_words[i];
        /* 字段0为成功读取次数。 */
        words[0] = sensor_stats.ok;
        /* 字段1为未就绪次数。 */
        words[1] = sensor_stats.not_ready;
        /* 字段2为错误次数。 */
        words[2] = sensor_stats.errors;
        /* 字段3为驱动最近一次返回值。 */
        words[3] = sensor_stats.result;
        /* 字段4默认未知，下面有成功记录时再算数据年龄。 */
        words[4] = UINT32_MAX;
        /* 只有成功过才有上次成功时刻可减。 */
        if (sensor_stats.ok != 0)
        {
            /* 快照时刻减去最近成功时刻，得到数据年龄，单位ms。 */
            words[4] = s_snapshot_ms - sensor_stats.good_ms;
        }
        /* 字段5为最近两次成功之间的间隔，单位ms。 */
        words[5] = sensor_stats.gap_last_ms;
        /* 字段6为历史最大成功间隔，单位ms。 */
        words[6] = sensor_stats.gap_max_ms;
        /* 字段7为最近一次事务耗时，单位us。 */
        words[7] = sensor_stats.work_last_us;
        /* 字段8为历史最大事务耗时，单位us。 */
        words[8] = sensor_stats.work_max_us;
        /* 字段9为驱动初始化返回值。 */
        words[9] = sensor_stats.init_result;
    }
    /* 读取LVGL内存池当前总量与剩余量。 */
    lv_mem_monitor(&memory);
    /* 已用字节数超过历史峰值时更新峰值。 */
    if (memory.total_size - memory.free_size > s_lvgl_peak_used)
    {
        /* 记录新的LVGL已用峰值。 */
        s_lvgl_peak_used = memory.total_size - memory.free_size;
    }
    /* 系统字段0为FreeRTOS堆当前剩余字节。 */
    s_system_words[0] = xPortGetFreeHeapSize();
    /* 系统字段1为FreeRTOS堆历史最小剩余字节。 */
    s_system_words[1] = xPortGetMinimumEverFreeHeapSize();
    /* 系统字段2为LVGL池当前剩余字节。 */
    s_system_words[2] = memory.free_size;
    /* 系统字段3为快照观察到的LVGL最大已用字节。 */
    s_system_words[3] = s_lvgl_peak_used;
    /* 系统字段4为蓝牙接收丢弃字节计数。 */
    s_system_words[4] = kt6368a_get_rx_dropped();
    /* 系统字段5为蓝牙接收错误计数，按驱动记录的口径原样上报。 */
    s_system_words[5] = kt6368a_get_rx_errors();
    /* 队列丢弃计数与写入方共用临界区。 */
    taskENTER_CRITICAL();
    /* 系统字段6为心率队列溢出累计次数。 */
    s_system_words[6] = s_queue_dropped;
    /* 读完退出临界区。 */
    taskEXIT_CRITICAL();
    /* 系统字段7为波形发送成功的样本数。 */
    s_system_words[7] = s_heart_tx_ok;
    /* 系统字段8为波形发送失败的样本数。 */
    s_system_words[8] = s_heart_tx_failed;
    /* 系统字段9为WDG1帧发送成功计数。 */
    s_system_words[9] = s_tx_ok;
    /* 系统字段10为WDG1帧发送失败计数。 */
    s_system_words[10] = s_tx_failed;
    /* 系统字段11为本轮快照的任务数。 */
    s_system_words[11] = s_task_count;
    /* 系统字段12固定填100，作为协议中的固定数值上报。 */
    s_system_words[12] = 100;
    /* 系统字段13填APP_DIAG_FRAME_MS，即实际限速间隔。 */
    s_system_words[13] = APP_DIAG_FRAME_MS;
    /* 系统字段14为诊断发送开关当前值。 */
    s_system_words[14] = s_enabled;
    /* 系统字段15为运行时间统计计数器当前总值。 */
    s_system_words[15] = total;
    /* 启动/EEPROM/校准三组字段在同一个临界区内复制。 */
    taskENTER_CRITICAL();
    /* 复制启动阶段字段。 */
    memcpy(s_boot_words, s_boot_values, sizeof(s_boot_words));
    /* 复制IMU保存进度字段。 */
    memcpy(s_imu_save_words, s_imu_save_values, sizeof(s_imu_save_words));
    /* 复制MPU校准诊断字段。 */
    memcpy(s_mpu_cal_words, s_mpu_cal_values, sizeof(s_mpu_cal_words));
    /* 三组都复制完再退出临界区。 */
    taskEXIT_CRITICAL();
}

/* CRC-16/CCITT-FALSE(初值0xFFFF、多项式0x1021、不反转)，覆盖帧头到数据末尾，主机按同法校验。 */
static uint16_t diag_crc16(const uint8_t *data, uint16_t size)
{
    /* CRC初值固定为0xFFFF。 */
    uint16_t crc = 0xFFFF;
    /* i为待校验字节下标。 */
    uint16_t i;
    /* bit为当前字节内逐位处理的位号。 */
    uint8_t bit;
    /* 逐字节累加。 */
    for (i = 0; i < size; ++i)
    {
        /* 把当前字节移到高8位后异或进CRC。 */
        crc ^= (uint16_t)data[i] << 8;
        /* 每个字节处理8位。 */
        for (bit = 0; bit < 8; ++bit)
        {
            /* 最高位为1时走多项式异或分支。 */
            if ((crc & 0x8000) != 0)
            {
                /* 左移一位后异或0x1021。 */
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            /* 最高位为0时只左移。 */
            else
            {
                /* 左移一位，低位补0。 */
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    /* 返回16位校验值，不做最终异或。 */
    return crc;
}

static void diag_put_u32(uint8_t *data, uint32_t value)
{
    /* 最低字节放在最前，即小端序。 */
    data[0] = (uint8_t)value;
    /* 次低字节放在偏移1。 */
    data[1] = (uint8_t)(value >> 8);
    /* 次高字节放在偏移2。 */
    data[2] = (uint8_t)(value >> 16);
    /* 最高字节放在偏移3。 */
    data[3] = (uint8_t)(value >> 24);
}

/* 仅由LVGL任务调用，每100ms最多发一行。帧格式: "WDG1" + 行类型 + 对象ID + 16位序号 +
 * 快照时刻 + 该行的若干32位字 + CRC16；行类型1=任务、2=传感器、3=系统、4=启动、
 * 5=IMU保存、6=MPU校准。frame[5]是该行对象在本类表中的ID（任务/传感器下标），
 * 不是全局行号；全局行号由s_row推进。整轮行号推进完后才回到第0行并重新采样；启动异步发送
 * 返回BUSY时保留行号和序号，下一拍重试同一行，HAL_OK则等回收结果后再计成败。 */
void app_diag_service(void)
{
    /* 固定帧缓冲，异步发送期间必须保持内容不变。 */
    uint8_t *frame = s_tx_frame;
    /* 本次调用开始的HAL毫秒时刻，用于限速和记录发送时刻。 */
    uint32_t now = HAL_GetTick();
    /* 保存异步发送的启动结果或取回的完成结果。 */
    HAL_StatusTypeDef status;
    /* 指向本行要打包的32位字段数组。 */
    const uint32_t *words;
    /* crc为本帧校验值，size为帧长(含CRC)。 */
    uint16_t crc, size;
    /* count为本行字段数，i为打包下标，id为帧内对象ID。 */
    uint8_t count, i, id;
    /* 关闭发送后仍收取在途结果，成功计数只在UART完成后更新。 */
    if (s_tx_pending != 0 && kt6368a_take_tx_result(&status) != 0)
    {
        /* 结果已被取走，清除在途标记。 */
        s_tx_pending = 0;
        /* HAL_OK表示UART发送完成，计入成功。 */
        if (status == HAL_OK)
        {
            /* 累计诊断帧发送成功次数。 */
            ++s_tx_ok;
        }
        /* 其他完成状态(HAL_ERROR/TIMEOUT等)都计入失败。 */
        else
        {
            /* 累计诊断帧发送失败次数。 */
            ++s_tx_failed;
        }
    }
    /* 未开启、蓝牙初始化未完成、发送忙或未到100ms间隔时本拍不发送。 */
    if (s_enabled == 0 || kt6368a_init_is_pending() != 0 || kt6368a_tx_is_busy() != 0 ||
        now - s_last_send_ms < APP_DIAG_FRAME_MS)
    {
        /* 直接返回，保留当前行号和快照，等待下一拍。 */
        return;
    }
    /* 新一轮的第一行且没有可用快照时，先整轮采样一次。 */
    if (s_row == 0 && s_snapshot_ready == 0)
    {
        /* 生成任务/传感器/系统/启动等字段。 */
        diag_snapshot();
        /* 标记本组帧已有快照，后续行复用同一份数据。 */
        s_snapshot_ready = 1;
    }
    /* 帧头前4字节固定为ASCII标识WDG1。 */
    memcpy(frame, "WDG1", 4);
    /* 行号落在任务段时按任务行打包。 */
    if (s_row < s_task_count)
    {
        /* 行类型1表示任务统计。 */
        frame[4] = 1;
        /* 对象ID为任务名映射出的业务ID。 */
        id = s_task_ids[s_row];
        /* 取该任务固定14字字段区。 */
        words = s_task_words[s_row];
        /* 任务行固定14个32位字段。 */
        count = 14;
    }
    /* 任务段之后是传感器段。 */
    else if (s_row < s_task_count + APP_DIAG_SENSOR_COUNT)
    {
        /* 行类型2表示传感器统计。 */
        frame[4] = 2;
        /* 用行号减去任务段长度得到传感器下标。 */
        id = s_row - s_task_count;
        /* 取该传感器的固定10字字段区。 */
        words = s_sensor_words[id];
        /* 传感器行固定10个32位字段。 */
        count = 10;
    }
    /* 传感器段之后的一行是系统统计。 */
    else if (s_row == s_task_count + APP_DIAG_SENSOR_COUNT)
    {
        /* 行类型3表示系统统计。 */
        frame[4] = 3;
        /* 系统行没有子对象，ID固定0。 */
        id = 0;
        /* 使用16字系统字段区。 */
        words = s_system_words;
        /* 系统行固定16个32位字段。 */
        count = 16;
    }
    /* 再下一行是启动阶段字段。 */
    else if (s_row == s_task_count + APP_DIAG_SENSOR_COUNT + 1)
    {
        /* 行类型4表示启动阶段记录。 */
        frame[4] = 4;
        /* 启动行ID固定0。 */
        id = 0;
        /* 使用启动字段副本。 */
        words = s_boot_words;
        /* 字段数由APP_BOOT_FIELD_COUNT决定。 */
        count = APP_BOOT_FIELD_COUNT;
    }
    /* 再下一行是IMU校准保存进度。 */
    else if (s_row == s_task_count + APP_DIAG_SENSOR_COUNT + 2)
    {
        /* 行类型5表示EEPROM保存进度。 */
        frame[4] = 5;
        /* 保存进度行ID固定0。 */
        id = 0;
        /* 使用4字保存进度副本。 */
        words = s_imu_save_words;
        /* 该行固定4个32位字段。 */
        count = 4;
    }
    /* 剩余最后一行为MPU校准诊断。 */
    else
    {
        /* 行类型6表示MPU零偏校准判定。 */
        frame[4] = 6;
        /* 校准行ID固定0。 */
        id = 0;
        /* 使用16字校准诊断副本。 */
        words = s_mpu_cal_words;
        /* 校准行固定16个32位字段。 */
        count = 16;
    }
    /* 写入对象ID，注意它不是全局行号。 */
    frame[5] = id;
    /* 序号低字节放在偏移6。 */
    frame[6] = (uint8_t)s_sequence;
    /* 序号高字节放在偏移7，构成16位小端序号。 */
    frame[7] = (uint8_t)(s_sequence >> 8);
    /* 偏移8起写4字节快照时刻，本组各行共用。 */
    diag_put_u32(frame + 8, s_snapshot_ms);
    /* 按字段数逐个写入32位字。 */
    for (i = 0; i < count; ++i)
    {
        /* 每字占4字节，从帧头12字节之后开始。 */
        diag_put_u32(frame + 12 + i * 4, words[i]);
    }
    /* 12字节头加字段区得到CRC之前的长度。 */
    size = 12 + count * 4;
    /* 对帧头到数据末尾计算CRC。 */
    crc = diag_crc16(frame, size);
    /* 追加CRC低字节。 */
    frame[size++] = (uint8_t)crc;
    /* 追加CRC高字节，size此时为完整帧长。 */
    frame[size++] = (uint8_t)(crc >> 8);
    /* 启动异步发送，缓冲区在结果取走前不可改写。 */
    status = kt6368a_write_async(frame, size, 10);
    /* 驱动忙时本次不推进任何状态。 */
    if (status == HAL_BUSY)
    {
        return; /* 保留本行和序号，下轮重试。 */
    }
    /* 启动即失败(非忙)时直接计一次发送失败。 */
    if (status != HAL_OK)
    {
        /* 累计诊断帧发送失败次数。 */
        ++s_tx_failed;
    }
    /* 启动成功，只说明已交给UART，尚不能计成功。 */
    else
    {
        /* 标记有在途发送，下一拍收取完成结果。 */
        s_tx_pending = 1;
    }
    /* 记录本次发送时刻，用于100ms限速。 */
    s_last_send_ms = now;
    /* 帧序号递增，便于主机发现缺口。 */
    ++s_sequence;
    /* 推进到下一行。 */
    ++s_row;
    /* 本轮各行均已尝试发送(非BUSY启动失败也会推进行号)，随后结束本轮。 */
    if (s_row >= s_task_count + APP_DIAG_SENSOR_COUNT + 4)
    {
        /* 回到第一行。 */
        s_row = 0;
        /* 清快照标记，下一轮发送前重新采样。 */
        s_snapshot_ready = 0;
    }
}
