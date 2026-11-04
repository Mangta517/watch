/**
  ******************************************************************************
  * @file    app_task.c
  * @brief   应用层-FreeRTOS 任务管理实现
  *          本工程所有 xTaskCreate / vTaskStartScheduler 只出现在本文件;
  *          业务模块 (app_display 等) 只提供 "在任务上下文里被调用" 的接口。
  *          内核: Middlewares/FreeRTOS V202212.01 (RVDS/ARM_CM4F port, heap_4)。
  ******************************************************************************
***/
#include "app_task.h"
#include "app_diag.h"
#include "app_reset.h"
#include "app_imu_store.h"
#include "app_uptime.h"
#include "app_weather_store.h"
#include "app_battery_log.h"
#include "app_ota.h"

#include "main.h" /* Error_Handler() 声明 */
#include "FreeRTOS.h"
#include "task.h"

#include "app_display.h"
#include "app_imu.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_aht21.h"
#include "app_battery.h"
#include "app_touch.h"
#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "my_i2c.h"
#include "mpu6050.h"
#include "spl006.h"
#include "lsm303dlhc.h"
#include "em7028.h"
#include "kt6368a.h"
#include "watch_rtc.h"
#include "tps3823.h"

/*********************
 *  任务配置
 *********************/
/* LVGL优先级3、sensor优先级2；显示分块主动让出CPU，MPU每轮只取最新一帧。 */
#define APP_LVGL_TASK_PRIO   3    /* configMAX_PRIORITIES=5 */
#define APP_LVGL_TASK_STK    1024 /* 字 = 4KB (Watch 工程实测足够跑 UI+刷新) */
#define APP_LVGL_TASK_PERIOD 10   /* LVGL服务周期10ms，与LV_DISP_DEF_REFR_PERIOD共用同一节拍 */

#define APP_SENSOR_TASK_PRIO                2   /* 低于LVGL，优先响应触摸和显示 */
#define APP_SENSOR_TASK_STK                 512 /* 字 = 2KB (IMU 读 + 零偏校准 + 气压读路径) */
#define APP_SENSOR_TASK_PERIOD              20  /* 姿态和校准均为50Hz。 */
#define APP_SENSOR_HEART_PERIOD_MS          10  /* 服务周期整除30ms心率读取下限，避免量化为40ms。 */
#define APP_SENSOR_IDLE_PERIOD_MS           40  /* 普通亮屏后台服务；加速度另按200ms读取。 */
#define APP_IDLE_POLL_MS                    20
#define APP_POWER_IDLE                      1
#define APP_POWER_MOTION                    2
#define APP_POWER_SCREEN_OFF                4
#define APP_POWER_SPORT                     8
#define APP_SENSOR_SPORT_PERIOD_MS          10
#define APP_SENSOR_SLEEP_WINDOW_MS          1600 /* 等待1Hz气压新转换。 */
#define APP_SENSOR_BACKGROUND_IMU_PERIOD_MS 200 /* 后台5Hz加速度循环，用于计步和辅助抬腕。 */
#define APP_SENSOR_BACKGROUND_IMU_IDLE_MS   1000 /* 静止时的软件读取周期；MPU硬件仍为5Hz。 */
#define APP_SENSOR_SLEEP_POLL_MS            200  /* 与后台5Hz采样合并，保留sensor进度检查。 */
#define APP_SENSOR_SLEEP_IDLE_POLL_MS       1000 /* 静止慢待机时放宽熄屏后台读取与LVGL稳定等待；心率10ms/姿态20ms/环境100ms不变。 */
#define APP_SENSOR_ENVIRONMENT_POLL_MS      100  /* AHT转换/恢复窗口仍在200ms读回期限内推进。 */
#define APP_SENSOR_BARO_PERIOD_MS           300000
#define APP_TOUCH_TASK_PRIO                 1    /* 触摸采集低于sensor；LVGL只消费缓存。 */
#define APP_WATCHDOG_FEED_PERIOD_MS         100
#define APP_WATCHDOG_SENSOR_STARTUP_MS      5000
#define APP_WATCHDOG_RENDER_TIMEOUT_MS      5000
#define APP_WATCHDOG_STOP_WINDOW_MS         200 /* 授权tickless暂停外部看门狗的喂狗新鲜度窗口，为100ms喂狗周期的两倍。 */

/* sensor 单写、LVGL 单读；Cortex-M4 对齐的 32 位访问不会撕裂，不共享复合快照。
 * 该计数只用于判断sensor主循环是否仍在推进，不承载任何业务数据。 */
static volatile uint32_t app_sensor_watchdog_progress;
/* sensor 单写、LVGL 单读：熄屏后台稳定等待节拍，静止慢待机时放宽到1000ms与sensor同步。 */
static volatile uint32_t s_sleep_poll_ms = APP_SENSOR_SLEEP_POLL_MS;
/* UI单写；电源标志打包在对齐32位值中，由各任务一次读取。
 * 该值只表达UI的意图，不代表屏幕/触摸的实际硬件状态。 */
static volatile uint32_t s_power_state;
static TaskHandle_t s_touch_task_handle;
static TaskHandle_t s_lvgl_task_handle;

/* 中断上下文只负责叫醒LVGL任务，不做GPIO消抖或页面操作。 */
void app_task_wake_from_isr(void)
{
    BaseType_t higher_priority_woken = pdFALSE; /* 先置"无需切换"，由通知调用在必要时改写。 */
    /* 任务尚未创建时没有可通知对象，直接跳过本次唤醒。 */
    if (s_lvgl_task_handle != NULL)
    {
        /* 给LVGL任务的通知计数加1，并把"是否唤醒了更高优先级任务"写入标志。 */
        vTaskNotifyGiveFromISR(s_lvgl_task_handle, &higher_priority_woken);
        /* 若刚唤醒的任务优先级高于当前被中断的任务，则在退出中断时立即切换。 */
        portYIELD_FROM_ISR(higher_priority_woken);
    }
}

/* UI任务发布电源标志：四个入参非0即置位，整体覆盖上一次状态。
 * 熄屏→亮屏边沿额外通知touch任务，通知可先于其阻塞到达，避免快速按键唤醒丢失。 */
void app_task_set_power_state(uint8_t idle, uint8_t motion, uint8_t screen_off, uint8_t sport)
{
    uint32_t state = 0; /* 本轮重新拼装的状态字，最终整体覆盖s_power_state。 */
    uint32_t previous_state = s_power_state; /* 先保存旧值，用于后面判断熄屏→亮屏的边沿。 */
    /* 待机请求非0时置APP_POWER_IDLE位，否则该位保持0。 */
    if (idle != 0)
    {
        state |= APP_POWER_IDLE;
    }
    /* 姿态页请求非0时置APP_POWER_MOTION位。 */
    if (motion != 0)
    {
        state |= APP_POWER_MOTION;
    }
    /* 熄屏请求非0时置APP_POWER_SCREEN_OFF位。 */
    if (screen_off != 0)
    {
        state |= APP_POWER_SCREEN_OFF;
    }
    /* 运动模式请求非0时置APP_POWER_SPORT位。 */
    if (sport != 0)
    {
        state |= APP_POWER_SPORT;
    }
    /* 四个位拼好后一次性写回，任务侧读到的始终是完整状态字。 */
    s_power_state = state;
    /* 旧值处于熄屏、新值已不熄屏，说明刚发生亮屏；touch任务已创建时补发一次通知。 */
    if ((previous_state & APP_POWER_SCREEN_OFF) != 0 && (state & APP_POWER_SCREEN_OFF) == 0 && s_touch_task_handle != NULL)
    {
        xTaskNotifyGive(s_touch_task_handle); /* 通知可先于阻塞到达，避免丢失快速按键唤醒。 */
    }
}

/* 返回1表示UI已发布全熄屏；仅供STOP资格和任务节拍判断，
 * 不反映LCD/触摸器件是否真的进入休眠。 */
uint8_t app_task_is_screen_off(void)
{
    /* 取出状态字中的熄屏位：非0返回1(UI已请求全熄屏)，为0返回0。 */
    return (s_power_state & APP_POWER_SCREEN_OFF) != 0;
}

/* 返回sensor最近发布的熄屏等待毫秒(200或1000)，LVGL用作通知等待超时。 */
uint32_t app_task_sleep_poll_ms(void)
{
    /* 原样返回sensor最近发布的等待毫秒数(200或1000)，不做换算。 */
    return s_sleep_poll_ms;
}

/* 看门狗状态：started_ms用于sensor启动宽限，render_started_ms记录本轮渲染起点，
 * last_feed_ms/last_sensor_progress保存上次喂狗时的时刻与sensor进度快照。 */
static struct
{
    uint32_t started_ms;
    uint32_t render_started_ms;
    uint32_t last_feed_ms;
    uint32_t last_sensor_progress;
    uint8_t sensor_started;
} app_watchdog;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void app_lvgl_task(void *pvParameters);
static void app_sensor_task(void *pvParameters);
static void app_touch_task(void *pvParameters);
static void app_lvgl_wait(uint32_t period_ms, uint8_t rtc_finished);
static void app_bluetooth_rx_from_isr(void);
static void app_watchdog_begin(void);
static void app_watchdog_service(void);
static void app_watchdog_render_progress(void);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
/* 创建LVGL显示任务前先初始化外部看门狗芯片；此处只建一个任务，
 * sensor/touch在首帧提交后由LVGL任务创建。 */
void App_Task_Create(void)
{
    /* 先把外部看门狗置为禁用状态；LVGL初始化显示后由app_watchdog_begin启用。 */
    tps3823_init();
    /* 创建LVGL任务：栈1024字、优先级3，句柄存入s_lvgl_task_handle供ISR唤醒。 */
    if (xTaskCreate(app_lvgl_task, "lvgl", APP_LVGL_TASK_STK, NULL, APP_LVGL_TASK_PRIO, &s_lvgl_task_handle) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }
}

/* 启动调度器；正常永不返回，返回说明内核异常，原地死守等看门狗。 */
void App_Task_Start(void)
{
    vTaskStartScheduler(); /* 正常永不返回 */
    /* 调度器异常返回时不再做任何初始化，原地空转等待外部看门狗复位。 */
    for (;;)
    {
    } /* 调度器异常返回时死守 */
}

/* 空闲合并时钟由app_tickless统一处理；喂狗只认LVGL与sensor两个任务的进度。 */

/* 仅返回授权，不喂狗、不改写last_feed_ms：喂狗时刻只由app_watchdog_begin初始化、
   app_watchdog_service在确认新的sensor进度后真实喂狗时更新。 */
uint8_t app_task_watchdog_stop_allowed(void)
{
    /* sensor尚未启动，没有可供判定的真实喂狗进度，拒绝暂停。 */
    if (app_watchdog.sensor_started == 0)
    {
        return 0;
    }
    /* 距最近一次由新sensor进度确认的喂狗超过授权窗口：不续期，拒绝暂停。 */
    if ((uint32_t)(HAL_GetTick() - app_watchdog.last_feed_ms) > APP_WATCHDOG_STOP_WINDOW_MS)
    {
        return 0;
    }
    /* 只有静止慢待机的1000ms节拍才需要跨过TPS3823约900ms的硬件超时。 */
    if (app_task_sleep_poll_ms() != APP_SENSOR_SLEEP_IDLE_POLL_MS)
    {
        return 0;
    }
    return 1;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
/* 启用外部看门狗并把本次起点写入复位记录；此时sensor还没创建，进度判据先置为"未启动"。 */
static void app_watchdog_begin(void)
{
    /* 让外部看门狗开始计时，此后必须周期性喂狗。 */
    tps3823_enable();
    /* 记录本次启动时刻，作为sensor启动宽限5s的起点。 */
    app_watchdog.started_ms = HAL_GetTick();
    /* 喂狗节拍基线同步取同一时刻，避免首轮立即触发100ms判定。 */
    app_watchdog.last_feed_ms = app_watchdog.started_ms;
    /* 把这次喂狗时刻写入复位记录，供复位原因诊断使用。 */
    app_reset_watchdog_feed(app_watchdog.started_ms);
}

/* 喂狗判据：sensor启动前只给5s宽限；启动后若本轮进度与上次相同，只跳过这一次喂狗，
 * 不锁存"永久停喂"——进度恢复前进后照常喂。满足条件时最多每100ms输出一次，
 * 两路调用方（渲染分块与LVGL主循环）共用这套状态。 */
static void app_watchdog_service(void)
{
    uint32_t now_ms = HAL_GetTick(); /* 本轮判定的当前毫秒时刻。 */
    uint32_t progress = app_sensor_watchdog_progress; /* 读一次sensor进度计数快照。 */

    /* sensor还没发布过进度(计数仍为0)时，先走启动宽限分支。 */
    if (app_watchdog.sensor_started == 0)
    {
        /* 计数已非0说明sensor主循环已跑起来，记下启动标志，后续改按进度判据。 */
        if (progress != 0)
        {
            app_watchdog.sensor_started = 1;
        }
        /* 计数仍为0且距启动已满5s：本轮不喂狗，返回等待调用方下次再判。 */
        else if ((uint32_t)(now_ms - app_watchdog.started_ms) >= APP_WATCHDOG_SENSOR_STARTUP_MS)
        {
            return;
        }
    }
    /* sensor已启动但进度与上次喂狗时相同：说明这一拍没有推进，只跳过本次喂狗。 */
    if (app_watchdog.sensor_started != 0 && progress == app_watchdog.last_sensor_progress)
    {
        return;
    }
    /* 距上次喂狗已满100ms才真正输出喂狗脉冲，避免过快重复喂。 */
    if ((uint32_t)(now_ms - app_watchdog.last_feed_ms) >= APP_WATCHDOG_FEED_PERIOD_MS)
    {
        tps3823_feed(); /* 对外部看门狗输出一次喂狗动作。 */
        app_reset_watchdog_feed(now_ms); /* 记录本次喂狗时刻。 */
        app_watchdog.last_sensor_progress = progress; /* 保存本次看到的sensor进度，作为下次比较基准。 */
        app_watchdog.last_feed_ms = now_ms; /* 更新喂狗节拍起点。 */
    }
}

/* 首帧之后的分块渲染回调：渲染仍在5s内才顺带推进喂狗。超时后只是该回调不再喂，
 * 主循环等其它路径仍可能继续喂狗；长时间无任何喂狗才由外部看门狗复位。 */
static void app_watchdog_render_progress(void)
{
    /* 本轮渲染已超过5s：不再由渲染回调喂狗，直接返回。 */
    if ((uint32_t)(HAL_GetTick() - app_watchdog.render_started_ms) >= APP_WATCHDOG_RENDER_TIMEOUT_MS)
    {
        return;
    }
    /* DMA可能已在渲染期间完成；阻塞一拍，让低优先级sensor推进后再检查喂狗。 */
    vTaskDelay(1);
    /* 渲染未超时，顺带执行一次常规喂狗判据。 */
    app_watchdog_service();
}

/* UART接收中断注册给蓝牙驱动：熄屏时LVGL正停在通知等待上，需要立即叫醒它消费字节，
 * 否则数据只能等到下一拍稳定节拍；亮屏时主循环本身在跑，不做任何事。 */
static void app_bluetooth_rx_from_isr(void)
{
    /* 只有熄屏状态下LVGL才停在通知等待上，此时收到UART字节需要立即叫醒它。 */
    if ((s_power_state & APP_POWER_SCREEN_OFF) != 0)
    {
        app_task_wake_from_isr();
    }
}

/* LVGL主循环的等待方式：熄屏且RTC初始化已结束、蓝牙初始化不再在途、PA4未按住时改用通知等待，
 * 由sensor发布的节拍或PA4/UART中断叫醒；其余情况保持固定10ms（待机20ms）轮询刷新UI。 */
static void app_lvgl_wait(uint32_t period_ms, uint8_t rtc_finished)
{
    /* 同时满足熄屏、PA4未按住、RTC初始化已结束、蓝牙初始化不在途，才改走通知等待。 */
    if ((s_power_state & APP_POWER_SCREEN_OFF) != 0 && App_UI_IsWakeKeyActive() == 0 &&
        rtc_finished != 0 && kt6368a_init_is_pending() == 0)
    {
        /* 蓝牙接收队列还有待消费字节：本轮只让出1个tick，下一拍继续处理，避免一次等太久。 */
        if (kt6368a_rx_pending() != 0)
        {
            vTaskDelay(1); /* 本轮256字节预算未清空队列，下一拍继续，仍让sensor运行。 */
        }
        else
        {
            /* PA4/UART立即通知；稳定熄屏由sensor按发布节拍通知，静止慢待机放宽到1000ms同步。 */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(app_task_sleep_poll_ms()));
        }
    }
    else
    {
        /* 亮屏或启动未完成时不做通知等待，按调用方给的周期毫秒换算成tick固定延时。 */
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

/* LVGL任务: 显示初始化、首帧提交、UI刷新，以及RTC/蓝牙的分步启动全部跑在本任务栈上。
 * 首帧完成后才创建sensor/touch；之后每轮依次推进RTC/蓝牙初始化、天气同步、显示服务、
 * 诊断发送与喂狗，最后按亮灭屏选择固定延时或通知等待。 */
static void app_lvgl_task(void *pvParameters)
{
    HAL_StatusTypeDef status; /* 保存本拍RTC/蓝牙初始化服务的返回状态。 */
    uint8_t rtc_finished = 0; /* RTC初始化是否已结束的标志。 */
    uint8_t bt_finished = 0; /* 蓝牙初始化是否已结束的标志。 */
    uint32_t period_ms; /* 本拍LVGL主循环的等待周期(ms)。 */
    (void)pvParameters; /* 未使用任务参数。 */

    /* 硬件拉起 + LVGL 注册 + 建表盘界面, 在本任务上下文一次性完成 */
    App_Display_Init();
    /* 首帧按新分块推进喂狗，卡在同一分块时不喂；总宽限仍限制为5s。 */
    app_watchdog_begin();
    /* 首帧渲染期间把喂狗回调挂到显示驱动，每个分块完成后调用一次。 */
    lv_port_disp_set_progress_callback(app_watchdog_service);
    lv_refr_now(NULL); /* 先提交首帧；DMA完成中断继续工作，不等待其他模块。 */
    /* 首帧之后换成带5s渲染上限的进度回调，避免单次渲染长时间无喂狗。 */
    lv_port_disp_set_progress_callback(app_watchdog_render_progress);
    /* 首帧渲染时间不占用sensor初始化的5s窗口；进入主循环后恢复双任务检查。 */
    app_watchdog.started_ms = HAL_GetTick();
    /* 记录首帧完成时刻到启动诊断。 */
    app_diag_boot_set(APP_BOOT_FIRST_FRAME_MS, HAL_GetTick());

    /* 首帧已提交，创建sensor采样任务(栈512字、优先级2)。 */
    if (xTaskCreate(app_sensor_task, "sensor", APP_SENSOR_TASK_STK, NULL, APP_SENSOR_TASK_PRIO, NULL) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }
    /* 创建touch任务(优先级1)，句柄保存下来供电源状态切换时发通知。 */
    if (xTaskCreate(app_touch_task, "touch", APP_TOUCH_STACK_WORDS, NULL, APP_TOUCH_TASK_PRIO, &s_touch_task_handle) != pdPASS)
    {
        Error_Handler();
    }

    /* 两种启动等待由本任务分阶段推进，sensor可在等待期间独立初始化/采集。 */
    app_diag_boot_set(APP_BOOT_RTC_BEGIN_MS, HAL_GetTick());
    /* 发起RTC初始化第一步，后续在循环里分拍推进。 */
    watch_rtc_init_begin();
    app_diag_boot_set(APP_BOOT_BT_BEGIN_MS, HAL_GetTick());
    /* 注册蓝牙接收中断回调，再把蓝牙初始化启动起来。 */
    kt6368a_set_rx_callback(app_bluetooth_rx_from_isr);
    kt6368a_init_begin();

    for (;;)
    {
        /* 标记LVGL任务本轮诊断开始。 */
        app_diag_task_begin(APP_DIAG_LVGL);
        /* RTC尚未完成时，本拍推进一步初始化。 */
        if (rtc_finished == 0)
        {
            status = watch_rtc_init_service();
            /* 驱动报告不再有待处理步骤：置完成标志并记录结束时刻与结果。 */
            if (watch_rtc_init_is_pending() == 0)
            {
                rtc_finished = 1;
                app_diag_boot_set(APP_BOOT_RTC_END_MS, HAL_GetTick());
                app_diag_boot_set(APP_BOOT_RTC_RESULT, (uint32_t)status);
            }
        }
        /* 蓝牙未完成，或已完成但驱动又报告有待处理步骤时，继续推进初始化。 */
        if (bt_finished == 0 || kt6368a_init_is_pending() != 0)
        {
            status = kt6368a_init_service();
            /* 首次结束且已无待处理步骤：置完成标志并记录结束时刻与结果。 */
            if (bt_finished == 0 && kt6368a_init_is_pending() == 0)
            {
                bt_finished = 1;
                app_diag_boot_set(APP_BOOT_BT_END_MS, HAL_GetTick());
                app_diag_boot_set(APP_BOOT_BT_RESULT, (uint32_t)status);
            }
        }
        /* 记下本轮渲染起点，供渲染分块回调判断5s上限。 */
        app_watchdog.render_started_ms = HAL_GetTick();
        /* 恢复天气缓存并发布新到的手机天气帧，临界区内只复制快照。 */
        app_weather_store_sync();
        App_Display_Service(); /* 锁 → lv_timer_handler → 解锁 */
        /* 处理本拍诊断输出。 */
        app_diag_service();
        /* 默认按10ms周期刷新。 */
        period_ms = APP_LVGL_TASK_PERIOD;
        /* 待机状态下降为20ms轮询，减少无谓刷新。 */
        if ((s_power_state & APP_POWER_IDLE) != 0)
        {
            period_ms = APP_IDLE_POLL_MS;
        }
        /* 记录LVGL本轮诊断结束及实际周期。 */
        app_diag_task_end(APP_DIAG_LVGL, period_ms);
        /* 主循环再走一次喂狗判据。 */
        app_watchdog_service();
        /* 按亮灭屏状态选择固定延时或通知等待。 */
        app_lvgl_wait(period_ms, rtc_finished);
    }
}

/* 触摸任务：独占CST816软件I2C，亮屏10ms、待机20ms轮询；熄屏成功后阻塞等通知，
 * 休眠写失败则按驱动给出的剩余毫秒退避。周期按实际唤醒时刻重排，超期不补跑旧轮询。 */
static void app_touch_task(void *pvParameters)
{
    TickType_t last_wake; /* 上一次周期起点tick，供vTaskDelayUntil对齐节拍。 */
    uint32_t period_ms; /* 本拍触摸轮询周期(ms)。 */
    uint32_t retry_ms; /* 熄屏休眠写失败后驱动给出的剩余退避毫秒。 */
    (void)pvParameters; /* 未使用任务参数。 */

    /* 初始化触摸控制器与软件I2C，之后本任务独占该总线。 */
    app_touch_init();
    /* 以当前tick作为周期对齐基准。 */
    last_wake = xTaskGetTickCount();
    for (;;)
    {
        /* 标记触摸任务本轮诊断开始。 */
        app_diag_task_begin(APP_DIAG_TOUCH);
        /* 默认按活跃周期10ms轮询。 */
        period_ms = APP_TOUCH_PERIOD_MS;
        /* 待机状态下降为20ms轮询，减少总线占用。 */
        if ((s_power_state & APP_POWER_IDLE) != 0)
        {
            period_ms = APP_IDLE_POLL_MS;
        }
        /* 执行一次触摸采集与事件缓存更新。 */
        app_touch_service();
        /* 记录本轮诊断结束及周期。 */
        app_diag_task_end(APP_DIAG_TOUCH, period_ms);
        /* 驱动报告已成功进入休眠：无限等待亮屏通知。 */
        if (app_touch_is_sleeping() != 0)
        {
            /* 休眠命令已写入成功，无限等待亮屏通知，完全不占用CPU轮询。 */
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            /* 被唤醒后以当前时刻重排周期，避免把休眠时长算进下一拍。 */
            last_wake = xTaskGetTickCount();
            continue;
        }
        /* 休眠写失败时驱动给出剩余退避毫秒；非0则本拍限时等待。 */
        retry_ms = app_touch_sleep_retry_ms();
        if (retry_ms != 0)
        {
            /* 退避期内按器件给的剩余毫秒限时等待；亮屏通知可提前打断，超时则重发休眠。 */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(retry_ms));
            last_wake = xTaskGetTickCount();
            continue;
        }
        /* 距上次周期起点已超过一个周期：说明本轮已超期，直接把起点拉到当前时刻。 */
        if ((TickType_t)(xTaskGetTickCount() - last_wake) >= pdMS_TO_TICKS(period_ms))
        {
            last_wake = xTaskGetTickCount(); /* 超时不连续补跑旧轮询。 */
        }
        /* 从更新后的起点睡到下一个周期边界。 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}

/* 传感器采样任务: SENSOR 总线 (PB13/PB14) 唯一 owner, MPU6050+SPL006+LSM303DLHC+EM7028+AHT21
 * 都只在本任务读写 —— 与 flight 端 flight_task 独占 I2C1 的约定同构。
 * 每轮顺序: 读UI发布的电源标志→按运动/心率/校准/熄屏决定采样模式→逐器件推进恢复与
 * 事务→按需读气压/电池/温湿度→串行推进EEPROM保存→按本轮活动重定周期并发布节拍。
 * 低频器件不新建任务，按墙钟错峰；校准、开机秒数、天气与OTA标志共用EEPROM，页写按拍推进。
 * EXTI 只记标志；GPIOB 的方向寄存器 RMW 保留短 PRIMASK 临界区。 */
static void app_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount(); /* 本任务周期调度的tick基准。 */
    TickType_t now_tick; /* 本轮结束前的当前tick，用于判断是否超期。 */
    mpu6050_cal_state cal_state; /* 零偏校准状态机的当前状态。 */
    mpu6050_offset_struct offset; /* EEPROM读出或校准得到的零偏值。 */
    app_imu_store_result store_result; /* EEPROM读写步骤的返回结果。 */
    uint16_t cal_stable, cal_frames; /* 校准的稳定帧数与已采集帧数。 */
    uint8_t cal_finished = 0; /* 零偏已恢复，或本轮校准流程已结束（成功/失败）。 */
    uint8_t imu_saving = 0, mag_saving = 0; /* IMU零偏/磁校准的EEPROM保存进度。 */
    uint8_t uptime_saving = 0, weather_saving = 0; /* 开机秒数/天气的保存进度。 */
    uint8_t ota_saving = 0; /* OTA标志保存进度；非0时其他保存让路。 */
    uint32_t baro_ms, baro_window_ms, battery_ms, now_ms, power_state; /* 气压周期起点、气压窗口起点、电池节拍、当前毫秒、电源状态快照。 */
    uint32_t period_ms = APP_SENSOR_TASK_PERIOD; /* 本轮调度周期，初值20ms。 */
    app_heart_struct heart; /* 心率快照，供本任务做调度判断。 */
    uint8_t sleeping = 0, bus_parked = 0, environment_active = 0; /* 熄屏睡眠、总线已挂起、温湿度窗口激活标志。 */
    uint8_t sleep_requested, imu_active, bus_needed, imu_warming = 0; /* 睡眠请求、本轮是否用IMU、是否需要总线、IMU预热中。 */
    uint8_t imu_sample_pending = 0, heart_pending, mpu_recovery, mpu_recovering = 0; /* 待取IMU帧、心率待处理、MPU恢复步骤结果、恢复进行中。 */
    uint8_t heart_bus_used; /* 心率服务本拍是否占用了总线。 */
    uint8_t imu_continuous, imu_background = 0, imu_mode_changed; /* 连续采样需求、后台5Hz模式、模式是否切换。 */
    uint8_t imu_reference = 0, reference_requested; /* 加速度参考模式当前值/请求值。 */
    uint8_t sporting = 0, sport_requested, sport_ready, sport_changed; /* 运动模式当前值、请求、可用条件、是否变化。 */
    uint8_t baro_active = 1, baro_read, baro_sleep_pending = 0, sleep_changed; /* 气压激活/本拍读气压/待写气压休眠/睡眠切换。 */
    uint32_t aht_period_ms = APP_AHT21_PERIOD_MS, requested_aht_period_ms; /* 温湿度当前周期与请求周期(ms)。 */
    uint32_t environment_ms = 0, window_ms = 0; /* 温湿度节拍起点与本次窗口起点。 */
    uint32_t imu_cycle_ms = 0, imu_wake_ms = 0; /* 上次IMU采样时刻与预热起点。 */
    uint32_t imu_warmup_ms, imu_period_ms; /* 预热时长与本轮IMU采样周期(ms)。 */
    uint32_t sensor_poll_ms = APP_SENSOR_SLEEP_POLL_MS, imu_read_period_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS; /* 熄屏轮询节拍与后台IMU读取周期。 */
    uint8_t stationary_slow = 0, stationary_slow_prev = 0, stationary_slow_entry = 0; /* 慢待机条件、上一轮慢待机、本次是否刚进入。 */

    (void)pvParameters; /* 未使用任务参数。 */
    /* 记录sensor任务启动时刻到启动诊断。 */
    app_diag_boot_set(APP_BOOT_SENSOR_BEGIN_MS, HAL_GetTick());

    /* SENSOR总线从初始化到采样均由本任务独占；失败仍进入原有故障处理。 */
    my_i2c_init(MY_I2C_BUS_SENSOR);
    /* 依次初始化MPU6050、气压计、磁力计、心率传感器，并把各自结果记入诊断。 */
    app_diag_sensor_init(APP_DIAG_MPU, my_mpu6050_init());
    app_diag_sensor_init(APP_DIAG_BARO, spl006_init());
    app_diag_sensor_init(APP_DIAG_MAG, lsm303dlhc_mag_init());
    app_diag_sensor_init(APP_DIAG_HEART, em7028_init()); /* 配置为待机，原有服务负责启动采样及照明。 */

    /* 数据锁须在任务上下文创建 (调度器已运行, 见 app_display 同款注释) */
    App_IMU_Init();
    /* 尝试从EEPROM读取IMU零偏，结果决定后续是恢复还是重新校准。 */
    store_result = app_imu_store_load(&offset);
    /* 初始化开机秒数、天气、电池记录、气压、心率、电池模块。 */
    app_uptime_init();
    app_weather_store_init();
    app_battery_log_init();
    App_Baro_Init();
    app_heart_init();
    app_battery_init();
    app_battery_service(); /* 零偏校准前启动采样；后续按秒完成电池启动均值。 */

    /* 初始化温湿度传感器。 */
    app_aht21_init();
    /* 记录sensor就绪与校准开始时刻。 */
    app_diag_boot_set(APP_BOOT_SENSOR_READY_MS, HAL_GetTick());
    app_diag_boot_set(APP_BOOT_CAL_BEGIN_MS, HAL_GetTick());
    /* EEPROM里有有效零偏记录：直接恢复到驱动，跳过本次静止校准。 */
    if (store_result == APP_IMU_STORE_OK)
    {
        my_mpu6050_restore_offset(&offset);
        cal_finished = 1;
        app_diag_boot_set(APP_BOOT_CAL_STATE, MPU6050_CAL_RESTORED);
        app_diag_boot_set(APP_BOOT_CAL_END_MS, HAL_GetTick());
        app_diag_boot_set(APP_BOOT_CAL_RESULT, 0);
    }
    else
    {
        /* EEPROM空白/无有效记录时才使用原校准，失败不写入。 */
        my_mpu6050_zero_offset_begin(HAL_GetTick());
        app_diag_boot_set(APP_BOOT_CAL_STATE, MPU6050_CAL_WAIT_STILL);
    }
    /* 校准与各周期起点统一以当前tick为基准。 */
    xLastWakeTime = xTaskGetTickCount();
    battery_ms = HAL_GetTick();
    baro_ms = battery_ms;
    baro_window_ms = battery_ms; /* 启动先取得一帧，此后保持5分钟相位，屏幕切换不重排。 */
    imu_cycle_ms = battery_ms;
    app_diag_boot_set(APP_BOOT_SAMPLING_BEGIN_MS, HAL_GetTick());

    for (;;)
    {
        /* 标记sensor任务本轮诊断开始。 */
        app_diag_task_begin(APP_DIAG_SENSOR_TASK);
        /* 取当前毫秒时刻，并快照一次UI发布的电源状态字。 */
        now_ms = HAL_GetTick();
        power_state = s_power_state;
        /* 从状态字取出运动模式请求位。 */
        sport_requested = (power_state & APP_POWER_SPORT) != 0;
        /* 运动模式要真正生效还需：校准流程已结束、无IMU/磁保存在途、磁校准未在进行。 */
        sport_ready = sport_requested != 0 && cal_finished != 0 && imu_saving == 0 && mag_saving == 0 && app_imu_is_calibrating() == 0;
        /* 比较本次可用状态与上一轮，得到是否发生切换。 */
        sport_changed = sport_ready != sporting;
        if (sport_changed != 0)
        {
            /* 记下新的运动模式值，并通知心率模块切换连续测量/恢复原请求。 */
            sporting = sport_ready;
            app_heart_set_sport_mode(sporting);
        }
        /* 非运动、无IMU/开机秒数/天气保存在途，且本sensor任务取到UI提交的校准请求时，重新开始零偏校准。 */
        if (sporting == 0 && imu_saving == 0 && uptime_saving == 0 && weather_saving == 0 && app_diag_take_imu_calibration() != 0)
        {
            /* 清零校准完成标志，让后续按未校准路径重新走一遍。 */
            cal_finished = 0;
            /* 丢弃旧的保存记录，并把校准结束时刻/结果置为无效值(UINT32_MAX)。 */
            app_diag_imu_save_record(APP_IMU_SAVE_NONE, 0, UINT32_MAX, UINT32_MAX);
            /* 启动驱动内零偏校准状态机，从"等待静止"开始。 */
            my_mpu6050_zero_offset_begin(HAL_GetTick());
            /* 诊断中记录校准开始时刻，并清空上一轮的结束时刻与结果。 */
            app_diag_boot_set(APP_BOOT_CAL_BEGIN_MS, HAL_GetTick());
            app_diag_boot_set(APP_BOOT_CAL_END_MS, UINT32_MAX);
            app_diag_boot_set(APP_BOOT_CAL_RESULT, UINT32_MAX);
        }
        /* 熄屏睡眠需同时满足：非运动、UI已请求熄屏、校准流程已结束、无保存/校准在途。 */
        sleep_requested = sporting == 0 && (power_state & APP_POWER_SCREEN_OFF) != 0 && cal_finished != 0 && imu_saving == 0 &&
                          mag_saving == 0 && app_imu_is_calibrating() == 0;
        /* 比较本次请求与上一轮睡眠状态，得到是否发生切换。 */
        sleep_changed = sleep_requested != sleeping;
        if (sleep_changed != 0)
        {
            /* 记录新的睡眠状态。 */
            sleeping = sleep_requested;
            /* 切换时清掉温湿度窗口，下一轮按新状态重新起窗口。 */
            environment_active = 0;
            /* IMU周期与待取帧一起重置，避免跨状态沿用旧节拍或旧帧。 */
            imu_cycle_ms = now_ms;
            imu_sample_pending = 0;
            /* 把熄屏/亮屏状态交给心率模块，决定是否允许后台自动测量。 */
            app_heart_set_sleep_mode(sleeping);
            /* 由睡眠切回非睡眠时，若总线已被挂起则重新初始化。 */
            if (sleeping == 0)
            {
                if (bus_parked != 0)
                {
                    my_i2c_init(MY_I2C_BUS_SENSOR);
                    bus_parked = 0;
                }
            }
        }
        /* 把熄屏与否交给电池模块：启动均值阶段仍按1s，完成后熄屏改为10s。 */
        app_battery_set_sleep_mode((power_state & APP_POWER_SCREEN_OFF) != 0);
        /* 温湿度默认按亮屏周期1s；熄屏时改用休眠周期。 */
        requested_aht_period_ms = APP_AHT21_PERIOD_MS;
        if ((power_state & APP_POWER_SCREEN_OFF) != 0)
        {
            requested_aht_period_ms = APP_AHT21_SLEEP_PERIOD_MS;
        }
        /* 请求周期与当前周期不同才下发，避免重复配置器件。 */
        if (requested_aht_period_ms != aht_period_ms)
        {
            app_aht21_set_period(requested_aht_period_ms);
            aht_period_ms = requested_aht_period_ms;
            /* 周期变了就重排温湿度节拍起点，并关闭当前窗口。 */
            environment_ms = HAL_GetTick();
            environment_active = 0;
        }
        /* 刚进入运动模式且气压还在采样：请求把它切到休眠，采样随之停止。 */
        if (sport_changed != 0 && sporting != 0 && baro_active != 0)
        {
            baro_sleep_pending = 1;
            baro_active = 0;
        }
        now_ms = HAL_GetTick(); /* 周期切换可能跨过毫秒，不能用更早的时间戳与新起点做无符号减法。 */
        /* 非运动且气压未激活，距上次周期起点已满5分钟：重新起一轮气压采样。 */
        if (sporting == 0 && baro_active == 0 && (uint32_t)(now_ms - baro_ms) >= APP_SENSOR_BARO_PERIOD_MS)
        {
            /* 起点按5分钟整数倍向前推进，保持相位不随本轮耗时漂移。 */
            baro_ms += (uint32_t)(now_ms - baro_ms) / APP_SENSOR_BARO_PERIOD_MS * APP_SENSOR_BARO_PERIOD_MS;
            /* 记下本轮窗口起点，供1.6s后读回气压。 */
            baro_window_ms = now_ms;
            baro_active = 1;
            baro_sleep_pending = 0;
            /* 总线若被挂起，先重新初始化再访问器件。 */
            if (bus_parked != 0)
            {
                my_i2c_init(MY_I2C_BUS_SENSOR);
                bus_parked = 0;
            }
            /* 唤醒气压计，开始本轮转换。 */
            spl006_set_sleep(0);
        }
        /* 睡眠中且温湿度窗口未激活，距上次窗口起点已满休眠周期：开一个新窗口。 */
        if (sleeping != 0 && environment_active == 0 && (uint32_t)(now_ms - environment_ms) >= APP_AHT21_SLEEP_PERIOD_MS)
        {
            /* 起点按休眠周期整数倍推进，保持固定相位。 */
            environment_ms += (uint32_t)(now_ms - environment_ms) / APP_AHT21_SLEEP_PERIOD_MS * APP_AHT21_SLEEP_PERIOD_MS;
            window_ms = now_ms;
            environment_active = 1;
            /* 总线若被挂起，先恢复再让温湿度服务访问。 */
            if (bus_parked != 0)
            {
                my_i2c_init(MY_I2C_BUS_SENSOR);
                bus_parked = 0;
            }
        }
        /* 取一份心率快照，并查询是否有待处理的熄屏自动测量。 */
        app_heart_get_snapshot(&heart);
        heart_pending = app_heart_sleep_pending();
        /* 需要加速度参考的条件：校准流程已结束、无保存在途/校准中、未在姿态页，且运动、心率开启或有待处理测量。 */
        reference_requested = cal_finished != 0 && imu_saving == 0 && mag_saving == 0 && app_imu_is_calibrating() == 0 &&
                              (power_state & APP_POWER_MOTION) == 0 && (sporting != 0 || heart.enabled != 0 || heart_pending != 0);
        /* 参考模式发生变化时，重排预热起点并置预热标志。 */
        if (reference_requested != imu_reference)
        {
            imu_reference = reference_requested;
            imu_wake_ms = now_ms;
            imu_warming = 1; /* 加速度参考切回完整姿态时，仍等待陀螺仪稳定100ms。 */
        }
        /* 把参考模式请求下发给IMU模块。 */
        app_imu_set_accel_reference(imu_reference);
        /* 连续采样需求：校准未完成、IMU/磁保存在途、磁校准中、运动、心率开启或有待处理、姿态页。 */
        imu_continuous = cal_finished == 0 || imu_saving != 0 || mag_saving != 0 || app_imu_is_calibrating() != 0 ||
                         sporting != 0 || heart.enabled != 0 || heart_pending != 0 || (power_state & APP_POWER_MOTION) != 0;
        /* 后台模式即"不需要连续采样"；与上一轮比较得出模式是否切换。 */
        imu_mode_changed = imu_background != (imu_continuous == 0);
        if (imu_mode_changed != 0)
        {
            /* 记录新的后台模式值。 */
            imu_background = imu_continuous == 0;
            /* 模式切换后重排采样起点；后台模式下先标一个待取帧，尽快取到第一帧。 */
            imu_cycle_ms = now_ms;
            imu_sample_pending = imu_background;
            /* 重新开始预热计时(完整姿态100ms、后台200ms)。 */
            imu_wake_ms = now_ms;
            imu_warming = 1;
        }
        /* 仅正常熄屏、无心率/运动/姿态/校准/保存且加速度连续稳定时才进入慢待机；
         * 任一活动或运动(新鲜帧清零stationary)立即回到200ms，计步与抬腕状态保持不变。 */
        stationary_slow = sleeping != 0 && imu_background != 0 && cal_finished != 0 && heart.enabled == 0 &&
                          heart_pending == 0 && sporting == 0 && imu_saving == 0 && mag_saving == 0 &&
                          uptime_saving == 0 && weather_saving == 0 && ota_saving == 0 && mpu_recovering == 0 &&
                          environment_active == 0 && baro_active == 0 && baro_sleep_pending == 0 &&
                          app_imu_is_calibrating() == 0 && app_imu_is_stationary() != 0;
        stationary_slow_entry = 0;
        if (stationary_slow != 0)
        {
            /* 满足慢待机条件：轮询节拍放宽到1000ms，后台IMU读取也放宽到1000ms。 */
            sensor_poll_ms = APP_SENSOR_SLEEP_IDLE_POLL_MS;
            imu_read_period_ms = APP_SENSOR_BACKGROUND_IMU_IDLE_MS;
            /* 转入慢待机的第一轮先取一帧新加速度再等1000ms：上一帧可能只过了200ms，
             * 直接改阈值会让真实读间隔达到1200ms并触发静止连续性上限、清掉计步/抬腕状态。 */
            stationary_slow_entry = stationary_slow_prev == 0;
            /* 记下已处于慢待机，供下一轮判断是否刚进入。 */
            stationary_slow_prev = 1;
        }
        else
        {
            /* 任一活动出现：轮询与后台IMU读取都回到200ms，并清除慢待机历史。 */
            sensor_poll_ms = APP_SENSOR_SLEEP_POLL_MS;
            imu_read_period_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS;
            stationary_slow_prev = 0;
        }
        s_sleep_poll_ms = sensor_poll_ms; /* 发布给LVGL，稳定等待与sensor读节拍同步。 */
        /* 后台模式下没有待取帧，且刚进入慢待机或已到读取周期：标一个待取帧并重排起点。 */
        if (imu_background != 0 && imu_sample_pending == 0 &&
            (stationary_slow_entry != 0 || (uint32_t)(now_ms - imu_cycle_ms) >= imu_read_period_ms))
        {
            imu_cycle_ms = now_ms; /* 超期只取最新一帧，不补跑休眠期间的旧采样。 */
            imu_sample_pending = 1;
        }
        /* 心率服务只在sensor任务操作硬件；准备到期的自动测量前先恢复总线。 */
        /* 本轮是否真的需要IMU：连续采样或存在待取帧。 */
        imu_active = imu_continuous != 0 || imu_sample_pending != 0;
        /* 总线需求：未睡眠，或温湿度/气压窗口激活，或气压休眠待写，或本轮要用IMU。 */
        bus_needed = sleeping == 0 || environment_active != 0 || baro_active != 0 || baro_sleep_pending != 0 || imu_active != 0;
        /* 需要总线但当前处于挂起状态：重新初始化软件I2C并清除挂起标志。 */
        if (bus_needed != 0 && bus_parked != 0)
        {
            my_i2c_init(MY_I2C_BUS_SENSOR);
            bus_parked = 0;
        }
        if (bus_needed != 0)
        {
            /* 有气压休眠待写时，尝试写一次；失败则保留标志，下一拍继续重试。 */
            if (baro_sleep_pending != 0)
            {
                baro_sleep_pending = spl006_set_sleep(1); /* 待机写失败只重试关转换，不重复采样或重排5分钟周期。 */
            }
            /* 非阻塞恢复：每个器件每拍至多推进一笔事务，失败冷却期间仍完成其他服务。 */
            if (imu_active != 0)
            {
                /* 推进一步MPU恢复流程，0=就绪、1=失败冷却、2=推进中。 */
                mpu_recovery = my_mpu6050_recover_step(HAL_GetTick());
                app_diag_sensor_recovery(APP_DIAG_MPU, mpu_recovery);
                /* 非0说明恢复尚未结束，记下恢复进行中标志。 */
                if (mpu_recovery != 0)
                {
                    mpu_recovering = 1;
                }
                /* 仍在推进(2)，或刚从进行中回到就绪(0)：需要重新预热。 */
                if (mpu_recovery == 2 || (mpu_recovering != 0 && mpu_recovery == 0))
                {
                    /* 恢复推进及首次就绪时重计预热，完整姿态在恢复后仍等陀螺仪稳定100ms。 */
                    if (imu_background != 0)
                    {
                        imu_sample_pending = 1;
                    }
                    imu_wake_ms = now_ms;
                    imu_warming = 1;
                }
                /* 恢复已就绪则清除"恢复进行中"标志。 */
                if (mpu_recovery == 0)
                {
                    mpu_recovering = 0;
                }
                /* 连续采样且不需要加速度参考时，顺带推进磁力计恢复一笔。 */
                if (imu_continuous != 0 && imu_reference == 0)
                {
                    app_diag_sensor_recovery(APP_DIAG_MAG, lsm303dlhc_mag_recover_step(HAL_GetTick()));
                }
            }
            /* 气压窗口激活时，每拍推进一步气压计恢复。 */
            if (baro_active != 0)
            {
                app_diag_sensor_recovery(APP_DIAG_BARO, spl006_recover_step(HAL_GetTick()));
            }
        }
        /* PPG先服务，沿用上一拍的新鲜运动参考；有事务时磁读取延后一拍。 */
        heart_bus_used = 0;
        if (bus_needed != 0)
        {
            /* 服务一次心率采样，返回1表示本拍占用了总线。 */
            heart_bus_used = app_heart_service();
            app_heart_get_snapshot(&heart); /* 本拍可能启动/结束自动测量，调度使用服务后的状态。 */
        }
        /* 先按普通亮屏后台服务取40ms。 */
        period_ms = APP_SENSOR_IDLE_PERIOD_MS;
        /* 校准未完成、心率开启、姿态页或有保存在途：收紧到20ms。 */
        if (cal_finished == 0 || heart.enabled != 0 || (power_state & APP_POWER_MOTION) != 0 || imu_saving != 0 || mag_saving != 0)
        {
            period_ms = APP_SENSOR_TASK_PERIOD;
        }
        /* 运动模式，或校准完成后的心率测量：再收紧到10ms。 */
        if (sporting != 0 || (heart.enabled != 0 && cal_finished != 0))
        {
            period_ms = APP_SENSOR_SPORT_PERIOD_MS;
        }
        /* 本轮确实要读IMU时，按模式配置采样周期并决定是否本拍服务。 */
        if (imu_active != 0)
        {
            /* 默认沿用本拍调度周期，预热按完整姿态的100ms。 */
            imu_period_ms = period_ms;
            imu_warmup_ms = 100;
            /* 后台模式固定200ms采样，预热也取同一周期。 */
            if (imu_background != 0)
            {
                imu_period_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS;
                imu_warmup_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS;
            }
            /* 把周期下发给IMU模块，返回值是驱动实际采用的周期。 */
            imu_period_ms = app_imu_set_sample_period(imu_period_ms);
            /* 连续模式下以驱动回报的周期作为本拍调度周期。 */
            if (imu_background == 0)
            {
                period_ms = imu_period_ms;
            }
            /* 后台模式只在200ms档且MPU未在恢复时读取；预热未结束则等满预热时长。 */
            if ((imu_background == 0 || (imu_period_ms == 200 && mpu_recovery == 0)) &&
                (imu_warming == 0 || (uint32_t)(now_ms - imu_wake_ms) >= imu_warmup_ms))
            {
                /* 预热结束，清标志。 */
                imu_warming = 0;
                /* 服务一轮IMU；本拍若心率占过总线则本轮延后磁读取。 */
                App_IMU_Service(heart_bus_used == 0);
                /* 记下本次采样时刻，并清掉待取帧。 */
                imu_cycle_ms = now_ms;
                imu_sample_pending = 0;
            }
        }
        else if (sleeping != 0)
        {
            /* 不需要IMU且处于熄屏：按熄屏轮询节拍(200/1000ms)等待。 */
            period_ms = sensor_poll_ms;
        }
        /* 心率开启时统一用10ms，满足心率服务30ms读取下限的整除关系。 */
        if (heart.enabled != 0)
        {
            /* 磁校准或MPU恢复可保留20ms配置，心率服务仍须整除30ms读取下限。 */
            period_ms = APP_SENSOR_HEART_PERIOD_MS;
        }
        /* 校准尚未完成时，每拍查询一次状态机进度并刷新诊断。 */
        if (cal_finished == 0)
        {
            cal_state = my_mpu6050_zero_offset_state();
            /* 取稳定帧数与已采集帧数，写入启动诊断。 */
            my_mpu6050_zero_offset_progress(&cal_stable, &cal_frames);
            app_diag_boot_set(APP_BOOT_CAL_STATE, (uint32_t)cal_state);
            app_diag_boot_set(APP_BOOT_CAL_FRAMES, cal_frames);
            app_diag_boot_set(APP_BOOT_CAL_STABLE_FRAMES, cal_stable);
            /* 状态机已结束(成功或失败)才置完成标志并记录结束时刻。 */
            if (cal_state == MPU6050_CAL_DONE || cal_state == MPU6050_CAL_FAILED)
            {
                cal_finished = 1;
                app_diag_boot_set(APP_BOOT_CAL_END_MS, HAL_GetTick());
                /* 成功：取回零偏值并置保存请求，EEPROM写入放到后续各拍。 */
                if (cal_state == MPU6050_CAL_DONE)
                {
                    my_mpu6050_get_offset(&offset);
                    imu_saving = 1; /* 已提交RAM偏移，EEPROM在后续各拍推进。 */
                }
                /* 失败：诊断结果记为1，不保存。 */
                else
                {
                    app_diag_boot_set(APP_BOOT_CAL_RESULT, 1);
                }
            }
        }
        /* 重新取当前毫秒，供下面的气压窗口与电池节拍判定。 */
        now_ms = HAL_GetTick();
        /* 气压窗口激活且距窗口起点满1.6s(等待1Hz背景转换出结果)：本拍读回气压。 */
        baro_read = baro_active != 0 && (uint32_t)(now_ms - baro_window_ms) >= APP_SENSOR_SLEEP_WINDOW_MS;
        if (baro_read != 0)
        {
            /* 读一次气压并写入历史，然后把气压计切到休眠，结束本轮窗口。 */
            App_Baro_Service();
            app_baro_record_history();
            baro_sleep_pending = spl006_set_sleep(1);
            baro_active = 0;
        }
        if (sleeping != 0)
        {
            /* 熄屏且无心率测量：按熄屏轮询节拍调度。 */
            if (heart.enabled == 0)
            {
                period_ms = sensor_poll_ms;
            }
            /* 温湿度窗口激活：本拍服务一次温湿度。 */
            if (environment_active != 0)
            {
                /* 无心率测量时把周期收紧到100ms，保证窗口内能推进转换步骤。 */
                if (heart.enabled == 0)
                {
                    period_ms = APP_SENSOR_ENVIRONMENT_POLL_MS;
                }
                app_aht21_service(); /* 窗口最后一拍也读回在途转换，不能因气压同拍结束而遗留旧结果。 */
                /* 首轮service后若无在途工作(转换/恢复/待读回)即提前收尾；否则保留至最大窗口或恢复边界。 */
                if ((uint32_t)(now_ms - window_ms) >= APP_SENSOR_SLEEP_WINDOW_MS || app_aht21_is_busy() == 0)
                {
                    environment_active = 0;
                }
            }
            /* 气压窗口仍在等1.6s时，无心率测量则用100ms周期推进。 */
            if (baro_active != 0 && heart.enabled == 0)
            {
                period_ms = APP_SENSOR_ENVIRONMENT_POLL_MS;
            }
            /* 熄屏下每满1000ms服务一次电池。 */
            if ((uint32_t)(now_ms - battery_ms) >= 1000)
            {
                battery_ms = now_ms;
                app_battery_service();
            }
            /* 所有硬件活动都已收尾且总线未挂起：释放软件I2C引脚并记挂起。 */
            if (imu_continuous == 0 && environment_active == 0 && baro_active == 0 && baro_sleep_pending == 0 && imu_sample_pending == 0 &&
                app_heart_sleep_pending() == 0 && bus_parked == 0)
            {
                /* MPU继续自主5Hz采样，磁力计已休眠；仅释放软件I2C引脚。 */
                my_i2c_suspend(MY_I2C_BUS_SENSOR);
                bus_parked = 1;
            }
        }
        /* 亮屏且本拍没读气压、距上次电池服务满1000ms：服务电池。 */
        else if (baro_read == 0 && (uint32_t)(now_ms - battery_ms) >= 1000)
        {
            battery_ms = now_ms;
            app_battery_service();
        }
        /* 亮屏且本拍没读气压也未到电池节拍：推进一次温湿度转换。 */
        else if (baro_read == 0)
        {
            app_aht21_service(); /* 转换等待仍分步检查，避开气压/电池读取轮次。 */
        }
        /* IMU零偏等待保存且其他保存都空闲：启动一次EEPROM写入，状态推进到2。 */
        if (imu_saving == 1 && mag_saving == 0 && uptime_saving == 0 && weather_saving == 0 && ota_saving == 0)
        {
            store_result = app_imu_store_save_begin(&offset);
            imu_saving = 2;
        }
        /* 写入已启动：每拍推进一步，直到不再是BUSY。 */
        if (imu_saving == 2)
        {
            store_result = app_imu_store_save_step();
            if (store_result != APP_IMU_STORE_BUSY)
            {
                /* 结束本轮保存，清进度标志。 */
                imu_saving = 0;
                /* 成功则诊断结果记0；失败记2(仅掉电保存失败，RAM偏移已生效)。 */
                if (store_result == APP_IMU_STORE_OK)
                {
                    app_diag_boot_set(APP_BOOT_CAL_RESULT, 0);
                }
                else
                {
                    app_diag_boot_set(APP_BOOT_CAL_RESULT, 2); /* RAM成功，掉电保存失败。 */
                }
            }
        }
        /* 其他保存都空闲，且(非运动或磁保存已在进行)时，推进磁校准保存。 */
        else if (uptime_saving == 0 && weather_saving == 0 && ota_saving == 0 && (sporting == 0 || mag_saving != 0))
        {
            mag_saving = App_IMU_SaveService();
        }
        /* 校准、开机时间、天气和OTA共用EEPROM；在途事务必须完成后才允许下一方开始。
         * OTA标志写入并读回确认后，其余保存都不再启动，等待复位进入升级。 */
        /* 开机秒数保存：OTA未在写、校准完成、其他保存在途为0时才允许启动。 */
        uptime_saving = app_uptime_service(ota_saving == 0 && cal_finished != 0 && imu_saving == 0 && mag_saving == 0 && weather_saving == 0 &&
                                          app_imu_is_calibrating() == 0);
        /* 天气保存：同样让开OTA与其余在途保存，且校准已完成。 */
        weather_saving = app_weather_store_service(ota_saving == 0 && cal_finished != 0 && imu_saving == 0 && mag_saving == 0 && uptime_saving == 0 &&
                                                  app_imu_is_calibrating() == 0);
        /* OTA标志保存优先级最高：其他保存在途为0时启动。 */
        ota_saving = app_ota_storage_service(imu_saving == 0 && mag_saving == 0 && uptime_saving == 0 && weather_saving == 0);
        /* OTA未在写时，按开机秒数服务电池记录。 */
        if (ota_saving == 0)
        {
            app_battery_log_service(app_uptime_get_seconds());
        }
        /* 采样与EEPROM服务之后重判当轮节拍：本拍读到运动、MPU仍在恢复、采样还欠账、心率启动或有任何在途保存时，
         * 当轮就退出慢待机，等待时长与发布给LVGL的节拍一起回到200ms，不被上一拍的1000ms覆盖。 */
        if (stationary_slow != 0 &&
            (app_imu_is_stationary() == 0 || mpu_recovering != 0 || imu_sample_pending != 0 || heart.enabled != 0 ||
             imu_saving != 0 || mag_saving != 0 || uptime_saving != 0 || weather_saving != 0 || ota_saving != 0 ||
             environment_active != 0 || baro_active != 0 || baro_sleep_pending != 0))
        {
            /* 本拍周期仍是慢待机节拍时，只把它改回200ms；更短的环境/心率周期不动。 */
            if (period_ms == sensor_poll_ms)
            {
                period_ms = APP_SENSOR_SLEEP_POLL_MS; /* 只缩短1000ms稳态等待，环境/心率等更短周期保持不变。 */
            }
            /* 慢待机参数一并回到活跃档，并重新发布给LVGL。 */
            sensor_poll_ms = APP_SENSOR_SLEEP_POLL_MS;
            imu_read_period_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS;
            s_sleep_poll_ms = sensor_poll_ms;
            stationary_slow_prev = 0; /* 再次静止时按慢待机入口重取新帧，跨切换读间隔仍不超过1000ms。 */
        }
        /* 记录本轮诊断结束及实际周期。 */
        app_diag_task_end(APP_DIAG_SENSOR_TASK, period_ms);
        /* 递增看门狗进度计数，表示本轮循环已完成。 */
        ++app_sensor_watchdog_progress;
        /* 熄屏且本轮按熄屏节拍等待(或蓝牙还有待收字节)：通知LVGL推进一拍。 */
        if (sleeping != 0 && (period_ms == sensor_poll_ms || kt6368a_rx_pending() != 0) && s_lvgl_task_handle != NULL)
        {
            xTaskNotifyGive(s_lvgl_task_handle);
        }
        now_tick = xTaskGetTickCount();
        /* 已超过一个周期：只让出1拍，以实际唤醒时刻重排，不补跑旧轮询或再等完整周期。 */
        if ((TickType_t)(now_tick - xLastWakeTime) >= pdMS_TO_TICKS(period_ms))
        {
            vTaskDelay(1);
            xLastWakeTime = xTaskGetTickCount();
        }
        else
        {
            /* 未超期：从上一拍起点睡到下一个周期边界，保持固定节拍。 */
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_ms));
        }
    }
}

/**********************
 *  栈溢出安全网 (configCHECK_FOR_STACK_OVERFLOW=1)
 *  触发说明: 某一任务栈越界, 停死便于调试器定位, 避免静默破坏。
 *********************/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    uint32_t task_tag = 0; /* 把任务名前4个字符打包成一个小端32位数，供复位记录区分任务。 */
    uint32_t i; /* 任务名字节循环下标。 */
    uintptr_t name_address = (uintptr_t)pcTaskName; /* 任务名指针的整数形式，用于范围检查。 */
    (void)xTask; /* 不使用TCB句柄，只记录任务名。 */
    /* 先关总中断，避免记录过程中被其它中断打断。 */
    __disable_irq();
    /* TCB可能已损坏，只读取仍落在F411 SRAM内的前四个任务名字节。 */
    if (name_address >= 0x20000000 && name_address <= 0x20020000 - 4)
    {
        /* 最多取4字节，遇字符串结束符提前停止；每字节左移8*i位拼入task_tag。 */
        for (i = 0; i < 4 && pcTaskName[i] != '\0'; ++i)
        {
            task_tag |= (uint32_t)(uint8_t)pcTaskName[i] << (8 * i);
        }
    }
    /* 以栈溢出类型、pc=0和任务名标签写入复位记录，然后死循环等待看门狗复位。 */
    app_reset_capture_stop(APP_RESET_STACK_OVERFLOW, 0, task_tag);
    for (;;)
    {
    }
}
