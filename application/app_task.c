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
#include "app_battery_log.h"

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
#define APP_LVGL_TASK_PERIOD 5    /* 保留原LVGL服务周期(ms) */

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
#define APP_SENSOR_SLEEP_PERIOD_MS          600000
#define APP_SENSOR_SLEEP_WINDOW_MS          1600 /* 等待1Hz气压新转换。 */
#define APP_SENSOR_BACKGROUND_IMU_PERIOD_MS 200 /* 后台5Hz加速度循环，用于计步和辅助抬腕。 */
#define APP_SENSOR_SLEEP_POLL_MS            100  /* 保留sensor进度检查和电池服务。 */
#define APP_TOUCH_TASK_PRIO                 1    /* 触摸采集低于sensor；LVGL只消费缓存。 */
#define APP_WATCHDOG_FEED_PERIOD_MS         100
#define APP_WATCHDOG_SENSOR_STARTUP_MS      5000
#define APP_WATCHDOG_RENDER_TIMEOUT_MS      5000

/* sensor 单写、LVGL 单读；Cortex-M4 对齐的 32 位访问不会撕裂，不共享复合快照。 */
static volatile uint32_t app_sensor_watchdog_progress;
/* UI单写；四个电源标志打包在对齐32位值中，由各任务一次读取。 */
static volatile uint32_t s_power_state;
static TaskHandle_t s_touch_task_handle;
static TaskHandle_t s_lvgl_task_handle;

void app_task_wake_from_isr(void)
{
    BaseType_t higher_priority_woken = pdFALSE;
    if (s_lvgl_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_lvgl_task_handle, &higher_priority_woken);
        portYIELD_FROM_ISR(higher_priority_woken);
    }
}

void app_task_set_power_state(uint8_t idle, uint8_t motion, uint8_t screen_off, uint8_t sport)
{
    uint32_t state = 0;
    uint32_t previous_state = s_power_state;
    if (idle != 0)
    {
        state |= APP_POWER_IDLE;
    }
    if (motion != 0)
    {
        state |= APP_POWER_MOTION;
    }
    if (screen_off != 0)
    {
        state |= APP_POWER_SCREEN_OFF;
    }
    if (sport != 0)
    {
        state |= APP_POWER_SPORT;
    }
    s_power_state = state;
    if ((previous_state & APP_POWER_SCREEN_OFF) != 0 && (state & APP_POWER_SCREEN_OFF) == 0 && s_touch_task_handle != NULL)
    {
        xTaskNotifyGive(s_touch_task_handle); /* 通知可先于阻塞到达，避免丢失快速按键唤醒。 */
    }
}

uint8_t app_task_is_screen_off(void)
{
    return (s_power_state & APP_POWER_SCREEN_OFF) != 0;
}

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
void App_Task_Create(void)
{
    tps3823_init();
    if (xTaskCreate(app_lvgl_task, "lvgl", APP_LVGL_TASK_STK, NULL, APP_LVGL_TASK_PRIO, &s_lvgl_task_handle) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }
}

void App_Task_Start(void)
{
    vTaskStartScheduler(); /* 正常永不返回 */
    for (;;)
    {
    } /* 调度器异常返回时死守 */
}

/* IDLE休眠统一由app_tickless处理，喂狗仍由LVGL及sensor任务进度决定。 */

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void app_watchdog_begin(void)
{
    tps3823_enable();
    app_watchdog.started_ms = HAL_GetTick();
    app_watchdog.last_feed_ms = app_watchdog.started_ms;
    app_reset_watchdog_feed(app_watchdog.started_ms);
}

static void app_watchdog_service(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t progress = app_sensor_watchdog_progress;

    if (app_watchdog.sensor_started == 0)
    {
        if (progress != 0)
        {
            app_watchdog.sensor_started = 1;
        }
        else if ((uint32_t)(now_ms - app_watchdog.started_ms) >= APP_WATCHDOG_SENSOR_STARTUP_MS)
        {
            return;
        }
    }
    if (app_watchdog.sensor_started != 0 && progress == app_watchdog.last_sensor_progress)
    {
        return;
    }
    if ((uint32_t)(now_ms - app_watchdog.last_feed_ms) >= APP_WATCHDOG_FEED_PERIOD_MS)
    {
        tps3823_feed();
        app_reset_watchdog_feed(now_ms);
        app_watchdog.last_sensor_progress = progress;
        app_watchdog.last_feed_ms = now_ms;
    }
}

static void app_watchdog_render_progress(void)
{
    if ((uint32_t)(HAL_GetTick() - app_watchdog.render_started_ms) >= APP_WATCHDOG_RENDER_TIMEOUT_MS)
    {
        return;
    }
    /* DMA可能已在渲染期间完成；阻塞一拍，让低优先级sensor推进后再检查喂狗。 */
    vTaskDelay(1);
    app_watchdog_service();
}

static void app_bluetooth_rx_from_isr(void)
{
    if ((s_power_state & APP_POWER_SCREEN_OFF) != 0)
    {
        app_task_wake_from_isr();
    }
}

static void app_lvgl_wait(uint32_t period_ms, uint8_t rtc_finished)
{
    if ((s_power_state & APP_POWER_SCREEN_OFF) != 0 && App_UI_IsWakeKeyActive() == 0 &&
        rtc_finished != 0 && kt6368a_init_is_pending() == 0)
    {
        if (kt6368a_rx_pending() != 0)
        {
            vTaskDelay(1); /* 本轮256字节预算未清空队列，下一拍继续，仍让sensor运行。 */
        }
        else
        {
            /* PA4或UART水位通知可先于等待到达；无接收时保留100ms喂狗期限。 */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_WATCHDOG_FEED_PERIOD_MS));
        }
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

/* LVGL 测试任务: 屏幕+触摸测试 UI 全部跑在本任务栈上 */
static void app_lvgl_task(void *pvParameters)
{
    HAL_StatusTypeDef status;
    uint8_t rtc_finished = 0;
    uint8_t bt_finished = 0;
    uint32_t period_ms;
    (void)pvParameters;

    /* 硬件拉起 + LVGL 注册 + 建测试界面, 在本任务上下文一次性完成 */
    App_Display_Init();
    /* 首帧按新分块推进喂狗，卡在同一分块时不喂；总宽限仍限制为5s。 */
    app_watchdog_begin();
    lv_port_disp_set_progress_callback(app_watchdog_service);
    lv_refr_now(NULL); /* 先提交首帧；DMA完成中断继续工作，不等待其他模块。 */
    lv_port_disp_set_progress_callback(app_watchdog_render_progress);
    /* 首帧渲染时间不占用sensor初始化的5s窗口；进入主循环后恢复双任务检查。 */
    app_watchdog.started_ms = HAL_GetTick();
    app_diag_boot_set(APP_BOOT_FIRST_FRAME_MS, HAL_GetTick());

    if (xTaskCreate(app_sensor_task, "sensor", APP_SENSOR_TASK_STK, NULL, APP_SENSOR_TASK_PRIO, NULL) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }
    if (xTaskCreate(app_touch_task, "touch", APP_TOUCH_STACK_WORDS, NULL, APP_TOUCH_TASK_PRIO, &s_touch_task_handle) != pdPASS)
    {
        Error_Handler();
    }

    /* 两种启动等待由本任务分阶段推进，sensor可在等待期间独立初始化/采集。 */
    app_diag_boot_set(APP_BOOT_RTC_BEGIN_MS, HAL_GetTick());
    watch_rtc_init_begin();
    app_diag_boot_set(APP_BOOT_BT_BEGIN_MS, HAL_GetTick());
    kt6368a_set_rx_callback(app_bluetooth_rx_from_isr);
    kt6368a_init_begin();

    for (;;)
    {
        app_diag_task_begin(APP_DIAG_LVGL);
        if (rtc_finished == 0)
        {
            status = watch_rtc_init_service();
            if (watch_rtc_init_is_pending() == 0)
            {
                rtc_finished = 1;
                app_diag_boot_set(APP_BOOT_RTC_END_MS, HAL_GetTick());
                app_diag_boot_set(APP_BOOT_RTC_RESULT, (uint32_t)status);
            }
        }
        if (bt_finished == 0 || kt6368a_init_is_pending() != 0)
        {
            status = kt6368a_init_service();
            if (bt_finished == 0 && kt6368a_init_is_pending() == 0)
            {
                bt_finished = 1;
                app_diag_boot_set(APP_BOOT_BT_END_MS, HAL_GetTick());
                app_diag_boot_set(APP_BOOT_BT_RESULT, (uint32_t)status);
            }
        }
        app_watchdog.render_started_ms = HAL_GetTick();
        App_Display_Service(); /* 锁 → lv_timer_handler → 解锁 */
        app_diag_service();
        period_ms = APP_LVGL_TASK_PERIOD;
        if ((s_power_state & APP_POWER_IDLE) != 0)
        {
            period_ms = APP_IDLE_POLL_MS;
        }
        app_diag_task_end(APP_DIAG_LVGL, period_ms);
        app_watchdog_service();
        app_lvgl_wait(period_ms, rtc_finished);
    }
}

/* 触摸总线由低优先级任务独占；初始化/恢复等待不会阻塞LVGL。 */
static void app_touch_task(void *pvParameters)
{
    TickType_t last_wake;
    uint32_t period_ms;
    (void)pvParameters;

    app_touch_init();
    last_wake = xTaskGetTickCount();
    for (;;)
    {
        app_diag_task_begin(APP_DIAG_TOUCH);
        period_ms = APP_TOUCH_PERIOD_MS;
        if ((s_power_state & APP_POWER_IDLE) != 0)
        {
            period_ms = APP_IDLE_POLL_MS;
        }
        app_touch_service();
        app_diag_task_end(APP_DIAG_TOUCH, period_ms);
        if (app_touch_is_sleeping() != 0)
        {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            last_wake = xTaskGetTickCount();
            continue;
        }
        if ((TickType_t)(xTaskGetTickCount() - last_wake) >= pdMS_TO_TICKS(period_ms))
        {
            last_wake = xTaskGetTickCount(); /* 超时不连续补跑旧轮询。 */
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period_ms));
    }
}

/* 传感器采样任务: SENSOR 总线 (PB13/PB14) 唯一 owner, MPU6050+SPL006+LSM303DLHC+EM7028+AHT21
 * 都只在本任务读写 —— 与 flight 端 flight_task 独占 I2C1 的约定同构。
 * 低频器件不新建任务，按墙钟错峰；EEPROM校准与时间记录串行，页写按拍推进。
 * EXTI 只记标志；GPIOB 的方向寄存器 RMW 保留短 PRIMASK 临界区。 */
static void app_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t now_tick;
    mpu6050_cal_state cal_state;
    mpu6050_offset_struct offset;
    app_imu_store_result store_result;
    uint16_t cal_stable, cal_frames;
    uint8_t cal_finished = 0;
    uint8_t imu_saving = 0, mag_saving = 0;
    uint8_t uptime_saving = 0;
    uint32_t baro_ms, battery_ms, now_ms, power_state;
    uint32_t period_ms = APP_SENSOR_TASK_PERIOD;
    app_heart_struct heart;
    uint8_t sleeping = 0, bus_parked = 0, environment_active = 0;
    uint8_t sleep_requested, imu_active, bus_needed, imu_warming = 0;
    uint8_t imu_sample_pending = 0, heart_pending, mpu_recovery;
    uint8_t heart_bus_used;
    uint8_t imu_continuous, imu_background = 0, imu_mode_changed;
    uint8_t sporting = 0, sport_requested, sport_ready, sport_changed;
    uint32_t environment_ms = 0, window_ms = 0;
    uint32_t imu_cycle_ms = 0, imu_wake_ms = 0;
    uint32_t imu_warmup_ms, imu_period_ms;

    (void)pvParameters;
    app_diag_boot_set(APP_BOOT_SENSOR_BEGIN_MS, HAL_GetTick());

    /* SENSOR总线从初始化到采样均由本任务独占；失败仍进入原有故障处理。 */
    my_i2c_init(MY_I2C_BUS_SENSOR);
    app_diag_sensor_init(APP_DIAG_MPU, my_mpu6050_init());
    app_diag_sensor_init(APP_DIAG_BARO, spl006_init());
    app_diag_sensor_init(APP_DIAG_MAG, lsm303dlhc_mag_init());
    app_diag_sensor_init(APP_DIAG_HEART, em7028_init()); /* 配置为待机，原有服务负责启动采样及照明。 */

    /* 数据锁须在任务上下文创建 (调度器已运行, 见 app_display 同款注释) */
    App_IMU_Init();
    store_result = app_imu_store_load(&offset);
    app_uptime_init();
    app_battery_log_init();
    App_Baro_Init();
    app_heart_init();
    app_battery_init();
    app_battery_service(); /* 零偏校准前启动采样；后续按秒完成电池启动均值。 */

    app_aht21_init();
    app_diag_boot_set(APP_BOOT_SENSOR_READY_MS, HAL_GetTick());
    app_diag_boot_set(APP_BOOT_CAL_BEGIN_MS, HAL_GetTick());
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
    xLastWakeTime = xTaskGetTickCount();
    battery_ms = HAL_GetTick();
    baro_ms = battery_ms - 500; /* 气压先在半秒点读取，电池在整秒点读取。 */
    imu_cycle_ms = battery_ms;
    app_diag_boot_set(APP_BOOT_SAMPLING_BEGIN_MS, HAL_GetTick());

    for (;;)
    {
        app_diag_task_begin(APP_DIAG_SENSOR_TASK);
        now_ms = HAL_GetTick();
        power_state = s_power_state;
        sport_requested = (power_state & APP_POWER_SPORT) != 0;
        sport_ready = sport_requested != 0 && cal_finished != 0 && imu_saving == 0 && mag_saving == 0 && app_imu_is_calibrating() == 0;
        sport_changed = sport_ready != sporting;
        if (sport_changed != 0)
        {
            sporting = sport_ready;
            app_heart_set_sport_mode(sporting);
        }
        if (sporting == 0 && imu_saving == 0 && uptime_saving == 0 && app_diag_take_imu_calibration() != 0)
        {
            cal_finished = 0;
            app_diag_imu_save_record(APP_IMU_SAVE_NONE, 0, UINT32_MAX, UINT32_MAX);
            my_mpu6050_zero_offset_begin(HAL_GetTick());
            app_diag_boot_set(APP_BOOT_CAL_BEGIN_MS, HAL_GetTick());
            app_diag_boot_set(APP_BOOT_CAL_END_MS, UINT32_MAX);
            app_diag_boot_set(APP_BOOT_CAL_RESULT, UINT32_MAX);
        }
        sleep_requested = sporting == 0 && (power_state & APP_POWER_SCREEN_OFF) != 0 && cal_finished != 0 && imu_saving == 0 &&
                          mag_saving == 0 && app_imu_is_calibrating() == 0;
        if (sleep_requested != sleeping)
        {
            sleeping = sleep_requested;
            environment_ms = now_ms;
            environment_active = 0;
            imu_cycle_ms = now_ms;
            imu_sample_pending = 0;
            if (sleeping != 0)
            {
                app_aht21_set_period(APP_SENSOR_SLEEP_PERIOD_MS);
            }
            else
            {
                app_aht21_set_period(APP_AHT21_PERIOD_MS);
            }
            app_heart_set_sleep_mode(sleeping);
            if (sleeping == 0)
            {
                if (bus_parked != 0)
                {
                    my_i2c_init(MY_I2C_BUS_SENSOR);
                    bus_parked = 0;
                }
                if (sporting == 0)
                {
                    spl006_set_sleep(0);
                }
                baro_ms = now_ms; /* 亮屏后等一次新的1Hz转换，避免读取休眠前旧值。 */
            }
            else
            {
                spl006_set_sleep(1);
            }
        }
        if (sport_changed != 0)
        {
            if (bus_parked != 0)
            {
                my_i2c_init(MY_I2C_BUS_SENSOR);
                bus_parked = 0;
            }
            if (sporting != 0)
            {
                spl006_set_sleep(1);
            }
            else if (sleeping == 0)
            {
                spl006_set_sleep(0);
                baro_ms = now_ms;
            }
        }
        if (sleeping != 0 && environment_active == 0 && (uint32_t)(now_ms - environment_ms) >= APP_SENSOR_SLEEP_PERIOD_MS)
        {
            environment_ms += APP_SENSOR_SLEEP_PERIOD_MS;
            window_ms = now_ms;
            environment_active = 1;
            if (bus_parked != 0)
            {
                my_i2c_init(MY_I2C_BUS_SENSOR);
                bus_parked = 0;
            }
            spl006_set_sleep(0);
        }
        app_heart_get_snapshot(&heart);
        heart_pending = app_heart_sleep_pending();
        imu_continuous = cal_finished == 0 || imu_saving != 0 || mag_saving != 0 || app_imu_is_calibrating() != 0 ||
                         sporting != 0 || heart.enabled != 0 || heart_pending != 0 || (power_state & APP_POWER_MOTION) != 0;
        imu_mode_changed = imu_background != (imu_continuous == 0);
        if (imu_mode_changed != 0)
        {
            imu_background = imu_continuous == 0;
            imu_cycle_ms = now_ms;
            imu_sample_pending = imu_background;
            imu_wake_ms = now_ms;
            imu_warming = 1;
        }
        if (imu_background != 0 && imu_sample_pending == 0 &&
            (uint32_t)(now_ms - imu_cycle_ms) >= APP_SENSOR_BACKGROUND_IMU_PERIOD_MS)
        {
            imu_cycle_ms = now_ms; /* 超期只取最新一帧，不补跑休眠期间的旧采样。 */
            imu_sample_pending = 1;
        }
        /* 心率服务只在sensor任务操作硬件；准备到期的自动测量前先恢复总线。 */
        imu_active = imu_continuous != 0 || imu_sample_pending != 0;
        bus_needed = sleeping == 0 || environment_active != 0 || imu_active != 0;
        if (bus_needed != 0 && bus_parked != 0)
        {
            my_i2c_init(MY_I2C_BUS_SENSOR);
            bus_parked = 0;
        }
        if (bus_needed != 0)
        {
            /* 非阻塞恢复：每个器件每拍至多推进一笔事务，失败冷却期间仍完成其他服务。 */
            if (imu_active != 0)
            {
                mpu_recovery = my_mpu6050_recover_step(HAL_GetTick());
                app_diag_sensor_recovery(APP_DIAG_MPU, mpu_recovery);
                if (imu_background != 0 && mpu_recovery == 2)
                {
                    /* 恢复完成后重新配置5Hz循环，再等待一次新的加速度转换。 */
                    imu_sample_pending = 1;
                    imu_wake_ms = now_ms;
                    imu_warming = 1;
                }
                if (imu_continuous != 0)
                {
                    app_diag_sensor_recovery(APP_DIAG_MAG, lsm303dlhc_mag_recover_step(HAL_GetTick()));
                }
            }
            if (sporting == 0 && (sleeping == 0 || environment_active != 0))
            {
                app_diag_sensor_recovery(APP_DIAG_BARO, spl006_recover_step(HAL_GetTick()));
            }
        }
        /* PPG先服务，沿用上一拍的新鲜运动参考；有事务时磁读取延后一拍。 */
        heart_bus_used = 0;
        if (bus_needed != 0)
        {
            heart_bus_used = app_heart_service();
        }
        period_ms = APP_SENSOR_IDLE_PERIOD_MS;
        if (cal_finished == 0 || heart.enabled != 0 || (power_state & APP_POWER_MOTION) != 0 || imu_saving != 0 || mag_saving != 0)
        {
            period_ms = APP_SENSOR_TASK_PERIOD;
        }
        if (sporting != 0 || (heart.enabled != 0 && cal_finished != 0))
        {
            period_ms = APP_SENSOR_SPORT_PERIOD_MS;
        }
        if (imu_active != 0)
        {
            imu_period_ms = period_ms;
            imu_warmup_ms = 100;
            if (imu_background != 0)
            {
                imu_period_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS;
                imu_warmup_ms = APP_SENSOR_BACKGROUND_IMU_PERIOD_MS;
            }
            imu_period_ms = app_imu_set_sample_period(imu_period_ms);
            if (imu_background == 0)
            {
                period_ms = imu_period_ms;
            }
            if ((imu_background == 0 || (imu_period_ms == 200 && mpu_recovery == 0)) &&
                (imu_warming == 0 || (uint32_t)(now_ms - imu_wake_ms) >= imu_warmup_ms))
            {
                imu_warming = 0;
                App_IMU_Service(heart_bus_used == 0);
                imu_cycle_ms = now_ms;
                imu_sample_pending = 0;
            }
        }
        else if (sleeping != 0)
        {
            period_ms = APP_SENSOR_SLEEP_POLL_MS;
        }
        if (heart.enabled != 0)
        {
            /* 磁校准或MPU恢复可保留20ms配置，心率服务仍须整除30ms读取下限。 */
            period_ms = APP_SENSOR_HEART_PERIOD_MS;
        }
        if (cal_finished == 0)
        {
            cal_state = my_mpu6050_zero_offset_state();
            my_mpu6050_zero_offset_progress(&cal_stable, &cal_frames);
            app_diag_boot_set(APP_BOOT_CAL_STATE, (uint32_t)cal_state);
            app_diag_boot_set(APP_BOOT_CAL_FRAMES, cal_frames);
            app_diag_boot_set(APP_BOOT_CAL_STABLE_FRAMES, cal_stable);
            if (cal_state == MPU6050_CAL_DONE || cal_state == MPU6050_CAL_FAILED)
            {
                cal_finished = 1;
                app_diag_boot_set(APP_BOOT_CAL_END_MS, HAL_GetTick());
                if (cal_state == MPU6050_CAL_DONE)
                {
                    my_mpu6050_get_offset(&offset);
                    imu_saving = 1; /* 已提交RAM偏移，EEPROM在后续各拍推进。 */
                }
                else
                {
                    app_diag_boot_set(APP_BOOT_CAL_RESULT, 1);
                }
            }
        }
        now_ms = HAL_GetTick();
        if (sleeping != 0)
        {
            if (heart.enabled == 0)
            {
                period_ms = APP_SENSOR_SLEEP_POLL_MS;
            }
            if (environment_active != 0)
            {
                app_aht21_service();
                if ((uint32_t)(now_ms - window_ms) >= APP_SENSOR_SLEEP_WINDOW_MS)
                {
                    App_Baro_Service();
                    app_baro_record_history();
                    spl006_set_sleep(1);
                    environment_active = 0;
                }
            }
            if ((uint32_t)(now_ms - battery_ms) >= 1000)
            {
                battery_ms = now_ms;
                app_battery_service();
            }
            if (imu_continuous == 0 && environment_active == 0 && imu_sample_pending == 0 &&
                app_heart_sleep_pending() == 0 && bus_parked == 0)
            {
                /* MPU继续自主5Hz采样，磁力计已休眠；仅释放软件I2C引脚。 */
                my_i2c_suspend(MY_I2C_BUS_SENSOR);
                bus_parked = 1;
            }
        }
        else if (sporting == 0 && (uint32_t)(now_ms - baro_ms) >= 1000)
        {
            baro_ms = now_ms;
            App_Baro_Service();
            app_baro_record_history();
        }
        else if ((uint32_t)(now_ms - battery_ms) >= 1000)
        {
            battery_ms = now_ms;
            app_battery_service();
        }
        else
        {
            app_aht21_service(); /* 转换等待仍分步检查，避开气压/电池读取轮次。 */
        }
        if (imu_saving == 1 && mag_saving == 0 && uptime_saving == 0)
        {
            store_result = app_imu_store_save_begin(&offset);
            imu_saving = 2;
        }
        if (imu_saving == 2)
        {
            store_result = app_imu_store_save_step();
            if (store_result != APP_IMU_STORE_BUSY)
            {
                imu_saving = 0;
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
        else if (uptime_saving == 0 && (sporting == 0 || mag_saving != 0))
        {
            mag_saving = App_IMU_SaveService();
        }
        /* 开机时间记录是运动模式下唯一的定期EEPROM访问；其余时间仍释放GPIO。 */
        uptime_saving = app_uptime_service(cal_finished != 0 && imu_saving == 0 && mag_saving == 0 && app_imu_is_calibrating() == 0);
        app_battery_log_service(app_uptime_get_seconds());
        app_diag_task_end(APP_DIAG_SENSOR_TASK, period_ms);
        ++app_sensor_watchdog_progress;
        now_tick = xTaskGetTickCount();
        /* 超期后只让出1拍，以实际唤醒时刻重排，不补跑旧轮询或再等完整周期。 */
        if ((TickType_t)(now_tick - xLastWakeTime) >= pdMS_TO_TICKS(period_ms))
        {
            vTaskDelay(1);
            xLastWakeTime = xTaskGetTickCount();
        }
        else
        {
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
    uint32_t task_tag = 0;
    uint32_t i;
    uintptr_t name_address = (uintptr_t)pcTaskName;
    (void)xTask;
    __disable_irq();
    /* TCB可能已损坏，只读取仍落在F411 SRAM内的前四个任务名字节。 */
    if (name_address >= 0x20000000 && name_address <= 0x20020000 - 4)
    {
        for (i = 0; i < 4 && pcTaskName[i] != '\0'; ++i)
        {
            task_tag |= (uint32_t)(uint8_t)pcTaskName[i] << (8 * i);
        }
    }
    app_reset_capture_stop(APP_RESET_STACK_OVERFLOW, 0, task_tag);
    for (;;)
    {
    }
}
