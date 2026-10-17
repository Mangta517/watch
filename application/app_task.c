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
#include "app_imu_store.h"

#include "main.h" /* Error_Handler() 声明 */
#include "FreeRTOS.h"
#include "task.h"

#include "app_display.h"
#include "app_imu.h"
#include "app_baro.h"
#include "app_heart.h"
#include "app_aht21.h"
#include "app_battery.h"
#include "lvgl.h"
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
#define APP_LVGL_TASK_PRIO   3    /* configMAX_PRIORITIES=5 */
#define APP_LVGL_TASK_STK    1024 /* 字 = 4KB (Watch 工程实测足够跑 UI+刷新) */
#define APP_LVGL_TASK_PERIOD 5    /* 保留原LVGL服务周期(ms) */

#define APP_SENSOR_TASK_PRIO   2                        /* 低于 lvgl: UI 响应优先, 采样是后台生产 */
#define APP_SENSOR_TASK_STK    512                      /* 字 = 2KB (IMU 读 + 零偏校准 + 气压读路径) */
#define APP_SENSOR_TASK_PERIOD MPU6050_SAMPLE_PERIOD_MS /* 与MPU100Hz输出一致 */
#define APP_SENSOR_BARO_DIV    25                       /* 气压250ms轮询，器件1Hz */
#define APP_WATCHDOG_FEED_PERIOD_MS 100U

/* sensor 单写、LVGL 单读；Cortex-M4 对齐的 32 位访问不会撕裂，不共享复合快照。 */
static volatile uint32_t app_sensor_watchdog_progress;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void app_lvgl_task(void *pvParameters);
static void app_sensor_task(void *pvParameters);
static void app_watchdog_service(void);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_Task_Create(void)
{
    tps3823_init();
    if (xTaskCreate(app_lvgl_task, "lvgl", APP_LVGL_TASK_STK, NULL, APP_LVGL_TASK_PRIO, NULL) != pdPASS)
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

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void app_watchdog_service(void)
{
    static uint32_t last_sensor_progress;
    static uint32_t last_feed_ms;
    static uint8_t started;
    uint32_t progress = app_sensor_watchdog_progress;

    if (progress == last_sensor_progress)
    {
        return;
    }
    if (started == 0)
    {
        /* 首帧和两任务首轮完成后才接通；外设初始化等待不消耗看门狗超时。 */
        tps3823_enable();
        started = 1;
    }
    else if ((uint32_t)(HAL_GetTick() - last_feed_ms) >= APP_WATCHDOG_FEED_PERIOD_MS)
    {
        tps3823_feed();
    }
    else
    {
        return;
    }
    last_sensor_progress = app_sensor_watchdog_progress;
    last_feed_ms = HAL_GetTick();
}

/* LVGL 测试任务: 屏幕+触摸测试 UI 全部跑在本任务栈上 */
static void app_lvgl_task(void *pvParameters)
{
    HAL_StatusTypeDef status;
    uint8_t rtc_finished = 0;
    uint8_t bt_finished = 0;
    (void)pvParameters;

    /* 硬件拉起 + LVGL 注册 + 建测试界面, 在本任务上下文一次性完成 */
    App_Display_Init();
    lv_refr_now(NULL); /* 先提交首帧；DMA完成中断继续工作，不等待其他模块。 */
    app_diag_boot_set(APP_BOOT_FIRST_FRAME_MS, HAL_GetTick());

    if (xTaskCreate(app_sensor_task, "sensor", APP_SENSOR_TASK_STK, NULL, APP_SENSOR_TASK_PRIO, NULL) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }

    /* 两种启动等待由本任务分阶段推进，sensor可在等待期间独立初始化/采集。 */
    app_diag_boot_set(APP_BOOT_RTC_BEGIN_MS, HAL_GetTick());
    watch_rtc_init_begin();
    app_diag_boot_set(APP_BOOT_BT_BEGIN_MS, HAL_GetTick());
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
        if (bt_finished == 0)
        {
            status = kt6368a_init_service();
            if (kt6368a_init_is_pending() == 0)
            {
                bt_finished = 1;
                app_diag_boot_set(APP_BOOT_BT_END_MS, HAL_GetTick());
                app_diag_boot_set(APP_BOOT_BT_RESULT, (uint32_t)status);
            }
        }
        App_Display_Service(); /* 锁 → lv_timer_handler → 解锁 */
        app_diag_service();
        app_diag_task_end(APP_DIAG_LVGL, APP_LVGL_TASK_PERIOD);
        app_watchdog_service();
        vTaskDelay(pdMS_TO_TICKS(APP_LVGL_TASK_PERIOD));
    }
}

/* 传感器采样任务: SENSOR 总线 (PB13/PB14) 唯一 owner, MPU6050+SPL006+LSM303DLHC+EM7028+AHT21
 * 都只在本任务读写 —— 与 flight 端 flight_task 独占 I2C1 的约定同构。
 * 低频器件不新建任务, 用static计数器在本任务内分频 (气压250ms)。
 * EXTI 只记标志；GPIOB 的方向寄存器 RMW 保留短 PRIMASK 临界区。 */
static void app_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    mpu6050_cal_state cal_state;
    mpu6050_offset_struct offset;
    app_imu_store_result store_result;
    uint16_t cal_stable, cal_frames;
    uint8_t cal_finished = 0;

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
    App_Baro_Init();
    app_heart_init();
    app_battery_init();
    app_battery_service(); /* 首次电池读数在零偏校准等待前发布。 */

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
    app_diag_boot_set(APP_BOOT_SAMPLING_BEGIN_MS, HAL_GetTick());

    for (;;)
    {
        static uint8_t baro_cnt = 0;

        app_diag_task_begin(APP_DIAG_SENSOR_TASK);
        /* 非阻塞恢复：每个器件每拍至多推进一笔事务，失败冷却期间仍完成其他服务。 */
        app_diag_sensor_recovery(APP_DIAG_MPU, my_mpu6050_recover_step(HAL_GetTick()));
        app_diag_sensor_recovery(APP_DIAG_BARO, spl006_recover_step(HAL_GetTick()));
        app_diag_sensor_recovery(APP_DIAG_MAG, lsm303dlhc_mag_recover_step(HAL_GetTick()));
        if (app_diag_take_imu_calibration() != 0)
        {
            cal_finished = 0;
            app_diag_imu_save_record(APP_IMU_SAVE_NONE, 0, UINT32_MAX, UINT32_MAX);
            my_mpu6050_zero_offset_begin(HAL_GetTick());
            app_diag_boot_set(APP_BOOT_CAL_BEGIN_MS, HAL_GetTick());
            app_diag_boot_set(APP_BOOT_CAL_END_MS, UINT32_MAX);
            app_diag_boot_set(APP_BOOT_CAL_RESULT, UINT32_MAX);
        }
        App_IMU_Service();
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
                    if (app_imu_store_save(&offset) == APP_IMU_STORE_OK)
                    {
                        app_diag_boot_set(APP_BOOT_CAL_RESULT, 0);
                    }
                    else
                    {
                        app_diag_boot_set(APP_BOOT_CAL_RESULT, 2); /* RAM校准成功，掉电保存失败。 */
                    }
                }
                else
                {
                    app_diag_boot_set(APP_BOOT_CAL_RESULT, 1);
                }
            }
        }
        app_heart_service();
        app_aht21_service();
        app_battery_service();
        if (++baro_cnt >= APP_SENSOR_BARO_DIV) /* 每25拍轮询一次气压 */
        {
            App_Baro_Service();
            baro_cnt = 0;
        }
        app_diag_task_end(APP_DIAG_SENSOR_TASK, APP_SENSOR_TASK_PERIOD);
        ++app_sensor_watchdog_progress;
        /* 校准保存会等待EEPROM写周期；错过周期后重新计时，避免连续补跑旧帧。 */
        if ((TickType_t)(xTaskGetTickCount() - xLastWakeTime) > pdMS_TO_TICKS(APP_SENSOR_TASK_PERIOD))
        {
            xLastWakeTime = xTaskGetTickCount();
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(APP_SENSOR_TASK_PERIOD));
    }
}

/**********************
 *  栈溢出安全网 (configCHECK_FOR_STACK_OVERFLOW=1)
 *  触发说明: 某一任务栈越界, 停死便于调试器定位, 避免静默破坏。
 *********************/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    for (;;)
    {
    }
}
