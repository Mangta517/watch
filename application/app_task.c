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

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void app_lvgl_task(void *pvParameters);
static void app_sensor_task(void *pvParameters);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void App_Task_Create(void)
{
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
/* LVGL 测试任务: 屏幕+触摸测试 UI 全部跑在本任务栈上 */
static void app_lvgl_task(void *pvParameters)
{
    (void)pvParameters;

    /* 硬件拉起 + LVGL 注册 + 建测试界面, 在本任务上下文一次性完成 */
    App_Display_Init();
    lv_refr_now(NULL); /* 先提交首帧；DMA完成中断继续工作，不等待其他模块。 */

    /* RTC、蓝牙由LVGL任务串行初始化，完成后才进入UI收发循环。
     * LSE/AT失败仍继续开机；初始化期间界面停留在首帧。 */
    (void)watch_rtc_init();
    (void)kt6368a_init();

    if (xTaskCreate(app_sensor_task, "sensor", APP_SENSOR_TASK_STK, NULL, APP_SENSOR_TASK_PRIO, NULL) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }

    for (;;)
    {
        App_Display_Service(); /* 锁 → lv_timer_handler → 解锁 */
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

    (void)pvParameters;

    /* SENSOR总线从初始化到采样均由本任务独占；失败仍进入原有故障处理。 */
    my_i2c_init(MY_I2C_BUS_SENSOR);
    (void)my_mpu6050_init();
    (void)spl006_init();
    (void)lsm303dlhc_mag_init();
    (void)em7028_init(); /* 配置为待机，原有服务负责启动采样及照明。 */

    /* 数据锁须在任务上下文创建 (调度器已运行, 见 app_display 同款注释) */
    App_IMU_Init();
    App_Baro_Init();
    app_heart_init();
    app_battery_init();
    app_battery_service(); /* 首次电池读数在零偏校准等待前发布。 */

    /* 上电零偏校准(第一级): 正常约2s，等待稳定最多3s；内部vTaskDelay让出CPU，
     * 只推迟首帧快照, 不影响 lvgl 任务。返回 1=超时放弃(校准时被搬动),
     * 保留原零点继续采样。第二级温漂抑制(静止微调)在 App_IMU_Service 内,
     * 策略同 flight 端 gyro_trim: 检测到静止才逐轮吃掉残余零偏。 */
    (void)my_mpu6050_zero_offset();
    app_aht21_init(); /* 仅启动阶段等待上电/初始化；周期测量由下方非阻塞服务处理。 */
    xLastWakeTime = xTaskGetTickCount(); /* 校准耗时不形成 vTaskDelayUntil 补跑 */

    for (;;)
    {
        static uint8_t baro_cnt = 0;

        App_IMU_Service();
        app_heart_service();
        app_aht21_service();
        app_battery_service();
        if (++baro_cnt >= APP_SENSOR_BARO_DIV) /* 每25拍轮询一次气压 */
        {
            App_Baro_Service();
            baro_cnt = 0;
        }
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
