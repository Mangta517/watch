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

#include "main.h"         /* Error_Handler() 声明 */
#include "FreeRTOS.h"
#include "task.h"

#include "app_display.h"
#include "app_imu.h"
#include "app_baro.h"
#include "lvgl.h"
#include "my_i2c.h"
#include "mpu6050.h"
#include "spl006.h"

/*********************
 *  任务配置
 *********************/
#define APP_LVGL_TASK_PRIO    3               /* configMAX_PRIORITIES=5 */
#define APP_LVGL_TASK_STK     1024            /* 字 = 4KB (Watch 工程实测足够跑 UI+刷新) */
#define APP_LVGL_TASK_PERIOD  5               /* LVGL 心跳周期(ms) */

#define APP_SENSOR_TASK_PRIO     2            /* 低于 lvgl: UI 响应优先, 采样是后台生产 */
#define APP_SENSOR_TASK_STK      512          /* 字 = 2KB (IMU 读 + 零偏校准 + 气压读路径) */
#define APP_SENSOR_TASK_PERIOD   10           /* 采样周期(ms): 软件 I2C 一轮读 ~2ms, 占空 ~20% */
#define APP_SENSOR_BARO_DIV      10           /* 气压分频: 每 10 拍(100ms)轮询一次, SPL006 已降为 1Hz 背景模式 */

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
    /* 传感器总线 (PB14=SCL, PB13=SDA) + MPU6050 一次性初始化。
     * 此处尚未启动调度器, 只做忙等待时序, 不调用任何阻塞 RTOS API。
     * 初始化失败(器件未接/总线异常)不阻塞开机: 后续读接口的返回值携带故障。 */
    my_i2c_init(MY_I2C_BUS_SENSOR);
    (void)my_mpu6050_init();
    (void)spl006_init();     /* 与 MPU6050 共用 SENSOR 总线, 同样调度器前一次性初始化;
                              * 内含 HAL_Delay(20/50) 复位等待, 失败不阻塞开机 */
    /* 零偏校准 my_mpu6050_zero_offset() 内含 vTaskDelay, 不放这里,
     * 已移到 sensor 任务体首跑时执行 (上电默认静置台架)。 */

    if (xTaskCreate(app_lvgl_task, "lvgl", APP_LVGL_TASK_STK, NULL,APP_LVGL_TASK_PRIO, NULL) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }
    if (xTaskCreate(app_sensor_task, "sensor", APP_SENSOR_TASK_STK, NULL, APP_SENSOR_TASK_PRIO, NULL) != pdPASS)
    {
        Error_Handler(); /* FreeRTOS 堆不足 */
    }
}

void App_Task_Start(void)
{
    vTaskStartScheduler();      /* 正常永不返回 */
    for (;;) { }                /* 调度器异常返回时死守 */
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

    for (;;) {
        App_Display_Service();                  /* 锁 → lv_timer_handler → 解锁 */
        vTaskDelay(pdMS_TO_TICKS(APP_LVGL_TASK_PERIOD));
    }
}

/* 传感器采样任务: SENSOR 总线 (PB13/PB14) 任务级唯一 owner, MPU6050+SPL006
 * 都只在本任务读写 —— 与 flight 端 flight_task 独占 I2C1 的约定同构。
 * 低频器件不新建任务, 用 static 计数器在本任务内分频 (气压 100ms)。
 * 总线互斥不靠关中断土办法 (驱动内 PRIMASK 临界区只挡 EXTI ISR 抢同一次突发读)。 */
static void app_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    (void)pvParameters;

    /* 数据锁须在任务上下文创建 (调度器已运行, 见 app_display 同款注释) */
    App_IMU_Init();
    App_Baro_Init();

    /* 上电零偏校准(第一级): 阻塞约 0.6~3s, 期间内部 vTaskDelay 让出 CPU,
     * 只推迟首帧快照, 不影响 lvgl 任务。返回 1=超时放弃(校准时被搬动),
     * 保留原零点继续采样。第二级温漂抑制(静止微调)在 App_IMU_Service 内,
     * 策略同 flight 端 gyro_trim: 检测到静止才逐轮吃掉残余零偏。 */
    (void)my_mpu6050_zero_offset();

    for (;;) {
        static uint8_t baro_cnt = 0;

        App_IMU_Service();
        if (++baro_cnt >= APP_SENSOR_BARO_DIV)   /* 每 10 拍轮询一次气压 */
        {
            App_Baro_Service();
            baro_cnt = 0;
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
    for (;;) { }
}
