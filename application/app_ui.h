/**
  ******************************************************************************
  * @file    app_ui.h
  * @brief   应用层-界面: lv_tabview 主页 (Message/Schedule/Meeting/Clock/IMU)
  *          IMU 页显示 MPU6050 加速度/角速度/温度与数据就绪中断速率。
  *          Clock 页演示"时间跳动": lv_label(数字秒跳) + lv_arc(秒环) + lv_timer。
  *          移植自 D:/3251/f411RET6/Watch 工程 application/app_ui.h。
  ******************************************************************************
***/
#ifndef __APP_UI_H
#define __APP_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

/* 在 lv_port_disp/indev 初始化完成后调用; 内部只建对象不起任务 */
void App_UI_Create(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UI_H */
