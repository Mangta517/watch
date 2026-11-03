/**
 * @file app_ui.h
 * @brief 手表界面：表盘、设置、运动、环境与蓝牙页面。
 */
#ifndef __APP_UI_H
#define __APP_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"

    /* 显示任务初始化LVGL和LCD后调用一次；触摸硬件由独立任务初始化。 */
    void App_UI_Create(void);
    /* 仅显示任务使用：熄屏时继续处理业务/按键，不运行LVGL动画和刷新定时器。 */
    uint8_t App_UI_IsScreenOn(void);
    /* 仅显示任务使用：PA4按住或任一边沿消抖期间维持短轮询。 */
    uint8_t App_UI_IsWakeKeyActive(void);
    void App_UI_ServiceSleeping(void);

    /* 请求返回上一页并保留垂直滚动位置；仅显示任务或持有App_Display_Lock()时调用。
     * 不在ISR中调用；KEY_1/PA5由UI定时器消抖后触发此动作。 */
    void App_UI_Back(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UI_H */
