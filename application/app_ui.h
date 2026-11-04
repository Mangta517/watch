/**
 * @file app_ui.h
 * @brief 手表界面：表盘、设置、运动、环境与蓝牙页面。
 *
 * 线程归属：UI创建、刷新和业务服务由LVGL显示任务执行；App_UI_Back也允许在持有
 * App_Display_Lock时提交返回请求。sensor任务维护业务快照，UI通过各模块的读取接口取数。
 *
 * 页面模型：表盘(PAGE_WATCH)与设置(PAGE_SETTINGS)两张主页在切换时保留控件、只恢复
 * 指针；详情页每次重建，返回时按s_history恢复原页面与垂直滚动位置。切页请求先挂起，
 * 等手指松开且上一段切页动画结束后才真正重建，保证LVGL事件对象在回调期间有效。
 *
 * 时间单位：UI_POLL_MS为定时器周期，UI_REFRESH_MS为数据刷新周期，UI_ANIM_MS为切页
 * 动画时长，均为毫秒；坐标与尺寸为屏幕像素。
 */
#ifndef __APP_UI_H
#define __APP_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"

    /* 显示任务初始化LVGL和LCD后调用一次；触摸硬件由独立任务初始化。
       只建表盘首帧与10ms UI定时器，不阻塞等待sensor数据。 */
    void App_UI_Create(void);
    /* 仅显示任务使用：返回UI的逻辑亮屏标志；熄屏请求会先将其清零，
       LCD休眠完成状态由内部待处理标志另外跟踪。 */
    uint8_t App_UI_IsScreenOn(void);
    /* 仅显示任务使用：PA4按住或任一边沿消抖期间维持短轮询。
       返回1时LVGL不能进入长通知等待，否则会漏掉按键松开。 */
    uint8_t App_UI_IsWakeKeyActive(void);
    /* 仅显示任务使用：熄屏时继续处理业务/按键，不运行LVGL动画和刷新定时器。
       与亮屏路径共用蓝牙收发、PA4/PA5消抖和抬腕检测。 */
    void App_UI_ServiceSleeping(void);

    /* 请求返回上一页并保留垂直滚动位置；仅显示任务或持有App_Display_Lock()时调用。
     * 不在ISR中调用；KEY_1/PA5由UI定时器消抖后触发此动作。 */
    void App_UI_Back(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UI_H */
