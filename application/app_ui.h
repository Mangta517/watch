/**
 * @file app_ui.h
 * @brief Dark watch UI: fixed watch, combined settings/motion/environment and BLE receive pages.
 */
#ifndef __APP_UI_H
#define __APP_UI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "lvgl.h"

    /* Call once in the display task after display and touch initialization. */
    void App_UI_Create(void);

    /* Queue a return to the previous page, preserving its vertical scroll position.
 * Call in the display task or under App_Display_Lock(); never from an ISR.
 * KEY_1/PA5 is debounced and connected to this action by the UI timer. */
    void App_UI_Back(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UI_H */
