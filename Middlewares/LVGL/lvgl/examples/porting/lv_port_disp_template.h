/**
 * @file lv_port_disp_templ.h
 *
 */

/*Copy this file as "lv_port_disp.h" and set this value to "1" to enable content*/
#if 1

#ifndef LV_PORT_DISP_TEMPL_H
#define LV_PORT_DISP_TEMPL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************
 *      INCLUDES
 *********************/

    /*********************
 *      DEFINES
 *********************/

    /**********************
 *      TYPEDEFS
 **********************/

    /**********************
 * GLOBAL PROTOTYPES
 **********************/
    void lv_port_disp_init(void);
    void lv_port_disp_service(void);
    /* 显示任务内每次提交新分块前调用；传NULL解除，等待同一笔DMA时不调用。 */
    void lv_port_disp_set_progress_callback(void (*callback)(void));

    /**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_PORT_DISP_TEMPL_H*/

#endif /*Disable/Enable content*/
