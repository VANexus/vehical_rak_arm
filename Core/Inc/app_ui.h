/**
 * @file  app_ui.h — LVGL 移植层与 UI 入口（L3 之上的表现层）
 *
 * 分层：app_tft（L1，把像素送进面板）← app_ui（本文件，LVGL 移植 + 界面）
 *
 * ⚠️ 所有 LVGL 调用都必须在**同一个任务**里（本工程 = defaultTask），
 *    LV_USE_OS 配的是 LV_OS_NONE，LVGL 自身不加锁。
 */
#ifndef APP_UI_H
#define APP_UI_H

#include <stdint.h>

/** 初始化 LVGL、注册显示与 tick、构建界面。会做一次全屏清屏（阻塞 ~31ms）。 */
void ui_init(void);

/** 在任务主循环里调，驱动 LVGL 定时器与重绘。 */
void ui_step(uint32_t now_ms);

/** 自检：最近一次 lv_timer_handler() 的耗时（DWT 周期）。用于量化 UI 开销。 */
uint32_t ui_last_handler_cycles(void);

/** 自检：累计完成的刷新次数（每次 flush 回调 +1）。 */
uint32_t ui_flush_count(void);

/* ---- 页面管理（实现在 app_ui.c，页面内容在 app_ui_pages.c）---- */

void        ui_page_next(int dir);                 /* +1 下一页，-1 上一页，0 回首页 */
const char *ui_page_name(void);                    /* 当前页名（OLED 镜像用） */
void        ui_page_summary(char *buf, int len);   /* 当前页一行摘要（OLED 镜像用） */
void        ui_key_poll(void);                     /* PE0 按键轮询消抖，默认任务循环里调 */
void        ui_key_irq_notify(void);               /* EXTI 回调置标志（ISR 上下文安全） */

#endif /* APP_UI_H */
