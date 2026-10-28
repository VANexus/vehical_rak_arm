/**
 * @file  app_ui_pages.h — 页面表内部契约（app_ui.c ↔ app_ui_pages.c 专用，勿对外）
 */
#ifndef APP_UI_PAGES_H
#define APP_UI_PAGES_H

#include "lvgl.h"

typedef struct {
    const char *name;                     /* ≤8 字符，OLED 镜像用 */
    void (*create)(lv_obj_t *scr);
    void (*tick)(uint32_t now_ms);        /* 60 ms 节拍，页面自己限流慢速项 */
} ui_page_t;

extern const ui_page_t k_pages[];
extern const int        k_page_count;

/* 页面共用配色（深黑底 + 单一强调色 + 警示琥珀） */
#define UI_COL_BG      0x000000
#define UI_COL_ACCENT  0x00E5A0
#define UI_COL_WARN    0xFFB000
#define UI_COL_DIM     0x303030
#define UI_COL_TEXT    0xE8E8E8

/* 页面共用：标题行（各页 create 里调一次） */
void ui_pages_header(lv_obj_t *scr, const char *title);

#endif /* APP_UI_PAGES_H */
