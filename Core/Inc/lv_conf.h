/**
 * @file  lv_conf.h — LVGL v9.6.0 工程配置（vehical_rak_edge_ai）
 *
 * 采用"最小覆盖"写法：只写与默认值不同的项，其余由 LVGL 的
 * `include/lvgl/config/lv_conf_internal.h` 提供默认值。
 * 完整可调项见 `Middlewares/Third_Party/LVGL/lv_conf_template.h`。
 *
 * 通过顶层 CMakeLists.txt 的 `target_compile_definitions(... LV_CONF_INCLUDE_SIMPLE)`
 * 让 LVGL 从 include 路径找到本文件。
 *
 * 资源约束（2026-10-14 实测，本板 STM32F407VET6）：
 *   RAM   : 112 KB（X-CUBE-AI 已占 16 KB），LVGL 接入前应用占用 46 KB
 *   FLASH : 512 KB，LVGL 接入前占用 36 KB
 *   显示  : ST7735S 128x160 SPI，**阻塞发送**，整帧约 31 ms @10.5 MHz
 */
#ifndef LV_CONF_H
#define LV_CONF_H

/* ------------------------------------------------------------------ */
/* 颜色与内存                                                          */
/* ------------------------------------------------------------------ */

/* ST7735S 是 RGB565 面板。
 * v9.6 起 LV_COLOR_DEPTH 已废弃，改用 LV_COLOR_FORMAT_DEFAULT。 */
#define LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565

/* LVGL 自带的内存池。RAM 只有 112 KB，不能吃默认的 64 KB。
 * 24 KB 够放下对象树 + 若干控件；不够时先调这里，再考虑换 LV_STDLIB_CLIB。 */
#define LV_MEM_SIZE             (16U * 1024U)

/* 单任务调用（本工程所有 LVGL 调用都在 defaultTask 里），不开 LVGL 自己的线程 */
#define LV_USE_OS               LV_OS_NONE

/* ------------------------------------------------------------------ */
/* 日志与断言（开发期开着，量产可关以省 Flash）                          */
/* ------------------------------------------------------------------ */
#define LV_USE_LOG              1
#define LV_LOG_LEVEL            LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF           0        /* 由 lv_port 里挂 serial 输出 */
#define LV_USE_ASSERT_NULL      1
#define LV_USE_ASSERT_MALLOC    1
#define LV_USE_ASSERT_STYLE     0

/* ------------------------------------------------------------------ */
/* 字体                                                                */
/* ------------------------------------------------------------------ */
/* UNSCII 是等宽点阵字体，正好用来做 ASCII 动画（矩阵雨/终端风格） */
#define LV_FONT_MONTSERRAT_14   1
#define LV_FONT_MONTSERRAT_16   1
#define LV_FONT_UNSCII_8        1
#define LV_FONT_UNSCII_16       1
#define LV_FONT_DEFAULT         &lv_font_montserrat_14

/* ------------------------------------------------------------------ */
/* 控件：只开用得上的，其余关掉省 Flash                                  */
/* ------------------------------------------------------------------ */
#define LV_USE_ANIMIMG          1        /* 帧动画 —— "动画过渡"要用 */
#define LV_USE_ARC              1
#define LV_USE_BAR              1
#define LV_USE_BUTTON           1
#define LV_USE_CANVAS           1        /* 自绘 —— ASCII 动画要用 */
#define LV_USE_CHART            1        /* 波形 */
#define LV_USE_IMAGE            1
#define LV_USE_LABEL            1
#define LV_USE_LED              1
#define LV_USE_LINE             1
#define LV_USE_SLIDER           1
#define LV_USE_SPAN             1
#define LV_USE_SPINNER          1
#define LV_USE_SWITCH           1
#define LV_USE_TABLE            1
#define LV_USE_TEXTAREA         0
#define LV_USE_KEYBOARD         0
#define LV_USE_CALENDAR         0
#define LV_USE_DROPDOWN         0
#define LV_USE_ROLLER           0
#define LV_USE_TABVIEW          0
#define LV_USE_TILEVIEW         0
#define LV_USE_WIN              0
#define LV_USE_MENU             0
#define LV_USE_MSGBOX           0
#define LV_USE_SPINBOX          0
#define LV_USE_COLORWHEEL       0
#define LV_USE_BUTTONMATRIX     0
#define LV_USE_IMAGEBUTTON      0
#define LV_USE_CHECKBOX         0
#define LV_USE_LIST             0
#define LV_USE_SCALE            0
#define LV_USE_ARCLABEL         0
#define LV_USE_BARCODE          0

/* ------------------------------------------------------------------ */
/* 渲染                                                                */
/* ------------------------------------------------------------------ */
#define LV_USE_DRAW_SW          1
/* 只保留 RGB565 与 ARGB8888，砍掉其余色深支持省 Flash */
#define LV_DRAW_SW_SUPPORT_RGB565        1
#define LV_DRAW_SW_SUPPORT_RGB565_SWAPPED 1
#define LV_DRAW_SW_SUPPORT_ARGB8888      1
#define LV_DRAW_SW_SUPPORT_RGB888        0
#define LV_DRAW_SW_SUPPORT_XRGB8888      0
#define LV_DRAW_SW_SUPPORT_RGB565A8      0
#define LV_DRAW_SW_SUPPORT_L8            0
#define LV_DRAW_SW_SUPPORT_AL88          0
#define LV_DRAW_SW_SUPPORT_A8            0
#define LV_DRAW_SW_SUPPORT_I1            0

#endif /* LV_CONF_H */
