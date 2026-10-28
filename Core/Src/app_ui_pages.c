/**
 * @file  app_ui_pages.c — TFT 四个页面（页面内容层）
 *
 * 页面表 + 内容。每页职责：
 *   SYS  系统状态仪表盘（默认页）：运行时间、两路串口收包、最近蓝牙指令
 *   AI   边缘 AI 推理可视化（占位数据，等推理接通换真值）
 *   WAVE 趋势波形（双曲线，占位正弦+噪声）
 *   ART  装饰页（呼吸圆弧，替代旧 ASCII 逐帧动画）
 *
 * 设计约定（见 app_ui_pages.h 配色宏）：
 *   - 全屏重绘只发生在切页瞬间；页内只改小部件 → 撕裂暴露窗口最小
 *   - 页面之间零依赖，只通过页管理器（app_ui.c）进入
 *   - 60 ms tick 由页管理器驱动，慢速项（≥1s）页面内部用 tick 计数限流
 */
#include "app_ui_pages.h"

#include "app_build.h"
#include "app_bt.h"
#include "app_serial.h"

#include "main.h"

#include <stdio.h>

/* ------------------------------------------------------------------ */
/* 页面共用                                                            */
/* ------------------------------------------------------------------ */

void ui_pages_header(lv_obj_t *scr, const char *title)
{
    lv_obj_t *t = lv_label_create(scr);
    lv_obj_set_style_text_color(t, lv_color_hex(UI_COL_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_label_set_text(t, title);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 4, 2);
}

/* ------------------------------------------------------------------ */
/* P0 — SYS 系统状态仪表盘                                              */
/* ------------------------------------------------------------------ */

static lv_obj_t *sys_rows;
static char      sys_buf[96];

static void sys_tick(uint32_t now_ms)
{
    (void)now_ms;
    static uint8_t div;
    if (++div < 17U) return;             /* ~1s 慢速限流 */
    div = 0;

    uint32_t up_s = HAL_GetTick() / 1000U;
    char cmd = bt_last_cmd();

    (void)snprintf(sys_buf, sizeof(sys_buf), "UP %lus\n\n"
                   "BT RX %lu  %c\n"
                   "DBG RX %lu\n\n"
                   "%s",
                   (unsigned long)up_s,
                   (unsigned long)serial_rx_count(SER_BT),
                   cmd ? cmd : '-',
                   (unsigned long)serial_rx_count(SER_DEBUG),
                   build_tag);
    lv_label_set_text(sys_rows, sys_buf);
}

static void sys_create(lv_obj_t *scr)
{
    ui_pages_header(scr, "SYS");

    sys_rows = lv_label_create(scr);
    lv_obj_set_style_text_color(sys_rows, lv_color_hex(UI_COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(sys_rows, &lv_font_unscii_8, LV_PART_MAIN);
    lv_obj_set_style_text_line_space(sys_rows, 4, LV_PART_MAIN);
    lv_label_set_text(sys_rows, "...");
    lv_obj_align(sys_rows, LV_ALIGN_TOP_LEFT, 6, 28);

    sys_tick(0xFFFFFFFFU);
}

/* ------------------------------------------------------------------ */
/* P1 — AI 推理可视化（占位数据）                                        */
/* ------------------------------------------------------------------ */

static const char *const k_ai_class[]  = { "CLEAR", "OBST", "PERSON" };
static const uint8_t     k_ai_conf[]   = { 97, 84, 71 };
#define AI_CLASSES (sizeof(k_ai_class) / sizeof(k_ai_class[0]))

static lv_obj_t *ai_class_lb;
static lv_obj_t *ai_conf_bar;
static lv_obj_t *ai_stat_lb;
static lv_obj_t *ai_lat_lb;
static char      ai_buf[24];
static uint8_t   ai_idx;
static uint32_t  ai_div;

static void ai_tick(uint32_t now_ms)
{
    (void)now_ms;
    static uint8_t jitter;
    if (++ai_div >= 50U) {               /* ~3s 换一次类别（占位演示） */
        ai_div = 0;
        ai_idx = (uint8_t)((ai_idx + 1U) % AI_CLASSES);
    }
    jitter = (uint8_t)((jitter + 3U) % 5U);

    int8_t conf = (int8_t)(k_ai_conf[ai_idx] - jitter);
    if (conf < 0) conf = 0;

    lv_label_set_text(ai_class_lb, k_ai_class[ai_idx]);
    lv_bar_set_value(ai_conf_bar, conf, LV_ANIM_OFF);

    (void)snprintf(ai_buf, sizeof(ai_buf), "%d%%  12ms", conf);
    lv_label_set_text(ai_lat_lb, ai_buf);
    lv_label_set_text(ai_stat_lb,
                      (conf >= 80) ? LV_SYMBOL_OK : LV_SYMBOL_WARNING);
    lv_obj_set_style_text_color(ai_stat_lb,
                                (conf >= 80) ? lv_color_hex(UI_COL_ACCENT)
                                             : lv_color_hex(UI_COL_WARN),
                                LV_PART_MAIN);
}

static void ai_create(lv_obj_t *scr)
{
    ui_pages_header(scr, "AI");

    ai_class_lb = lv_label_create(scr);
    lv_obj_set_style_text_color(ai_class_lb, lv_color_hex(UI_COL_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(ai_class_lb, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_label_set_text(ai_class_lb, k_ai_class[0]);
    lv_obj_align(ai_class_lb, LV_ALIGN_TOP_LEFT, 6, 30);

    ai_conf_bar = lv_bar_create(scr);
    lv_obj_set_size(ai_conf_bar, 116, 14);
    lv_obj_align(ai_conf_bar, LV_ALIGN_TOP_LEFT, 6, 56);
    lv_bar_set_range(ai_conf_bar, 0, 100);
    lv_obj_set_style_bg_color(ai_conf_bar, lv_color_hex(UI_COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_bg_color(ai_conf_bar, lv_color_hex(UI_COL_ACCENT), LV_PART_INDICATOR);

    ai_lat_lb = lv_label_create(scr);
    lv_obj_set_style_text_color(ai_lat_lb, lv_color_hex(UI_COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(ai_lat_lb, &lv_font_unscii_8, LV_PART_MAIN);
    lv_label_set_text(ai_lat_lb, "--");
    lv_obj_align(ai_lat_lb, LV_ALIGN_TOP_LEFT, 6, 78);

    ai_stat_lb = lv_label_create(scr);
    lv_obj_set_style_text_font(ai_stat_lb, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_label_set_text(ai_stat_lb, LV_SYMBOL_OK);
    lv_obj_align(ai_stat_lb, LV_ALIGN_TOP_RIGHT, -6, 30);

    /* 占位提示：真数据等推理链路接通后替换 */
    lv_obj_t *hint = lv_label_create(scr);
    lv_obj_set_style_text_color(hint, lv_color_hex(UI_COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(hint, &lv_font_unscii_8, LV_PART_MAIN);
    lv_label_set_text(hint, "sim data");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 6, -6);
}

/* ------------------------------------------------------------------ */
/* P2 — WAVE 趋势波形                                                   */
/* ------------------------------------------------------------------ */

static lv_obj_t    *wave_chart;
static lv_chart_series_t *wave_s1;
static lv_chart_series_t *wave_s2;

static void wave_tick(uint32_t now_ms)
{
    (void)now_ms;
    static uint8_t div;
    static uint32_t phase;
    if (++div < 3U) return;              /* ~180ms 一点 */
    div = 0;
    phase++;

    lv_chart_set_next_value(wave_chart, wave_s1,
                            (int32_t)(50 + 40 * (phase & 1U ? -1 : 1)
                                      + (int32_t)(phase % 7U)));
    lv_chart_set_next_value(wave_chart, wave_s2, (int32_t)(50 + (phase % 11U) - 5));
}

static void wave_create(lv_obj_t *scr)
{
    ui_pages_header(scr, "WAVE");

    wave_chart = lv_chart_create(scr);
    lv_obj_set_size(wave_chart, 118, 122);
    lv_obj_align(wave_chart, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_chart_set_type(wave_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(wave_chart, 40);
    lv_chart_set_range(wave_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
    lv_chart_set_div_line_count(wave_chart, 3, 3);
    lv_obj_set_style_bg_color(wave_chart, lv_color_hex(UI_COL_BG), LV_PART_MAIN);
    lv_obj_set_style_border_color(wave_chart, lv_color_hex(UI_COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_line_color(wave_chart, lv_color_hex(0x181818), LV_PART_MAIN);

    wave_s1 = lv_chart_add_series(wave_chart, lv_color_hex(UI_COL_ACCENT),
                                  LV_CHART_AXIS_PRIMARY_Y);
    wave_s2 = lv_chart_add_series(wave_chart, lv_color_hex(UI_COL_WARN),
                                  LV_CHART_AXIS_PRIMARY_Y);
    for (int i = 0; i < 40; ++i) {
        lv_chart_set_next_value(wave_chart, wave_s1, 50);
        lv_chart_set_next_value(wave_chart, wave_s2, 50);
    }
}

/* ------------------------------------------------------------------ */
/* P3 — ART 装饰页（呼吸圆弧，替代旧 ASCII 逐帧动画）                    */
/* ------------------------------------------------------------------ */

static lv_obj_t *art_arc;

static void art_create(lv_obj_t *scr)
{
    lv_obj_t *t = lv_label_create(scr);
    lv_obj_set_style_text_color(t, lv_color_hex(UI_COL_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_font(t, &lv_font_unscii_16, LV_PART_MAIN);
    lv_label_set_text(t, "EDGE-AI");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t *sub = lv_label_create(scr);
    lv_obj_set_style_text_color(sub, lv_color_hex(UI_COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_text_font(sub, &lv_font_unscii_8, LV_PART_MAIN);
    lv_label_set_text(sub, build_tag);
    lv_obj_align(sub, LV_ALIGN_TOP_MID, 0, 54);

    art_arc = lv_arc_create(scr);
    lv_obj_set_size(art_arc, 96, 96);
    lv_obj_align(art_arc, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_arc_set_bg_angles(art_arc, 0, 360);
    lv_arc_set_value(art_arc, 0);
    lv_obj_remove_style(art_arc, NULL, LV_PART_KNOB);   /* 去掉中心旋钮 */
    lv_obj_set_clickable(art_arc, false);
    lv_obj_set_style_arc_color(art_arc, lv_color_hex(UI_COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_arc_color(art_arc, lv_color_hex(UI_COL_ACCENT), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(art_arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(art_arc, 6, LV_PART_INDICATOR);

    /* 呼吸动画：0→100→0 无限循环；重绘只有圆弧区域，撕裂暴露窗口小 */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, art_arc);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_arc_set_value);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_time(&a, 1200);
    lv_anim_set_playback_time(&a, 1200);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

static void art_tick(uint32_t now_ms) { (void)now_ms; }   /* 动画由 lv_anim 驱动 */

/* ------------------------------------------------------------------ */
/* 页面表                                                              */
/* ------------------------------------------------------------------ */

const ui_page_t k_pages[] = {
    { "SYS",  sys_create,  sys_tick  },
    { "AI",   ai_create,   ai_tick   },
    { "WAVE", wave_create, wave_tick },
    { "ART",  art_create,  art_tick  },
};
const int k_page_count = (int)(sizeof(k_pages) / sizeof(k_pages[0]));
