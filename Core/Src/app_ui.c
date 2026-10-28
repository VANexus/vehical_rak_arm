/**
 * @file  app_ui.c — LVGL v9.6.0 移植层 + 页面管理器
 *
 * 职责只有两块：
 *   1) LVGL 移植三件事：tick（HAL_GetTick）/ 显示（flush → app_tft）/ 日志（调试串口）
 *   2) 页面管理器：页面表（app_ui_pages.c）→ 多 screen 直切 + 按键轮询
 *
 * ⚠️ 所有 LVGL 调用只在 defaultTask 里（LV_USE_OS = LV_OS_NONE，LVGL 不加锁）。
 * ⚠️ 字节序由 tft_blit() 内部处理，flush 里不要再交换。
 * ⚠️ 切页用无动画 lv_scr_load 直切：转场动画=整屏重绘，软 SPI 屏会撕裂风暴。
 */
#include "app_ui.h"

#include "app_build.h"
#include "app_tft.h"

#include "lvgl.h"
#include "main.h"

#include <stdio.h>

/* ------------------------------------------------------------------ */
/* 显示缓冲                                                            */
/* ------------------------------------------------------------------ */

/* 40 行的部分缓冲：128 x 40 x 2 = 10240 字节 */
#define UI_BUF_LINES   40
static uint8_t s_drawbuf[TFT_W * UI_BUF_LINES * 2];

static lv_display_t *s_disp;
static volatile uint32_t s_flush_n;
static volatile uint32_t s_handler_cycles;

/* ------------------------------------------------------------------ */
/* flush 回调                                                          */
/* ------------------------------------------------------------------ */

static void ui_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint16_t w = (uint16_t)lv_area_get_width(area);
    uint16_t h = (uint16_t)lv_area_get_height(area);

    tft_blit((uint16_t)area->x1, (uint16_t)area->y1, w, h, (const uint16_t *)px_map);
    s_flush_n++;
    lv_display_flush_ready(disp);
}

/* ------------------------------------------------------------------ */
/* 日志 → 调试串口                                                     */
/* ------------------------------------------------------------------ */

#if LV_USE_LOG
static void ui_log_cb(lv_log_level_t level, const char *buf)
{
    (void)level;
    for (const char *s = buf; *s; ++s) {
        while ((USART1->SR & 0x40U) == 0U) { }
        USART1->DR = (uint8_t)*s;
    }
}
#endif

/* ------------------------------------------------------------------ */
/* 页面管理                                                             */
/* ------------------------------------------------------------------ */

/* 页面表由 app_ui_pages.c 提供（页面之间互不依赖） */
#include "app_ui_pages.h"

#define UI_PAGE_MAX 8

static int        s_page;
static lv_obj_t  *s_screens[UI_PAGE_MAX];
static volatile uint8_t s_key_irq;          /* EXTI 回调置位（NVIC 使能后生效） */

static void ui_anim_timer(lv_timer_t *t)
{
    (void)t;
    k_pages[s_page].tick(HAL_GetTick());
}

void ui_page_next(int dir)
{
    if (dir > 0)      s_page = (s_page + 1) % k_page_count;
    else if (dir < 0) s_page = (s_page + k_page_count - 1) % k_page_count;
    else              s_page = 0;
    lv_scr_load(s_screens[s_page]);
}

const char *ui_page_name(void) { return k_pages[s_page].name; }

void ui_page_summary(char *buf, int len)
{
    (void)snprintf(buf, (size_t)len, "%s UP%lus", k_pages[s_page].name,
                   (unsigned long)(HAL_GetTick() / 1000U));
}

/* PE0 按键：双路径。
 * - EXTI 路径：HAL_GPIO_EXTI_Callback → ui_key_irq_notify()（NVIC 使能后生效）
 * - 轮询路径：30ms 消抖（NVIC 未使能时的兜底，两者共存无冲突）
 * 低有效，按下 → 下一页。 */
void ui_key_irq_notify(void) { s_key_irq = 1U; }

void ui_key_poll(void)
{
    if (s_key_irq) { s_key_irq = 0U; ui_page_next(1); return; }

    static uint8_t   stable = 1U;       /* 稳定态（1=松开） */
    static uint8_t   raw_prev = 1U;
    static uint32_t  t_change;

    uint8_t raw = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0U : 1U;
    if (raw != raw_prev) {
        raw_prev = raw;
        t_change = HAL_GetTick();
        return;
    }
    if (raw != stable && (HAL_GetTick() - t_change) >= 30U) {
        stable = raw;
        if (raw == 0U) ui_page_next(1);
    }
}

/* ------------------------------------------------------------------ */
/* 对外入口                                                            */
/* ------------------------------------------------------------------ */

void ui_init(void)
{
    lv_init();
    lv_tick_set_cb((lv_tick_get_cb_t)HAL_GetTick);

#if LV_USE_LOG
    lv_log_register_print_cb(ui_log_cb);
#endif

    s_disp = lv_display_create(TFT_W, TFT_H);
    lv_display_set_flush_cb(s_disp, ui_flush_cb);
    lv_display_set_buffers(s_disp, s_drawbuf, NULL, sizeof(s_drawbuf),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    for (int i = 0; i < k_page_count; ++i) {
        lv_obj_t *scr = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
        s_screens[i] = scr;
        k_pages[i].create(scr);
    }
    lv_scr_load(s_screens[0]);

    lv_timer_create(ui_anim_timer, 60, NULL);
}

/* ------------------------------------------------------------------ */
/* 自检口径（'?'/banner 统计用）                                        */
/* ------------------------------------------------------------------ */

void ui_step(uint32_t now_ms)
{
    (void)now_ms;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t t0 = DWT->CYCCNT;
    lv_timer_handler();
    s_handler_cycles = (uint32_t)(DWT->CYCCNT - t0);
}

uint32_t ui_last_handler_cycles(void) { return s_handler_cycles; }
uint32_t ui_flush_count(void)         { return s_flush_n; }
