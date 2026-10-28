/**
 * @file  app_oled_render.c — edge_ai 的 OLED 内容层
 *
 * 布局（128x64，全大写 —— 字库只到 'Z'，小写会被静默吞成空格）：
 *   y=0  : build_tag + 运行秒数              ← AGENTS.md §5 规则 4 的硬要求
 *   y=10 : BT 收到的字节数 + 最近一条指令
 *   y=20 : DEBUG 口收到的字节数（CH9102F / COM3）
 *   y=30+: 留空，等 A2（页面/按键/波形/中文字库）
 */
#include "app_oled_render.h"

#include "app_build.h"
#include "app_bt.h"
#include "app_oled.h"
#include "app_serial.h"
#include "app_ui.h"

#include "main.h"
#include <stdio.h>

#define OLED_REFRESH_MS 200U        /* 5 Hz */

static void render(void)
{
    char row[32];

    oled_clear();

    /* ---- y=0：build_tag + 运行秒数（上板判据）---- */
    uint32_t up_s = HAL_GetTick() / 1000U;
    if (up_s > 9999U) up_s = 9999U;
    (void)snprintf(row, sizeof(row), "%s U%lu", build_tag, (unsigned long)up_s);
    oled_text(0, 0, row);

    /* ---- y=10：蓝牙链路状态 ---- */
    char cmd = bt_last_cmd();
    (void)snprintf(row, sizeof(row), "BT%lu %c", (unsigned long)serial_rx_count(SER_BT),
                   cmd ? cmd : '-');
    oled_text(0, 10, row);

    /* ---- y=20：调试串口状态 ---- */
    (void)snprintf(row, sizeof(row), "DBG%lu", (unsigned long)serial_rx_count(SER_DEBUG));
    oled_text(0, 20, row);

    /* ---- y=30/40：TFT 页面镜像（双屏联动）---- */
    oled_text(0, 30, ui_page_name());
    char sum[24];
    ui_page_summary(sum, (int)sizeof(sum));
    oled_text(0, 40, sum);

    oled_flush();
}

void oled_show_step(uint32_t now_ms)
{
    static uint32_t last;
    static uint8_t  started;

    if (!started) {
        started = 1U;
        oled_init();            /* 幂等；惰性初始化不占 boot 路径时间 */
        last = now_ms;
    }
    if ((uint32_t)(now_ms - last) < OLED_REFRESH_MS) return;
    last = now_ms;
    render();
}
