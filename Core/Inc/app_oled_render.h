/**
 * @file  app_oled_render.h — edge_ai 的 OLED 内容层（A1 最小集）
 *
 * 分层：platform/ 提供"画到帧缓冲 + 推屏"，本文件决定**显示什么**。
 * 内容层是各工程私有的，不进 platform/。
 */

#ifndef APP_OLED_RENDER_H
#define APP_OLED_RENDER_H

#include <stdint.h>

/**
 * 按节拍刷新屏。内部限流到 5 Hz（软 SPI 推一帧阻塞 ~1ms，不能每个 tick 都推）。
 *
 * ⚠️ 只在**低优先级任务**里调（架构文档 §3 的 L1 约束）。
 *   绝不可在任何 ISR 里调 —— 见 platform/Inc/app_oled.h 的阻塞说明。
 *
 * @param now_ms 当前毫秒时基（传 HAL_GetTick()）
 */
void oled_show_step(uint32_t now_ms);

#endif /* APP_OLED_RENDER_H */
