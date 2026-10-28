/**
 * @file  app_tft.h — ST7735S SPI TFT 驱动（L1，本仓私有）
 *
 * 为什么放在 edge_ai 仓而不是 platform/：
 *   这块屏是**外接模块**，不在 C30D 板上（板载的只有 SSD1306 OLED）。
 *   motion 仓永远不会用到它，放进 platform/ 只会给那边加死代码。
 *   若将来两块板都装，再上移 —— 引脚已经通过 board_c30d.h 外置，上移无阻力。
 *
 * ⚠️ 分辨率：ST7735S 常见三种规格，本驱动按 **1.8" 128x160**（最普遍）实现。
 *    实物若是 128x128 或 80x160，改 TFT_W/TFT_H + init 里的行列偏移即可。
 *    模块只有 8 根线（VCC/GND/BLK/SDA/SCL/CS/DC/RST）、**没有 MISO**，
 *    所以无法靠读 RDDID 寄存器自动识别 —— 只能按实物确认。
 *
 * ⚠️ 刷新率的物理上限（实测口径，非估算）：
 *    整帧 = 128x160x2 = 40 960 字节。
 *    SPI3 挂 APB1(42 MHz)，当前分频 /4 = **10.5 MHz** → 整帧 **31 ms ≈ 32 fps**。
 *    ST7735S 写周期上限 tSCYCW≈66ns ≈ 15 MHz，所以不能靠提到 21 MHz 翻倍。
 *    面板自身扫描率固定 ~60-70 Hz（FRMCTR1 的 RTNA/RTNB），**不可编程到 144 Hz**。
 *    → 结论：**144 Hz 在这套硬件上做不到**。可达目标是"面板满速 60 Hz + 局部刷新"。
 *
 * ⚠️ 撕裂：模块没引出 TE（Tearing Effect）引脚，CPU 写 GRAM 与面板扫描无法同步，
 *    **无法做到严格无撕裂**。缓解手段是减小每次刷新的区域（局部刷新）。
 */
#ifndef APP_TFT_H
#define APP_TFT_H

#include <stdint.h>

/* 见文件头关于分辨率的说明 —— 实物不符时改这两行 */
#define TFT_W   128
#define TFT_H   160

/* 面板 GRAM 起始偏移（1.8" 128x160 红板为 0/0；128x128 绿板常见 2/1） */
#define TFT_XOFF  0
#define TFT_YOFF  0

/* 颜色按 RGB565 */
#define TFT_RGB565(r,g,b)  ((uint16_t)((((r) & 0xF8u) << 8) | (((g) & 0xFCu) << 3) | ((b) >> 3)))

void tft_init(void);

/** 背光开关（PC2，纯 GPIO，无调光）。 */
void tft_backlight(int on);

/** 整屏填色。阻塞约 31 ms（10.5 MHz 全屏）—— 只在低优先级任务调。 */
void tft_fill(uint16_t color);

/** 把一块 RGB565 像素写到 (x,y) 起、宽 w 高 h 的区域。阻塞。 */
void tft_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *px);

/** 自检：最近一次 tft_fill() 的 DWT 周期数。除以 SystemCoreClock 即秒数。 */
uint32_t tft_fill_cycles(void);

/** 面板识别字（初始化序列写进去的 MADCTL 值），供诊断打印。 */
uint8_t tft_madctl(void);

#endif /* APP_TFT_H */
