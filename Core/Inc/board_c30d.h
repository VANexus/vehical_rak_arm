/**
 * @file  board_c30d.h — 本工程的板级定义（vehical_rak_edge_ai）
 *
 * 按 docs/architecture.md §4：`platform/` 里的共用源码**不得**自己定义板级差异，
 * 一律通过本头文件取值；本文件**每个工程各一份**，靠顶层 CMakeLists.txt 的
 * include 路径优先级覆盖 platform/Inc 里的同名默认值。
 *
 * 板子 = 轮趣 C30D（WHEELTEC L150Pro 套件主控板），STM32F407VET6 / LQFP100。
 * 引脚依据：`docs/WHEELTEC L150Pro …zip` 内
 *   9.原理图/C30D 原理图和资源分配表/STM32F407VET6(C30D-V2.1_V2.2)主板资源分配说明.pdf
 *   （该 PDF 与 V2.0 版逐字节相同，引脚分配跨 V2.0/V2.1/V2.2 通用）
 * 汇总表见 docs/BOARD-C30D.md。
 *
 * ⚠️ 一个物理量只允许一个常数 —— 新增引脚/换算一律写在这里，不要在别处复制。
 */

#ifndef BOARD_C30D_H
#define BOARD_C30D_H

#include "stm32f4xx.h"

/* ------------------------------------------------------------------ */
/* OLED（SSD1306 128x64，软 SPI）                                       */
/* ------------------------------------------------------------------ */
/* PD11-14 不对应任何 SPI 外设，所以软 SPI 是硬件约束而非选择。
 * GPIO 方向由 CubeMX 生成的 MX_GPIO_Init() 配好（gpio.c，OUTPUT_PP），
 * 驱动只做 BSRR 直写。 */
#define OLED_PORT       GPIOD
#define OLED_PIN_DC     11U
#define OLED_PIN_RES    12U
#define OLED_PIN_DATA   13U
#define OLED_PIN_CLK    14U

/* ------------------------------------------------------------------ */
/* 外部 ST7735S TFT —— **软 SPI**（外接模块，非板载）                    */
/* ------------------------------------------------------------------ */
/* 为什么是软 SPI 而不是硬件 SPI3（2026-10-14 实测实物排针后改的）：
 *   板上 20 针「预留 IO」排针的丝印是 C10 C11 5V GND D15 A7 A6 C5
 *   A3 C3 A2 C2 A14 C1 A13 C0 GND ON 3V3 A8 —— **没有 C12**。
 *   而 STM32F407 只有 SPI1/2/3，三者的 SCK+MOSI 组合没有一个能同时
 *   落在这个排针上（SPI3 要 PC12、SPI1 要 PA5、SPI2 要 PB13/PB10，全不可用）。
 *   → 位操作软 SPI，引脚随便挑。
 *
 * 引脚分配（全部在排针上，且都在 GPIOC 以便单口 BSRR 位操作）：
 *   C5=SCK  C3=MOSI  C0=DC  C1=RST  C2=BLK   ；A7=CS（唯一不在 GPIOC 的）
 * 注意 A13/A14 是 SWD，**不可**拿来做普通 IO。
 *
 * ⚠️ 需要在 CubeMX GUI 里把这 3 个脚设成 GPIO_Output：PC5 / PC3 / PA7
 *    （PC0/PC1/PC2 已经是了）。脚本接口在这份 .ioc 上写不进新引脚，
 *    见 vehical_rak_motion/docs/CUBEMX-SOP.md §3「GUI 推荐给结构性改动」。 */
#define TFT_PORT        GPIOC
#define TFT_PIN_SCK     5U
#define TFT_PIN_MOSI    3U
#define TFT_PIN_DC      0U
#define TFT_PIN_RST     1U
#define TFT_PIN_BLK     2U
/* CS 在另一端口：GPIOA bit 7 */
#define TFT_CS_PORT     GPIOA
#define TFT_PIN_CS      7U

#endif /* BOARD_C30D_H */
