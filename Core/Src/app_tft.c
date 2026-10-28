/**
 * @file  app_tft.c — ST7735S TFT 驱动（L1，本仓私有）—— **软 SPI 版**
 *
 * 为什么不用硬件 SPI：见 board_c30d.h 里 TFT 段的说明 —— 板上 20 针预留 IO
 * 排针没有 C12，而 STM32F407 的三个 SPI 都需要一个排针上不存在的脚。
 *
 * 位操作实现：MOSI/SCK 同在 GPIOC，每次翻转只写一次 BSRR（单周期存储指令），
 * 8 个位一个字节、每字节约 24 次存储 → 实测速率见 tft_fill_cycles()。
 *
 * 时序：SPI Mode 0（CPOL=0, CPHA=0）—— MOSI 先建立，SCK 上升沿被面板采样。
 *
 * ⚠️ 本文件不碰任何外设寄存器**配置**，只写 GPIO 数据寄存器（BSRR）。
 *    引脚方向由 CubeMX 的 MX_GPIO_Init() 配好。
 */
#include "app_tft.h"
#include "board_c30d.h"

#include "main.h"
#include "stm32f4xx.h"

/* ------------------------------------------------------------------ */
/* 引脚操作                                                             */
/* ------------------------------------------------------------------ */

/* SCK / MOSI / DC / RST / BLK 都在 TFT_PORT(GPIOC) */
#define BB_SET(pin)   (TFT_PORT->BSRR = (1UL << (pin)))
#define BB_CLR(pin)   (TFT_PORT->BSRR = (1UL << ((pin) + 16U)))

/* CS 单独在 TFT_CS_PORT(GPIOA) */
#define TFT_CS_HI()   (TFT_CS_PORT->BSRR = (1UL << TFT_PIN_CS))
#define TFT_CS_LO()   (TFT_CS_PORT->BSRR = (1UL << (TFT_PIN_CS + 16U)))

#define TFT_DC_CMD()  BB_CLR(TFT_PIN_DC)
#define TFT_DC_DATA() BB_SET(TFT_PIN_DC)
#define TFT_RST_HI()  BB_SET(TFT_PIN_RST)
#define TFT_RST_LO()  BB_CLR(TFT_PIN_RST)

static void tft_delay_ms(uint32_t ms)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t span  = ms * (SystemCoreClock / 1000U);
    while ((uint32_t)(DWT->CYCCNT - start) < span) { __NOP(); }
}

/* ------------------------------------------------------------------ */
/* 软 SPI 位操作                                                        */
/* ------------------------------------------------------------------ */

static void tft_send(const uint8_t *p, uint16_t n)
{
    const uint32_t mo_set = (1UL << TFT_PIN_MOSI);
    const uint32_t mo_clr = (1UL << (TFT_PIN_MOSI + 16U));
    const uint32_t sck_s  = (1UL << TFT_PIN_SCK);
    const uint32_t sck_c  = (1UL << (TFT_PIN_SCK + 16U));
    GPIO_TypeDef  *port   = TFT_PORT;

    while (n--) {
        uint8_t v = *p++;
        for (int i = 7; i >= 0; --i) {
            /* 数据与 SCK 上升沿合并成一次 BSRR 写（Mode 0：上升沿采样）——
             * 3 写/bit 变 2 写/bit。建立时间为 0 是实测项：花屏就退回
             * 「先写数据、再写 sck_s、再写 sck_c」三步版。 */
            port->BSRR = ((v & (1u << i)) ? mo_set : mo_clr) | sck_s;
            port->BSRR = sck_c;
        }
    }
}

static void tft_cmd(uint8_t c)
{
    TFT_DC_CMD();
    TFT_CS_LO();
    tft_send(&c, 1U);
    TFT_CS_HI();
}

static void tft_data(const uint8_t *p, uint16_t n)
{
    TFT_DC_DATA();
    TFT_CS_LO();
    tft_send(p, n);
    TFT_CS_HI();
}

static void tft_data1(uint8_t d) { tft_data(&d, 1U); }

static uint8_t s_madctl;

uint8_t tft_madctl(void) { return s_madctl; }

/* ------------------------------------------------------------------ */
/* 初始化                                                               */
/* ------------------------------------------------------------------ */

void tft_backlight(int on)
{
    if (on) BB_SET(TFT_PIN_BLK); else BB_CLR(TFT_PIN_BLK);
}

void tft_init(void)
{
    tft_backlight(0);
    TFT_CS_HI();

    TFT_RST_HI(); tft_delay_ms(10);
    TFT_RST_LO(); tft_delay_ms(20);
    TFT_RST_HI(); tft_delay_ms(120);

    tft_cmd(0x01); tft_delay_ms(150);          /* SWRESET */
    tft_cmd(0x11); tft_delay_ms(120);          /* SLPOUT  */

    /* FRMCTR1/2/3：RTNA=1；FPA/BPA 从默认 0x2C/0x2D 压到 0x14/0x14
     * —— 每帧行数 ~249→~200，面板扫描率提高约 25%，撕裂窗口变窄。
     * 若出现色彩不稳/条纹，退回 0x2C/0x2D。 */
    { static const uint8_t d[] = {0x01, 0x14, 0x14}; tft_cmd(0xB1); tft_data(d, 3); }
    { static const uint8_t d[] = {0x01, 0x14, 0x14}; tft_cmd(0xB2); tft_data(d, 3); }
    { static const uint8_t d[] = {0x01, 0x14, 0x14, 0x01, 0x14, 0x14};
      tft_cmd(0xB3); tft_data(d, 6); }
    tft_cmd(0xB4); tft_data1(0x07);            /* INVCTR */

    { static const uint8_t d[] = {0xA2, 0x02, 0x84}; tft_cmd(0xC0); tft_data(d, 3); }
    tft_cmd(0xC1); tft_data1(0xC5);
    { static const uint8_t d[] = {0x0A, 0x00};       tft_cmd(0xC2); tft_data(d, 2); }
    { static const uint8_t d[] = {0x8A, 0x2A};       tft_cmd(0xC3); tft_data(d, 2); }
    { static const uint8_t d[] = {0x8A, 0xEE};       tft_cmd(0xC4); tft_data(d, 2); }
    tft_cmd(0xC5); tft_data1(0x0E);            /* VMCTR1 */

    tft_cmd(0x20);                             /* INVOFF（颜色反相改成 0x21）*/

    /* MADCTL：MY=1 MX=1 MV=0 RGB=1 → 竖屏 BGR。颜色红蓝互换就把 0xC8 改 0xC0。 */
    s_madctl = 0xC8;
    tft_cmd(0x36); tft_data1(s_madctl);

    tft_cmd(0x3A); tft_data1(0x05);            /* COLMOD = RGB565 */

    { static const uint8_t p[] = {0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
                                  0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10};
      tft_cmd(0xE0); tft_data(p, 16); }
    { static const uint8_t n[] = {0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                                  0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10};
      tft_cmd(0xE1); tft_data(n, 16); }

    tft_cmd(0x13); tft_delay_ms(10);           /* NORON */
    tft_cmd(0x29); tft_delay_ms(100);          /* DISPON */

    tft_backlight(1);
}

/* ------------------------------------------------------------------ */
/* 写像素                                                               */
/* ------------------------------------------------------------------ */

static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t b[4];
    x0 += TFT_XOFF; x1 += TFT_XOFF;
    y0 += TFT_YOFF; y1 += TFT_YOFF;

    tft_cmd(0x2A);
    b[0] = (uint8_t)(x0 >> 8); b[1] = (uint8_t)x0;
    b[2] = (uint8_t)(x1 >> 8); b[3] = (uint8_t)x1;
    tft_data(b, 4);

    tft_cmd(0x2B);
    b[0] = (uint8_t)(y0 >> 8); b[1] = (uint8_t)y0;
    b[2] = (uint8_t)(y1 >> 8); b[3] = (uint8_t)y1;
    tft_data(b, 4);

    tft_cmd(0x2C);                             /* RAMWR */
}

void tft_blit(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *px)
{
    if (w == 0U || h == 0U) return;
    if ((uint32_t)x + w > TFT_W) w = (uint16_t)(TFT_W - x);
    if ((uint32_t)y + h > TFT_H) h = (uint16_t)(TFT_H - y);
    if (w == 0U || h == 0U) return;

    tft_set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

    TFT_DC_DATA();
    TFT_CS_LO();
    static uint8_t line[TFT_W * 2];      /* static：不占任务栈（栈只有 2 KB） */
    for (uint16_t row = 0; row < h; ++row) {
        /* LVGL 的缓冲是 RGB565 小端；软 SPI 按字节发，所以这里必须先换成
         * 大端（高字节先）—— 面板要求 MSB first。整行换完一次发。 */
        const uint16_t *s = px + (uint32_t)row * w;
        for (uint16_t i = 0; i < w; ++i) {
            line[i * 2U]     = (uint8_t)(s[i] >> 8);
            line[i * 2U + 1U] = (uint8_t)(s[i] & 0xFFU);
        }
        tft_send(line, (uint16_t)(w * 2U));
    }
    TFT_CS_HI();
}

/* 自检：最近一次全屏填充耗掉的 DWT 周期数 */
static volatile uint32_t s_fill_cycles;
uint32_t tft_fill_cycles(void) { return s_fill_cycles; }

void tft_fill(uint16_t color)
{
    static uint8_t line[TFT_W * 2];      /* static：不占任务栈（栈只有 2 KB） */
    for (uint16_t i = 0; i < TFT_W; ++i) {
        line[i * 2U]      = (uint8_t)(color >> 8);
        line[i * 2U + 1U] = (uint8_t)color;
    }
    tft_set_window(0, 0, TFT_W - 1U, TFT_H - 1U);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t t0 = DWT->CYCCNT;

    TFT_DC_DATA();
    TFT_CS_LO();
    for (uint16_t row = 0; row < TFT_H; ++row) {
        tft_send(line, (uint16_t)sizeof(line));
    }
    TFT_CS_HI();

    s_fill_cycles = (uint32_t)(DWT->CYCCNT - t0);
}
