/**
 * [AI]  @file  app_oled.c — SSD1306 OLED 调试显示（软 SPI PD11..PD14）
 *
 * A1（机械臂板）：显示内容改为六轴实际角 / 模式机 / 指令流 / 复位原因 / build_tag，
 * 驱动与时序沿用迁移前底盘板验证过的实现（软 SPI + DWT 延时）。
 * 本文件不依赖 HAL tick（DWT 延时），不碰 MX 生成文件。
 */
#include "app_oled.h"
#include "app_arm.h"
#include "app_build.h"
#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>

#define OLED_REFRESH_MS 200U          /* 5Hz 重绘（R4 同款） */

/* ---- DWT 毫秒延时（init 路径禁 HAL tick 依赖——R3 教训,与 app_imu 同套路） ---- */
static void oled_delay_ms(uint32_t ms)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t start = DWT->CYCCNT;
    uint32_t span = ms * (SystemCoreClock / 1000U);
    while ((uint32_t)(DWT->CYCCNT - start) < span) { __NOP(); }
}

/* ---- 驱动（R4 验证过的软 SPI 实现） ---- */
static void pin_set(unsigned pin, int high)
{
    GPIOD->BSRR = high ? (1UL << pin) : (1UL << (pin + 16U));
}

static void oled_byte(uint8_t value, int data)
{
    pin_set(11, data);
    for (unsigned bit = 0; bit < 8; ++bit) {
        pin_set(14, 0);
        pin_set(13, (value & 0x80U) != 0);
        pin_set(14, 1);
        value <<= 1;
    }
}

static void oled_cmd(uint8_t value)  { oled_byte(value, 0); }
static void oled_data(uint8_t value) { oled_byte(value, 1); }

static void oled_init(void)
{
    /* [AI] R20 CubeMX 迁移：PD11-14 已由 MX_GPIO_Init 配置为推挽输出
       （gpio.c, OUTPUT_PP），删寄存器版 GPIO 配置块 */
    pin_set(14, 1);
    pin_set(12, 0);
    oled_delay_ms(100);
    pin_set(12, 1);
    oled_delay_ms(10);
    oled_cmd(0xAE);
    oled_cmd(0xD5); oled_cmd(0x50);
    oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xD3); oled_cmd(0x00);
    oled_cmd(0x40);
    oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0x20); oled_cmd(0x02);
    oled_cmd(0xA0); oled_cmd(0xC0);
    oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0x81); oled_cmd(0xEF);
    oled_cmd(0xD9); oled_cmd(0xF1);
    oled_cmd(0xDB); oled_cmd(0x30);
    oled_cmd(0xA4); oled_cmd(0xA6);
    oled_cmd(0xAF);
}

/* 5x7 字库（R4 原版 + [AI] 补 H/I/K/Q/X/Z，其余未列字形=空白） */
static const uint8_t font[59][5] = {
    ['+' - ' '] = {0x08, 0x08, 0x3E, 0x08, 0x08},
    ['-' - ' '] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['.' - ' '] = {0x00, 0x60, 0x60, 0x00, 0x00},
    ['0' - ' '] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    ['1' - ' '] = {0x00, 0x42, 0x7F, 0x40, 0x00},
    ['2' - ' '] = {0x42, 0x61, 0x51, 0x49, 0x46},
    ['3' - ' '] = {0x21, 0x41, 0x45, 0x4B, 0x31},
    ['4' - ' '] = {0x18, 0x14, 0x12, 0x7F, 0x10},
    ['5' - ' '] = {0x27, 0x45, 0x45, 0x45, 0x39},
    ['6' - ' '] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
    ['7' - ' '] = {0x01, 0x71, 0x09, 0x05, 0x03},
    ['8' - ' '] = {0x36, 0x49, 0x49, 0x49, 0x36},
    ['9' - ' '] = {0x06, 0x49, 0x49, 0x29, 0x1E},
    [':' - ' '] = {0x00, 0x36, 0x36, 0x00, 0x00},
    ['A' - ' '] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    ['B' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D' - ' '] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['F' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x01},
    ['G' - ' '] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H' - ' '] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I' - ' '] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['K' - ' '] = {0x7F, 0x10, 0x28, 0x44, 0x00},
    ['L' - ' '] = {0x7F, 0x40, 0x40, 0x40, 0x40},
    ['M' - ' '] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    ['N' - ' '] = {0x7F, 0x04, 0x08, 0x10, 0x7F},
    ['O' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x3E},
    ['P' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x06},
    ['Q' - ' '] = {0x3E, 0x41, 0x51, 0x21, 0x5E},
    ['R' - ' '] = {0x7F, 0x09, 0x19, 0x29, 0x46},
    ['S' - ' '] = {0x46, 0x49, 0x49, 0x49, 0x31},
    ['T' - ' '] = {0x01, 0x01, 0x7F, 0x01, 0x01},
    ['U' - ' '] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
    ['V' - ' '] = {0x1F, 0x20, 0x40, 0x20, 0x1F},
    ['W' - ' '] = {0x3F, 0x40, 0x38, 0x40, 0x3F},
    ['X' - ' '] = {0x63, 0x14, 0x08, 0x14, 0x63},
    ['Y' - ' '] = {0x07, 0x08, 0x70, 0x08, 0x07},
    ['Z' - ' '] = {0x61, 0x51, 0x49, 0x45, 0x43},
};

static uint8_t frame[8][128];
static uint8_t oled_ready;

static void pixel(unsigned x, unsigned y)
{
    if (x < 128 && y < 64) {
        unsigned page = (y / 16U) * 2U + ((y % 16U) / 8U);
        frame[page][x] |= (uint8_t)(1U << (y % 8U));
    }
}

static void glyph(unsigned x, unsigned y, char c)
{
    if (c < ' ' || c > 'Z') c = ' ';
    for (unsigned col = 0; col < 5; ++col) {
        uint8_t bits = font[(unsigned)(c - ' ')][col];
        for (unsigned row = 0; row < 7; ++row) {
            if (bits & (1U << row)) pixel(x + col, y + row);
        }
    }
}

static void display_text(unsigned x, unsigned y, const char *s)
{
    for (unsigned i = 0; s[i] && x + i * 6U < 128; ++i)
        glyph(x + i * 6U, y, s[i]);
}

static void oled_flush(void)
{
    for (unsigned page = 0; page < 8; ++page) {
        oled_cmd((uint8_t)(0xB0U + page));
        oled_cmd(0x00); oled_cmd(0x10);
        for (unsigned x = 0; x < 128; ++x) oled_data(frame[page][x]);
    }
}

/* [AI] ---- 布局（R13 调试重点,全部大写:字库仅大写） ----
   y=0 : B12.3V E1 P- R13        电池/使能/复位原因/版本
   y=10: IMU MPU S1 Y+123.4      芯片/状态/积分航向
   y=20: RT+12.3 C+120 S256      角速度/稳定修正/降额比例
   y=30: W+012 -012 +012 -012    四轮实测转速(counts/ms)
   y=40: V+123 +000 +000 SL-     车体速度(轮速正算)/打滑标志
   y=54: 状态文本 ... U999s      模式/遥控/自检/急停 + 运行秒数 */
static void render(void)
{
    char row[32];

    for (unsigned p = 0; p < 8; ++p)
        for (unsigned x = 0; x < 128; ++x) frame[p][x] = 0;

    /* y=0 : 构建标识 + 运行秒数（AGENTS.md §5 规则 4：冷启动先认标签） */
    uint32_t up_s = arm_uptime_s(HAL_GetTick());
    if (up_s > 999U) up_s = 999U;
    (void)snprintf(row, sizeof(row), "P%c U%lus",
                   (char)arm_reset_cause(), (unsigned long)up_s);
    display_text(0, 0, row);
    display_text(102, 0, build_tag);

    /* y=10: 六轴实际角（0.1° 取整到度，避免 float printf） */
    (void)snprintf(row, sizeof(row), "J1%3u J2%3u J3%3u",
                   (unsigned)(arm_axis_deg10(0U) / 10U),
                   (unsigned)(arm_axis_deg10(1U) / 10U),
                   (unsigned)(arm_axis_deg10(2U) / 10U));
    display_text(0, 10, row);
    (void)snprintf(row, sizeof(row), "J4%3u J5%3u J6%3u",
                   (unsigned)(arm_axis_deg10(3U) / 10U),
                   (unsigned)(arm_axis_deg10(4U) / 10U),
                   (unsigned)(arm_axis_deg10(5U) / 10U));
    display_text(0, 20, row);

    /* y=30: 模式机 + 指令流状态 + 异常计数 */
    static const char *const mode_name[4] = { "IDLE", "REMOTE", "ESTOP", "FAULT" };
    uint8_t mode = arm_mode();
    (void)snprintf(row, sizeof(row), "%s ST%c ER%u",
                   mode_name[mode > 3U ? 0U : mode],
                   arm_cmd_stale(HAL_GetTick()) ? '!' : '-',
                   (unsigned)arm_servo_errcnt());
    display_text(0, 30, row);

    /* y=40: Home 安全位自检提示（上电后未收到任何指令前应停在 Home） */
    display_text(0, 40, arm_estop_latched() ? "ARMD" : "READY");

    oled_flush();
}

void oled_show_step(uint32_t now_ms)
{
    static uint32_t last;
    if (oled_ready == 0U) {
        oled_init();
        oled_ready = 1U;
        last = now_ms;
    }
    if ((uint32_t)(now_ms - last) < OLED_REFRESH_MS) return;
    last = now_ms;
    render();
}
