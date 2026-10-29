/*
 * app_oled.c — [AI] A13 SSD1306 OLED（**I2C**，软 I2C，PD11..PD14 开漏）
 *
 * 【为什么重写】用户实测：三块屏都是 **4 脚 I2C（VCC/GND/SCL/SDA）**，
 *   而旧驱动是 4 线 SPI（DC/RES/CLK/DIN）。两者对不上，所以屏从来没亮过
 *   （历史上"OLED 显示正确"是 SWD 读回帧缓冲 RAM 得出的，不是肉眼确认）。
 *
 * 【引脚】板子中央 OLED 口 = PD11..PD14 四根。I2C 只用其中 2 根（SCL/SDA），
 *   但排线顺序未知 → **运行时自动试探**：{(11,12),(12,11),(13,14),(14,13)}
 *   × 地址 {0x3C,0x3D}，谁 ACK 就用谁。
 *   两脚已由 CubeMX 配成 GPIO_MODE_OUTPUT_OD（开漏，I2C 必需）。
 *
 * 【时序】DWT 忙等半周期 ~3 µs（≈160 kHz，够 SSD1306 用；不占定时器）。
 *
 * 【对外】
 *   oled_show_step(now)  10 Hz 调用，内部 5 Hz 重绘
 *   oled_set_page(p)     CAN 0x033 UI_CMD：0=自动 1=关节 2=IMU 3=诊断 4=自检反白
 *   oled_set_contrast(c)
 */
#include "app_oled.h"

#include "app_arm.h"
#include "app_build.h"
#include "app_dbg.h"
#include "app_imu.h"
#include "app_joint.h"
#include "main.h"

#include <stdio.h>

#define OLED_REFRESH_MS 200U
#define OLED_I2C_DLY    48U      /* 忙等圈数，约 3 µs */

/* ---- 帧缓冲（页式 8×128）---- */
static uint8_t frame[8][128];

/* ---- 探测结果 ---- */
static uint8_t s_scl, s_sda;     /* 引脚号（11..14） */
static uint8_t s_addr;           /* 7 位地址 */
static uint8_t s_ready;
static uint8_t s_page;           /* 0=自动 */
static uint8_t s_contrast = 0xCF;
static uint8_t s_invert;

/* 5x7 字库（仅 0x20..0x5A，与旧驱动一致，只支持大写） */
static const uint8_t font[64][5] = {
    [' ' - ' '] = {0x00, 0x00, 0x00, 0x00, 0x00},
    ['-' - ' '] = {0x08, 0x08, 0x08, 0x08, 0x08},
    ['.' - ' '] = {0x00, 0x60, 0x60, 0x00, 0x00},
    [':' - ' '] = {0x00, 0x36, 0x36, 0x00, 0x00},
    ['/' - ' '] = {0x20, 0x10, 0x08, 0x04, 0x02},
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
    ['A' - ' '] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
    ['B' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x36},
    ['C' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x22},
    ['D' - ' '] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
    ['E' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x41},
    ['F' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x01},
    ['G' - ' '] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
    ['H' - ' '] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
    ['I' - ' '] = {0x00, 0x41, 0x7F, 0x41, 0x00},
    ['J' - ' '] = {0x20, 0x40, 0x41, 0x3F, 0x01},
    ['K' - ' '] = {0x7F, 0x08, 0x14, 0x22, 0x41},
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
    ['W' - ' '] = {0x7F, 0x20, 0x18, 0x20, 0x7F},
    ['X' - ' '] = {0x63, 0x14, 0x08, 0x14, 0x63},
    ['Y' - ' '] = {0x03, 0x04, 0x78, 0x04, 0x03},
    ['Z' - ' '] = {0x61, 0x51, 0x49, 0x45, 0x43},
};

/* ======================= 软 I2C（开漏） ======================= */

static void od_hi(uint8_t pin) { GPIOD->BSRR = (1UL << pin); }          /* 释放 → 上拉拉高 */
static void od_lo(uint8_t pin) { GPIOD->BSRR = (1UL << (pin + 16U)); }  /* 拉低 */
static int  od_rd(uint8_t pin) { return (GPIOD->IDR & (1UL << pin)) ? 1 : 0; }

static void i2c_dly(void)
{
    volatile uint32_t n = OLED_I2C_DLY;
    while (n--) { __NOP(); }
}

static void i2c_start(void)
{
    od_hi(s_sda); od_hi(s_scl); i2c_dly();
    od_lo(s_sda); i2c_dly();
    od_lo(s_scl); i2c_dly();
}

static void i2c_stop(void)
{
    od_lo(s_sda); i2c_dly();
    od_hi(s_scl); i2c_dly();
    od_hi(s_sda); i2c_dly();
}

static int i2c_wr(uint8_t b)   /* 0 = ACK */
{
    for (unsigned i = 0; i < 8U; ++i) {
        if (b & 0x80U) od_hi(s_sda); else od_lo(s_sda);
        b = (uint8_t)(b << 1);
        i2c_dly(); od_hi(s_scl); i2c_dly(); od_lo(s_scl);
    }
    od_hi(s_sda); i2c_dly();
    od_hi(s_scl); i2c_dly();
    int ack = od_rd(s_sda);
    od_lo(s_scl); i2c_dly();
    return ack;
}

/* 写一帧：控制字节 + n 字节数据 */
static int oled_write(uint8_t ctrl, const uint8_t *buf, unsigned n)
{
    i2c_start();
    if (i2c_wr((uint8_t)(s_addr << 1))) { i2c_stop(); return 1; }
    (void)i2c_wr(ctrl);
    for (unsigned i = 0; i < n; ++i) (void)i2c_wr(buf[i]);
    i2c_stop();
    return 0;
}

static void oled_cmd1(uint8_t c) { (void)oled_write(0x00U, &c, 1U); }

/* ======================= 探测 / 初始化 ======================= */

static int probe(uint8_t scl, uint8_t sda, uint8_t addr)
{
    s_scl = scl; s_sda = sda; s_addr = addr;
    od_hi(scl); od_hi(sda); i2c_dly();
    i2c_start();
    int nack = i2c_wr((uint8_t)(addr << 1));
    i2c_stop();
    return (nack == 0) ? 0 : 1;
}

static void oled_init(void)
{
    BOOT_MARK(8);

    /* 自动试探 SCL/SDA 组合与地址 */
    static const uint8_t pairs[4][2] = { {11, 12}, {13, 14}, {12, 11}, {14, 13} };
    static const uint8_t addrs[2] = { 0x3C, 0x3D };
    int found = 0;
    for (unsigned p = 0; p < 4U && found == 0; ++p) {
        for (unsigned a = 0; a < 2U && found == 0; ++a) {
            if (probe(pairs[p][0], pairs[p][1], addrs[a]) == 0) found = 1;
        }
    }
    if (found == 0) { s_ready = 0U; return; }   /* 没找到：任务继续跑，不阻塞 */

    static const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (unsigned i = 0; i < sizeof(init_seq); ++i) oled_cmd1(init_seq[i]);
    BOOT_MARK(9);
    s_ready = 1U;
}

/* ======================= 绘制 ======================= */

static void pixel(unsigned x, unsigned y)
{
    if (x < 128U && y < 64U) frame[y / 8U][x] |= (uint8_t)(1U << (y % 8U));
}

static void glyph(unsigned x, unsigned y, char c)
{
    if (c < ' ' || c > 'Z') c = ' ';
    const uint8_t *g = font[(unsigned)(c - ' ')];
    for (unsigned col = 0; col < 5U; ++col) {
        for (unsigned row = 0; row < 7U; ++row) {
            if (g[col] & (1U << row)) pixel(x + col, y + row);
        }
    }
}

static void text(unsigned x, unsigned y, const char *s)
{
    for (unsigned i = 0; s[i] != '\0' && (x + i * 6U) < 128U; ++i) glyph(x + i * 6U, y, s[i]);
}

static void clear_frame(void)
{
    for (unsigned p = 0; p < 8U; ++p)
        for (unsigned x = 0; x < 128U; ++x) frame[p][x] = 0U;
}

static void flush(void)
{
    BOOT_MARK(10);
    for (unsigned page = 0; page < 8U; ++page) {
        uint8_t cmds[3];
        cmds[0] = (uint8_t)(0xB0U + page);
        cmds[1] = 0x00U;          /* 列低 4 位 */
        cmds[2] = 0x10U;          /* 列高 4 位 */
        (void)oled_write(0x00U, cmds, 3U);
        (void)oled_write(0x40U, frame[page], 128U);
    }
}

/* 全屏自检图案（CAN 0x033 PAGE=4）*/
static void selftest_pattern(void)
{
    for (unsigned p = 0; p < 8U; ++p) {
        for (unsigned x = 0; x < 128U; ++x) {
            frame[p][x] = (uint8_t)(((x / 8U) % 2U) ? 0xAAU : 0x55U);
        }
    }
}

static void render(void)
{
    char row[24];
    clear_frame();

    if (s_page == 4U) { selftest_pattern(); flush(); return; }

    /* 首行：标签 + 运行秒数 */
    uint32_t up = arm_uptime_s(HAL_GetTick());
    if (up > 9999U) up = 9999U;
    (void)snprintf(row, sizeof(row), "P%c U%lus", (char)arm_reset_cause(), (unsigned long)up);
    text(0, 0, row);
    text(96, 0, build_tag);

    if (s_page == 0U || s_page == 1U) {
        (void)snprintf(row, sizeof(row), "J1%3u J2%3u J3%3u",
                       (unsigned)(arm_axis_deg10(0U) / 10U),
                       (unsigned)(arm_axis_deg10(1U) / 10U),
                       (unsigned)(arm_axis_deg10(2U) / 10U));
        text(0, 12, row);
        (void)snprintf(row, sizeof(row), "J4%3u J5%3u J6%3u",
                       (unsigned)(arm_axis_deg10(3U) / 10U),
                       (unsigned)(arm_axis_deg10(4U) / 10U),
                       (unsigned)(arm_axis_deg10(5U) / 10U));
        text(0, 24, row);
    }

    if (s_page == 0U || s_page == 2U) {
        (void)snprintf(row, sizeof(row), "IMU C%u %uH %uMG",
                       (unsigned)imu_arm_chip(),
                       (unsigned)(imu_arm_rate_x10() / 10U),
                       (unsigned)imu_arm_amag_mg());
        text(0, 36, row);
        (void)snprintf(row, sizeof(row), "IMPACT %lu",
                       (unsigned long)joint_impact_cnt());
        text(0, 48, row);
        return;
    }

    if (s_page == 3U) {
        static const char *const mode_name[4] = { "IDLE", "REMOTE", "ESTOP", "FAULT" };
        uint8_t mode = arm_mode();
        (void)snprintf(row, sizeof(row), "%s E%u", mode_name[mode > 3U ? 0U : mode],
                       (unsigned)(arm_estop_latched() ? 1U : 0U));
        text(0, 12, row);
        (void)snprintf(row, sizeof(row), "Z%c F%c ST%u",
                       joint_zeroed() ? '1' : '0', joint_fault() ? '1' : '0',
                       (unsigned)joint_state(0U));
        text(0, 24, row);
    }
}

/* ======================= 对外 ======================= */

void oled_set_page(uint8_t page) { s_page = (page > 4U) ? 0U : page; }

void oled_set_contrast(uint8_t c)
{
    s_contrast = c;
    if (s_ready != 0U) { oled_cmd1(0x81U); oled_cmd1(c); }
}

void oled_set_invert(uint8_t on)
{
    s_invert = on;
    if (s_ready != 0U) oled_cmd1(on ? 0xA7U : 0xA6U);
}

void oled_show_step(uint32_t now_ms)
{
    static uint32_t last;
    if (s_ready == 0U) {
        oled_init();
        if (s_ready == 0U) return;   /* 没探到屏：静默降级，不阻塞 */
        last = now_ms;
    }
    if ((uint32_t)(now_ms - last) < OLED_REFRESH_MS) return;
    last = now_ms;
    render();
    flush();
}
