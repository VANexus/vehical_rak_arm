/**
 * @file  app_bt.c — 蓝牙应用层（WHEELTEC ASCII 协议）
 *
 * 行为：
 *   1) 两路串口收到的字节都排空；DEBUG 路回显（终端自证），BT 路解析指令
 *   2) 每 500 ms 往 BT 发一帧状态 {A%d:%d:%d:%d}$（WHEELTEC APP 状态区格式）
 *   3) 收到 '?' 或 'h' 时回一段自检文本（两路都回）
 *
 * 实测口径：本仓无电机、无 IMU，所以状态帧里的数值是**占位**（速度 0、电压 0、倾角 0），
 * 目的只是让 APP 的状态区有数据可显示、证明链路通。真实数据要等 CAN / 电池 ADC 子项目。
 */
#include "app_bt.h"

#include "app_build.h"
#include "app_serial.h"
#include "app_tft.h"
#include "app_ui.h"

#include <stdio.h>

#define BT_REPORT_MS    500U

static char     s_last_cmd;
static uint32_t s_rx_total;
static uint32_t s_last_report;

/* WHEELTEC 遥控指令表（ASCII → 含义） */
static const char *cmd_name(char c)
{
    switch (c) {
    case 'A': return "FWD";
    case 'B': return "UP-RIGHT";
    case 'C': return "RIGHT";
    case 'D': return "DN-RIGHT";
    case 'E': return "BACK";
    case 'F': return "DN-LEFT";
    case 'G': return "LEFT";
    case 'H': return "UP-LEFT";
    case 'X': return "FASTER";
    case 'Y': return "SLOWER";
    case 'Z': return "STOP";
    case 'I': return "UI-NEXT";
    case 'J': return "UI-PREV";
    case 'K': return "UI-HOME";
    default:  return NULL;
    }
}

static void send_banner(int port)
{
    serial_printf(port, "\r\n=== vehical_rak_edge_ai %s ===\r\n", build_tag);
    serial_printf(port, "USART1(debug)=115200  USART2(BT)=9600\r\n");
    serial_printf(port, "keys: A-H dir, X/Y speed, Z stop, I/J/K ui, ?=this\r\n");
    serial_printf(port, "rx=%lu\r\n", (unsigned long)s_rx_total);
    serial_printf(port, "tft: fill=%lu cy flush=%lu ui=%lu cy\r\n",
                  (unsigned long)tft_fill_cycles(),
                  (unsigned long)ui_flush_count(),
                  (unsigned long)ui_last_handler_cycles());
}

/* 处理一个收到的字节。port = 它从哪来。 */
static void handle_byte(int port, char c)
{
    s_rx_total++;

    /* 调试口回显，方便终端确认链路 */
    if (port == SER_DEBUG) {
        if (c == '\r' || c == '\n') {
            serial_puts(SER_DEBUG, "\r\n");
            return;
        }
        if (c == '?' || c == 'h') { send_banner(SER_DEBUG); return; }
        serial_printf(SER_DEBUG, "[%02X '%c'] ", (unsigned)(unsigned char)c, (c >= 32 && c < 127) ? c : '.');
        return;
    }

    /* 蓝牙口：按 WHEELTEC 协议解析 */
    if (c == '?' || c == 'h') { send_banner(SER_BT); return; }

    const char *nm = cmd_name(c);
    if (nm) {
        s_last_cmd = c;
        serial_printf(SER_BT, "OK %c %s\r\n", c, nm);
        serial_printf(SER_DEBUG, "BT cmd: %c %s\r\n", c, nm);
        /* I/J/K → 页面切换（LVGL 只在 defaultTask 上下文跑，这里只发意图） */
        if (c == 'I')      ui_page_next(1);
        else if (c == 'J') ui_page_next(-1);
        else if (c == 'K') ui_page_next(0);
        return;
    }
    /* 自定义按键 a..o */
    if (c >= 'a' && c <= 'o') {
        s_last_cmd = c;
        serial_printf(SER_BT, "KEY %c\r\n", c);
        serial_printf(SER_DEBUG, "BT key: %c\r\n", c);
        return;
    }
    /* 其它（含 {n：v} 参数帧）只计数，不回 */
}

void bt_init(void)
{
    s_last_cmd = 0;
    s_rx_total = 0U;
    s_last_report = 0U;
    serial_init();
}

void bt_step(uint32_t now_ms)
{
    serial_poll();                      /* 先把 RX 排空 */

    int c;
    while ((c = serial_getc(SER_DEBUG)) >= 0) handle_byte(SER_DEBUG, (char)c);
    while ((c = serial_getc(SER_BT))    >= 0) handle_byte(SER_BT,    (char)c);

    /* 周期上报 WHEELTEC 状态帧，让 APP 状态区有数据 */
    if ((uint32_t)(now_ms - s_last_report) >= BT_REPORT_MS) {
        s_last_report = now_ms;
        /* {A左轮速度:右轮速度:电池电压:倾角}$ —— 本仓无电机无 IMU，数值为占位 */
        serial_printf(SER_BT, "{A%d:%d:%d:%d}$\r\n", 0, 0, 0, 0);
    }
}

char bt_last_cmd(void)      { return s_last_cmd; }
uint32_t bt_rx_total(void)  { return s_rx_total; }
