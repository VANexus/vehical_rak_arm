/**
 * @file  app_serial.h — 两路串口（L5 通信，本仓私有）
 *
 *   SER_DEBUG : USART1  PA9/PA10  → 板载 CH9102F → PC 的 USB 串口（本机 COM3）
 *   SER_BT    : USART2  PD5/PD6   → 板载蓝牙模块（WHEELTEC ASCII 协议）
 *
 * 波特率（取自 WHEELTEC 官方 V2.0 源码 `system.c`：uart1_init(115200) / uart2_init(9600)）：
 *   USART1 = 115200 8N1   USART2 = 9600 8N1
 *
 * ⚠️ 接收走**轮询**而非中断：CubeMX 脚本接口设不了 NVIC（实测 `set ip parameters
 *    NVIC USART1_IRQn true` 返回 OK 但不生效，.ioc 里没有条目、it.c 里也没有 handler）。
 *    轮询由 serial_poll() 在任务里排空 RX，配合 1 ms 外循环足以吃下 115200（≈11.5 B/ms）。
 *    要改成中断驱动，需在 CubeMX GUI 里勾 System Core → NVIC → USART1/USART2 global interrupt。
 */
#ifndef APP_SERIAL_H
#define APP_SERIAL_H

#include <stdint.h>

#define SER_DEBUG   0   /* USART1 / CH9102F / PC 的 USB 串口 */
#define SER_BT      1   /* USART2 / 蓝牙 */
#define SER_NPORTS  2

void serial_init(void);

/* 排空两路 RX。放在任务主循环里，每次调用前先跑一遍。 */
void serial_poll(void);

/* 取一个已收到的字节；无数据返回 -1。 */
int  serial_getc(int port);

/* 发送。阻塞到发完（9600 下一字节 ~1ms，勿在高优先级任务里发长串）。 */
void serial_puts(int port, const char *s);
void serial_printf(int port, const char *fmt, ...);

/* 统计（给 OLED / 诊断用） */
uint32_t serial_rx_count(int port);
uint32_t serial_tx_count(int port);

#endif /* APP_SERIAL_H */
