/**
 * @file  app_serial.c — 两路串口的收发（USART1 调试 / USART2 蓝牙）
 *
 * 外设本身完全由 CubeMX 配置（波特率、引脚 AF、时钟）—— 本文件只负责"怎么读怎么写"。
 * 接收用轮询排空（原因见 app_serial.h 的说明）。
 */
#include "app_serial.h"

#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RXBUF_SZ    256U        /* 2 的幂，环形缓冲 */
#define RXBUF_MASK  (RXBUF_SZ - 1U)

typedef struct {
    UART_HandleTypeDef *huart;
    volatile uint8_t    buf[RXBUF_SZ];
    volatile uint16_t   head;   /* 写指针（poll 里写） */
    volatile uint16_t   tail;   /* 读指针（任务里读） */
    volatile uint32_t   rx_n;
    volatile uint32_t   tx_n;
} serial_port_t;

static serial_port_t s_port[SER_NPORTS];

void serial_init(void)
{
    s_port[SER_DEBUG].huart = &huart1;
    s_port[SER_BT].huart    = &huart2;
    for (int i = 0; i < SER_NPORTS; ++i) {
        s_port[i].head = s_port[i].tail = 0U;
        s_port[i].rx_n = s_port[i].tx_n = 0U;
    }
}

static void rx_push(serial_port_t *p, uint8_t c)
{
    uint16_t next = (uint16_t)((p->head + 1U) & RXBUF_MASK);
    if (next == p->tail) return;        /* 满则丢弃，不阻塞 */
    p->buf[p->head] = c;
    p->head = next;
    p->rx_n++;
}

void serial_poll(void)
{
    for (int i = 0; i < SER_NPORTS; ++i) {
        serial_port_t *p = &s_port[i];
        uint8_t c;
        /* HAL_UART_Receive(..., Timeout=0) 每次读 1 字节、无数据立刻返回。
         * 紧循环排空 —— 115200 下 1 ms 到 ~11.5 字节，循环远快于此。 */
        while (HAL_UART_Receive(p->huart, &c, 1U, 0U) == HAL_OK) {
            rx_push(p, c);
        }
    }
}

int serial_getc(int port)
{
    if (port < 0 || port >= SER_NPORTS) return -1;
    serial_port_t *p = &s_port[port];
    if (p->tail == p->head) return -1;
    uint8_t c = p->buf[p->tail];
    p->tail = (uint16_t)((p->tail + 1U) & RXBUF_MASK);
    return (int)c;
}

void serial_puts(int port, const char *s)
{
    if (port < 0 || port >= SER_NPORTS || !s) return;
    serial_port_t *p = &s_port[port];
    size_t n = strlen(s);
    if (n == 0U) return;
    /* 阻塞发送；9600 下每字节约 1 ms，长串会占住调用它的任务 —— 只在低优先级任务里发。 */
    if (HAL_UART_Transmit(p->huart, (uint8_t *)s, (uint16_t)n, 1000U) == HAL_OK) {
        p->tx_n += (uint32_t)n;
    }
}

void serial_printf(int port, const char *fmt, ...)
{
    char buf[80];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    serial_puts(port, buf);
}

uint32_t serial_rx_count(int port)
{
    if (port < 0 || port >= SER_NPORTS) return 0U;
    return s_port[port].rx_n;
}

uint32_t serial_tx_count(int port)
{
    if (port < 0 || port >= SER_NPORTS) return 0U;
    return s_port[port].tx_n;
}
