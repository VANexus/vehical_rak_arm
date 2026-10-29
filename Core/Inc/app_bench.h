#ifndef APP_BENCH_H
#define APP_BENCH_H

#include <stdint.h>

/*
 * app_bench.h — [AI] A6 台架串口控制台（USART1，USB-CH9102F 那个 Type-C 口）
 *
 * 【为什么有这个模块（2026-10-29 台架）】
 *   首次上板要摸清 J1/J2 的**方向、行程、起转占空比**，而 A5 起本板已无 OLED；
 *   靠 Jetson 发 ARM_CMD 又不适合"手边一点点试"（还要先归零、还有 100 ms deadman）。
 *   所以给一个**本地点动控制台**：USB 一插、串口终端一开，按键点动，撞到就停。
 *
 * 【命令（单字符，无需回车）】
 *   1 / 2 / 3 / 4  J1+ / J1- / J2+ / J2-   （每次点动 1.2 s，到点自动刹）
 *   0              取消点动（立即刹）
 *   + / -          点动占空比 ±5%（5%..60%，默认 25%）
 *   p              打印一次 J1/J2 状态
 *   m              连续监视开关（5 Hz）
 *   k              软件置零（等同长按 PE0 >=1.5 s）
 *   c              清故障（CLEAR_FAULT 同语义）
 *   ? / h          帮助
 *
 * 【安全语义】
 *   - 点动是开环恒占空比 + 碰撞即停（app_joint），**不经过位置环、不需要先归零**；
 *   - 每条命令都在 1.2 s 后自动刹，串口拔掉/终端关掉不会留下满转的电机；
 *   - 急停锁存 / 该轴故障时 `joint_jog()` 拒绝启动；
 *   - 点动期间 app_arm 会丢弃来自 Jetson 的 J1/J2 目标（两套控制不抢同一个轴）。
 *
 * 【打印】
 *   `bench_write()` 是**本板唯一**的串口写入口（带互斥），USART1 与 USART3 同时发
 *   （C30D 两个 Type-C 都是 CH9102F，台架上插哪个都能看到）。CAN 诊断打印也走它。
 */

/* 调度器起来后调用一次：建队列/互斥量 + 启动 USART1 单字节中断接收 + 打横幅 */
void bench_init(void);

/* 控制台线程主体（由 freertos.c 的 task_bench 调用，不返回） */
void bench_task(void *argument);

/* 线程安全地写两路串口（任务上下文；ISR 里不要调） */
void bench_write(const char *buf, uint16_t len);

/* 格式化打印（单行 <=160 字节，超出截断） */
void bench_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* APP_BENCH_H */
