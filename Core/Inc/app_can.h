#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>

/*
 * 机械臂板 CAN 层（v4 臂帧，见 docs/PROTOCOL.md）。
 *
 * 下行：0x010 SAFETY_CMD（e2e，ISR 内校验 CRC8）、
 *       0x110 ARM_CMD_A(J1-J4) / 0x111 ARM_CMD_B(J5,J6,夹爪) —— 50 Hz。
 * 上行：0x210/0x211 SERVO_STATE_A/B（50 Hz）、
 *       0x221 ARM_DIAG（10 Hz）、0x031 ARM_NODE_INFO（1 Hz）。
 *
 * 帧 ID / Data ID / CRC 全部取自生成的 docs/gen/can_protocol.h
 * （单一真源 = docs/protocol-signals.yaml，勿在本文件手写帧表）。
 */

void can_init(void);                     /* 硬件 + 队列 + NVIC（先于调度器调用） */
void can_rx_task_step(void);             /* 阻塞于队列；Task_CAN_Rx 调用 */
void can_tx_task_step(uint32_t now_ms);  /* 1 kHz；Task_CAN_Tx 调用 */

uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init);   /* 协议 CRC8（与底盘板同名同语义） */

/* [AI] A3 CAN 诊断取值（供 app_can_diag.c 打印；不参与任何控制决策） */
uint32_t app_can_rx_total(void);      /* 收到的帧总数（任何 ID） */
uint32_t app_can_rx_of(uint32_t id);   /* 某个标准 ID 的收帧计数 */
uint32_t app_can_tx_ok(void);         /* 入队成功的发帧次数 */
uint32_t app_can_tx_fail(void);       /* 邮箱满/发不出去（= 没人 ACK 的征兆） */
uint32_t app_can_busoff_cnt(void);    /* bus-off 发生次数 */
uint32_t app_can_esr(void);           /* bxCAN ESR 原值（EPVF/BOFF/LEC/TEC/REC） */

#endif /* APP_CAN_H */
