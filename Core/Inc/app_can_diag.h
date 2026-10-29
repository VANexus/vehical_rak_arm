#ifndef APP_CAN_DIAG_H
#define APP_CAN_DIAG_H

/*
 * [AI] A3 CAN 板间互通诊断 —— 只加在机械臂板上，用 USART1(PA9/PA10) 把
 * CAN 收帧统计打出来。目的：**不依赖 Jetson、不依赖 CAN 嗅探器**，用一块板
 * 就能判定"两台 C30D 之间 CAN 是否真的通"。
 *
 * 判据（都在 app_can_diag.c 的打印行里）：
 *   收帧计数涨      → 对方（底盘板）的帧到本板了（下行方向通）
 *   txfail 不涨 + ESR 无 EPVF/BOFF → 本板发的帧被对方 ACK 了（上行方向通；
 *                     CAN 的 ACK 由任何"收到且认可该帧"的节点发出，与应用层
 *                     是否消费无关 —— 所以对方只要 CAN 控制器活着就会 ACK）
 *   ESR 的 EPVF/BOFF 或 txfail 涨 → 总线上没人 ACK：查 CANH/CANL 接反、
 *                     没共地、波特率不一致、或同 ID 双发打架
 */

void app_can_diag_task(void *argument);   /* FreeRTOS 任务入口（1 Hz 打印） */

#endif /* APP_CAN_DIAG_H */
