#ifndef APP_CANOPEN_H
#define APP_CANOPEN_H

#include <stdint.h>

/**
 * app_canopen.h —— CANopen v5 适配层对外接口（**唯一**的 CAN 链路实现）
 *
 * 2026-10-30：v4（`app_can.c` + 手写帧表）与 `APP_USE_CANOPEN` 回滚开关已**整体删除**。
 * 下列符号是**历史接口名**，保留只为让 `main.c` / `freertos.c` / `app_bench.c` /
 * `app_can_diag.c` 的调用点不必改写；语义现在全由 CANopenNode 承担。
 * 注意：本板 `app_can_*` 里的计数器/ESR 目前是**降级实现**（返回 0 或直读寄存器），
 * 只有在诊断面板里才需要精确值——真要精确统计请在 CANopen 侧接 CO_EMCY / bus-off 回调。
 */

/* 硬件 + 协议栈初始化（先于调度器调用）。 */
void can_init(void);

/* 接收侧：跑 CANopen 后台处理（内部自带 1 ms 让出）。由 Task_CAN_Rx 调用。 */
void can_rx_task_step(void);

/* 1 ms 节拍：驱动 CANopenNode 的 SYNC / RPDO / TPDO 处理。 */
void can_tx_task_step(uint32_t now_ms);

/* CRC8（poly 0x07）。CANopen 自身的 CRC 在栈内；本函数只服务 USB v2 串口通道。 */
uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init);

/* ---- 诊断计数（app_can_diag / app_bench 用；保持与 v4 同名同签名） ---- */
uint32_t app_can_rx_total(void);
uint32_t app_can_rx_of(uint32_t id);
uint32_t app_can_tx_ok(void);
uint32_t app_can_tx_fail(void);
uint32_t app_can_busoff_cnt(void);
uint32_t app_can_esr(void);
uint8_t app_can_vehicle_moving(void);

/* 50 Hz 调用：把 CANopen RPDO 命令落到 app_arm，再把反馈写回 OD。
 * 由 task_arm_ctrl 调用。 */
void app_canopen_step(uint32_t now_ms);

#endif /* APP_CANOPEN_H */
