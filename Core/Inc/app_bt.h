/**
 * @file  app_bt.h — 蓝牙应用层（WHEELTEC ASCII 协议）
 *
 * 协议出自资料包 `2.使用与开发手册/2.单片机与APP通信协议.pdf`（WHEELTEC 官方，V1.0）：
 *
 *   APP → 单片机（单字符 ASCII）
 *     A/B/C/D/E/F/G/H = 前进/右上/右转/右下/后退/左下/左转/左上
 *     X = 加速   Y = 减速   Z = 停车
 *     I/J/K = 切到 重力/摇杆/按键 界面
 *     a..o  = 15 个自定义按键
 *     {n：v} / {Q：W} / {Q：P} / {#...} = 参数调试
 *
 *   单片机 → APP（printf 形式）
 *     状态区 {A%d:%d:%d:%d}$   左轮速度:右轮速度:电池电压:倾角
 *     波形区 {B%d:%d:%d:%d:%d}$
 *     调试区 {C%d:%d:%d:%d:%d:%d:%d:%d:%d}$
 *
 * 本文件在 edge_ai 上只做**链路自证**：解析收到的指令、回显、周期性上报状态帧。
 * 真正的业务动作（推理触发等）留给后续子项目。
 */
#ifndef APP_BT_H
#define APP_BT_H

#include <stdint.h>

void bt_init(void);

/** 在任务主循环里调。内部限流，不必自己算节拍。 */
void bt_step(uint32_t now_ms);

/** 最近一次收到的遥控指令（0 = 无），供 OLED / 诊断显示。 */
char bt_last_cmd(void);

/** 收到的字节总数（两路合计）。 */
uint32_t bt_rx_total(void);

#endif /* APP_BT_H */
