/*
 * app_build.c — 固件构建标识（单点定义）
 *
 * A4（2026-10-29）：诊断打印同时走 USART1 与 USART3（C30D 有两个 CH9102F
 *   USB 口，台架插的是哪一个不确定，两路都打）。
 * A3（2026-10-29）：新增 CAN 板间互通诊断（app_can_diag.c）—— 1 Hz 经 USART1
 *   打印收帧/发帧/ESR 统计，让"两块 C30D 之间 CAN 通不通"不依赖 Jetson 就能判定。
 *   只加统计与打印，**不改任何帧 ID、字节布局与控制行为**。
 * A2（2026-10-28）：J1(腰)/J2(大臂) 从舵机换成"轮子同款"MG513P30 增量编码器电机，
 *   新增 app_joint（25 kHz 采样 + 1 kHz 定点位置 PI + 软件归零 + 急停刹车）；
 *   J3..J6 仍是舵机。CAN 0x110/111 与 0x210/211 的 ID 与字节布局**一字未动**
 *   （Jetson 侧零改动）；CLEAR_FAULT(0x010 cmd=6) 现在会复位 J1/J2 位置环故障。
 *   改动前（A1）：机械臂板首版——6 路舵机 PWM + CAN v4 臂帧。
 */
#include "app_build.h"

const char build_tag[] = "A4";
