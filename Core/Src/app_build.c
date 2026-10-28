/*
 * app_build.c — 固件构建标识（单点定义）
 *
 * A2（2026-10-28）：J1(腰)/J2(大臂) 从舵机换成"轮子同款"MG513P30 增量编码器电机，
 *   新增 app_joint（25 kHz 采样 + 1 kHz 定点位置 PI + 软件归零 + 急停刹车）；
 *   J3..J6 仍是舵机。CAN 0x110/111 与 0x210/211 的 ID 与字节布局**一字未动**
 *   （Jetson 侧零改动）；CLEAR_FAULT(0x010 cmd=6) 现在会复位 J1/J2 位置环故障。
 *   改动前（A1）：机械臂板首版——6 路舵机 PWM + CAN v4 臂帧。
 */
#include "app_build.h"

const char build_tag[] = "A2";
