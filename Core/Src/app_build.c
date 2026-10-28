/*
 * app_build.c — 固件构建标识（单点定义）
 *
 * A1（2026-10-28）：机械臂板首版——6 路舵机 PWM + CAN v4 臂帧
 * （0x110/111 ARM_CMD 下行、0x210/211 SERVO_STATE 上行、0x031/0x221 板级遥测）。
 */
#include "app_build.h"

const char build_tag[] = "A1";
