#ifndef APP_ARM_H
#define APP_ARM_H

#include <stdint.h>

/*
 * 机械臂板应用层（L3 语义层）。
 *
 * 职责边界（2026-10-28 双板架构）：
 *   - 底盘板 vehical_rak_motion：4 轮 25 kHz 闭环 + IMU。
 *   - 机械臂板 vehical_rak_arm（本板）：6 路舵机 PWM（50 Hz）+ CAN 臂帧。
 *   - Jetson 负责 IK/轨迹规划；本板只做"指令角度 → 舵机脉宽"的执行与本地安全兜底。
 *
 * 轴序单点定义（协议语义见 docs/PROTOCOL.md v4）：
 *   axis[0..5] = J1(腰) J2(大臂) J3(肘) J4(小臂) J5(腕俯仰) J6(腕旋转)
 *   舵机通道反向：J1=ch5 J2=ch4 J3=ch3 J4=ch2 J5=ch1 J6=ch0（PC6..PC9 + PB14/PB15）
 *   夹爪（协议第 7 轴 GRIP）**本板无硬件通道**，只回显不落地。
 *   位置语义：舵机输出轴角 0..270° 满量程 ↔ 500..2500 µs。
 *   ⚠️ J1/J6 实测为连续旋转舵机 —— 其角度值按"速度偏置"解释（中位=停转），
 *   这是上位机标定层的约定，固件按线性映射照做（与迁移前底盘板行为一致）。
 */

#define ARM_AXIS_COUNT        6U
#define ARM_PULSE_MIN_US      500U
#define ARM_PULSE_MAX_US      2500U
#define ARM_PULSE_MID_US      1500U
#define ARM_DEG10_FULL_SCALE  2700U   /* 270.0°（0.1° 单位） */
#define ARM_CMD_DEADMAN_MS    100U    /* v4 deadman：ARM 指令断流超时 */

/* 上电初始化：舵机输出 Home 安全位（500/2166/2500/500/500/500 µs）+ 状态复位 */
void arm_init(void);

/* 落地一组臂指令：deg10[] = 7 轴（0.1° 单位，J1..J6 + 夹爪占位），
   valid_mask 第 i 位=1 才更新该轴；无效轴回 1500 µs 中位。
   急停锁存期间**不落地**（舵机保持当前位置），并累计异常计数。 */
void arm_command_deg10(const uint16_t deg10[7], uint8_t valid_mask);

/* 急停锁存 / 清除（CAN SAFETY 0x010 快速路径调用） */
void arm_estop_latch(void);
void arm_estop_clear(void);
uint8_t arm_estop_latched(void);

/* 远程门（SAFETY START/STOP） */
void arm_set_remote(uint8_t on);
uint8_t arm_remote(void);

/* 指令流活性（deadman） */
void arm_note_cmd_rx(uint32_t now_ms);
uint8_t arm_cmd_stale(uint32_t now_ms);

/* 状态位图（v4 state_bits）与模式机（mode_enum） */
uint16_t arm_state_bitmap(uint8_t busoff_now, uint32_t now_ms);
uint8_t arm_mode(void);

/* 舵机指令异常计数（饱和 255），进 ARM_DIAG */
uint8_t arm_servo_errcnt(void);

/* 单轴实际脉宽 → 0.1°（脉宽→角：(us-500)×27/20），用于 SERVO_STATE 遥测 */
uint16_t arm_axis_deg10(unsigned axis);

/* 复位原因（'B'/'I'/'S'/'P'/'-'），OLED 显示用 */
uint8_t arm_reset_cause(void);

/* 运行秒数（OLED 显示） */
uint32_t arm_uptime_s(uint32_t now_ms);

#endif /* APP_ARM_H */
