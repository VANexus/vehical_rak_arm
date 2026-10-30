#ifndef APP_ARM_H
#define APP_ARM_H

#include <stdint.h>

/*
 * 机械臂板应用层（L3 语义层）。
 *
 * 职责边界（2026-10-28 双板架构）：
 *   - 底盘板 vehical_rak_motion：4 轮 25 kHz 闭环 + IMU。
 *   - 机械臂板 vehical_rak_arm（本板）：J1/J2 增量编码器电机位置闭环
 *     + J3..J6 舵机 PWM（50 Hz）+ CAN 臂帧。
 *   - Jetson 负责 IK/轨迹规划；本板只做"指令角度 → 执行机构"的执行与本地安全兜底。
 *
 * 轴序单点定义（协议语义见 docs/CANOPEN-V5.md）：
 *   axis[0..5] = J1(腰) J2(大臂) J3(肘) J4(小臂) J5(腕俯仰) J6(腕旋转)
 *   **[AI] A17 定稿（2026-10-30 用户确认）**：
 *   - J1/J2 = "轮子同款"MG513P30 增量编码器电机（AT8236 双输入 + 正交编码器），
 *     走 app_joint 的位置闭环；协议字段**逐轴上限**：J1 0..360.0°（360° 有限位
 *     旋转，手动归零）、J2 0..360.0°（零点选在行程一端，行程实测后收软限位）；
 *   - J3..J6 = 四路 **PWM 位置舵机**（不再有任何"连续旋转/速度偏置"解释）：
 *     J3=PC6 J4=PC7 J5=PC8 J6=PC9（TIM8_CH1..CH4），0..270° 满量程 ↔ 500..2500 µs；
 *   - 夹爪（协议第 7 轴 GRIP）**本板无硬件通道**，只回显不落地。
 */

#define ARM_AXIS_COUNT        6U
#define ARM_JOINT_AXIS_COUNT  2U   /* [AI] A2: axis[0..1] = J1/J2（app_joint 电机轴） */
#define ARM_PULSE_MIN_US      500U
#define ARM_PULSE_MAX_US      2500U
#define ARM_PULSE_MID_US      1500U
#define ARM_DEG10_FULL_SCALE  2700U   /* 270.0°（0.1° 单位） */
#define ARM_CMD_DEADMAN_MS    100U    /* v4 deadman：ARM 指令断流超时 */

/* 上电初始化：J3..J6 舵机输出 Home 安全位（500/2500/500/500 µs）+
   J1/J2 电机刹车（joint_init，不驱动）+ 状态复位。 */
void arm_init(void);

/* 落地一组臂指令：deg10[] = 7 轴（0.1° 单位，J1..J6 + 夹爪占位），
   valid_mask 第 i 位=1 才更新该轴。
   - J1/J2（电机）：仅**远程门开**时接受目标（否则保持当前姿态）；
     未归零/故障时 app_joint 内部刹车。
   - J3..J6（舵机）：按 0.1°→脉宽映射；无效轴回 1500 µs 中位。
   急停锁存期间**不落地**（J3..J6 保持当前位置，J1/J2 由 joint_set_estop 刹车），
   并累计异常计数。 */
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

/* 单轴实际角 0.1°（J1/J2 = 编码器回读，J3..J6 = 脉宽→角 (us-500)×27/20），
   用于 SERVO_STATE 遥测 */
uint16_t arm_axis_deg10(unsigned axis);

/* 复位原因（'B'/'I'/'S'/'P'/'-'），OLED 显示用 */
uint8_t arm_reset_cause(void);

/* 运行秒数（OLED 显示） */
uint32_t arm_uptime_s(uint32_t now_ms);

#endif /* APP_ARM_H */
