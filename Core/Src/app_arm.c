/*
 * app_arm.c — 机械臂应用层：指令角度 → 舵机脉宽 + 本地安全兜底
 *
 * 【为什么单独一层】
 *   CAN 层只搬字节，舵机层只写 CCR，本层负责两者之间的语义：
 *   角度→脉宽换算、轴↔通道映射、急停锁存、指令断流判定。
 *   换算常数与轴序都单点定义在 app_arm.h，不在多处复制。
 *
 * 【安全语义（docs/PROTOCOL.md v4 §6.2，2026-10-28 新增）】
 *   - 急停锁存期间收到的 ARM 指令**一律不落地**（舵机保持当前位置）。
 *   - 指令断流 100 ms → STATE.bit3 cmd_stale=1；舵机本身无目标概念，
 *     硬件 PWM 自续波保持最后一帧位置，固件不需要额外动作。
 */
#include "app_arm.h"

#include "app_buzz.h"
#include "app_imu.h"
#include "app_joint.h"
#include "app_servo.h"
#include "main.h"
#include "stm32f4xx.h"

static volatile uint8_t  s_estop;
static volatile uint8_t  s_remote;
static volatile uint8_t  s_errcnt;
static volatile uint32_t s_last_cmd_ms;
static volatile uint8_t  s_cmd_seen;
static uint32_t          s_boot_ms;
static uint8_t           s_reset_cause = '-';

/* [AI] A17（2026-10-30 用户定稿，旧 ch3..ch0 映射作废）：
   J3..J6 → 舵机通道 ch0..ch3 = **PC6/PC7/PC8/PC9**（TIM8_CH1..CH4）。
   四路全是 **PWM 位置舵机**——J1/J6 曾按"连续旋转舵机（角度值=速度偏置）"解释的
   约定随之**整体作废**（J1 已是编码器电机，J6 现在也是位置舵机）。 */
static const unsigned k_servo_ch[ARM_AXIS_COUNT - ARM_JOINT_AXIS_COUNT] = { 0U, 1U, 2U, 3U };

void arm_init(void)
{
    /* 复位原因（HAL 宏，不直写 RCC->CSR——R45 起的外设纪律） */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) s_reset_cause = (uint8_t)'B';
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) s_reset_cause = (uint8_t)'I';
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) s_reset_cause = (uint8_t)'S';
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) s_reset_cause = (uint8_t)'P';
    else s_reset_cause = (uint8_t)'-';
    __HAL_RCC_CLEAR_RESET_FLAGS();

    s_estop = 0U;
    s_remote = 0U;
    s_errcnt = 0U;
    s_cmd_seen = 0U;
    s_last_cmd_ms = HAL_GetTick();
    s_boot_ms = HAL_GetTick();

    /* 舵机输出 Home 安全位（app_servo.c 单点定义 500/2166/2500/500/500/500 µs） */
    servo_init();

    /* [AI] A2: J1/J2 增量编码器电机闭环（app_joint）。
       上电只刹车、不驱动；零点未定义（未归零前 J1/J2 不接受指令）。 */
    joint_init();

    /* [AI] A7: 板载 IMU（PB10/PB11 软 I2C，双芯片探测）。
       只做碰撞确认/重力方向/陀螺互校；找不到芯片不影响其它功能。 */
    (void)imu_arm_init();

    /* [AI] A8: 蜂鸣器（PA8）—— 上电自检短响一声 = 固件起来了、任务都建住了 */
    buzz_init();
    buzz_alert(BUZZ_BOOT);
}

void arm_command_deg10(const uint16_t deg10[7], uint8_t valid_mask)
{
    if (deg10 == NULL) {
        return;
    }
    if (s_estop != 0U) {
        /* 急停锁存 → 不落地；只记异常计数（饱和 255） */
        if (s_errcnt < 255U) {
            s_errcnt++;
        }
        return;
    }
    for (unsigned axis = 0; axis < ARM_AXIS_COUNT; ++axis) {
        if (axis < ARM_JOINT_AXIS_COUNT) {
            /* [AI] A2: J1/J2 = 增量编码器电机位置闭环（app_joint）。
               仅在**远程门开**时接受目标（否则保持当前姿态）；
               未归零 / 急停 / 故障时 app_joint 内部自行刹车，本层无需再判。
               [AI] A6: 该轴正在台架点动时**丢弃**远程目标 —— 否则位置环与点动
               会抢同一个轴（点动结束后目标已重同步到当前位置，需重新发指令）。 */
            if ((valid_mask & (1U << axis)) != 0U && s_remote != 0U &&
                joint_jog_axis_active((uint8_t)axis) == 0U) {
                joint_set_target_deg10(axis, deg10[axis]);
            }
            continue;
        }
        unsigned us;
        if ((valid_mask & (1U << axis)) != 0U) {
            /* 0.1° → 脉宽：us = 500 + deg10×2000/2700 = 500 + deg10×20/27 */
            uint32_t raw = deg10[axis];
            if (raw > ARM_DEG10_FULL_SCALE) {
                raw = ARM_DEG10_FULL_SCALE;
            }
            us = ARM_PULSE_MIN_US + (raw * 20U) / 27U;
        } else {
            us = ARM_PULSE_MID_US;   /* 无效轴回中位（与迁移前底盘板行为一致） */
        }
        servo_set_pulse(k_servo_ch[axis - ARM_JOINT_AXIS_COUNT], us);
    }
}

void arm_estop_latch(void)
{
    s_estop = 1U;
    s_remote = 0U;
    buzz_alert(BUZZ_FAULT);   /* [AI] A8: 急停锁存长响 */
    /* 舵机（J3..J6）不动 PWM：保持当前位置（急停语义 = 冻结，不是归零）。
       [AI] A2: J1/J2 是电机，必须**主动断电**——置门后下一个 25 kHz 节拍
       （≤40 µs）立即短路制动，本函数可在 CAN ISR 上下文调用。 */
    joint_set_estop(1U);
}

void arm_estop_clear(void)
{
    s_estop = 0U;
    joint_set_estop(0U);   /* [AI] A2: 清门后 J1/J2 恢复位置环（若已归零） */
}

uint8_t arm_estop_latched(void) { return s_estop; }

void arm_set_remote(uint8_t on) { s_remote = (on != 0U) ? 1U : 0U; }

uint8_t arm_remote(void) { return s_remote; }

void arm_note_cmd_rx(uint32_t now_ms)
{
    s_last_cmd_ms = now_ms;
    s_cmd_seen = 1U;
}

uint8_t arm_cmd_stale(uint32_t now_ms)
{
    if (s_cmd_seen == 0U) {
        return 0U;
    }
    return ((uint32_t)(now_ms - s_last_cmd_ms) > ARM_CMD_DEADMAN_MS) ? 1U : 0U;
}

uint16_t arm_state_bitmap(uint8_t busoff_now, uint32_t now_ms)
{
    uint16_t s = 0U;
    if (s_estop != 0U)    s |= 1U << 0;   /* estop_latched */
    if (s_remote != 0U)   s |= 1U << 1;   /* remote_running */
    if (arm_cmd_stale(now_ms) != 0U) s |= 1U << 3;   /* cmd_stale */
    if (busoff_now != 0U) s |= 1U << 5;   /* can_recovering */
    /* [AI] A2: bit2 fault = J1/J2 位置环故障（跟随误差 >30° 或软限位越界）。
       未归零**不算** fault（是"未就绪"），OLED 单独显示 NOTZRO。
       bit4 bat_uv：本板未接电池 ADC 采样，恒 0 */
    if (joint_fault() != 0U) s |= 1U << 2;
    return s;
}

uint8_t arm_mode(void)
{
    if (s_estop != 0U) {
        return 2U;   /* ESTOP */
    }
    return (s_remote != 0U) ? 1U : 0U;   /* REMOTE / IDLE */
}

uint8_t arm_servo_errcnt(void) { return s_errcnt; }

uint16_t arm_axis_deg10(unsigned axis)
{
    if (axis >= ARM_AXIS_COUNT) {
        return 0U;
    }
    /* [AI] A2: J1/J2 回读来自编码器（app_joint），J3..J6 仍来自舵机 CCR */
    if (axis < ARM_JOINT_AXIS_COUNT) {
        return joint_actual_deg10(axis);
    }
    unsigned us = servo_get_pulse(k_servo_ch[axis - ARM_JOINT_AXIS_COUNT]);
    if (us < ARM_PULSE_MIN_US) {
        us = ARM_PULSE_MIN_US;
    }
    /* 脉宽→角：deg10 = (us-500)×27/20 */
    return (uint16_t)(((uint32_t)(us - ARM_PULSE_MIN_US) * 27U) / 20U);
}

uint8_t arm_reset_cause(void) { return s_reset_cause; }

uint32_t arm_uptime_s(uint32_t now_ms)
{
    return (uint32_t)(now_ms - s_boot_ms) / 1000U;
}
