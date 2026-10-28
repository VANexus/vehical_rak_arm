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

/* 轴 → 舵机通道（装臂实测接线）：J1=ch5 J2=ch4 J3=ch3 J4=ch2 J5=ch1 J6=ch0 */
static const unsigned k_axis_to_ch[ARM_AXIS_COUNT] = { 5U, 4U, 3U, 2U, 1U, 0U };

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
        servo_set_pulse(k_axis_to_ch[axis], us);
    }
}

void arm_estop_latch(void)
{
    s_estop = 1U;
    s_remote = 0U;
    /* 不动 PWM：舵机保持当前位置（急停语义 = 冻结，不是归零） */
}

void arm_estop_clear(void)
{
    s_estop = 0U;
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
    /* bit2 fault：臂板当前无故障码源（无电流采样/编码器闭环），恒 0
       bit4 bat_uv：本板未接电池 ADC 采样，恒 0 */
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
    unsigned us;
    if (axis >= ARM_AXIS_COUNT) {
        return 0U;
    }
    us = servo_get_pulse(k_axis_to_ch[axis]);
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
