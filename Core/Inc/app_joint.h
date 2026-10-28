#ifndef APP_JOINT_H
#define APP_JOINT_H

#include <stdint.h>

/*
 * app_joint.h — J1(腰) / J2(大臂) 增量编码器电机位置闭环 + 软件归零（机械臂板）
 *
 * 【为什么有这个模块（2026-10-28 改造）】
 *   机械臂的**底座(J1)**与**大臂(J2)**电机从舵机换成了"轮子同款"MG513P30
 *   增量编码器电机（AT8236 双输入驱动 + 正交编码器）。舵机只认脉宽、且连续旋转型
 *   根本没有角度反馈；电机 + 编码器则可以做**位置闭环**，也能**软件归零**。
 *   本模块是这两轴唯一的实时写者：25 kHz 采样编码器、1 kHz 跑定点位置 PI、
 *   按状态机放行/刹车。J3..J6 仍是舵机（app_servo），不归本模块管。
 *
 * 【协议零改动】CAN 0x110/0x111 ARM_CMD（0.1° 目标）与 0x210/0x211 SERVO_STATE
 *   （0.1° 回读）的 ID 与字节布局一字未动 —— 只是 J1/J2 两个字段的执行机构从
 *   舵机换成了电机，Jetson 侧零改动。
 *
 * 【事实分级纪律（AGENTS.md）】
 *   本模块里标 `[AI] 待实测` 的常数（电机口/编码器口映射、出力方向符号、位置环增益、
 *   软限位角、静摩擦起步占空比）**都是待上板确认的初值**，不得当既定事实引用。
 *   首版上板流程见仓内 README §7「J1/J2 电机轴首次上板」。
 */

#define JOINT_COUNT 2U

#define JOINT_AXIS_J1 0U   /* 腰（底座） */
#define JOINT_AXIS_J2 1U   /* 大臂 */

/* 关节状态（OLED / 诊断用） */
typedef enum {
    JOINT_ST_BRAKE = 0,   /* 未归零 / 急停 / 故障 → 刹车输出（无保持力） */
    JOINT_ST_HOLD,        /* 已归零且在位 → 位置环顶住当前姿态 */
    JOINT_ST_MOVE,        /* 已归零且有跟随误差 → 位置环正在出力 */
    JOINT_ST_FAULT        /* 跟随误差越界 / 软限位越界（需置零或 CLEAR_FAULT 复位） */
} joint_state_t;

/* 上电初始化：刹车优先的 PWM 启停 + 编码器启动 + 状态复位。
   上电**不驱动电机**（AGENTS.md §5）；零点默认未定义（见 joint_zeroed）。 */
void joint_init(void);

/* 25 kHz 中断宿主：**仅供 stm32f4xx_it.c 的 TIM6_DAC_IRQHandler 调用**（自检/清 UIF）。 */
void joint_isr(void);

/* 调度器运行后调用一次：使能 DWT 周期计数 + 启动 TIM6 25 kHz 中断 */
void joint_start_isr(void);
uint8_t joint_isr_ready(void);

/* 急停门（app_arm 的急停锁存/清除调用；可在 CAN ISR 上下文调用）。
   置 1 → 下一个 25 kHz 节拍（≤40 µs）立即刹车并复位调节器。 */
void joint_set_estop(uint8_t on);

/* 软件归零：把 J1/J2 **当前**编码器位置定义为 0°，目标同时置为 0°。
   请求由 25 kHz ISR 落地（任务上下文调用安全）。
   规程：人工把两轴推到机械基准位 → 长按板上用户键 PE0 ≥1.5 s（或调本函数）。 */
void joint_set_zero(void);
uint8_t joint_zeroed(void);

/* 故障（位图：bit0=J1, bit1=J2）。故障轴刹车，直到置零或 joint_fault_clear()。
   触发条件：跟随误差 >30° 或超出软限位 + 容差（防飞车）。 */
uint8_t joint_fault(void);
void joint_fault_clear(void);   /* 请求；ISR 复检条件消失后才真正清位 */

/* 目标角（0.1°，相对零点）；超出软限位会被钳位。未收到目标时保持上一次目标。 */
void joint_set_target_deg10(unsigned axis, uint16_t deg10);

/* 反馈（0.1°，相对零点；由编码器累计计数换算，分辨率 0.005859°/计数） */
uint16_t joint_actual_deg10(unsigned axis);
int32_t  joint_pos_counts(unsigned axis);   /* 原始累计计数（相对零点） */
int32_t  joint_err_counts(unsigned axis);   /* 目标 - 实际（counts） */

uint8_t joint_state(unsigned axis);         /* joint_state_t */
uint8_t joint_btn_pressed(void);            /* PE0 按住中（OLED 用） */

/* [AI] ISR 占用率量具（AGENTS.md 要求 ≤35%，DWT 周期计数；SWD 直读）。
   周期 40 µs @168 MHz = 6720 周期，预算 2352 周期。 */
extern volatile uint32_t dbg_joint_isr_last_cycles;
extern volatile uint32_t dbg_joint_isr_max_cycles;

#endif /* APP_JOINT_H */
