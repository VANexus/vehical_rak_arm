/*
 * app_joint.c — J1/J2 增量编码器电机位置闭环 + 软件归零（机械臂板）
 *
 * 【结构（与底盘板 app_car 的 25 kHz 回路同构，但只做位置环、只有 2 轴）】
 *   TIM6 25 kHz ISR（joint_isr，stm32f4xx_it.c 调用）：
 *     每 40 µs  —— 读 2 路编码器 CNT → 累加关节计数（单写者）；急停快速路径刹车；
 *     每 25 拍 —— 1 kHz 定点位置 PI → 占空比 → 写 AT8236 双输入 PWM。
 *   ISR 内禁浮点/除法链/阻塞；位置/占空比/积分器全部 ISR 单写者，上层只读。
 *
 * 【为什么 1 kHz 而不是 25 kHz 控制】
 *   关节机械带宽 ~10–50 Hz，1 kHz 位置环已有 20–100× 余量；PWM 载波固定 25 kHz，
 *   一拍本就只能改一次占空比。详见 vehical_rak_motion/docs/LOOP-RATE-AND-IMU-ANALYSIS.md。
 *
 * 【0.1° ↔ 计数】counts = deg10 × 1707 / 100（= 61440/3600×100，分辨率 0.005859°/计数，
 *   协议量化 0.1° = 17.07 计数 → 环路比指令细约 10 倍）。换算常数单点定义在下方。
 */
#include "app_joint.h"

#include "main.h"
#include "tim.h"
#include "stm32f4xx.h"

/* ======================= 单点配置（改这里，不改别处） ======================= */

/* ---- 接线映射 ----------------------------------------------------------
   [AI] 2026-10-29 用户确认实物接线：**TJ1 = 电机 A，TJ2 = 电机 B**
        （C30D 板顶端子：TJ1/TJ2/TJ3/TJ4 = 电机 A/B/C/D 接口）。
        机械臂 J1（腰）插 TJ1、J2（大臂）插 TJ2 —— 与下面 {A, B} 一致，**无需改代码**。
        电机口 A = TIM10_CH1/TIM11_CH1 = PB8/PB9，编码器 A = TIM2 = PA15/PB3
        电机口 B = TIM9_CH1/CH2       = PE5/PE6，编码器 B = TIM3 = PB4/PB5
        电机口 C = TIM1_CH1/CH2       = PE9/PE11，编码器 C = TIM4 = PB6/PB7
        电机口 D = TIM1_CH3/CH4       = PE13/PE14，编码器 D = TIM5 = PA0/PA1
        机械臂板不驱动轮子，四路电机口都空闲 —— 只用 A/B 两路。
        ⚠️ 极性 kJointSign 仍是 [AI] 待实测：反了就是**正反馈飞车**，
           首版上板必须先做极性确认（README §7），确认后再改这里。 */
static const uint8_t kJointMotorCh[JOINT_COUNT] = { 0U, 1U };   /* [AI] A27：J1→A、J2→B 恢复（A26 的 C 口裁决被 R75 同板对照取代——硬件无问题，根因是驱动方法，见 app_build.c A27） */
static const uint8_t kJointEncIdx[JOINT_COUNT]  = { 0U, 1U };   /* {J1, J2} → TIM2, TIM3 */

/* 电机出力方向 ↔ 关节角增大方向（+1 / -1）。
   [AI] 极性史：A23 手感测试翻到 {-1,-1}；A28 台架验收"+5° 指令 J2 朝角度增大
   方向实转"判 {-1,-1} 正确。但 A32（2026-10-31 下午，台架实录）实测：当前
   接线下 duty>0 → 编码器计数**减小**（j2+ 点动回读走负，线掉事件前后两次
   独立测量一致），位置环正目标 err>0 → cmd>0 → 计数背离目标 → 任何位置
   指令都顶向限位 / 30° 跟随误差故障（J2 位置指令全部不动）。与 A28 结论
   矛盾的最可能解释：A28 之后发生过接线变动（2026-10-31 下午台架线掉重插，
   同批 J1 编码器读数冻结异常）或 A28 方向判定有误。以当前接线实测为准回
   {1,1}：duty>0 → 计数增大 = 朝目标，环路可收敛。**烧录后首测必须用小角度
   （+5°）复验方向，若仍发散立即回滚并排查接线**。点动位物理方向随之对调
   （j2+ = 回读增大），属标定语义，不影响环路稳定性。
   [AI] A35 台架实录（2026-10-31 晚，CANopen 路径首测）：J1 位置指令 +10°
   在 {1,1} 下正反馈飞车——err>0 → duty>0 → 回读走负，0.1° 有符号回读实测
   -541（-54.1°）后被碰撞/堵转闸刹车（joint_fault=1，无硬件损伤）。与 A32
   的 j2+ 点动结论互证：当前接线 J1 需 -1。J2 本轮未驱动，维持 +1 待
   小角度点动复验。 */
static const int8_t  kJointSign[JOINT_COUNT]    = { -1, 1 };

/* ---- 换算常数（一个物理量只允许一个常数） ---- */
#define JOINT_CNT_PER_DEG10_X100  1707U   /* counts per 0.1° × 100 = 61440/3600×100 */

/* ---- 软限位（相对零点，有符号 0.1°）----
   [AI] A6：改成**每轴可关**。理由（2026-10-29 台架）：
     - J1（腰）= 连续旋转，没有机械限位 —— 老的固定 0..2700 会让它在转过 270° 时锁 FAULT；
     - J2（大臂）= 有明确机械限位（一定会撞），但**实际行程还没实测**
       （台架当前约 4-5 点方向 ↔ 7-8 点方向，具体角度随装配变化）。
   默认两轴都**关闭**软限位，靠下面的碰撞/堵转判定做保护；实测出行程后把
   kJointLimitOn[axis] 置 1 并填 min/max（相对零点），重新编译即生效。 */
/* ---- 协议字段上限（无符号 0.1°，**逐轴**）----
   [AI] A17：按用户 2026-10-30 确认的机械臂结构定稿：
   - J1（腰）= 360° 有限位旋转（**不**连续转；手动归零，软限位后补）→ 上限 3600；
   - J2（大臂）= 机械限位轴，零点选在行程一端（协议字段无符号，负角发不出）；
     行程实测前上限同样放宽到 3600（软限位仍关，靠碰撞/堵转判定兜底；
     实测行程后回填 kJointLimit* 并可收紧本值）。
   J3..J6 是舵机，不走本表（app_arm.c 满量程 2700 = 270.0°）。 */
static const uint16_t kJointProtoMaxDeg10[JOINT_COUNT] = { 3600U, 3600U };
#define JOINT_TRAVEL_MARGIN_DEG10 200U    /* 越界容差 20°：只拦飞车，不误报正常超调 */

static const uint8_t kJointLimitOn[JOINT_COUNT]       = { 0U, 0U };
static const int32_t kJointLimitMinDeg10[JOINT_COUNT] = { 0, -1350 };   /* J2 占位 -135° */
static const int32_t kJointLimitMaxDeg10[JOINT_COUNT] = { 2700, 1350 }; /* J2 占位 +135° */

/* ---- 碰撞/堵转判定（本板无电流采样，只能看"给了占空比却不动"）----
   [AI] A6：100 ms 窗口累加位移，连续 2 窗（共 200 ms）不够 30 counts
   （≈300 counts/s ≈1.8°/s）就判"顶住" → 立即刹车。
   ⚠️ 这条判据的速度下限就是 ~1.8°/s：比这更慢的点动会被判成"不动"（设计取舍）。 */
#define JOINT_STALL_DUTY_Q   1500L   /* ≥15% 占空比才判：低于此不足以顶住机械限位 */
#define JOINT_STALL_WIN_MS    100U   /* 判速窗口 */
#define JOINT_STALL_CPM100     30L   /* 每 100 ms 位移下限（counts） */
#define JOINT_STALL_WINS        30U   /* 连续窗口数（A24：400ms→1s；A33：1s→3s——
                                         重触发 100ms 内接管，闸门只兜"贴目标研磨"
                                         型真堵转，避免与重触发赛跑误锁） */

/* ---- 台架点动（A6）---- */
#define JOINT_JOG_DUTY_DEF   6000L   /* A30：25%→60%。默认 25% 破不了静摩擦（台架
                                        实录点动探墙推不动），60% = 点动上限 */
#define JOINT_JOG_DUTY_MIN    500L   /* 5% */
#define JOINT_JOG_DUTY_MAX   6000L   /* 60%：再高就不算"慢速探测"了 */
#define JOINT_JOG_MS_DEF     1200U   /* 单次点动 1.2 s，到点自动刹 */
#define JOINT_JOG_MS_MAX     3000U   /* 连续按键可续到 3 s（仍不是"按住不停"） */

/* ---- A34：速度环（1 kHz 定点，对标底盘 R76：位置→限速速度→速度 PI+摩擦前馈）----
   为什么换架构（A33 提交说明有全程实录）：本对象 = 大静摩擦（顺重力 ~56% duty、
   逆重力 >60%，95% 可破）+ 低动摩擦 + 大滑行惯性。位置 PI + kick + 地板 + 重触发 +
   堵转闸门是多机制赛跑，每个常数都踩别的机制的触发沿（A33 七轮台架迭代的教训）。
   速度环用 1 kHz 位置差分做速度反馈：刹车（v_des=0、v_meas>0 → 反向占空比）、
   破静摩擦（FF 满额 + 速度 P）、粘滑（速度闭环稳住滑行速率）由**同一个律**覆盖。 */
#define JOINT_V_MAX        6L    /* 速度限幅 counts/ms ≈ 35°/s（与 JOINT_TGT_SLEW 同源） */
#define JOINT_KPP_SHIFT     3L    /* 位置误差 → 速度设定：v_des = err >> 3（8 counts 误差 = 1 c/ms，
                                    0.25° 误差即满速） */
#define JOINT_KVV_NUM      40L    /* 速度环 P：1 c/ms 速度误差 → 1.5% 占空比。
                                    从静止 dv=6 → 90%+FF 满额 = 破摩擦级；滑行超速 10%
                                    （0.6 c/ms）→ 减 9% 修量 */
#define JOINT_KVV_DEN       1L
#define JOINT_KVFF        300L    /* 摩擦前馈（随速度比例）：v_des=6 c/ms → 18% 巡航底。
                                    ⚠️ A34 首测教训：FF=90% 常量时平衡速度 = v_des +
                                    FF/KVV ≈ 46 c/ms（269°/s）直接飞车（回程冲 -89°）——
                                    FF 只能给巡航量级，突破交给助推 */
#define JOINT_BOOST_MS     100L    /* 破静摩擦助推：想动但 |v_meas|≤1 c/ms 持续 100ms →
                                    全力 95% 直到动起来（|v_meas|≥2 释放）。有速度反馈
                                    闸门，无 A33 交棒竞速 */
#define JOINT_BOOST_ERR_MIN 350L  /* [AI] A36：助推的最小误差门槛（350 counts ≈ 2°）——
                                    微小误差（0.05° 级）也触发 95% 助推 → 过冲 → 反向 →
                                    极限环抖振（台架实录），大误差才配破静摩擦 */
#define JOINT_KI_NUM     1L      /* 位置积分（消稳态差/抗重力）：单位误差·ms → 0.01% */
#define JOINT_KI_DEN    50L
#define JOINT_I_LIMIT   120000L  /* 积分限幅（counts·ms）≈ 100 counts 误差持续 3 s */
#define JOINT_DEADBAND    24L    /* [AI] A36：8→24 counts（0.05°→0.14°）——死区内 v_des=0
                                    （见速度环段），±1 count 量化噪声不再出满速指令 */
#define JOINT_DUTY_LIM  9500L    /* 0.01% → 95%（A19：台架实录 J1 静摩擦下 85% 零位移） */
#define JOINT_SLEW_Q    2000L    /* 每 1 ms 允许的占空比变化（0.01%）→ 20%/ms 斜坡 */
/* 每 1 kHz 拍最多 6 counts ≈ 35°/s（35°/s × 170.7 counts/° ÷ 1000 ≈ 5.98）。
   ⚠️ 历史教训（A32 台架实录）：旧值 1000 的注释"≈35°/s"算错了 170 倍——等效无斜坡，
   是历次炮弹过冲的总根源。改常数时先算量纲。 */
#define JOINT_TGT_SLEW     6L
#define JOINT_ERR_FAULT 5120L    /* 跟随误差（对**斜坡后目标**）>30° 判 FAULT（防飞车闸门） */

/* [AI] A33/A27 的 kick 常数与交棒机制已随 A34 速度环整体退役——破静摩擦
   由 FF（满额摩擦前馈）+ 速度环 P 从静止 dv=6 c/ms → 95% 覆盖，见上。 */

/* ---- 节拍 ---- */
#define JOINT_WIN_TICKS          25U       /* 25 kHz ÷ 25 = 1 kHz 控制节拍 */
#define JOINT_BTN_DEBOUNCE_TICKS 1000U     /* PE0 消抖 40 ms @25 kHz */
#define JOINT_BTN_HOLD_TICKS     37500U    /* PE0 长按 1.5 s → 置零 */

/* ======================= 状态（ISR 单写者 / volatile 只读） ======================= */

static uint32_t s_pwm_arr;

/* 编码器累计（ISR 单写者） */
static uint32_t s_prev_cnt[JOINT_COUNT];
static volatile int32_t s_pos_counts[JOINT_COUNT];    /* 原始累计计数 */
static volatile int32_t s_zero_counts[JOINT_COUNT];   /* 零点对应的原始计数 */
static volatile int32_t s_target_counts[JOINT_COUNT]; /* 目标（原始计数口径） */

/* 调节器（ISR 单写者） */
static int32_t s_integral[JOINT_COUNT];
static int16_t s_boost_ms[JOINT_COUNT];    /* A34：破静摩擦助推计时（ISR 单写者） */
static int32_t s_duty_q[JOINT_COUNT];
static int32_t s_tgt_slew[JOINT_COUNT];   /* 斜坡后的目标（调节器真正的设定点） */
/* [AI] A34：s_kick_ms / s_kick_pos0 已随 kick 机制退役（速度环 FF 覆盖破静摩擦），
   各处置零语句保留以兼容旧调用点，变量本身删除。 */

/* 门 / 状态 */
static volatile uint8_t s_estop;
static volatile uint8_t s_zeroed;
static volatile uint8_t s_fault[JOINT_COUNT];
static volatile uint8_t s_zero_req;
static volatile uint8_t s_clear_req;
static volatile uint8_t s_impact_req;      /* [AI] A7: 加速度计碰撞确认请求（ISR 落地） */
static volatile uint32_t s_impact_cnt;
static volatile uint8_t s_btn_pressed;
static uint32_t s_btn_low, s_btn_hold;
static uint8_t  s_btn_fired;
static uint8_t  s_isr_started;
static uint32_t s_win_cnt;

/* [AI] A6：台架点动 + 碰撞/堵转状态（点动字段由任务侧写、ISR 读，见 joint_jog 注释） */
static volatile int32_t  s_jog_duty_q[JOINT_COUNT];
static volatile int8_t   s_jog_dir[JOINT_COUNT];
static volatile uint16_t s_jog_ms[JOINT_COUNT];      /* 剩余点动时间，0 = 未点动 */
static volatile uint8_t  s_collide[JOINT_COUNT];
static volatile int32_t  s_jog_duty_sel = JOINT_JOG_DUTY_DEF;
static int32_t  s_pos_prev_1k[JOINT_COUNT];          /* 1 kHz 位移统计 */
static int32_t  s_move_acc[JOINT_COUNT];
static uint16_t s_win_ms[JOINT_COUNT];
static uint8_t  s_still_wins[JOINT_COUNT];

/* [AI] 25 kHz ISR 占用率量具（AGENTS.md：预算 ≤35% = 2352 周期） */
volatile uint32_t dbg_joint_isr_last_cycles;
volatile uint32_t dbg_joint_isr_max_cycles;

/* ======================= 输出：AT8236 双输入 + 编码器（L1） ======================= */

/* [AI] CubeMX 迁移纪律：TIM1/9/10/11 的 PSC=0、ARR=6719(25 kHz) 与引脚 AF 由
   MX_TIMx_Init 配置；本层只启动输出通道、写 CCR。 */
static void joint_pwm_start(uint8_t ch)
{
    switch (ch) {
    case 0U: (void)HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);   /* PB8 */
             (void)HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); break; /* PB9 */
    case 1U: (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);    /* PE5 */
             (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); break; /* PE6 */
    case 2U: (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    /* PE9 */
             (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); break; /* PE11 */
    case 3U: (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    /* PE13 */
             (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); break; /* PE14 */
    default: break;
    }
}

static void joint_pair(uint8_t ch, uint32_t forward, uint32_t reverse)
{
    switch (ch) {
    case 0U: __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, forward);
             __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, reverse); break;
    case 1U: __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_1, forward);
             __HAL_TIM_SET_COMPARE(&htim9,  TIM_CHANNEL_2, reverse); break;
    case 2U: __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, forward);
             __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_2, reverse); break;
    case 3U: __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_3, forward);
             __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_4, reverse); break;
    default: break;
    }
}

/* 刹车 = IN1=IN2=1（AT8236 双输入短路制动；board-c30d.md §1 实测口径） */
static void joint_brake(unsigned axis)
{
    joint_pair(kJointMotorCh[axis], s_pwm_arr + 1U, s_pwm_arr + 1U);
}

static void joint_apply(unsigned axis, int32_t duty_q)
{
    if (duty_q == 0) { joint_brake(axis); return; }
    int32_t mag = (duty_q > 0) ? duty_q : -duty_q;
    int dir = (duty_q > 0) ? 1 : -1;
    if (kJointSign[axis] < 0) dir = -dir;
    unsigned pct = (unsigned)(mag / 100L);
    if (pct == 0U) { joint_brake(axis); return; }   /* 不足 1% 一律刹车，不留滑行 */
    if (pct > 95U) pct = 95U;                       /* [AI] A24：85→95 对齐 JOINT_DUTY_LIM——
                                                       A19 提限到 95% 但被本钳位截回 85%，
                                                       95% 上限从未真正生效（台架"85% 零位移"
                                                       实测的正是这个被截断的值） */
    uint32_t pulse = (s_pwm_arr + 1U) * pct / 100U;
    if (dir > 0) joint_pair(kJointMotorCh[axis], pulse, 0U);
    else         joint_pair(kJointMotorCh[axis], 0U, pulse);
}

static TIM_TypeDef *const kEncTim[4] = { TIM2, TIM3, TIM4, TIM5 };

static void joint_encoder_start(uint8_t idx)
{
    switch (idx) {
    case 0U: (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); break;
    case 1U: (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); break;
    case 2U: (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); break;
    case 3U: (void)HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); break;
    default: break;
    }
}

/* ======================= 换算 ======================= */

static int32_t counts_of_deg10(int32_t deg10)
{
    return (deg10 * (int32_t)JOINT_CNT_PER_DEG10_X100) / 100L;
}

/* 有符号 0.1°（J1 连续旋转可为负 / 超 2700） */
static int32_t deg10_of_counts(int32_t counts)
{
    return (counts * 100L) / (int32_t)JOINT_CNT_PER_DEG10_X100;
}

/* 相对零点 → 协议无符号字段（0..逐轴上限，饱和） */
static uint16_t deg10_proto(unsigned axis, int32_t counts_rel)
{
    int32_t d = deg10_of_counts(counts_rel);
    int32_t hi = (int32_t)kJointProtoMaxDeg10[axis];
    if (d < 0L) return 0U;
    if (d > hi) return (uint16_t)hi;
    return (uint16_t)d;
}

/* 软限位越界（含容差）；该轴未启用软限位时恒 0 */
static uint8_t joint_out_of_limit(unsigned a)
{
    if (kJointLimitOn[a] == 0U) return 0U;
    int32_t rel = s_pos_counts[a] - s_zero_counts[a];
    int32_t lo = counts_of_deg10(kJointLimitMinDeg10[a] - (int32_t)JOINT_TRAVEL_MARGIN_DEG10);
    int32_t hi = counts_of_deg10(kJointLimitMaxDeg10[a] + (int32_t)JOINT_TRAVEL_MARGIN_DEG10);
    return (uint8_t)((rel < lo || rel > hi) ? 1U : 0U);
}

/* ======================= A6：碰撞/堵转 + 点动（ISR 侧） ======================= */

/* 1 kHz 位移统计：100 ms 窗累加 |Δcounts|，判定"动没动" */
static void joint_track_motion(unsigned a)
{
    int32_t d = s_pos_counts[a] - s_pos_prev_1k[a];
    s_pos_prev_1k[a] = s_pos_counts[a];
    if (d < 0L) d = -d;
    if (s_move_acc[a] < 1000000L) s_move_acc[a] += d;
    if (++s_win_ms[a] < JOINT_STALL_WIN_MS) return;

    uint8_t still = (s_move_acc[a] < JOINT_STALL_CPM100) ? 1U : 0U;
    s_move_acc[a] = 0L;
    s_win_ms[a] = 0U;
    /* [AI] A33 关键修复：still 窗口只在**真的在驱动**（|duty|≥15%）时累计——
       旧代码静置时无条件累加到饱和（255），任何位置指令让 duty 一过 15% 就在
       同一拍被判"堵转"锁 FAULT（SWD 实录：指令开始后 slew 只走 18 counts 即
       全冻结）。这也是"清障后第一小步能动、之后全死"的根因。 */
    if (still != 0U) {
        int32_t dq = s_duty_q[a];
        if (dq < 0L) dq = -dq;
        if (dq >= JOINT_STALL_DUTY_Q) {
            if (s_still_wins[a] < 255U) s_still_wins[a]++;
        } else {
            s_still_wins[a] = 0U;
        }
    } else {
        s_still_wins[a] = 0U;
    }
}

/* 判"给足占空比仍不动"（小占空比顶不动不算，避免小目标/保持姿态误报） */
static uint8_t joint_stalled(unsigned a)
{
    if (s_still_wins[a] < JOINT_STALL_WINS) return 0U;
    int32_t dq = s_duty_q[a];
    if (dq < 0L) dq = -dq;
    return (uint8_t)((dq >= JOINT_STALL_DUTY_Q) ? 1U : 0U);
}

/* 点动途中判到顶住：刹车 + 记碰撞，不锁 FAULT（探边界本来就会撞） */
static void joint_jog_abort(unsigned a)
{
    joint_brake(a);
    s_integral[a] = 0;
    s_duty_q[a] = 0;
    s_jog_ms[a] = 0U;
    s_collide[a] = 1U;
    s_still_wins[a] = 0U;
    s_move_acc[a] = 0L;
    s_win_ms[a] = 0U;
    s_target_counts[a] = s_pos_counts[a];
    s_tgt_slew[a] = s_pos_counts[a];
}

/* 点动一拍：按 JOINT_SLEW_Q 斜坡到恒占空比，到点自动刹 + 目标重同步 */
static void joint_jog_step(unsigned a)
{
    int32_t want = (int32_t)s_jog_dir[a] * s_jog_duty_q[a];
    int32_t dq = want - s_duty_q[a];
    if (dq > JOINT_SLEW_Q) dq = JOINT_SLEW_Q;
    else if (dq < -JOINT_SLEW_Q) dq = -JOINT_SLEW_Q;
    s_duty_q[a] += dq;
    joint_apply(a, s_duty_q[a]);

    if (s_jog_ms[a] == 0U) return;
    s_jog_ms[a] = (uint16_t)(s_jog_ms[a] - 1U);
    if (s_jog_ms[a] == 0U) {
        joint_brake(a);
        s_duty_q[a] = 0;
        s_target_counts[a] = s_pos_counts[a];
        s_tgt_slew[a] = s_pos_counts[a];
    }
}

/* ======================= 生命周期 ======================= */

void joint_init(void)
{
    s_pwm_arr = htim1.Init.Period;      /* 6719 = 168 MHz / 25 kHz - 1 */
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        s_pos_counts[a] = 0;
        s_zero_counts[a] = 0;
        s_target_counts[a] = 0;
        s_integral[a] = 0;
        s_duty_q[a] = 0;
        s_tgt_slew[a] = 0;
        s_fault[a] = 0U;
        s_prev_cnt[a] = kEncTim[kJointEncIdx[a]]->CNT;
        /* [AI] A6：点动/判撞状态清零 —— 上电不允许任何点动在跑 */
        s_jog_duty_q[a] = 0;
        s_jog_dir[a] = 0;
        s_jog_ms[a] = 0U;
        s_collide[a] = 0U;
        s_pos_prev_1k[a] = 0;
        s_move_acc[a] = 0;
        s_win_ms[a] = 0U;
        s_still_wins[a] = 0U;
    }
    s_zeroed = 0U;                      /* 零点未定义：未归零前不驱动 */
    s_estop = 0U;
    s_zero_req = 0U;
    s_clear_req = 0U;
    s_impact_req = 0U;
    s_impact_cnt = 0U;
    s_win_cnt = 0U;
    s_btn_low = 0U; s_btn_hold = 0U; s_btn_fired = 0U; s_btn_pressed = 0U;

    /* 先刹车再启动 PWM 输出（AT8236 IN1=IN2=1 = 短路制动；上电不得自动驱动） */
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        joint_brake(a);
        joint_pwm_start(kJointMotorCh[a]);
        joint_encoder_start(kJointEncIdx[a]);
        s_prev_cnt[a] = kEncTim[kJointEncIdx[a]]->CNT;
    }
}

void joint_start_isr(void)
{
    if (s_isr_started != 0U) return;
    s_isr_started = 1U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* [AI] 占用率自测用；幂等 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    (void)HAL_TIM_Base_Start_IT(&htim6);              /* 25 kHz 首拍即进 joint_isr */
}

uint8_t joint_isr_ready(void) { return s_isr_started; }

/* ======================= 门 / 请求（任务或 CAN ISR 上下文） ======================= */

void joint_set_estop(uint8_t on) { s_estop = (on != 0U) ? 1U : 0U; }

void joint_set_zero(void) { s_zero_req = 1U; }

uint8_t joint_zeroed(void) { return s_zeroed; }

uint8_t joint_fault(void)
{
    uint8_t b = 0U;
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) if (s_fault[a] != 0U) b |= (uint8_t)(1U << a);
    return b;
}

void joint_fault_clear(void) { s_clear_req = 1U; }

uint8_t joint_btn_pressed(void) { return s_btn_pressed; }

/* ======================= A6：台架点动（任务上下文调用） ======================= */

/* 多个 volatile 字段分步写，ISR 可能读到"半更新"；所以先把 s_jog_ms 置 0 停一拍，
   写完其余字段再最后置时间 —— 宁可多剥一个 1 ms 周期的刹车。 */
void joint_jog(uint8_t axis, int8_t dir, int32_t duty_q, uint16_t duration_ms)
{
    if (axis >= JOINT_COUNT || dir == 0) return;
    if (s_estop != 0U || s_fault[axis] != 0U) return;   /* 故障/急停时不接受点动 */

    if (duty_q < 0) duty_q = -duty_q;
    if (duty_q < JOINT_JOG_DUTY_MIN) duty_q = JOINT_JOG_DUTY_MIN;
    if (duty_q > JOINT_JOG_DUTY_MAX) duty_q = JOINT_JOG_DUTY_MAX;
    if (duration_ms == 0U) duration_ms = JOINT_JOG_MS_DEF;

    int8_t d = (dir > 0) ? (int8_t)1 : (int8_t)-1;

    /* 同轴同向的重复按键 = 续时（终端按键自动重复时会连发），
       不去打断正在跑的点动 —— 否则每按一次都要先刹 1 ms，动作会一顿一顿。 */
    if (s_jog_ms[axis] != 0U && s_jog_dir[axis] == d) {
        uint32_t t = (uint32_t)s_jog_ms[axis] + (uint32_t)duration_ms;
        if (t > JOINT_JOG_MS_MAX) t = JOINT_JOG_MS_MAX;
        s_jog_duty_q[axis] = duty_q;
        s_jog_ms[axis] = (uint16_t)t;
        return;
    }

    s_jog_ms[axis] = 0U;                 /* 先停一拍 */
    s_jog_dir[axis] = d;
    s_jog_duty_q[axis] = duty_q;
    s_collide[axis] = 0U;                /* 新一次点动清该轴的碰撞记录 */
    s_still_wins[axis] = 0U;
    s_move_acc[axis] = 0L;
    s_win_ms[axis] = 0U;
    s_pos_prev_1k[axis] = s_pos_counts[axis];
    /* 点动期间位置环不执行，但目标也不能留在旧值 —— 否则点动结束的瞬间
       位置环会"向旧目标追赶"，把点动位移当成误差。 */
    s_target_counts[axis] = s_pos_counts[axis];
    s_tgt_slew[axis] = s_pos_counts[axis];
    s_jog_ms[axis] = duration_ms;        /* 最后放行 */
}

void joint_jog_stop(void)
{
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) s_jog_ms[a] = 0U;
}

uint8_t joint_jog_active(void)
{
    uint8_t b = 0U;
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) if (s_jog_ms[a] != 0U) b |= (uint8_t)(1U << a);
    return b;
}

uint8_t joint_jog_axis_active(uint8_t axis)
{
    return (axis >= JOINT_COUNT) ? 0U : (uint8_t)((s_jog_ms[axis] != 0U) ? 1U : 0U);
}

void joint_jog_set_duty(int32_t duty_q)
{
    if (duty_q < JOINT_JOG_DUTY_MIN) duty_q = JOINT_JOG_DUTY_MIN;
    if (duty_q > JOINT_JOG_DUTY_MAX) duty_q = JOINT_JOG_DUTY_MAX;
    s_jog_duty_sel = duty_q;
}

int32_t joint_jog_get_duty(void) { return s_jog_duty_sel; }

uint8_t joint_collide(void)
{
    uint8_t b = 0U;
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) if (s_collide[a] != 0U) b |= (uint8_t)(1U << a);
    return b;
}

int32_t joint_duty_q(unsigned axis)
{
    return (axis >= JOINT_COUNT) ? 0L : s_duty_q[axis];
}

/* [AI] A7：加速度计碰撞确认（任务上下文置请求，ISR 下一拍落地） */
void joint_note_impact(void) { s_impact_req = 1U; }

uint32_t joint_impact_cnt(void) { return s_impact_cnt; }

void joint_set_target_deg10(unsigned axis, uint16_t deg10)
{
    if (axis >= JOINT_COUNT) return;
    int32_t d = (int32_t)deg10;
    /* [AI] A6：软限位逐轴可关，启用时同时钳制目标（J1 连续旋转默认不钳） */
    if (kJointLimitOn[axis] != 0U) {
        if (d < kJointLimitMinDeg10[axis]) d = kJointLimitMinDeg10[axis];
        if (d > kJointLimitMaxDeg10[axis]) d = kJointLimitMaxDeg10[axis];
    }
    /* [AI] A17：协议上限逐轴（J1/J2 = 3600 = 360.0°，见 kJointProtoMaxDeg10 注释） */
    int32_t hi = (int32_t)kJointProtoMaxDeg10[axis];
    if (d > hi) d = hi;
    s_target_counts[axis] = s_zero_counts[axis] + counts_of_deg10(d);
}

uint16_t joint_actual_deg10(unsigned axis)
{
    if (axis >= JOINT_COUNT) return 0U;
    return deg10_proto(axis, s_pos_counts[axis] - s_zero_counts[axis]);
}

int32_t joint_actual_deg10_signed(unsigned axis)
{
    if (axis >= JOINT_COUNT) return 0L;
    return deg10_of_counts(s_pos_counts[axis] - s_zero_counts[axis]);
}

int32_t joint_pos_counts(unsigned axis)
{
    return (axis >= JOINT_COUNT) ? 0L : (s_pos_counts[axis] - s_zero_counts[axis]);
}

int32_t joint_err_counts(unsigned axis)
{
    return (axis >= JOINT_COUNT) ? 0L : (s_target_counts[axis] - s_pos_counts[axis]);
}

uint8_t joint_state(unsigned axis)
{
    if (axis >= JOINT_COUNT) return (uint8_t)JOINT_ST_BRAKE;
    if (s_fault[axis] != 0U) return (uint8_t)JOINT_ST_FAULT;
    if (s_zeroed == 0U || s_estop != 0U) return (uint8_t)JOINT_ST_BRAKE;
    int32_t e = s_tgt_slew[axis] - s_pos_counts[axis];
    if (e > JOINT_DEADBAND || e < -JOINT_DEADBAND) return (uint8_t)JOINT_ST_MOVE;
    return (uint8_t)JOINT_ST_HOLD;
}

/* ======================= ISR 侧状态迁移 ======================= */

static void joint_zero_apply(void)
{
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        s_zero_counts[a] = s_pos_counts[a];
        s_target_counts[a] = s_pos_counts[a];   /* 目标 = 当前 = 0.0° */
        s_tgt_slew[a] = s_pos_counts[a];
        s_integral[a] = 0;
        s_duty_q[a] = 0;
        s_fault[a] = 0U;
        joint_brake(a);
        /* [AI] A6：置零同时取消点动/碰撞记录（置零是人主动定义基准位） */
        s_jog_ms[a] = 0U;
        s_collide[a] = 0U;
    }
    s_zeroed = 1U;
}

static void joint_fault_clear_apply(void)
{
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        if (s_fault[a] == 0U) continue;
        /* [AI] A6：清位时先判软限位（未启用软限位的轴永远为 0）；跟随误差在清位时
           **不参与判定** —— 目标重同步到当前位置，否则撞过限位后永远清不了位。 */
        if (joint_out_of_limit(a) == 0U) {
            s_fault[a] = 0U;
            s_integral[a] = 0;
            s_duty_q[a] = 0;
                s_target_counts[a] = s_pos_counts[a];
            s_tgt_slew[a] = s_pos_counts[a];   /* 斜坡从当前位置重新起步 */
            s_still_wins[a] = 0U;
            s_move_acc[a] = 0L;
            s_win_ms[a] = 0U;
        }
    }
}

static void isr_end(uint32_t t0)
{
    /* [AI] 量的是"清 UIF 之后到退出"的跨度，不含 NVIC 入/出栈（与底盘板 R19 同口径） */
    uint32_t dt = DWT->CYCCNT - t0;
    dbg_joint_isr_last_cycles = dt;
    if (dt > dbg_joint_isr_max_cycles) dbg_joint_isr_max_cycles = dt;
}

/* ======================= 25 kHz ISR ======================= */

void joint_isr(void)
{
    if ((TIM6->SR & TIM_SR_UIF) == 0U) return;
    TIM6->SR = (uint16_t)~TIM_SR_UIF;
    uint32_t t0 = DWT->CYCCNT;

    /* ---- 1) 编码器累计（单写者） ---- */
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        uint8_t idx = kJointEncIdx[a];
        uint32_t cnt = kEncTim[idx]->CNT;
        /* TIM3/TIM4 是 16 位（差值按 int16 回绕），TIM2/TIM5 是 32 位 */
        int32_t delta = (idx == 1U || idx == 2U)
            ? (int32_t)(int16_t)(uint16_t)(cnt - s_prev_cnt[a])
            : (int32_t)(cnt - s_prev_cnt[a]);
        s_prev_cnt[a] = cnt;
        s_pos_counts[a] += delta;
    }

    /* ---- 2) PE0（上拉，低有效）：消抖 + 长按 1.5 s → 软件置零 ---- */
    {
        uint8_t low = ((GPIOE->IDR & GPIO_IDR_ID0) == 0U) ? 1U : 0U;
        s_btn_pressed = low;
        if (low != 0U) {
            if (s_btn_low < JOINT_BTN_DEBOUNCE_TICKS) s_btn_low++;
            else if (s_btn_hold < JOINT_BTN_HOLD_TICKS) s_btn_hold++;
            else if (s_btn_fired == 0U) { s_btn_fired = 1U; s_zero_req = 1U; }
        } else {
            s_btn_low = 0U; s_btn_hold = 0U; s_btn_fired = 0U;
        }
    }

    /* ---- 3) 急停快速路径：本周期（≤40 µs）立即刹车 + 复位调节器 ---- */
    if (s_estop != 0U) {
        for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            s_jog_ms[a] = 0U;            /* [AI] A6: 急停同时取消点动 */
        }
        isr_end(t0);
        return;
    }

    if (++s_win_cnt < JOINT_WIN_TICKS) { isr_end(t0); return; }
    s_win_cnt = 0U;

    /* ---- 4) 1 kHz：状态请求 + 位置 PI + 输出 ---- */
    if (s_zero_req != 0U)  { s_zero_req = 0U;  joint_zero_apply(); }
    if (s_clear_req != 0U) { s_clear_req = 0U; joint_fault_clear_apply(); }
    if (s_impact_req != 0U) {
        /* [AI] A7: 加速度计确认撞击 —— 取消正在跑的点动并记碰撞（不在 ISR 里算浮点） */
        s_impact_req = 0U;
        if (s_impact_cnt < 0xFFFFFFFFUL) s_impact_cnt++;
        for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
            if (s_jog_ms[a] != 0U) joint_jog_abort(a);
        }
    }

    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        /* [AI] A6: 1 kHz 位移统计 —— 与是否点动无关，判撞只看位移与占空比 */
        joint_track_motion(a);

        if (s_fault[a] != 0U) {
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            s_jog_ms[a] = 0U;
            continue;
        }

        /* [AI] A6: 点动优先于位置环（且**不需要先归零**）——
           判到顶住就刹车，不锁故障；时间到自动刹。 */
        if (s_jog_ms[a] != 0U) {
            if (joint_stalled(a) != 0U) {
                joint_jog_abort(a);
                continue;
            }
            joint_jog_step(a);
            continue;
        }

        if (s_zeroed == 0U) {
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            continue;
        }

        /* ---- A34：速度环（位置→限速速度→速度 PI+摩擦前馈，底盘 R76 同构）----
           1kHz 单一律覆盖刹车/破静摩擦/粘滑/保持，替代 A33 的
           kick+地板+重触发+闸门多机制赛跑（历史教训见 A33 提交说明）。 */

        /* 1) 目标斜坡：35°/s 限速（速度环自身不炮弹，斜坡只管"目标走多快"） */
        {
            int32_t ds = s_target_counts[a] - s_tgt_slew[a];
            if (ds > JOINT_TGT_SLEW) ds = JOINT_TGT_SLEW;
            else if (ds < -JOINT_TGT_SLEW) ds = -JOINT_TGT_SLEW;
            s_tgt_slew[a] += ds;
        }

        int32_t err = s_tgt_slew[a] - s_pos_counts[a];
        if (err > JOINT_ERR_FAULT || err < -JOINT_ERR_FAULT) {
            s_fault[a] = 1U;                      /* 防飞车闸门 */
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            continue;
        }

        /* 堵转闸门（A33 已修：still 窗口只在 |duty|≥15% 时累计）：追目标 3s
           仍低于 1.8°/s 判真堵转。 */
        if ((err > JOINT_DEADBAND || err < -JOINT_DEADBAND) && joint_stalled(a) != 0U) {
            s_fault[a] = 1U;
            s_collide[a] = 1U;
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            continue;
        }

        /* 软限位（含容差，逐轴可关）：越界即 FAULT */
        if (joint_out_of_limit(a) != 0U) {
            s_fault[a] = 1U;
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            continue;
        }

        /* 2) 速度设定：位置误差 → 限速速度（counts/ms）。
           [AI] A36：死区内 v_des=0——位置 P 对 ±1 count 量化噪声照样出速度
           指令（KPP_SHIFT=3 时 8 counts 即满速），是目标附近抖振的主源；
           死区内改由积分/刹车保持（积分在死区内本就冻结+泄放）。 */
        int32_t v_des = err >> JOINT_KPP_SHIFT;
        if (err < JOINT_DEADBAND && err > -JOINT_DEADBAND) v_des = 0L;
        if (v_des > JOINT_V_MAX) v_des = JOINT_V_MAX;
        else if (v_des < -JOINT_V_MAX) v_des = -JOINT_V_MAX;

        /* 3) 速度测量：1 kHz 位置差分（track_motion 本拍已更新 pos_prev_1k）。
           毛刺钳位：单拍 |Δ|>30 counts（≈175°/s）超出机械可能，按编码器毛刺
           处理——不钳位的话速度 P 会对毛刺打出反向满占空比（J1"自己动"的根因）。 */
        int32_t v_meas = s_pos_counts[a] - s_pos_prev_1k[a];
        if (v_meas > 30L) v_meas = 30L;
        else if (v_meas < -30L) v_meas = -30L;

        /* 4) 摩擦前馈（随速度比例，巡航底）+ 速度 P */
        int32_t ff = JOINT_KVFF * v_des;
        int32_t cmd = ((v_des - v_meas) * JOINT_KVV_NUM) / JOINT_KVV_DEN + ff;

        /* 4b) 破静摩擦助推：想动（|v_des|>0）但 100ms 没动（|v_meas|≤1 c/ms）→
           全力 95% 直到动起来（|v_meas|≥2 释放）。速度反馈闸门 = 动起来的瞬间
           自然交棒，无 A33 的 handoff 竞速。助推期冻结位置积分（防泵入过冲）。
           [AI] A36：加 |err|≥JOINT_BOOST_ERR_MIN（2°）门槛——微小误差触发的
           95% 助推是目标附近极限环抖振的第二主源。 */
        if (v_des != 0L &&
            (err > JOINT_BOOST_ERR_MIN || err < -JOINT_BOOST_ERR_MIN) &&
            v_meas <= 1L && v_meas >= -1L) {
            if (s_boost_ms[a] < 32767) s_boost_ms[a]++;
            if (s_boost_ms[a] > 8000) {
                /* 95% 助推 8s 仍不动 = 真堵转（语义明确，无竞速） */
                s_fault[a] = 1U;
                s_collide[a] = 1U;
                joint_brake(a);
                s_duty_q[a] = 0;
                s_integral[a] = 0;
                continue;
            }
            if (s_boost_ms[a] > JOINT_BOOST_MS) {
                cmd = (v_des > 0L) ? JOINT_DUTY_LIM : -JOINT_DUTY_LIM;
                /* 助推期冻结堵转窗口（95% 是指定突破机制，别让闸门抢跑） */
                s_still_wins[a] = 0U;
                s_move_acc[a] = 0L;
                s_win_ms[a] = 0U;
                s_pos_prev_1k[a] = s_pos_counts[a];
            }
        } else {
            s_boost_ms[a] = 0;
        }

        if (s_boost_ms[a] <= JOINT_BOOST_MS) {
            /* 助推期（>100ms 未动）冻结积分，防释放瞬间泵入过冲 */
            if (err > JOINT_DEADBAND)       s_integral[a] += err - JOINT_DEADBAND;
            else if (err < -JOINT_DEADBAND) s_integral[a] += err + JOINT_DEADBAND;
        }
        else if (s_integral[a] != 0L) {
            /* 死区内积分泄放（防"呆顶"）：满积分 ~1.5s 泄完 */
            s_integral[a] += (s_integral[a] > 0L) ? -200 : 200;
            if (s_integral[a] > -200L && s_integral[a] < 200L) s_integral[a] = 0;
        }
        /* [AI] A34：常态弱泄漏（0.1%/ms）——大幅摆动期误差大、积分风箱满罐，
           到位后 +60% 偏置直接把关节顶过目标（台架实录 ±60° 极限环）。 */
        s_integral[a] -= s_integral[a] / 1000;
        if (s_integral[a] > JOINT_I_LIMIT) s_integral[a] = JOINT_I_LIMIT;
        if (s_integral[a] < -JOINT_I_LIMIT) s_integral[a] = -JOINT_I_LIMIT;
        cmd += s_integral[a] / JOINT_KI_DEN;

        if (cmd > JOINT_DUTY_LIM) cmd = JOINT_DUTY_LIM;
        if (cmd < -JOINT_DUTY_LIM) cmd = -JOINT_DUTY_LIM;

        int32_t dq = cmd - s_duty_q[a];
        if (dq > JOINT_SLEW_Q) dq = JOINT_SLEW_Q;
        else if (dq < -JOINT_SLEW_Q) dq = -JOINT_SLEW_Q;
        s_duty_q[a] += dq;
        joint_apply(a, s_duty_q[a]);
    }

    isr_end(t0);
}
