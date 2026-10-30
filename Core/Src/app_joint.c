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
   （j2+ = 回读增大），属标定语义，不影响环路稳定性。 */
static const int8_t  kJointSign[JOINT_COUNT]    = { 1, 1 };

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
#define JOINT_STALL_WINS        10U   /* 连续窗口数（A24：400ms→1s——用户台架实录两轴都
                                         "初始给点力才开始转"，静摩擦突破需要时间；
                                         位置环满占空比 1s 零位移再判顶住，仍能兜住真堵转） */

/* ---- 台架点动（A6）---- */
#define JOINT_JOG_DUTY_DEF   6000L   /* A30：25%→60%。默认 25% 破不了静摩擦（台架
                                        实录点动探墙推不动），60% = 点动上限 */
#define JOINT_JOG_DUTY_MIN    500L   /* 5% */
#define JOINT_JOG_DUTY_MAX   6000L   /* 60%：再高就不算"慢速探测"了 */
#define JOINT_JOG_MS_DEF     1200U   /* 单次点动 1.2 s，到点自动刹 */
#define JOINT_JOG_MS_MAX     3000U   /* 连续按键可续到 3 s（仍不是"按住不停"） */

/* ---- 位置环（1 kHz 定点；[AI] 未上板整定的初值，首版上板必须重标） ---- */
#define JOINT_KP_NUM     3L      /* 单位误差 → 0.01% 占空比（A28：10→3。10 时 5° 误差
                                    = 85% duty，破静摩擦瞬间直接打满→炮弹式过冲到
                                    30° 故障闸门。3 时 5°→25%、30°→76%，重力稳态
                                    靠积分补） */
#define JOINT_KP_DEN     1L
#define JOINT_KI_NUM     1L      /* 单位误差·ms → 0.01%：100 counts 持续 1 s → 20% */
#define JOINT_KI_DEN    50L
#define JOINT_I_LIMIT   300000L  /* 积分限幅（counts·ms）≈ 100 counts 误差持续 3 s */
#define JOINT_DEADBAND     8L    /* counts，≈0.05°：死区内冻结积分，防抖动泵占空比 */
#define JOINT_DUTY_LIM  9500L    /* 0.01% → 95%（A19：85%→95%。台架实录 J1 静摩擦下
                                    85% 零位移；底盘 R 系列起步静摩擦 ~75% 转述，
                                    臂减速箱更沉。95% + 堵转窗口 400ms 兜底） */
#define JOINT_SLEW_Q    2000L    /* 每 1 ms 允许的占空比变化（0.01%）→ 20%/ms 斜坡 */
#define JOINT_TGT_SLEW  1000L    /* 目标斜坡：每 1 ms 最多 1000 counts ≈ 35°/s（[AI] 待整定）
                                    作用：把"指令大跳变"变成受控斜坡，从而让下面的
                                    JOINT_ERR_FAULT 只拦**真的跟不上/飞车**，不误报正常大位移。
                                    若某轴物理速度低于 35°/s 出现误报，改小本常数。 */
#define JOINT_ERR_FAULT 5120L    /* 跟随误差（对**斜坡后目标**）>30° 判 FAULT（防飞车闸门） */

/* ---- A27：静摩擦突破前馈（对标底盘 R33 判向 kick 的"持续到突破"语义） ----
   台架实录（A25-A26 + R75 同板对照）：同板同口同电机，底盘代码能转、
   臂板代码不能——差异 = 底盘 kick 是**开环 75% 持续输出直到编码器看到运动**，
   臂板 A25 kick 只打 150ms 就交 PI，1s 堵转窗口跟着锁故障。
   修正：kick 持续满扭矩，直到位移 ≥JOINT_BREAKAWAY_MOVED（突破，PI 接管）
   或 JOINT_BREAKAWAY_MS_MAX（真堵转上限，之后堵转判定接管锁 FAULT）。
   kick 期间冻结堵转窗口计数。 */
#define JOINT_BREAKAWAY_ERR    350L    /* 误差 ≥2°（350 counts）才 kick，小修正不打 */
/* [AI] A30：kick 斜坡——30% 起步、每秒 +10%、封顶 95%。静摩擦在尽可能低的
   占空比下被突破（最小齿轮箱预紧 = 最温和弹射），突破用时不确定也没关系
   （斜坡会一直爬到 95%）。 */
#define JOINT_BREAKAWAY_DUTY0 3000L    /* kick 起始 30% */
#define JOINT_BREAKAWAY_RAMP    1L    /* 每 ms +0.01%（= +10%/s） */
#define JOINT_BREAKAWAY_DUTY_MAX 9500L /* kick 封顶 95% */
#define JOINT_BREAKAWAY_MS_MAX 8000U   /* kick 最长 8s（A29：3s 不够——A27 实测突破
                                          用时 ~4s 且逐次漂移；底盘判向 kick 无上限，
                                          8s + 1s 堵转已保守） */
#define JOINT_BREAKAWAY_MOVED   100L   /* 位移 ≥100 counts（0.6°）= 已突破 */
#define JOINT_HANDOFF_DUTY    3000L    /* A28：突破交棒占空比 30%（对标底盘 50% hand-off，臂更轻） */

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
static int32_t s_duty_q[JOINT_COUNT];
static int32_t s_tgt_slew[JOINT_COUNT];   /* 斜坡后的目标（调节器真正的设定点） */
static volatile uint16_t s_kick_ms[JOINT_COUNT];   /* A27 剩余 kick 预算 */
static int32_t s_kick_pos0[JOINT_COUNT];           /* A27 kick 起点位移 */

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
    if (still != 0U) {
        if (s_still_wins[a] < 255U) s_still_wins[a]++;
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
        s_kick_ms[a] = 0U;   /* A25 */
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
        s_kick_ms[a] = 0U;   /* A25 */
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
            s_kick_ms[a] = 0U;   /* A25 */
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

        /* 目标斜坡：把指令跳变变成受控位移，保护机械 + 让防飞车闸门不误报 */
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

        /* [AI] A6: 位置环模式的堵转/撞限位 —— 正在追目标（超死区）且占空比已≥阈值
           却 200 ms 不动 → 锁 FAULT（比点动严重：说明跟踪已失效）。 */
        if ((err > JOINT_DEADBAND || err < -JOINT_DEADBAND) && joint_stalled(a) != 0U) {
            s_fault[a] = 1U;
            s_collide[a] = 1U;
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            continue;
        }

        /* 软限位（含容差，逐轴可关）：越界即 FAULT（未启用时恒不越界） */
        if (joint_out_of_limit(a) != 0U) {
            s_fault[a] = 1U;
            joint_brake(a);
            s_duty_q[a] = 0;
            s_integral[a] = 0;
            continue;
        }

        /* 死区：死区内不加积分（防在噪声带里泵占空比导致嗡嗡抖） */
        int32_t pe = err;
        if (pe > JOINT_DEADBAND) pe -= JOINT_DEADBAND;
        else if (pe < -JOINT_DEADBAND) pe += JOINT_DEADBAND;
        else pe = 0L;

        /* [AI] A27：静摩擦突破 kick —— 从静止起步且误差超阈值时开环满扭矩
           **持续输出直到突破**（底盘 R33 同语义），位移达标即交 PI；
           kick 期间冻结堵转窗口计数（否则 1s 就被误判顶住）。 */
        if (s_kick_ms[a] != 0U) {
            s_kick_ms[a]--;
            int32_t moved = s_pos_counts[a] - s_kick_pos0[a];
            if (moved < 0L) moved = -moved;
            if (moved >= JOINT_BREAKAWAY_MOVED || pe == 0L) {
                /* A28：突破瞬间降速交棒——带着 95% 交接会炮弹式过冲（台架实录
                   J2 冲到 76.5° 撞 30° 故障闸门）。降到 30% + 清掉 kick 期间
                   充爆的积分器，让 PI 从温和起点接管。 */
                s_kick_ms[a] = 0U;
                if (s_duty_q[a] > JOINT_HANDOFF_DUTY)      s_duty_q[a] = JOINT_HANDOFF_DUTY;
                else if (s_duty_q[a] < -JOINT_HANDOFF_DUTY) s_duty_q[a] = -JOINT_HANDOFF_DUTY;
                s_integral[a] = 0;
            } else {
                s_still_wins[a] = 0U;
                s_move_acc[a] = 0L;
                s_win_ms[a] = 0U;
                s_pos_prev_1k[a] = s_pos_counts[a];
                int32_t elapsed = (int32_t)(JOINT_BREAKAWAY_MS_MAX - s_kick_ms[a]);
                int32_t kick = (err > 0L ? 1L : -1L) *
                               (JOINT_BREAKAWAY_DUTY0 + elapsed * JOINT_BREAKAWAY_RAMP);
                if (kick > JOINT_BREAKAWAY_DUTY_MAX)  kick = JOINT_BREAKAWAY_DUTY_MAX;
                if (kick < -JOINT_BREAKAWAY_DUTY_MAX) kick = -JOINT_BREAKAWAY_DUTY_MAX;
                int32_t kdq = kick - s_duty_q[a];
                if (kdq > JOINT_SLEW_Q) kdq = JOINT_SLEW_Q;
                else if (kdq < -JOINT_SLEW_Q) kdq = -JOINT_SLEW_Q;
                s_duty_q[a] += kdq;
                joint_apply(a, s_duty_q[a]);
                continue;
            }
        }
        if (s_duty_q[a] == 0L &&
            (pe > JOINT_BREAKAWAY_ERR || pe < -JOINT_BREAKAWAY_ERR)) {
            s_kick_ms[a] = JOINT_BREAKAWAY_MS_MAX;
            s_kick_pos0[a] = s_pos_counts[a];
            continue;   /* 下一拍起按 kick 通道输出 */
        }

        if (pe != 0L) {
            s_integral[a] += pe;
            if (s_integral[a] > JOINT_I_LIMIT) s_integral[a] = JOINT_I_LIMIT;
            if (s_integral[a] < -JOINT_I_LIMIT) s_integral[a] = -JOINT_I_LIMIT;
        }

        int32_t cmd = (pe * JOINT_KP_NUM) / JOINT_KP_DEN +
                      (s_integral[a] * JOINT_KI_NUM) / JOINT_KI_DEN;
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
