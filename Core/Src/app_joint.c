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
   [AI] 待实测：下面两行是**默认占位**，必须按装臂时的实际接线改。
        电机口 A = TIM10_CH1/TIM11_CH1 = PB8/PB9，编码器 A = TIM2 = PA15/PB3
        电机口 B = TIM9_CH1/CH2       = PE5/PE6，编码器 B = TIM3 = PB4/PB5
        电机口 C = TIM1_CH1/CH2       = PE9/PE11，编码器 C = TIM4 = PB6/PB7
        电机口 D = TIM1_CH3/CH4       = PE13/PE14，编码器 D = TIM5 = PA0/PA1
        机械臂板不驱动轮子，四路电机口都空闲 —— 选哪两路只取决于实物插头。 */
static const uint8_t kJointMotorCh[JOINT_COUNT] = { 0U, 1U };   /* {J1, J2} → A, B */
static const uint8_t kJointEncIdx[JOINT_COUNT]  = { 0U, 1U };   /* {J1, J2} → TIM2, TIM3 */

/* 电机出力方向 ↔ 关节角增大方向（+1 / -1）。[AI] 待实测：反了就是**正反馈飞车**，
   首版上板必须先做极性确认（README §7），确认后再改这里。 */
static const int8_t  kJointSign[JOINT_COUNT]    = { 1, 1 };

/* ---- 换算常数（一个物理量只允许一个常数） ---- */
#define JOINT_CNT_PER_DEG10_X100  1707U   /* counts per 0.1° × 100 = 61440/3600×100 */

/* ---- 软限位（相对零点，0.1°）。[AI] 待实测：J1/J2 允许角度范围未确认。 ---- */
#define JOINT_ANGLE_MIN_DEG10     0U
#define JOINT_ANGLE_MAX_DEG10     2700U
#define JOINT_TRAVEL_MARGIN_DEG10 200U    /* 越界容差 20°：只拦飞车，不误报正常超调 */

/* ---- 位置环（1 kHz 定点；[AI] 未上板整定的初值，首版上板必须重标） ---- */
#define JOINT_KP_NUM    10L      /* 单位误差 → 0.01% 占空比：100 counts(0.59°) → 10% */
#define JOINT_KP_DEN     1L
#define JOINT_KI_NUM     1L      /* 单位误差·ms → 0.01%：100 counts 持续 1 s → 20% */
#define JOINT_KI_DEN    50L
#define JOINT_I_LIMIT   300000L  /* 积分限幅（counts·ms）≈ 100 counts 误差持续 3 s */
#define JOINT_DEADBAND     8L    /* counts，≈0.05°：死区内冻结积分，防抖动泵占空比 */
#define JOINT_DUTY_LIM  8500L    /* 0.01% → 85%（与底盘板 MAX_DUTY_PERCENT 同口径） */
#define JOINT_SLEW_Q    2000L    /* 每 1 ms 允许的占空比变化（0.01%）→ 20%/ms 斜坡 */
#define JOINT_TGT_SLEW  1000L    /* 目标斜坡：每 1 ms 最多 1000 counts ≈ 35°/s（[AI] 待整定）
                                    作用：把"指令大跳变"变成受控斜坡，从而让下面的
                                    JOINT_ERR_FAULT 只拦**真的跟不上/飞车**，不误报正常大位移。
                                    若某轴物理速度低于 35°/s 出现误报，改小本常数。 */
#define JOINT_ERR_FAULT 5120L    /* 跟随误差（对**斜坡后目标**）>30° 判 FAULT（防飞车闸门） */

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

/* 门 / 状态 */
static volatile uint8_t s_estop;
static volatile uint8_t s_zeroed;
static volatile uint8_t s_fault[JOINT_COUNT];
static volatile uint8_t s_zero_req;
static volatile uint8_t s_clear_req;
static volatile uint8_t s_btn_pressed;
static uint32_t s_btn_low, s_btn_hold;
static uint8_t  s_btn_fired;
static uint8_t  s_isr_started;
static uint32_t s_win_cnt;

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
    if (pct > 85U) pct = 85U;                       /* JOINT_DUTY_LIM 同口径双保险 */
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

static int32_t counts_of_deg10(uint32_t deg10)
{
    return (int32_t)((deg10 * JOINT_CNT_PER_DEG10_X100) / 100U);
}

static uint16_t deg10_of_counts(int32_t counts)
{
    int32_t full = (int32_t)((JOINT_ANGLE_MAX_DEG10 * JOINT_CNT_PER_DEG10_X100) / 100U);
    if (counts <= 0L) return 0U;
    if (counts >= full) return (uint16_t)JOINT_ANGLE_MAX_DEG10;   /* 防 int32 乘法溢出 */
    return (uint16_t)((counts * 100L) / (int32_t)JOINT_CNT_PER_DEG10_X100);
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
    }
    s_zeroed = 0U;                      /* 零点未定义：未归零前不驱动 */
    s_estop = 0U;
    s_zero_req = 0U;
    s_clear_req = 0U;
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

void joint_set_target_deg10(unsigned axis, uint16_t deg10)
{
    if (axis >= JOINT_COUNT) return;
    if (deg10 > JOINT_ANGLE_MAX_DEG10) deg10 = JOINT_ANGLE_MAX_DEG10;
    s_target_counts[axis] = s_zero_counts[axis] + counts_of_deg10(deg10);
}

uint16_t joint_actual_deg10(unsigned axis)
{
    if (axis >= JOINT_COUNT) return 0U;
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
    }
    s_zeroed = 1U;
}

static void joint_fault_clear_apply(void)
{
    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        if (s_fault[a] == 0U) continue;
        int32_t e = s_target_counts[a] - s_pos_counts[a];
        int32_t rel = s_pos_counts[a] - s_zero_counts[a];
        int32_t lo = counts_of_deg10(JOINT_ANGLE_MIN_DEG10) -
                     counts_of_deg10(JOINT_TRAVEL_MARGIN_DEG10);
        int32_t hi = counts_of_deg10(JOINT_ANGLE_MAX_DEG10) +
                     counts_of_deg10(JOINT_TRAVEL_MARGIN_DEG10);
        if (e <= JOINT_ERR_FAULT && e >= -JOINT_ERR_FAULT && rel > lo && rel < hi) {
            s_fault[a] = 0U;
            s_integral[a] = 0;
            s_duty_q[a] = 0;
            s_tgt_slew[a] = s_pos_counts[a];   /* 斜坡从当前位置重新起步 */
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
        }
        isr_end(t0);
        return;
    }

    if (++s_win_cnt < JOINT_WIN_TICKS) { isr_end(t0); return; }
    s_win_cnt = 0U;

    /* ---- 4) 1 kHz：状态请求 + 位置 PI + 输出 ---- */
    if (s_zero_req != 0U)  { s_zero_req = 0U;  joint_zero_apply(); }
    if (s_clear_req != 0U) { s_clear_req = 0U; joint_fault_clear_apply(); }

    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        if (s_zeroed == 0U || s_fault[a] != 0U) {
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

        /* 软限位（含容差）：越界即 FAULT（未标定范围时容差给足，只拦飞车） */
        {
            int32_t rel = s_pos_counts[a] - s_zero_counts[a];
            int32_t lo = counts_of_deg10(JOINT_ANGLE_MIN_DEG10) -
                         counts_of_deg10(JOINT_TRAVEL_MARGIN_DEG10);
            int32_t hi = counts_of_deg10(JOINT_ANGLE_MAX_DEG10) +
                         counts_of_deg10(JOINT_TRAVEL_MARGIN_DEG10);
            if (rel < lo || rel > hi) {
                s_fault[a] = 1U;
                joint_brake(a);
                s_duty_q[a] = 0;
                s_integral[a] = 0;
                continue;
            }
        }

        /* 死区：死区内不加积分（防在噪声带里泵占空比导致嗡嗡抖） */
        int32_t pe = err;
        if (pe > JOINT_DEADBAND) pe -= JOINT_DEADBAND;
        else if (pe < -JOINT_DEADBAND) pe += JOINT_DEADBAND;
        else pe = 0L;

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
