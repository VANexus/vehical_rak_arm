/*
 * app_bench.c — [AI] A6 台架串口控制台（USART1 @921600，单字符命令）
 *
 * 【它解决什么问题】
 *   2026-10-29 台架：三节点 CAN 已互通，J1/J2 是"轮子同款"增量编码器电机，
 *   但**方向、行程、起转占空比全都没实测**，J2 还有明确机械限位（一定会撞）。
 *   不能拿位置环去顶限位（积分会把占空比推到 85%），所以：
 *
 *     USB(Type-C) → CH9102F → USART1 → 本控制台
 *       按键 → joint_jog()（开环恒占空比 + 碰撞即停）→ 1 kHz ISR 执行
 *
 *   点动**不需要先归零**（这正是它的用途）；每次 1.2 s 自动刹；撞到立即刹并把
 *   位移/占空比打回来 —— 这一次点动走了多少、在哪个方向停的，就是标定数据。
 *
 * 【与其它模块的边界】
 *   - 只管"人按键 → 调 app_joint 的公开 API"，不直接碰 PWM/编码器；
 *   - bench_write() 是本板**唯一**串口写入口（互斥 + 双路），CAN 诊断打印也走它，
 *     避免两个任务同时调 HAL_UART_Transmit 把 huart 状态打乱。
 */
#include "app_bench.h"

#include "app_build.h"
#include "app_dbg.h"
#include "app_buzz.h"
#include "app_can.h"
#include "app_imu.h"
#include "app_joint.h"
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <stdarg.h>
#include <stdio.h>

#define BENCH_QUEUE_LEN   32U
#define BENCH_LINE_MAX    160U
#define BENCH_DUTY_STEP   500L    /* 点动占空比步长 5% */
#define BENCH_PRINT_MS    200U    /* 点动/监视时的状态行周期（5 Hz） */

static QueueHandle_t      s_rxq;
static SemaphoreHandle_t  s_tx_lock;
static uint8_t            s_rx_byte;
static uint8_t            s_monitor;
static int32_t            s_jog_pos0[JOINT_COUNT];   /* 本次点动起点（counts） */
static uint16_t           s_jog_was[JOINT_COUNT];

/* ======================= 串口（唯一写入口） ======================= */

void bench_write(const char *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) return;

    if (s_tx_lock != NULL) (void)xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50));
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100U);
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100U);
    if (s_tx_lock != NULL) (void)xSemaphoreGive(s_tx_lock);
}

void bench_printf(const char *fmt, ...)
{
    char line[BENCH_LINE_MAX];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return;

    uint16_t len = (uint16_t)((n < (int)sizeof(line)) ? n : ((int)sizeof(line) - 1));
    bench_write(line, len);
}

/* [AI] A11: 串口接收改为**轮询**（见 bench_task）。这里原来是一对
   HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback：CubeMX 的 -q 脚本开不了
   USART1 的 NVIC，中断接收在 ORE/FE 噪声下会形成错误中断风暴，把调度器饿死
   （实测：boot_stage 停在 61，OLED/蜂鸣器全无）。已整体删除，不启用 RX 中断。 */

/* ======================= 打印 ======================= */

static void bench_help(void)
{
    bench_printf("[bench] 1=J1+ 2=J1- 3=J2+ 4=J2- 0=stop | +/-=duty(%ld%%) | "
                 "p=print m=monitor k=zero c=clearfault i=imu j=isr\r\n",
                 (long)(joint_jog_get_duty() / 100L));
}

/* [AI] A7: IMU 状态 —— 碰撞确认/重力方向/实测采样率 */
static void bench_imu_status(void)
{
    float a[3];
    float g[3];
    imu_arm_accel_mg(a);
    imu_arm_gyro_dps(g);
    bench_printf("[imu] chip=%u rate=%u.%uHz age=%ums evt=0x%02X |a|=%umg impact=%lu fail=%lu\r\n",
                 (unsigned)imu_arm_chip(),
                 (unsigned)(imu_arm_rate_x10() / 10U),
                 (unsigned)(imu_arm_rate_x10() % 10U),
                 (unsigned)imu_arm_age_ms(),
                 (unsigned)imu_arm_events(),
                 (unsigned)imu_arm_amag_mg(),
                 (unsigned long)joint_impact_cnt(),
                 (unsigned long)imu_arm_fail_cnt());
    bench_printf("  a=%+ld,%+ld,%+ld mg  g=%+ld,%+ld,%+ld dps\r\n",
                 (long)a[0], (long)a[1], (long)a[2],
                 (long)g[0], (long)g[1], (long)g[2]);
}

/* [AI] A7: 单轴状态行 */
static void bench_axis_line(unsigned a)
{
    uint8_t f = (uint8_t)((joint_fault() >> a) & 1U);
    uint8_t c = (uint8_t)((joint_collide() >> a) & 1U);
    bench_printf("  J%u c=%+ld d10=%+ld st=%u F=%u C=%u duty=%+ld jog=%u\r\n",
                 (unsigned)(a + 1U),
                 (long)joint_pos_counts(a),
                 (long)joint_actual_deg10_signed(a),
                 (unsigned)joint_state((uint8_t)a),
                 (unsigned)f, (unsigned)c,
                 (long)joint_duty_q(a),
                 (unsigned)joint_jog_axis_active((uint8_t)a));
}

static void bench_print_status(void)
{
    bench_printf("[%s] zeroed=%u jog=0x%02X collide=0x%02X duty=%ld%%\r\n",
                 build_tag,
                 (unsigned)joint_zeroed(),
                 (unsigned)joint_jog_active(),
                 (unsigned)joint_collide(),
                 (long)(joint_jog_get_duty() / 100L));
    bench_axis_line(0U);
    bench_axis_line(1U);
}

/* ======================= 命令 ======================= */

static void bench_jog_press(uint8_t axis, int8_t dir)
{
    uint8_t f = (uint8_t)((joint_fault() >> axis) & 1U);
    joint_jog(axis, dir, joint_jog_get_duty(), 0U);   /* 0 → 默认 1.2 s */
    if (joint_jog_axis_active(axis) == 0U) {
        bench_printf("[bench] J%u jog refused (fault=%u; press c to clear)\r\n",
                     (unsigned)(axis + 1U), (unsigned)f);
    } else {
        buzz_alert(BUZZ_JOG);
        bench_printf("[bench] J%u jog dir=%+d duty=%ld%% 1200ms\r\n",
                     (unsigned)(axis + 1U), (int)dir,
                     (long)(joint_jog_get_duty() / 100L));
    }
}

static void bench_handle(char ch)
{
    switch (ch) {
    case '1': bench_jog_press(0U, 1); break;      /* J1 正转（角度增大方向） */
    case '2': bench_jog_press(0U, -1); break;
    case '3': bench_jog_press(1U, 1); break;      /* J2 */
    case '4': bench_jog_press(1U, -1); break;
    case '0':
        joint_jog_stop();
        bench_printf("[bench] jog stop\r\n");
        break;
    case '+':
        joint_jog_set_duty(joint_jog_get_duty() + BENCH_DUTY_STEP);
        bench_printf("[bench] duty=%ld%%\r\n", (long)(joint_jog_get_duty() / 100L));
        break;
    case '-':
        joint_jog_set_duty(joint_jog_get_duty() - BENCH_DUTY_STEP);
        bench_printf("[bench] duty=%ld%%\r\n", (long)(joint_jog_get_duty() / 100L));
        break;
    case 'p': bench_print_status(); break;
    case 'i': bench_imu_status(); break;
    case 'j':
        bench_printf("[isr] last=%lu max=%lu cycles (budget 2352)\r\n",
                     (unsigned long)dbg_joint_isr_last_cycles,
                     (unsigned long)dbg_joint_isr_max_cycles);
        break;
    case 'm':
        s_monitor = (uint8_t)((s_monitor == 0U) ? 1U : 0U);
        bench_printf("[bench] monitor=%u\r\n", (unsigned)s_monitor);
        break;
    case 'k':
        joint_set_zero();
        bench_printf("[bench] zero requested (current pose = 0.0 deg)\r\n");
        break;
    case 'c':
        joint_fault_clear();
        bench_printf("[bench] fault clear requested\r\n");
        break;
    case '?':
    case 'h': bench_help(); break;
    case '\r':
    case '\n': break;
    default: break;
    }
}

/* ======================= 生命周期 ======================= */

void bench_init(void)
{
    BOOT_MARK(64);
    s_rxq = xQueueCreate(BENCH_QUEUE_LEN, sizeof(s_rx_byte));
    BOOT_MARK(65);
    s_tx_lock = xSemaphoreCreateMutex();
    BOOT_MARK(66);
    if (s_rxq == NULL || s_tx_lock == NULL) {
        Error_Handler();
    }

    for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
        s_jog_pos0[a] = 0;
        s_jog_was[a] = 0U;
    }


    BOOT_MARK(67);
    /* [AI] A12: 横幅不在 init 里打 —— 实测 HAL_UART_Transmit 在调度器刚起步时
       会把 StartDefaultTask 卡死（boot_stage 停在 67，OLED/蜂鸣器全无）。
       改到任务首次迭代里打，init 只做资源创建。 */
}

void bench_task(void *argument)
{
    (void)argument;
    char ch;
    uint32_t last_print = 0U;
    uint8_t collide_seen = 0U;

    static uint8_t banner_done;
    for (;;) {
        if (banner_done == 0U) {
            banner_done = 1U;
            bench_printf("[%s] bench console ready (USART1/3 @921600). ? for help\r\n",
                         build_tag);
        }
        /* [AI] A11: **轮询** RXNE 代替中断接收。原因：CubeMX 脚本开不了 USART1 的 NVIC，
           而 HAL_UART_Receive_IT 在 ORE/FE 噪声下会触发错误中断风暴（回调里重复装载），
           把调度器饿死 —— 实测卡在 bench_init 之后（boot_stage=61）屏黑、无声。
           轮询只读 SR/DR（读 DR 自动清 RXNE/ORE），不碰寄存器初始化。 */
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
            char c = (char)(huart1.Instance->DR & 0xFFU);
            bench_handle(c);
        }
        if (xQueueReceive(s_rxq, &ch, pdMS_TO_TICKS(10)) == pdTRUE) {
            bench_handle(ch);
        }

        /* [AI] A7: 加速度计冲击 —— **只有车静止时**才算“机械臂碰撞确认”。
           车在动时加速度尖峰分不清是压到坑洼还是臂撞了限位，只当 bump 事件。 */
        if (imu_arm_take_impact() != 0U) {
            if (app_can_vehicle_moving() == 0U) {
                joint_note_impact();
                buzz_alert(BUZZ_IMPACT);
                bench_printf("[imu] IMPACT (stationary) -> jog aborted\r\n");
            } else {
                bench_printf("[imu] bump (moving) -> ignored for arm\r\n");
            }
        }

        /* 点动结束回报：走了多少 counts / 是不是撞停的 —— 这就是标定数据 */
        for (unsigned a = 0U; a < JOINT_COUNT; ++a) {
            uint8_t act = joint_jog_axis_active((uint8_t)a);
            if (act != 0U && s_jog_was[a] == 0U) {
                s_jog_pos0[a] = joint_pos_counts(a);
            } else if (act == 0U && s_jog_was[a] != 0U) {
                int32_t moved = joint_pos_counts(a) - s_jog_pos0[a];
                uint8_t hit = (uint8_t)((joint_collide() >> a) & 1U);
                bench_printf("[bench] J%u end: moved=%+ldc (%+ld d10) hit=%u\r\n",
                             (unsigned)(a + 1U), (long)moved,
                             (long)joint_actual_deg10_signed((uint8_t)a),
                             (unsigned)hit);
                bench_print_status();
            }
            s_jog_was[a] = act;
        }

        uint8_t cm = joint_collide();
        if (cm != collide_seen) {
            collide_seen = cm;
            if (cm != 0U) {
                bench_printf("[bench] COLLIDE mask=0x%02X -> braked\r\n", (unsigned)cm);
            }
        }

        uint32_t now = HAL_GetTick();
        if ((s_monitor != 0U || joint_jog_active() != 0U) &&
            ((uint32_t)(now - last_print) >= BENCH_PRINT_MS)) {
            last_print = now;
            if (s_monitor != 0U) {
                bench_print_status();
            } else {
                bench_axis_line(0U);
                bench_axis_line(1U);
            }
        }
    }
}
