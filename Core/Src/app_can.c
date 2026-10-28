/**
 * app_can.c — 机械臂板 CAN v4（臂帧编解码 / 收发任务）
 *
 * 帧 ID、Data ID、CRC 全部来自 docs/gen/can_protocol.h（由 tools/can_check.py
 * 从 docs/protocol-signals.yaml 生成），本文件不手写帧表。
 *
 * 板间分工（2026-10-28 双板架构）：0x110/111 只由机械臂板落地；
 * 底盘板不处理臂帧。两板的 0x030 / 0x220 同 ID 之争由本板改用
 * 0x031 ARM_NODE_INFO / 0x221 ARM_DIAG 规避。
 *
 * 安全：0x010 SAFETY_CMD 是 e2e 帧，CRC8（初值 = Data ID）在 **ISR 内**校验，
 * EMERGENCY_STOP 直接锁存（舵机立即冻结，不落地后续指令）。
 */
#include "app_can.h"
#include "app_arm.h"
#include "app_build.h"
#include "app_joint.h"
#include "main.h"
#include "can.h"
#include "can_protocol.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

/* ---- v4 量化（单点定义在 YAML；换算只在此处）----
   ARM_CMD:   uint16 × 0.1°  → 本板直接用 0.1°（app_arm 接受 deg10），无需换算。
   SERVO_STATE 同 0.1°。 */
#define PROTO_VER    CAN_PROTO_VER
#define HW_REV       1U
#define CMD_DEADMAN_MS  100U

struct can_frame {
    uint32_t id;
    uint8_t data[8];
};

static QueueHandle_t rx_queue;
static volatile uint8_t  can_clear_req;   /* SAFETY CLEAR_ESTOP 请求（ISR 置位，任务落地） */
static volatile uint8_t  can_clear_fault_req;  /* [AI] A2: SAFETY CLEAR_FAULT（复位 J1/J2 位置环故障） */
static volatile uint8_t  busoff_cnt;
static CAN_FilterTypeDef can_flt_saved;

uint8_t crc8(const uint8_t *p, uint8_t n, uint8_t init)
{
    uint8_t crc = init;
    while (n--) {
        crc ^= *p++;
        for (uint8_t b = 0; b < 8U; ++b)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* [AI] R57 教训沿用：bxCAN 过滤器寄存器不在 CAN_TypeDef 内，只能直读地址
   （FMR=0x200, FM1R=0x204, FS1R=0x20C, FFA1R=0x214, FA1R=0x21C, FR1=0x240）。 */
#define CAN1_FFA1R_ADDR  (*(volatile uint32_t *)0x40006614UL)
#define CAN1_FA1R_ADDR   (*(volatile uint32_t *)0x4000661CUL)

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void can_init(void)
{
    rx_queue = xQueueCreate(16U, sizeof(struct can_frame));

    CAN_FilterTypeDef flt = {0};
    flt.FilterBank = 0U;
    flt.FilterMode = CAN_FILTERMODE_IDMASK;
    flt.FilterScale = CAN_FILTERSCALE_32BIT;
    flt.FilterIdHigh = 0U;
    flt.FilterIdLow = 0U;
    flt.FilterMaskIdHigh = 0U;
    flt.FilterMaskIdLow = 0U;
    flt.FilterFIFOAssignment = CAN_RX_FIFO0;
    flt.FilterActivation = ENABLE;
    flt.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(&hcan1, &flt) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    can_flt_saved = flt;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t d[8];
    BaseType_t woken = pdFALSE;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        struct can_frame f;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) != HAL_OK) break;
        f.id = hdr.StdId;
        for (unsigned k = 0; k < 8U; ++k) f.data[k] = d[k];

        /* 0x010 SAFETY_CMD：e2e 快速路径，CRC 在 ISR 内校验。
           急停锁存立即生效（≤40µs 语义：舵机冻结不需要 ISR 参与，
           因为冻结 = 不写 CCR，任务侧也不会再落地）。 */
        if (f.id == SAFETY_CMD_ID) {
            if (crc8(f.data, 7U, SAFETY_CMD_DATA_ID) == f.data[7]) {
                switch (f.data[0] & 0x0FU) {
                case 1U:                                  /* START */
                    if (arm_estop_latched() == 0U) arm_set_remote(1U);
                    break;
                case 2U:                                  /* STOP */
                    arm_set_remote(0U);
                    break;
                case 4U:                                  /* EMERGENCY_STOP */
                    arm_estop_latch();
                    break;
                case 5U:                                  /* CLEAR_ESTOP */
                    can_clear_req = 1U;                   /* 任务侧落地 */
                    break;
                case 6U:                                  /* CLEAR_FAULT */
                    can_clear_fault_req = 1U;             /* [AI] A2: 复位 J1/J2 位置环故障 */
                    break;
                default:                                  /* 7（SOFT_RESET）：臂板不做 */
                    break;
                }
            }
            continue;   /* 不进队列：安全帧已在 ISR 处理完 */
        }

        if (f.id == ARM_CMD_A_ID || f.id == ARM_CMD_B_ID) {
            (void)xQueueSendFromISR(rx_queue, &f, &woken);
        }
        /* 其余 ID（轮速/轮遥测/其他节点）本节点不消费，静默丢弃 */
    }
    portYIELD_FROM_ISR(woken);
}

/* Task_CAN_Rx：出队 → 解码 → 落地。A/B 两帧凑齐 7 轴才下发一次（避免半套指令）。 */
void can_rx_task_step(void)
{
    struct can_frame f;
    if (xQueueReceive(rx_queue, &f, portMAX_DELAY) != pdTRUE) return;

    uint32_t now = HAL_GetTick();
    static uint16_t pending[7];
    static uint8_t have_a, have_b;

    if (f.id == ARM_CMD_A_ID) {
        for (unsigned i = 0; i < 4U; ++i)
            pending[i] = get_le16(&f.data[i * 2U]);
        have_a = 1U;
    } else if (f.id == ARM_CMD_B_ID) {
        for (unsigned i = 0; i < 3U; ++i)
            pending[4U + i] = get_le16(&f.data[i * 2U]);
        have_b = 1U;
    } else {
        return;
    }

    arm_note_cmd_rx(now);
    if (have_a == 0U || have_b == 0U) return;
    arm_command_deg10(pending, 0x7FU);
}

static int can_tx(uint32_t std_id, const uint8_t *d)
{
    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = std_id;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = 8U;
    return (HAL_CAN_AddTxMessage(&hcan1, &hdr, d, NULL) == HAL_OK) ? 1 : 0;
}

/* Task_CAN_Tx：1 kHz 底拍。相位错开避免同拍挤爆 3 个发送邮箱：
   轮位类底盘帧本板不发；臂帧 20+5 ms（A/B 两帧分相位）、DIAG 55 ms、NODE_INFO 999 ms。 */
void can_tx_task_step(uint32_t now_ms)
{
    static unsigned ph;
    static uint8_t busoff_prev;
    uint8_t d[8];

    /* 邮箱死锁自愈（沿用底盘板方案：任一邮箱忙超 150 ms 强制 ABRQ） */
    {
        static uint16_t tx_busy_ms[3];
        static const uint32_t tx_tme[3]  = { CAN_TSR_TME0, CAN_TSR_TME1, CAN_TSR_TME2 };
        static const uint32_t tx_abrq[3] = { CAN_TSR_ABRQ0, CAN_TSR_ABRQ1, CAN_TSR_ABRQ2 };
        for (unsigned m = 0U; m < 3U; ++m) {
            if ((CAN1->TSR & tx_tme[m]) != 0U) {
                tx_busy_ms[m] = 0U;
            } else if (++tx_busy_ms[m] > 150U) {
                CAN1->TSR = tx_abrq[m];
                tx_busy_ms[m] = 0U;
            }
        }
    }

    if (can_clear_req != 0U) {
        can_clear_req = 0U;
        arm_estop_clear();
    }
    if (can_clear_fault_req != 0U) {
        can_clear_fault_req = 0U;
        joint_fault_clear();   /* [AI] A2: 复检条件已消失才由 25 kHz ISR 真正清位 */
    }

    /* bus-off 计数（ABOM 自动恢复，ESR.BOFF 上升沿累计） */
    {
        uint8_t bo = (CAN1->ESR & CAN_ESR_BOFF) ? 1U : 0U;
        if (bo != 0U && busoff_prev == 0U && busoff_cnt < 255U) ++busoff_cnt;
        busoff_prev = bo;
    }

    /* RX 链路自愈 @1 Hz（过滤器被篡改 / FMPIE0 被清 → RX 静默死亡的实测签名） */
    if (ph == 500U) {
        if ((CAN1_FA1R_ADDR & 0x1UL) == 0U || (CAN1_FFA1R_ADDR & 0x1UL) != 0U) {
            (void)HAL_CAN_ConfigFilter(&hcan1, &can_flt_saved);
        }
        if ((CAN1->IER & CAN_IER_FMPIE0) == 0U) {
            (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }

    if (ph % 20U == 5U) {                        /* 50 Hz：SERVO_STATE_A（J1..J4） */
        for (unsigned i = 0; i < 4U; ++i)
            put_le16(&d[i * 2U], arm_axis_deg10(i));
        (void)can_tx(SERVO_STATE_A_ID, d);
    }
    if (ph % 20U == 15U) {                       /* 50 Hz：SERVO_STATE_B（J5,J6,夹爪,保留）
                                                    —— 相位于 A 帧错开，避免同拍抢邮箱 */
        put_le16(&d[0], arm_axis_deg10(4U));     /* J5 */
        put_le16(&d[2], arm_axis_deg10(5U));     /* J6 */
        put_le16(&d[4], 0U);                     /* 夹爪：本板无硬件通道 */
        put_le16(&d[6], 0U);                     /* RSVD16 */
        (void)can_tx(SERVO_STATE_B_ID, d);
    }
    if (ph % 100U == 55U) {                      /* 10 Hz：ARM_DIAG（55: 避开臂帧相位） */
        /* [AI] A2: FAULT_u16 仍发 0（冻结协议：不新增位语义）；
           J1/J2 位置环故障通过 **STATE_u16.bit2 fault_active**（YAML 已定义）上报。 */
        put_le16(&d[0], 0U);
        put_le16(&d[2], arm_state_bitmap(busoff_prev, now_ms));
        d[4] = arm_mode();
        d[5] = arm_servo_errcnt();
        put_le16(&d[6], 0U);                     /* RSVD16 */
        (void)can_tx(ARM_DIAG_ID, d);
    }
    if (ph == 999U) {                            /* 1 Hz：ARM_NODE_INFO（999: 独占毫秒） */
        uint16_t st = arm_state_bitmap(busoff_prev, now_ms);
        d[0] = (uint8_t)PROTO_VER;
        d[1] = (uint8_t)HW_REV;
        /* BUILD_u16：build_tag 两字符折成 16 位（"A1" → 0x4131），
           与底盘板的 build_hash16 不是同一算法，但同为"版本指纹"用途。 */
        d[2] = (uint8_t)build_tag[0];
        d[3] = (uint8_t)build_tag[1];
        put_le16(&d[4], 0U);                     /* FAULT */
        put_le16(&d[6], st);
        (void)can_tx(ARM_NODE_INFO_ID, d);
    }
    if (++ph >= 1000U) ph = 0U;
}