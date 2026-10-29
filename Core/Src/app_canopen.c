/**
 * app_canopen.c — 机械臂板 CANopen v5 适配层（P0 基础）
 *
 * 与底盘板同构：CANopenNode v4 + 官方 STM32 移植层，node 2，1 Mbps。
 * 上电停在 Pre-operational，等 Jetson NMT Start 后进 Operational。
 *
 * P0 只验证协议栈（心跳/SDO/Pre-operational）；臂帧命令与 SRDO 在 P1/P2
 * 接回 app_arm / app_joint，见 docs/superpowers/specs/2026-10-29-can-v5-canopen-design.md。
 */
#include "app_can.h"
#include "app_canopen.h"
#include "app_arm.h"
#include "app_build.h"
#include "app_imu.h"
#include "app_joint.h"

#include "CO_app_STM32.h"
#include "OD.h"
#include "can.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#define APP_CANOPEN_NODE_ID 2U
#define APP_CANOPEN_BAUDRATE 1000U
#define APP_CMD_DEADMAN_MS 100U

static CANopenNodeSTM32 s_node = {
    .desiredNodeID = APP_CANOPEN_NODE_ID,
    .baudrate = APP_CANOPEN_BAUDRATE,
    .timerHandle = NULL,
    .CANHandle = &hcan1,
    .HWInitFunction = MX_CAN1_Init,
};

static volatile uint32_t s_busoff_cnt;
static volatile uint8_t  s_busoff_prev;

void can_init(void)
{
    if (canopen_app_init(&s_node) != 0) {
        Error_Handler();
    }
}

void can_rx_task_step(void)
{
    canopen_app_process();
    vTaskDelay(pdMS_TO_TICKS(1));
}

void can_tx_task_step(uint32_t now_ms)
{
    (void)now_ms;
    canopen_app_interrupt();

    /* bus-off 上升沿计数（CANopenNode 把 bxCAN 错误状态聚合在 CANerrorStatus） */
    CO_t *co = canopenNodeSTM32->canOpenStack;
    if (co != NULL) {
        uint8_t bo = ((co->CANmodule->CANerrorStatus & CO_CAN_ERRTX_BUS_OFF) != 0U) ? 1U : 0U;
        if (bo != 0U && s_busoff_prev == 0U && s_busoff_cnt < 0xFFFFFFFFU) {
            ++s_busoff_cnt;
        }
        s_busoff_prev = bo;
    }
}

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

/* P0 诊断 shim：CANopenNode 不直接暴露累计收/发计数；P1 从 OD/EMCY 补齐。
   ESR 仍直读 bxCAN，bus-off 计数来自上面的上升沿。 */
uint32_t app_can_rx_total(void) { return 0U; }
uint32_t app_can_rx_of(uint32_t id) { (void)id; return 0U; }
uint32_t app_can_tx_ok(void) { return 0U; }
uint32_t app_can_tx_fail(void) { return 0U; }
uint32_t app_can_busoff_cnt(void) { return s_busoff_cnt; }
uint32_t app_can_esr(void) { return (uint32_t)CAN1->ESR; }
uint8_t app_can_vehicle_moving(void) { return 0U; }

/* ======================= P1 应用桥接 ======================= */

static uint32_t s_last_seq;
static uint32_t s_seq_ms;
static uint8_t  s_seq_seen;

static uint8_t node_operational(void)
{
    CO_t *co = (canopenNodeSTM32 != NULL) ? canopenNodeSTM32->canOpenStack : NULL;
    return (co != NULL && co->NMT != NULL &&
            co->NMT->operatingState == CO_NMT_OPERATIONAL) ? 1U : 0U;
}

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767)  return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

void app_canopen_step(uint32_t now_ms)
{
    CO_t *co = (canopenNodeSTM32 != NULL) ? canopenNodeSTM32->canOpenStack : NULL;
    if (co == NULL) return;

    uint32_t seq = OD_RAM.x2140_cmd_seq;
    if (seq != s_last_seq) {
        s_last_seq = seq;
        s_seq_ms = now_ms;
        s_seq_seen = 1U;
    }

    uint8_t enable = 0U;
    if (node_operational() != 0U && s_seq_seen != 0U &&
        (uint32_t)(now_ms - s_seq_ms) <= APP_CMD_DEADMAN_MS &&
        (OD_RAM.x2141_cmd_flags & 0x0001U) != 0U) {
        enable = 1U;
    }

    arm_set_remote(enable);
    if (enable != 0U) {
        uint16_t deg10[7] = {
            OD_RAM.x2100_J1cmd, OD_RAM.x2101_J2cmd, OD_RAM.x2102_J3cmd,
            OD_RAM.x2103_J4cmd, OD_RAM.x2104_J5cmd, OD_RAM.x2105_J6cmd,
            OD_RAM.x2106_gripcmd
        };
        arm_command_deg10(deg10, 0x7FU);
        arm_note_cmd_rx(now_ms);
    }

    /* 反馈：J1..J6 实际角；夹爪/第 8 轴本板无硬件，回 0 */
    OD_RAM.x2110_J1state = arm_axis_deg10(0U);
    OD_RAM.x2111_J2state = arm_axis_deg10(1U);
    OD_RAM.x2112_J3state = arm_axis_deg10(2U);
    OD_RAM.x2113_J4state = arm_axis_deg10(3U);
    OD_RAM.x2114_J5state = arm_axis_deg10(4U);
    OD_RAM.x2115_J6state = arm_axis_deg10(5U);
    OD_RAM.x2116_gripstate = 0U;
    OD_RAM.x2117_J8state = 0U;

    float g[3];
    imu_arm_gyro_dps(g);
    OD_RAM.x2120_imu_gz   = clamp_i16((int32_t)(g[2] * 100.0f));
    OD_RAM.x2121_imu_amag = imu_arm_amag_mg();
    OD_RAM.x2122_imu_evt  = imu_arm_events();
    OD_RAM.x2123_imu_age  = imu_arm_age_ms();
    OD_RAM.x2124_imu_rsvd = 0U;

    OD_RAM.x2130_state  = arm_state_bitmap(0U, now_ms);
    OD_RAM.x2131_fault  = 0U;
    OD_RAM.x2132_mode   = arm_mode();
    OD_RAM.x2133_errcnt = arm_servo_errcnt();

    OD_RAM.x2150_cmd_en  = enable;
    OD_RAM.x2151_cmd_age_ms = (s_seq_seen != 0U)
                               ? (uint16_t)((now_ms - s_seq_ms) > 0xFFFFU
                                                ? 0xFFFFU : (now_ms - s_seq_ms))
                               : 0xFFFFU;
    OD_RAM.x2154_joint_fault_bm = joint_fault();
    OD_RAM.x2155_joint_zeroed   = joint_zeroed();

    uint32_t packed = 0U;
    for (unsigned i = 0U; i < 4U && build_tag[i] != '\0'; ++i) {
        packed |= (uint32_t)(uint8_t)build_tag[i] << (8U * i);
    }
    OD_RAM.x2F00_tag_packed  = packed;
    OD_RAM.x2F01_reset_cause = arm_reset_cause();
}
