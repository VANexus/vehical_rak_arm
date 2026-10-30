/**
 * app_can_diag.c — [AI] A3 CAN 板间互通诊断打印（USART1 @921600，1 Hz 一行）
 *
 * 为什么需要它（2026-10-29 台架）：
 *   两块 C30D 之间要验证 CAN 互通，但当时**没有第三个节点**（Jetson 还没并进
 *   总线），而两板固件都没把 CAN 收帧计数露到 OLED —— 光看屏幕无法判定。
 *   本模块让**机械臂板**把自己看到的总线状况从 USART1 打出来，USB(CH9102F)
 *   一插，PC 端 /dev/ttyACM1 就能读到，判定只看一行数字。
 *
 * 打印内容与判据见 app_can_diag.h。
 */
#include "app_can_diag.h"
#include "app_bench.h"
#include "app_build.h"
#include "app_canopen.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>

/* 关注的对端（底盘板）帧 ID —— 取自 docs/protocol-signals.yaml */
#define DIAG_ID_CHASSIS_NODE   0x030U   /* NODE_INFO   1 Hz  */
#define DIAG_ID_CHASSIS_POS_A  0x200U   /* WHEEL_POS_A 100 Hz */
#define DIAG_ID_CHASSIS_POS_B  0x201U   /* WHEEL_POS_B 100 Hz */
#define DIAG_ID_CHASSIS_SPD    0x202U   /* WHEEL_SPD   100 Hz */
#define DIAG_ID_CHASSIS_DIAG   0x220U   /* DIAG        10 Hz  */

void app_can_diag_task(void *argument)
{
    (void)argument;
    char line[192];

    for (;;) {
        int n = snprintf(line, sizeof(line),
                         "[%s] CAN rx=%lu tx=%lu txfail=%lu bo=%lu esr=%08lX"
                         " | 030=%lu 200=%lu 201=%lu 202=%lu 220=%lu\r\n",
                         build_tag,
                         (unsigned long)app_can_rx_total(),
                         (unsigned long)app_can_tx_ok(),
                         (unsigned long)app_can_tx_fail(),
                         (unsigned long)app_can_busoff_cnt(),
                         (unsigned long)app_can_esr(),
                         (unsigned long)app_can_rx_of(DIAG_ID_CHASSIS_NODE),
                         (unsigned long)app_can_rx_of(DIAG_ID_CHASSIS_POS_A),
                         (unsigned long)app_can_rx_of(DIAG_ID_CHASSIS_POS_B),
                         (unsigned long)app_can_rx_of(DIAG_ID_CHASSIS_SPD),
                         (unsigned long)app_can_rx_of(DIAG_ID_CHASSIS_DIAG));
        if (n > 0) {
            uint16_t len = (uint16_t)((n < (int)sizeof(line))
                                      ? n : (int)sizeof(line) - 1);
            /* [AI] A3: 两路串口都发。C30D 有两个 CH9102F USB 口：
               USART1(PA9/PA10) = 调试口，USART3(PD8/PD9) = 对外通讯口。
               台架上插的是哪一个不确定，所以两路都打，插哪个都能读到。
               [AI] A6: 改走 bench_write() —— 与控制台共用唯一串口写入口与互斥，
               否则两个任务同时调 HAL_UART_Transmit 会把 huart 状态打乱。 */
            bench_write(line, len);
        }
        osDelay(1000U);
    }
}
