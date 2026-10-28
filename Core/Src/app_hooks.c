/**
 * @file  app_hooks.c — FreeRTOS 钩子
 *
 * 目前只实现栈溢出钩子。为什么需要它：
 *   任务栈是从 FreeRTOS 堆（heap_4）里分配的，跟 TCB、其他任务的栈**紧挨着**。
 *   栈一溢出就静默踩坏邻居，症状是"调度器卡死但**没有 HardFault**" ——
 *   非常难查。开了 configCHECK_FOR_STACK_OVERFLOW=2（在 FreeRTOSConfig.h 的
 *   USER CODE 块里）之后，FreeRTOS 会在每次任务切换时查栈末端魔数，
 *   踩了就直接调到这里。
 *
 * 钩子里不能再用任何 RTOS API（调度器已经不可信），只直写 USART1 寄存器，
 * 然后停在一个可被调试器打断的死循环里。
 */
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"

static void panic_puts(const char *s)
{
    for (; *s; ++s) {
        while ((USART1->SR & 0x40U) == 0U) { }
        USART1->DR = (uint8_t)*s;
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    panic_puts("\r\n*** STACK OVERFLOW in task: ");
    panic_puts(pcTaskName ? pcTaskName : "?");
    panic_puts(" ***\r\n");
    for (;;) { /* 保持可被 SWD 挂上查看 */ }
}

/* 堆耗尽钩子（heap_4 提供），同样是"静默失败"的重灾区 */
void vApplicationMallocFailedHook(void)
{
    panic_puts("\r\n*** MALLOC FAILED (FreeRTOS heap exhausted) ***\r\n");
    for (;;) { }
}
