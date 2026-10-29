/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "iwdg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_arm.h"
#include "app_can.h"
#include "app_can_diag.h"
#include "app_joint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void task_can_rx(void *argument);
static void task_can_tx(void *argument);
static void task_status(void *argument);
static void task_can_diag(void *argument);   /* [AI] A3: CAN 互通诊断打印 */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  static const osThreadAttr_t canrx_attr = {
    .name = "canrx", .stack_size = 4096, .priority = (osPriority_t) osPriorityAboveNormal };
  static const osThreadAttr_t cantx_attr = {
    .name = "cantx", .stack_size = 4096, .priority = (osPriority_t) osPriorityNormal };
  static const osThreadAttr_t stat_attr = {
    .name = "stat", .stack_size = 4096, .priority = (osPriority_t) osPriorityBelowNormal };
  /* [AI] A3: 诊断线程（1 Hz 打一行 CAN 统计）。栈给 2048 B —— snprintf 需要。 */
  static const osThreadAttr_t candg_attr = {
    .name = "candg", .stack_size = 2048, .priority = (osPriority_t) osPriorityLow };
  /* [AI]  创建失败 = 堆不足。静默跑下去板子看起来"活着"但 CAN 全哑——
     这是最难查的失败模式，所以显式兜底（A1: 已无 ctrl 线程）。 */
  osThreadId_t h_canrx = osThreadNew(task_can_rx, NULL, &canrx_attr);  /* [AI]  CAN 接收解析 */
  osThreadId_t h_cantx = osThreadNew(task_can_tx, NULL, &cantx_attr);  /* [AI]  1kHz 发送 */
  osThreadId_t h_stat  = osThreadNew(task_status, NULL, &stat_attr);   /* [AI]  10Hz 喂狗 */
  osThreadId_t h_candg = osThreadNew(task_can_diag, NULL, &candg_attr);/* [AI] A3: CAN 诊断打印 */
  if (defaultTaskHandle == NULL || h_canrx == NULL ||
      h_cantx == NULL || h_stat == NULL || h_candg == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;   /* [AI]  未使用（其余四个任务同款写法），-Wextra 要求显式 */

  /* [AI] A2: 启动 J1/J2 的 25 kHz 位置环（TIM6 中断宿主 = stm32f4xx_it.c 的
     TIM6_DAC_IRQHandler）。与底盘板同思路：boot 与实时中断解耦，调度器起来后再开。 */
  joint_start_isr();

  /* [AI] A2: J3..J6 舵机标定沿用迁移前实测记录（ch1=PC7=J5、ch2=PC8=J4、
     ch3=PC9=J3、ch0=PC6=J6；MG995/996R 500µs=0°、2500µs=270°）。
     ⚠️ 转述自迁移前底盘板实测记录，本板装臂后需按实物复核一次。
     J1(ch5)/J2(ch4) 已改电机，不再适用本条。 */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* [AI] A1: 臂板没有 25 kHz 闭环，原 task_control（100 Hz 监督器）整体移除——
   舵机是纯硬件 PWM（app_servo），本板无周期性控制回路需要跑。 */

static void task_can_rx(void *argument)
{
  (void)argument;
  for (;;) {
    can_rx_task_step();                             /* [AI]  阻塞于队列 */
  }
}

static void task_can_tx(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    can_tx_task_step(HAL_GetTick());
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));       /* 1 kHz */
  }
}

static void task_status(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    HAL_IWDG_Refresh(&hiwdg);                       /* [AI]  A1: 10Hz 喂狗(原 car_feed_watchdog 同体) */
    /* [AI] A5: OLED 已从本板移除（占 PD11-14 四根 GPIO，且调试信息改由 USART 输出） */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));     /* 10 Hz */
  }
}

/* [AI] A3: CAN 互通诊断线程 —— 只打印，不做任何控制决策 */
static void task_can_diag(void *argument)
{
  app_can_diag_task(argument);
}

/* [AI]  ---- FreeRTOS 失败钩子（configCHECK_FOR_STACK_OVERFLOW=2 / malloc hook）----
   板子上没有控制台。做法：把现场留成 SWD 可直读的全局量，然后关中断空转，
   让 IWDG(2s) 复位。复位原因 'I'(IWDGRSTF) 会显示在 OLED 的 P 字段上
   （app_arm.c 的 arm_reset_cause），所以"反复显示 P I"= 有任务栈溢出或堆耗尽。 */
volatile char     dbg_overflow_task[configMAX_TASK_NAME_LEN];
volatile uint32_t dbg_overflow_cnt;
volatile uint32_t dbg_malloc_fail_cnt;

/* 签名按 FreeRTOS 侧的 char*（cmsis_os2.c 里那份 __WEAK 空实现是
   signed char*，属 ST 的笔误；强符号覆盖它即可）。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  dbg_overflow_cnt++;
  for (unsigned i = 0U; i < configMAX_TASK_NAME_LEN; ++i) {
    dbg_overflow_task[i] = pcTaskName[i];
    if (pcTaskName[i] == '\0') break;
  }
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
  dbg_malloc_fail_cnt++;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}

/* USER CODE END Application */

