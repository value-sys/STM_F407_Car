/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : STM32F407 FreeRTOS application framework
  ******************************************************************************
  * @note 任务框架参考MSPM0工程，仍使用当前CubeMX生成的CMSIS-RTOS v2封装。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_init.h"
#include "chassis.h"
#include "debug_task.h"
#include "imu_task.h"
#include "motor_control.h"
#include "project_config.h"
#include "vofa_task.h"
#include "ui_task.h"
#include "vision_task.h"
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
volatile osThreadId_t g_xStackOverflowTask;
volatile const char *g_pcStackOverflowTaskName;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTest_Task */
osThreadId_t MotorTest_TaskHandle;
const osThreadAttr_t MotorTest_Task_attributes = {
  .name = "MotorTest_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GrayTrace_Task */
osThreadId_t GrayTrace_TaskHandle;
const osThreadAttr_t GrayTrace_Task_attributes = {
  .name = "GrayTrace_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for debugTask */
osThreadId_t debugTaskHandle;
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for vofaTask06 */
osThreadId_t vofaTask06Handle;
const osThreadAttr_t vofaTask06_attributes = {
  .name = "vofaTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorTest(void *argument);
void ImuTask03(void *argument);
void GrayTrace(void *argument);
void debug(void *argument);
void vofa06(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  vAppModuleInit();
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

  /* creation of MotorTest_Task */
  MotorTest_TaskHandle = osThreadNew(MotorTest, NULL, &MotorTest_Task_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(ImuTask03, NULL, &ImuTask_attributes);

  /* creation of GrayTrace_Task */
  GrayTrace_TaskHandle = osThreadNew(GrayTrace, NULL, &GrayTrace_Task_attributes);

  /* creation of debugTask */
  debugTaskHandle = osThreadNew(debug, NULL, &debugTask_attributes);

  /* creation of vofaTask06 */
  vofaTask06Handle = osThreadNew(vofa06, NULL, &vofaTask06_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  /* Infinite loop */
  for(;;)
  {
    vVisionTaskUpdate();
    vUiTaskUpdate();
    osDelay(2);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorTest */
/**
* @brief Function implementing the MotorTest_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTest */
void MotorTest(void *argument)
{
  /* USER CODE BEGIN MotorTest */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorTest */
}

/* USER CODE BEGIN Header_ImuTask03 */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask03 */
void ImuTask03(void *argument)
{
  /* USER CODE BEGIN ImuTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask03 */
}

/* USER CODE BEGIN Header_GrayTrace */
/**
* @brief Function implementing the GrayTrace_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GrayTrace */
void GrayTrace(void *argument)
{
  /* USER CODE BEGIN GrayTrace */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GrayTrace */
}

/* USER CODE BEGIN Header_debug */
/**
* @brief Function implementing the debugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_debug */
void debug(void *argument)
{
  /* USER CODE BEGIN debug */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END debug */
}

/* USER CODE BEGIN Header_vofa06 */
/**
* @brief Function implementing the vofaTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vofa06 */
void vofa06(void *argument)
{
  /* USER CODE BEGIN vofa06 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vofa06 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  FreeRTOS栈溢出钩子
  * @note   保存任务信息后停机，便于调试器定位。
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  g_xStackOverflowTask = (osThreadId_t)xTask;
  g_pcStackOverflowTaskName = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END Application */

