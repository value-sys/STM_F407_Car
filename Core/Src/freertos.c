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
#include "grayscale_task.h"
#include "imu_task.h"
#include "line_ab_curve_test.h"
#include "line_route.h"
#include "line_track.h"
#include "motor_control.h"
#include "project_config.h"
#include "vofa_task.h"
#include "ui_task.h"
#include "vision_task.h"
#include "qd4310_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * CMSIS-RTOS v2的stack_size单位为字节；参考工程的配置单位为
 * StackType_t字数，因此在这里统一换算，避免直接照搬后缩小4倍。
 */
#define MOTOR_TASK_STACK_WORDS       640U
#define CHASSIS_TASK_STACK_WORDS     512U
#define IMU_TASK_STACK_WORDS         512U
#define DEBUG_TASK_STACK_WORDS       512U
#define VOFA_TASK_STACK_WORDS        1024U
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
  /* StartDefaultTask同时执行循迹状态机、IMU/灰度融合和底盘运动学。 */
  .stack_size = CHASSIS_TASK_STACK_WORDS * sizeof(StackType_t),
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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for vofaTask06 */
osThreadId_t vofaTask06Handle;
const osThreadAttr_t vofaTask06_attributes = {
  .name = "vofaTask06",
  /* VOFA帧使用浮点snprintf，增大栈空间避免格式化时栈溢出。 */
  .stack_size = VOFA_TASK_STACK_WORDS * sizeof(StackType_t),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for QD4310 test task */
osThreadId_t Qd4310TestTaskHandle;
const osThreadAttr_t Qd4310TestTask_attributes = {
  .name = "Qd4310TestTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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

  /* VOFA is temporarily disabled because USART1 is reserved for vision. */
  vofaTask06Handle = NULL;

  /* creation of QD4310 test task */
  Qd4310TestTaskHandle = osThreadNew(vQd4310TestTask, NULL,
      &Qd4310TestTask_attributes);

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
  uint32_t ulWakeTick = osKernelGetTickCount();

  (void)argument;
  for (;;)
  {
    vVisionTaskUpdate();
    vUiTaskUpdate();
    const stChassisDeviceParamTdf *pstChassis;

    /*
     * debug任务只写入vy/omega目标；本任务完成差速运动学换算，并把
     * 左右轮RPM交给电机速度环。DC_MOTOR1为左轮，DC_MOTOR2为右轮。
     */
    if ((emChassisImuRotateGetState() == emChassisImuRotateRunning) ||
        (g_emDebugMode == emDebugModeNone) ||
        (g_emDebugMode == emDebugModeChassis) ||
        (g_emDebugMode == emDebugModeImuRotate) ||
        (g_emDebugMode == emDebugModeLineTrackImu) ||
        (g_emDebugMode == emDebugModeCurveLineTrackImu) ||
        (g_emDebugMode == emDebugModeGrayLineTrack) ||
        (g_emDebugMode == emDebugModeLineRoute) ||
        (g_emDebugMode == emDebugModeAbCurveTest) ||
        (g_emDebugMode == emDebugModeCascadeRotate))
    {
      if (emChassisImuRotateGetState() == emChassisImuRotateRunning)
      {
        vChassisImuRotateUpdate();
      }

      if (g_emDebugMode == emDebugModeLineTrackImu)
      {
        /* 灰度负责位置外环，IMU角速度负责抑制弯道中的转动误差。 */
        vLineTrackImuUpdateByTargetRpm(LINE_TRACK_TARGET_RPM);
      }
      else if (g_emDebugMode == emDebugModeCurveLineTrackImu)
      {
        /* 弯道丢线时由循迹模块保持最近一次有效线速度。 */
        vLineTrackCurveImuUpdateByTargetRpm(LINE_TRACK_TARGET_RPM);
      }
      else if (g_emDebugMode == emDebugModeGrayLineTrack)
      {
        /* Pure grayscale tracking speed is configured by LINE_TRACK_GRAY_ONLY_TARGET_RPM. */
        vLineTrackUpdateByTargetRpm(LINE_TRACK_GRAY_ONLY_TARGET_RPM);
      }
      else if (g_emDebugMode == emDebugModeLineRoute)
      {
        /* 第2/5/6问共用路线：直线用灰度+IMU，弯道用纯灰度。 */
        vLineRouteUpdate();
        if (emLineRouteGetState() == emLineRouteStopped)
        {
          vUiStop();
        }
      }
      else if (g_emDebugMode == emDebugModeAbCurveTest)
      {
        /* AB直线后切换弯道循迹，并按独立时长停车。 */
        vLineAbCurveUpdate();
        if (emLineAbCurveGetState() == emLineAbCurveStopped)
        {
          vUiStop();
        }
      }

      vChassisUpdate();
      pstChassis = c_pstGetChassisDeviceParam();
      vMotorControlSetTargetRpm(DC_MOTOR1,
          pstChassis->stRunningParam.fLeftTargetSpeed);
      vMotorControlSetTargetRpm(DC_MOTOR2,
          pstChassis->stRunningParam.fRightTargetSpeed);
    }

    ulWakeTick += MOTOR_SAMPLE_TIME;
    (void)osDelayUntil(ulWakeTick);
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
  uint32_t ulWakeTick = osKernelGetTickCount();

  (void)argument;
  for (;;)
  {
    /* 固定10ms更新两路定时器编码器测速、速度PID和PWM。 */
    vMotorControlUpdate();
    ulWakeTick += MOTOR_SAMPLE_TIME;
    (void)osDelayUntil(ulWakeTick);
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
  /* IMU底层继续使用当前工程USART3中断，本任务只处理最新有效数据。 */
  vImuTask(argument);
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
  /* 每10ms读取一次8路数字灰度数据，循迹处理由后续功能单独调用。 */
  vGrayscaleTask(argument);
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
  /* 默认进入底盘测试：停车保护3秒后，以100RPM对应线速度前进。 */
  vDebugTask(argument);
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
  /* 复用当前工程VOFA_SendFloat，每100ms打印一次两路电机状态。 */
  vVofaTask(argument);
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

