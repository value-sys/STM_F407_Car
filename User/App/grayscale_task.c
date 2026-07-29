/**
  * @file       grayscale_task.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      灰度传感器10ms周期采集任务
  */

#include "grayscale_task.h"
#include "GrayscaleSensor.h"
#include "cmsis_os2.h"
#include "project_config.h"

void vGrayscaleTask(void *pvParameters)
{
    uint32_t ulWakeTick = osKernelGetTickCount();
    (void)pvParameters;

    for (;;)
    {
        /* 任务只更新原始8位数字量，不调用后续循迹处理。 */
        vGrayscaleSensorTask(GRAYSCALE1);
        ulWakeTick += GRAYSCALE_SAMPLE_PERIOD_MS;
        (void)osDelayUntil(ulWakeTick);
    }
}
