/**
  * @file       imu_task.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      IMU周期数据处理任务
  * @note       USART3中断负责接收和解析，本任务只消费最新数据并计算角加速度。
  */

#include "imu_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "imu.h"
#include "project_config.h"

volatile float g_fImuAngularVelocityZ = 0.0f;
volatile float g_fImuAngularAccelerationZ = 0.0f;
volatile uint8_t g_ucImuDataValid = 0U;

float fImuTaskGetAngularVelocityZ(void)
{
    return g_fImuAngularVelocityZ;
}

float fImuTaskGetAngularAccelerationZ(void)
{
    return g_fImuAngularAccelerationZ;
}

void vImuTask(void *pvParameters)
{
    uint32_t ulWakeTick = osKernelGetTickCount();
    uint32_t ulPreviousSampleTick = 0U;
    uint32_t ulPreviousSampleCount = 0U;
    float fPreviousAngularVelocity = 0.0f;

    (void)pvParameters;
    for (;;)
    {
        uint32_t ulSampleCount;
        float fAngularVelocity;

        /* 在同一临界区读取帧计数和数据，保证二者来自同一个有效帧。 */
        taskENTER_CRITICAL();
        ulSampleCount = ulImuGetGyroSampleCount(IMU_1);
        fAngularVelocity = fImuGetGyroZ(IMU_1);
        taskEXIT_CRITICAL();

        if (ulSampleCount != ulPreviousSampleCount)
        {
            uint32_t ulCurrentSampleTick = osKernelGetTickCount();

            g_fImuAngularVelocityZ = fAngularVelocity;
            if (g_ucImuDataValid != 0U)
            {
                uint32_t ulElapsedTicks =
                    ulCurrentSampleTick - ulPreviousSampleTick;
                if (ulElapsedTicks != 0U)
                {
                    g_fImuAngularAccelerationZ =
                        (fAngularVelocity - fPreviousAngularVelocity) /
                        ((float)ulElapsedTicks /
                         (float)configTICK_RATE_HZ);
                }
            }
            else
            {
                /* 第一帧只建立差分基准，角加速度从下一帧开始有效。 */
                g_fImuAngularAccelerationZ = 0.0f;
                g_ucImuDataValid = 1U;
            }

            fPreviousAngularVelocity = fAngularVelocity;
            ulPreviousSampleTick = ulCurrentSampleTick;
            ulPreviousSampleCount = ulSampleCount;
        }

        ulWakeTick += IMU_TASK_PERIOD_MS;
        (void)osDelayUntil(ulWakeTick);
    }
}
