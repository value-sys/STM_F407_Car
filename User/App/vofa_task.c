/**
  * @file       vofa_task.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      使用当前工程USART1的VOFA+电机监视任务
  */

#include "vofa_task.h"
#include "cmsis_os2.h"
#include "debug_task.h"
#include "encoder.h"
#include "function.h"
#include "GrayscaleSensor.h"
#include "imu.h"
#include "imu_task.h"
#include "motor_control.h"
#include "project_config.h"
#include "vofa_firewater.h"

/// @brief      同一FireWater帧发送两路电机的目标和实际转速
/// @note       通道顺序为：左目标RPM、左实际RPM、右目标RPM、右实际RPM。
/* Channels: M1 target RPM, M1 actual RPM, M1 target count, M1 actual count,
 * M2 target RPM, M2 actual RPM, M2 target count, M2 actual count. */
void vVofaSendMotorInfo(void)
{
    const stDcMotorEncoderDeviceParamTdf *pstEncoder1 =
        c_pstGetEncoderDeviceParam(DC_MOTOR1);
    const stDcMotorEncoderDeviceParamTdf *pstEncoder2 =
        c_pstGetEncoderDeviceParam(DC_MOTOR2);
    float afData[10] = {0.0f};

    afData[0] = fMotorControlGetTargetRpm(DC_MOTOR1);
    afData[1] = g_fMotor1ActualRpm;
    afData[2] = (float)lMotorControlGetCascadeTargetCount(DC_MOTOR1);
    if (pstEncoder1 != NULL)
    {
        afData[3] = (float)pstEncoder1->stRunningParam.lCount;
    }
    afData[4] = fMotorControlGetCascadeProgressRatio(DC_MOTOR1);
    afData[5] = fMotorControlGetTargetRpm(DC_MOTOR2);
    afData[6] = g_fMotor2ActualRpm;
    afData[7] = (float)lMotorControlGetCascadeTargetCount(DC_MOTOR2);
    if (pstEncoder2 != NULL)
    {
        afData[8] = (float)pstEncoder2->stRunningParam.lCount;
    }
    afData[9] = fMotorControlGetCascadeProgressRatio(DC_MOTOR2);
    vVofaFireWaterSend(afData, 10U);
}

/// @brief      发送IMU航向角、Z轴角速度和Z轴角加速度
/// @note       三个通道单位依次为度、度每秒、度每二次方秒。
void vVofaSendImuInfo(void)
{
    VOFA_SendFloat(fImuGetYaw(IMU_1),
        fImuTaskGetAngularVelocityZ(),
        fImuTaskGetAngularAccelerationZ());
}

/// @brief      同一FireWater帧发送灰度传感器D1~D8数字状态
/// @note       通道顺序固定为D1到D8，输出值仅为0或1。
void vVofaSendGrayscaleInfo(void)
{
    float afData[8];
    uint8_t ucRaw = ucGrayscaleSensorGetDigital(GRAYSCALE1);
    uint8_t ucBlackMask = (uint8_t)(~ucRaw);
    uint8_t ucIndex;

    for (ucIndex = 0U; ucIndex < 8U; ucIndex++)
    {
        afData[ucIndex] = (float)((ucBlackMask >> ucIndex) & 0x01U);
    }
    vVofaFireWaterSend(afData, 8U);
}

void vVofaTask(void *pvParameters)
{
    uint32_t ulWakeTick = osKernelGetTickCount();
    (void)pvParameters;

    for (;;)
    {
        if (g_emDebugMode == emDebugModeImu)
        {
            vVofaSendImuInfo();
        }
        else if (g_emDebugMode == emDebugModeGrayscale)
        {
            vVofaSendGrayscaleInfo();
        }
        else
        {
            vVofaSendMotorInfo();
        }
        ulWakeTick += VOFA_SEND_PERIOD_MS;
        (void)osDelayUntil(ulWakeTick);
    }
}
