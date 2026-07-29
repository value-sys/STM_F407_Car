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
#include "imu.h"
#include "imu_task.h"
#include "motor.h"
#include "motor_control.h"
#include "project_config.h"

/// @brief      使用当前工程VOFA_SendFloat封装两路电机状态打印
/// @note       每个电机发送一帧，三个通道依次为：实际RPM、累计计数、PWM。
///             第一帧是左电机DC_MOTOR1，第二帧是右电机DC_MOTOR2。
void vVofaSendMotorInfo(void)
{
    const stDcMotorDeviceParamTdf *pstMotor1 =
        c_pstGetDcMotorDeviceParam(DC_MOTOR1);
    const stDcMotorDeviceParamTdf *pstMotor2 =
        c_pstGetDcMotorDeviceParam(DC_MOTOR2);

    VOFA_SendFloat(g_fMotor1ActualRpm,
        (float)c_pstGetEncoderDeviceParam(DC_MOTOR1)->stRunningParam.lCount,
        (float)pstMotor1->stRunningParam.usPwmValue);
    VOFA_SendFloat(g_fMotor2ActualRpm,
        (float)c_pstGetEncoderDeviceParam(DC_MOTOR2)->stRunningParam.lCount,
        (float)pstMotor2->stRunningParam.usPwmValue);
}

/// @brief      发送IMU航向角、Z轴角速度和Z轴角加速度
/// @note       三个通道单位依次为度、度每秒、度每二次方秒。
void vVofaSendImuInfo(void)
{
    VOFA_SendFloat(fImuGetYaw(IMU_1),
        fImuTaskGetAngularVelocityZ(),
        fImuTaskGetAngularAccelerationZ());
}

void vVofaTask(void *pvParameters)
{
    uint32_t ulWakeTick = osKernelGetTickCount();
    (void)pvParameters;

    for (;;)
    {
        /* IMU调试模式发送IMU三通道，其余模式打印左右电机状态。 */
        if (g_emDebugMode == emDebugModeImu)
        {
            vVofaSendImuInfo();
        }
        else
        {
            vVofaSendMotorInfo();
        }
        ulWakeTick += VOFA_SEND_PERIOD_MS;
        (void)osDelayUntil(ulWakeTick);
    }
}
