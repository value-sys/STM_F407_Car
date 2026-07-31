/**
  * @file       debug_task.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      调试任务头文件
  */

#ifndef _DEBUG_TASK_H_
#define _DEBUG_TASK_H_

#include <stdint.h>

typedef enum
{
    emDebugModeNone = 0,             ///< 不执行主动测试，底盘保持停止
    emDebugModeMotorOpenLoop,        ///< 直接输出PWM，不执行速度PID
    emDebugModeMotorSpeedLoop,       ///< 使用编码器和PID测试两路电机速度
    emDebugModeEncoder,              ///< 保持停止，仅观察编码器测速
    emDebugModeImu,                  ///< 保持停止，通过VOFA观察IMU数据
    emDebugModeChassis,              ///< 上电保护延时后持续执行底盘前进测试
    emDebugModeImuRotate,            ///< 执行IMU双环定角旋转测试
    emDebugModeGrayscale,             ///< 保持停止，通过VOFA观察8路灰度数字量
    emDebugModeLineTrackImu,         ///< 灰度位置外环+IMU角速度内环循迹
    emDebugModeCurveLineTrackImu,    ///< 弯道循迹，丢线保持丢线前速度
    emDebugModeGrayLineTrack,        ///< 纯灰度循迹，不使用IMU
    emDebugModeLineRoute,            ///< ABCD整圈路线状态机
    emDebugModeCascadeRotate         ///< 位置-速度串级PID旋转测试
} emDebugModeTdf;

extern volatile emDebugModeTdf g_emDebugMode;
extern volatile float g_fDebugMotor1TargetRpm;
extern volatile float g_fDebugMotor2TargetRpm;
extern volatile uint16_t g_usDebugMotor1Pwm;
extern volatile uint16_t g_usDebugMotor2Pwm;
extern volatile float g_fDebugChassisVy;
extern volatile float g_fDebugChassisOmega;
extern volatile float g_fDebugImuRotateAngleDeg;
extern volatile float g_fDebugCascadeRotateRadiusMm;
extern volatile float g_fDebugCascadeRotateAngleDeg;
extern volatile float g_fDebugCascadeRotateOmegaRadS;
/// 0表示前进前停车，1表示前进，2表示后退前停车，3表示后退。
extern volatile uint8_t g_ucDebugChassisTestState;

void vDebugTask(void *pvParameters);

#endif
