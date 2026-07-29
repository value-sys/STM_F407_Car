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
    emDebugModeImuRotate             ///< 执行IMU双环定角旋转测试
} emDebugModeTdf;

extern volatile emDebugModeTdf g_emDebugMode;
extern volatile float g_fDebugMotor1TargetRpm;
extern volatile float g_fDebugMotor2TargetRpm;
extern volatile uint16_t g_usDebugMotor1Pwm;
extern volatile uint16_t g_usDebugMotor2Pwm;
extern volatile float g_fDebugChassisVy;
extern volatile float g_fDebugChassisOmega;
extern volatile float g_fDebugImuRotateAngleDeg;
/// 0表示停车保护阶段，1表示正在前进。
extern volatile uint8_t g_ucDebugChassisTestState;

void vDebugTask(void *pvParameters);

#endif
