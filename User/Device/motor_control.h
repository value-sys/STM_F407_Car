/**
  * @file       motor_control.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      双路电机速度闭环模块头文件
  */

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "motor.h"
#include <stdint.h>

extern volatile float g_fMotor1TargetRpm;
extern volatile float g_fMotor1ActualRpm;
extern volatile float g_fMotor2TargetRpm;
extern volatile float g_fMotor2ActualRpm;

void vMotorControlInit(void);
void vMotorControlUpdate(void);
void vMotorControlSetEnable(uint8_t ucEnable);
void vMotorControlSetTargetRpm(emDcMotorDevNumTdf emDevNum, float fTargetRpm);
float fMotorControlGetTargetRpm(emDcMotorDevNumTdf emDevNum);
float fMotorControlGetOutput(emDcMotorDevNumTdf emDevNum);
void vMotorControlStop(void);

#endif
