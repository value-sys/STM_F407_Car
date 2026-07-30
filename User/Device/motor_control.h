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
void vMotorControlStartCascadePosition(int32_t lMotor1TargetCount,
    int32_t lMotor2TargetCount);
void vMotorControlStartCascadePositionWithSpeed(
    int32_t lMotor1TargetCount, int32_t lMotor2TargetCount,
    float fMaxTargetRpm);
void vMotorControlStartCascadePositionWithFeedforward(
    int32_t lMotor1TargetCount, int32_t lMotor2TargetCount,
    float fMotor1FeedforwardRpm, float fMotor2FeedforwardRpm,
    float fMaxTargetRpm);
void vMotorControlCancelCascadePosition(void);
uint8_t ucMotorControlCascadePositionIsActive(void);
uint8_t ucMotorControlCascadePositionIsComplete(void);
float fMotorControlGetTargetRpm(emDcMotorDevNumTdf emDevNum);
float fMotorControlGetOutput(emDcMotorDevNumTdf emDevNum);
int32_t lMotorControlGetCascadeTargetCount(emDcMotorDevNumTdf emDevNum);
float fMotorControlGetCascadeProgressRatio(emDcMotorDevNumTdf emDevNum);
void vMotorControlStop(void);

#endif
