/**
  * @file       cascade_pid.h
  * @version    V1.0.0
  * @date       20260730
  * @brief      电机位置-速度串级PID模块
  * @note       外环为位置式PID，输出目标转速；内环为增量式PID，输出PWM修正量。
  */

#ifndef _CASCADE_PID_H_
#define _CASCADE_PID_H_

#include "pid_controller.h"

typedef struct
{
    PID_t stPositionPid;          /* 位置式外环，输出目标RPM */
    PID_t stSpeedPid;             /* 增量式内环，输出PWM修正量 */
    float fTargetPosition;        /* 目标位置，编码器计数 */
    float fFeedforwardSpeed;      /* 运动学解算的前馈速度，RPM */
    float fTargetSpeedMax;        /* 总目标速度限幅，RPM */
    float fTargetSpeed;           /* 外环输出的目标速度，RPM */
    float fPosition;              /* 当前实际位置，编码器计数 */
    float fSpeed;                 /* 当前实际速度，RPM */
    float fOutput;                /* 内环输出，PWM相对停止值的修正量 */
} stCascadePidTdf;

void vCascadePidInit(stCascadePidTdf *pstCascade,
    float fPositionKp, float fPositionKi, float fPositionKd,
    float fPositionMaxSpeed,
    float fSpeedKp, float fSpeedKi, float fSpeedKd,
    float fSpeedMaxOutput, float fSpeedIntegralLimit,
    float fControlPeriod);
void vCascadePidReset(stCascadePidTdf *pstCascade);
void vCascadePidSetPositionMaxSpeed(stCascadePidTdf *pstCascade,
    float fPositionMaxSpeed);
void vCascadePidSetFeedforwardSpeed(stCascadePidTdf *pstCascade,
    float fFeedforwardSpeed);
void vCascadePidSetTargetSpeedMax(stCascadePidTdf *pstCascade,
    float fTargetSpeedMax);
float fCascadePidCalculate(stCascadePidTdf *pstCascade,
    float fPosition, float fSpeed, float fTargetPosition);

/**
 * @brief      仅执行串级PID的速度内环
 * @note       位置目标由外部的编码器完成比例控制生成。
 */
float fCascadePidCalculateSpeedOnly(stCascadePidTdf *pstCascade,
    float fSpeed, float fTargetSpeed);

#endif
