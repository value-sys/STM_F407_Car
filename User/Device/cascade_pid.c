/**
  * @file       cascade_pid.c
  * @version    V1.0.0
  * @date       20260730
  * @brief      电机位置-速度串级PID模块
  */

#include "cascade_pid.h"
#include <stddef.h>
#include <string.h>

static float fCascadePidLimit(float fValue, float fLimit)
{
    if (fValue > fLimit)
    {
        return fLimit;
    }
    if (fValue < -fLimit)
    {
        return -fLimit;
    }
    return fValue;
}

void vCascadePidInit(stCascadePidTdf *pstCascade,
    float fPositionKp, float fPositionKi, float fPositionKd,
    float fPositionMaxSpeed,
    float fSpeedKp, float fSpeedKi, float fSpeedKd,
    float fSpeedMaxOutput, float fSpeedIntegralLimit,
    float fControlPeriod)
{
    if (pstCascade == NULL)
    {
        return;
    }

    memset(pstCascade, 0, sizeof(*pstCascade));

    /* 外环使用位置式PID，输出目标转速。 */
    pstCascade->stPositionPid.fKp = fPositionKp;
    pstCascade->stPositionPid.fKi = fPositionKi;
    pstCascade->stPositionPid.fKd = fPositionKd;
    pstCascade->stPositionPid.fMaxOut = fPositionMaxSpeed;
    pstCascade->stPositionPid.fIntegralLimit = fPositionMaxSpeed;
    pstCascade->stPositionPid.fControlPeriod = fControlPeriod;
    pstCascade->fTargetSpeedMax = fPositionMaxSpeed;
    pstCascade->stPositionPid.usImprove = PID_INTEGRAL_LIMIT |
        PID_TRAPEZOID_INTEGRAL;
    PID_Init(&pstCascade->stPositionPid);

    /* 内环使用增量式PID，输出相对PWM停止值的修正量。 */
    pstCascade->stSpeedPid.fKp = fSpeedKp;
    pstCascade->stSpeedPid.fKi = fSpeedKi;
    pstCascade->stSpeedPid.fKd = fSpeedKd;
    pstCascade->stSpeedPid.fMaxOut = fSpeedMaxOutput;
    pstCascade->stSpeedPid.fIntegralLimit = fSpeedIntegralLimit;
    pstCascade->stSpeedPid.fControlPeriod = fControlPeriod;
    pstCascade->stSpeedPid.usImprove = PID_INCREMENTAL_OUTPUT |
        PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL |
        PID_DERIVATIVE_ON_MEASUREMENT;
    PID_Init(&pstCascade->stSpeedPid);
}

void vCascadePidReset(stCascadePidTdf *pstCascade)
{
    if (pstCascade == NULL)
    {
        return;
    }

    PID_Reset(&pstCascade->stPositionPid);
    PID_Reset(&pstCascade->stSpeedPid);
    pstCascade->fTargetPosition = 0.0f;
    pstCascade->fFeedforwardSpeed = 0.0f;
    pstCascade->fTargetSpeedMax = pstCascade->stPositionPid.fMaxOut;
    pstCascade->fTargetSpeed = 0.0f;
    pstCascade->fPosition = 0.0f;
    pstCascade->fSpeed = 0.0f;
    pstCascade->fOutput = 0.0f;
}

void vCascadePidSetPositionMaxSpeed(stCascadePidTdf *pstCascade,
    float fPositionMaxSpeed)
{
    if (pstCascade == NULL)
    {
        return;
    }
    if (fPositionMaxSpeed < 0.0f)
    {
        fPositionMaxSpeed = -fPositionMaxSpeed;
    }
    pstCascade->stPositionPid.fMaxOut = fPositionMaxSpeed;
    pstCascade->stPositionPid.fIntegralLimit = fPositionMaxSpeed;
}

void vCascadePidSetFeedforwardSpeed(stCascadePidTdf *pstCascade,
    float fFeedforwardSpeed)
{
    if (pstCascade != NULL)
    {
        pstCascade->fFeedforwardSpeed = fFeedforwardSpeed;
    }
}

void vCascadePidSetTargetSpeedMax(stCascadePidTdf *pstCascade,
    float fTargetSpeedMax)
{
    if (pstCascade == NULL)
    {
        return;
    }
    if (fTargetSpeedMax < 0.0f)
    {
        fTargetSpeedMax = -fTargetSpeedMax;
    }
    pstCascade->fTargetSpeedMax = fTargetSpeedMax;
}

float fCascadePidCalculate(stCascadePidTdf *pstCascade,
    float fPosition, float fSpeed, float fTargetPosition)
{
    if (pstCascade == NULL)
    {
        return 0.0f;
    }

    pstCascade->fPosition = fPosition;
    pstCascade->fSpeed = fSpeed;
    pstCascade->fTargetPosition = fTargetPosition;
    pstCascade->fTargetSpeed = pstCascade->fFeedforwardSpeed +
        PID_Calculate(
        &pstCascade->stPositionPid, fPosition, fTargetPosition);
    pstCascade->fTargetSpeed = fCascadePidLimit(
        pstCascade->fTargetSpeed, pstCascade->fTargetSpeedMax);
    pstCascade->fOutput = PID_Calculate(
        &pstCascade->stSpeedPid, fSpeed, pstCascade->fTargetSpeed);
    return pstCascade->fOutput;
}

float fCascadePidCalculateSpeedOnly(stCascadePidTdf *pstCascade,
    float fSpeed, float fTargetSpeed)
{
    if (pstCascade == NULL)
    {
        return 0.0f;
    }

    pstCascade->fPosition = 0.0f;
    pstCascade->fSpeed = fSpeed;
    pstCascade->fTargetPosition = 0.0f;
    pstCascade->fTargetSpeed = fCascadePidLimit(fTargetSpeed,
        pstCascade->fTargetSpeedMax);
    pstCascade->fOutput = PID_Calculate(&pstCascade->stSpeedPid,
        fSpeed, pstCascade->fTargetSpeed);
    return pstCascade->fOutput;
}
