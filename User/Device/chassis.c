/**
  * @file       chassis.c
  * @version    V1.1.0
  * @date       20260729
  * @brief      差速底盘运动学和IMU定角旋转模块
  * @note       PWM闭环由motor_control任务完成；IMU读取仍使用当前STM32驱动。
  */

#include "chassis.h"
#include "imu.h"
#include "pid_controller.h"
#include "project_config.h"
#include <stddef.h>
#include <string.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_DEG_TO_RAD (CHASSIS_PI / 180.0f)
#define CHASSIS_RAD_TO_DEG (180.0f / CHASSIS_PI)

static stChassisDeviceParamTdf s_stChassisDeviceParam;
static PID_t s_stImuTurnAnglePid;
static PID_t s_stImuTurnRatePid;

static float fChassisAbs(float fValue)
{
    return (fValue >= 0.0f) ? fValue : -fValue;
}

static float fChassisLimit(float fValue, float fLimit)
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

/// @brief      将航向角归一化到[-180°, 180°]
static float fChassisWrapYaw(float fAngle)
{
    while (fAngle > 180.0f)
    {
        fAngle -= 360.0f;
    }
    while (fAngle < -180.0f)
    {
        fAngle += 360.0f;
    }
    return fAngle;
}

static void vChassisPidInit(PID_t *pstPid, float fKp, float fKi,
    float fKd, float fMaxOut)
{
    memset(pstPid, 0, sizeof(PID_t));
    pstPid->fKp = fKp;
    pstPid->fKi = fKi;
    pstPid->fKd = fKd;
    pstPid->fMaxOut = fMaxOut;
    pstPid->fIntegralLimit = fMaxOut;
    pstPid->fControlPeriod = (float)MOTOR_SAMPLE_TIME / 1000.0f;
    pstPid->usImprove = PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL;
    PID_Init(pstPid);
}

const stChassisDeviceParamTdf *c_pstGetChassisDeviceParam(void)
{
    return &s_stChassisDeviceParam;
}

void vChassisDeviceInit(const stChassisStaticParamTdf *pstInit)
{
    if (pstInit == NULL)
    {
        return;
    }
    memcpy(&s_stChassisDeviceParam.stStaticParam,
        pstInit, sizeof(stChassisStaticParamTdf));
    memset(&s_stChassisDeviceParam.stRunningParam,
        0, sizeof(stChassisRunningParamTdf));
    s_stChassisDeviceParam.stRunningParam.emImuRotateState =
        emChassisImuRotateIdle;

    /* 航向角外环输出目标偏航角速度，角速度内环输出底盘omega。 */
    vChassisPidInit(&s_stImuTurnAnglePid,
        pstInit->fImuTurnAngleKp, pstInit->fImuTurnAngleKi,
        pstInit->fImuTurnAngleKd, pstInit->fImuTurnMaxYawRate);
    vChassisPidInit(&s_stImuTurnRatePid,
        pstInit->fImuTurnRateKp, pstInit->fImuTurnRateKi,
        pstInit->fImuTurnRateKd,
        pstInit->fImuTurnMaxOmega * CHASSIS_RAD_TO_DEG);
    vChassisStop();
}

/// @brief      设置底盘目标速度
/// @note       只更新目标，不进行PWM操作，可由其他任务安全地调用。
void vChassisSetSpeed(float fVy, float fOmega)
{
    s_stChassisDeviceParam.stRunningParam.vy = fVy;
    s_stChassisDeviceParam.stRunningParam.omega = fOmega;
}

/// @brief      将底盘目标换算为左右轮RPM
/// @note       DC_MOTOR1为左轮，DC_MOTOR2为右轮。
void vChassisUpdate(void)
{
    const stChassisStaticParamTdf *pstStatic =
        &s_stChassisDeviceParam.stStaticParam;
    stChassisRunningParamTdf *pstRunning =
        &s_stChassisDeviceParam.stRunningParam;
    float fLeftSpeed;
    float fRightSpeed;

    if (pstStatic->fWheelRadius <= 0.0f)
    {
        pstRunning->fLeftTargetSpeed = 0.0f;
        pstRunning->fRightTargetSpeed = 0.0f;
        return;
    }

    fLeftSpeed = pstRunning->vy -
        pstRunning->omega * pstStatic->fWheelBase * 0.5f;
    fRightSpeed = pstRunning->vy +
        pstRunning->omega * pstStatic->fWheelBase * 0.5f;
    pstRunning->fLeftTargetSpeed = fLeftSpeed * 60.0f /
        (2.0f * CHASSIS_PI * pstStatic->fWheelRadius);
    pstRunning->fRightTargetSpeed = fRightSpeed * 60.0f /
        (2.0f * CHASSIS_PI * pstStatic->fWheelRadius);
    if (pstStatic->ucRightMotorReverse != 0U)
    {
        pstRunning->fRightTargetSpeed = -pstRunning->fRightTargetSpeed;
    }
}

void vChassisStop(void)
{
    vChassisSetSpeed(0.0f, 0.0f);
}

void vChassisMove(float fVy)
{
    vChassisSetSpeed(fVy, 0.0f);
}

void vChassisRotate(float fOmega)
{
    vChassisSetSpeed(0.0f, fOmega);
}

/// @brief      单轮支点旋转
/// @param      fOmega 正值左轮停止，负值右轮停止。
void vChassisPivotRotate(float fOmega)
{
    float fAbsOmega = (fOmega >= 0.0f) ? fOmega : -fOmega;
    vChassisSetSpeed(fAbsOmega *
        s_stChassisDeviceParam.stStaticParam.fWheelBase * 0.5f, fOmega);
}

void vChassisImuRotateStart(float fRelativeAngleDeg)
{
    stChassisRunningParamTdf *pstRunning =
        &s_stChassisDeviceParam.stRunningParam;

    PID_Reset(&s_stImuTurnAnglePid);
    PID_Reset(&s_stImuTurnRatePid);
    pstRunning->ucImuTurnStableCount = 0U;

    /* 没收到有效角速度帧时禁止启动，避免使用默认零值误动作。 */
    if (ulImuGetGyroSampleCount(IMU_1) == 0U)
    {
        pstRunning->emImuRotateState = emChassisImuRotateNoImu;
        vChassisStop();
        return;
    }

    fRelativeAngleDeg = fChassisLimit(fRelativeAngleDeg, 180.0f);
    pstRunning->fImuTurnTargetYaw = fChassisWrapYaw(
        fImuGetYaw(IMU_1) + fRelativeAngleDeg);
    pstRunning->fImuTurnYawError = fRelativeAngleDeg;
    pstRunning->fImuTurnTargetYawRate = 0.0f;
    pstRunning->fImuTurnActualYawRate = fImuGetGyroZ(IMU_1);
    pstRunning->fImuTurnOutputOmega = 0.0f;
    pstRunning->emImuRotateState = emChassisImuRotateRunning;
}

void vChassisImuRotateUpdate(void)
{
    const stChassisStaticParamTdf *pstStatic =
        &s_stChassisDeviceParam.stStaticParam;
    stChassisRunningParamTdf *pstRunning =
        &s_stChassisDeviceParam.stRunningParam;
    float fRateOutput;
    float fDesiredOmega;
    float fOmegaStep;

    if (pstRunning->emImuRotateState != emChassisImuRotateRunning)
    {
        return;
    }
    if (ulImuGetGyroSampleCount(IMU_1) == 0U)
    {
        pstRunning->emImuRotateState = emChassisImuRotateNoImu;
        vChassisStop();
        return;
    }

    pstRunning->fImuTurnYawError = fChassisWrapYaw(
        pstRunning->fImuTurnTargetYaw - fImuGetYaw(IMU_1));
    pstRunning->fImuTurnActualYawRate = fImuGetGyroZ(IMU_1);
    pstRunning->fImuTurnTargetYawRate = PID_Calculate(
        &s_stImuTurnAnglePid, 0.0f, pstRunning->fImuTurnYawError);
    fRateOutput = PID_Calculate(&s_stImuTurnRatePid,
        pstRunning->fImuTurnActualYawRate,
        pstRunning->fImuTurnTargetYawRate);

    /* 当前IMU正角速度表示右转，而底盘负omega表示右转。 */
    fDesiredOmega = fChassisLimit(-fRateOutput * CHASSIS_DEG_TO_RAD,
        pstStatic->fImuTurnMaxOmega);
    fOmegaStep = pstStatic->fImuTurnOmegaSlewRate *
        ((float)MOTOR_SAMPLE_TIME / 1000.0f);
    if (fOmegaStep > 0.0f)
    {
        pstRunning->fImuTurnOutputOmega += fChassisLimit(
            fDesiredOmega - pstRunning->fImuTurnOutputOmega, fOmegaStep);
    }
    else
    {
        pstRunning->fImuTurnOutputOmega = fDesiredOmega;
    }

    /* 角度和角速度同时稳定若干周期后才判定完成，抑制过零抖动。 */
    if ((fChassisAbs(pstRunning->fImuTurnYawError) <=
         pstStatic->fImuTurnAngleTolerance) &&
        (fChassisAbs(pstRunning->fImuTurnActualYawRate) <=
         pstStatic->fImuTurnRateTolerance))
    {
        if (pstRunning->ucImuTurnStableCount < 255U)
        {
            pstRunning->ucImuTurnStableCount++;
        }
        if (pstRunning->ucImuTurnStableCount >=
            pstStatic->ucImuTurnStableCycles)
        {
            pstRunning->emImuRotateState = emChassisImuRotateCompleted;
            pstRunning->fImuTurnOutputOmega = 0.0f;
            vChassisStop();
            return;
        }
    }
    else
    {
        pstRunning->ucImuTurnStableCount = 0U;
    }

    vChassisPivotRotate(pstRunning->fImuTurnOutputOmega);
}

void vChassisImuRotateCancel(void)
{
    stChassisRunningParamTdf *pstRunning =
        &s_stChassisDeviceParam.stRunningParam;

    PID_Reset(&s_stImuTurnAnglePid);
    PID_Reset(&s_stImuTurnRatePid);
    pstRunning->fImuTurnTargetYawRate = 0.0f;
    pstRunning->fImuTurnOutputOmega = 0.0f;
    pstRunning->ucImuTurnStableCount = 0U;
    pstRunning->emImuRotateState = emChassisImuRotateIdle;
    vChassisStop();
}

emChassisImuRotateStateTdf emChassisImuRotateGetState(void)
{
    return s_stChassisDeviceParam.stRunningParam.emImuRotateState;
}
