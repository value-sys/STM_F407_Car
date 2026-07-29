/**
  * @file       motor_control.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      双路电机速度闭环模块
  */

#include "motor_control.h"
#include "encoder.h"
#include "pid_controller.h"
#include "project_config.h"
#include "ui_task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static PID_t s_astMotorPid[DC_MOTOR_DEV_NUM];
static volatile float s_afTargetRpm[DC_MOTOR_DEV_NUM];
static volatile float s_afPidOutput[DC_MOTOR_DEV_NUM];
static volatile uint8_t s_ucControlEnable = 1U;

volatile float g_fMotor1TargetRpm;
volatile float g_fMotor1ActualRpm;
volatile float g_fMotor2TargetRpm;
volatile float g_fMotor2ActualRpm;

static void vMotorControlInitPid(PID_t *pstPid,
    float fKp, float fKi, float fKd)
{
    memset(pstPid, 0, sizeof(PID_t));
    pstPid->fKp = fKp;
    pstPid->fKi = fKi;
    pstPid->fKd = fKd;
    pstPid->fMaxOut = MOTOR_PID_OUTPUT_MAX;
    pstPid->fIntegralLimit = MOTOR_PID_INTEGRAL_MAX;
    pstPid->fDeadBand = MOTOR_PID_DEADBAND;
    pstPid->fControlPeriod = (float)MOTOR_SAMPLE_TIME / 1000.0f;
    pstPid->fCoefA = MOTOR_PID_INTEGRAL_COEF_A;
    pstPid->fCoefB = MOTOR_PID_INTEGRAL_COEF_B;
    pstPid->fOutputLpfRc = MOTOR_PID_OUTPUT_LPF_RC;
    pstPid->fDerivativeLpfRc = MOTOR_PID_DERIVATIVE_LPF_RC;
    pstPid->usImprove = PID_INCREMENTAL_OUTPUT |
        PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL |
        PID_DERIVATIVE_ON_MEASUREMENT;
    PID_Init(pstPid);
}

void vMotorControlInit(void)
{
    vMotorControlInitPid(&s_astMotorPid[DC_MOTOR1],
        MOTOR1_PID_KP, MOTOR1_PID_KI, MOTOR1_PID_KD);
    vMotorControlInitPid(&s_astMotorPid[DC_MOTOR2],
        MOTOR2_PID_KP, MOTOR2_PID_KI, MOTOR2_PID_KD);
    vMotorControlStop();
    s_ucControlEnable = 1U;
}

/// @brief      更新两路编码器测速、PID和PWM
/// @note       必须以MOTOR_SAMPLE_TIME为固定周期调用。
void vMotorControlUpdate(void)
{
    uint8_t ucMotor;

    vEncoderUpdate(DC_MOTOR1);
    vEncoderUpdate(DC_MOTOR2);
    g_fMotor1ActualRpm = c_pstGetEncoderDeviceParam(DC_MOTOR1)->
        stRunningParam.fCurrentSpeed;
    g_fMotor2ActualRpm = c_pstGetEncoderDeviceParam(DC_MOTOR2)->
        stRunningParam.fCurrentSpeed;

    if (ucUiRunEnabled() == 0U)
    {
        vMotorControlStop();
        return;
    }

    if (s_ucControlEnable == 0U)
    {
        return;
    }
    for (ucMotor = 0U; ucMotor < DC_MOTOR_DEV_NUM; ucMotor++)
    {
        emDcMotorDevNumTdf emMotor = (emDcMotorDevNumTdf)ucMotor;
        const stDcMotorDeviceParamTdf *pstMotor =
            c_pstGetDcMotorDeviceParam(emMotor);
        float fCurrentRpm = c_pstGetEncoderDeviceParam(emMotor)->
            stRunningParam.fCurrentSpeed;
        float fOutput = PID_Calculate(&s_astMotorPid[emMotor],
            fCurrentRpm, s_afTargetRpm[emMotor]);
        int32_t lPwm = (int32_t)pstMotor->stStaticParam.usPwmStopValue +
            (int32_t)roundf(fOutput);

        if (lPwm < 0)
        {
            lPwm = 0;
        }
        else if (lPwm > (int32_t)pstMotor->stStaticParam.usPwmMaxValue)
        {
            lPwm = (int32_t)pstMotor->stStaticParam.usPwmMaxValue;
        }
        s_afPidOutput[emMotor] = fOutput;
        vDcMotorSetSpeed(emMotor, (uint16_t)lPwm);
    }
}

void vMotorControlSetEnable(uint8_t ucEnable)
{
    s_ucControlEnable = (ucEnable != 0U) ? 1U : 0U;
    if (s_ucControlEnable == 0U)
    {
        PID_Reset(&s_astMotorPid[DC_MOTOR1]);
        PID_Reset(&s_astMotorPid[DC_MOTOR2]);
    }
}

void vMotorControlSetTargetRpm(emDcMotorDevNumTdf emDevNum, float fTargetRpm)
{
    if (emDevNum < emDcMotorDevNumMax)
    {
        s_afTargetRpm[emDevNum] = fTargetRpm;
        if (emDevNum == DC_MOTOR1)
        {
            g_fMotor1TargetRpm = fTargetRpm;
        }
        else
        {
            g_fMotor2TargetRpm = fTargetRpm;
        }
    }
}

float fMotorControlGetTargetRpm(emDcMotorDevNumTdf emDevNum)
{
    return (emDevNum < emDcMotorDevNumMax) ? s_afTargetRpm[emDevNum] : 0.0f;
}

float fMotorControlGetOutput(emDcMotorDevNumTdf emDevNum)
{
    return (emDevNum < emDcMotorDevNumMax) ? s_afPidOutput[emDevNum] : 0.0f;
}

void vMotorControlStop(void)
{
    s_afTargetRpm[DC_MOTOR1] = 0.0f;
    s_afTargetRpm[DC_MOTOR2] = 0.0f;
    g_fMotor1TargetRpm = 0.0f;
    g_fMotor2TargetRpm = 0.0f;
    s_afPidOutput[DC_MOTOR1] = 0.0f;
    s_afPidOutput[DC_MOTOR2] = 0.0f;
    PID_Reset(&s_astMotorPid[DC_MOTOR1]);
    PID_Reset(&s_astMotorPid[DC_MOTOR2]);
    vDcMotorStop(DC_MOTOR1);
    vDcMotorStop(DC_MOTOR2);
}
