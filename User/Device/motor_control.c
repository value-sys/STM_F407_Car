/**
  * @file       motor_control.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      双路电机速度闭环模块
  */

#include "motor_control.h"
#include "cascade_pid.h"
#include "encoder.h"
#include "pid_controller.h"
#include "project_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static PID_t s_astMotorPid[DC_MOTOR_DEV_NUM];
static stCascadePidTdf s_astCascadePid[DC_MOTOR_DEV_NUM];
static PID_t s_stCascadeSyncPid;
static volatile float s_afTargetRpm[DC_MOTOR_DEV_NUM];
static volatile float s_afPidOutput[DC_MOTOR_DEV_NUM];
static volatile float s_afCascadeFeedforwardRpm[DC_MOTOR_DEV_NUM];
static volatile int32_t s_alCascadeTargetCount[DC_MOTOR_DEV_NUM];
static volatile int32_t s_alCascadeStartCount[DC_MOTOR_DEV_NUM];
static volatile uint8_t s_aucCascadeMotorReached[DC_MOTOR_DEV_NUM];
static volatile uint8_t s_ucCascadeControlEnable;
static volatile uint8_t s_ucCascadeControlComplete;
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

static float fMotorControlAbs(float fValue)
{
    return (fValue >= 0.0f) ? fValue : -fValue;
}

static float fMotorControlSign(float fValue)
{
    if (fValue > 0.0f)
    {
        return 1.0f;
    }
    if (fValue < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

static float fMotorControlProgressRatio(emDcMotorDevNumTdf emMotor,
    float fPosition)
{
    float fTargetDelta = (float)s_alCascadeTargetCount[emMotor] -
        (float)s_alCascadeStartCount[emMotor];
    float fActualDelta = fPosition -
        (float)s_alCascadeStartCount[emMotor];

    if (fMotorControlAbs(fTargetDelta) < 1.0f)
    {
        return 1.0f;
    }
    return fActualDelta / fTargetDelta;
}

static uint8_t ucMotorControlCascadeReached(
    emDcMotorDevNumTdf emMotor, float fPosition)
{
    float fTargetDelta = (float)s_alCascadeTargetCount[emMotor] -
        (float)s_alCascadeStartCount[emMotor];
    float fTolerance = CASCADE_ROTATE_POSITION_TOLERANCE_COUNTS;

    if (fTargetDelta > 0.0f)
    {
        return (fPosition >=
            (float)s_alCascadeTargetCount[emMotor] - fTolerance) ? 1U : 0U;
    }
    if (fTargetDelta < 0.0f)
    {
        return (fPosition <=
            (float)s_alCascadeTargetCount[emMotor] + fTolerance) ? 1U : 0U;
    }
    return 1U;
}

static void vMotorControlInitCascadeSyncPid(void)
{
    memset(&s_stCascadeSyncPid, 0, sizeof(s_stCascadeSyncPid));
    s_stCascadeSyncPid.fKp = CASCADE_ROTATE_SYNC_PID_KP;
    s_stCascadeSyncPid.fKi = CASCADE_ROTATE_SYNC_PID_KI;
    s_stCascadeSyncPid.fKd = CASCADE_ROTATE_SYNC_PID_KD;
    s_stCascadeSyncPid.fMaxOut = CASCADE_ROTATE_SYNC_MAX_CORRECTION_RPM;
    s_stCascadeSyncPid.fIntegralLimit =
        CASCADE_ROTATE_SYNC_MAX_CORRECTION_RPM;
    s_stCascadeSyncPid.fControlPeriod =
        (float)MOTOR_SAMPLE_TIME / 1000.0f;
    s_stCascadeSyncPid.usImprove = PID_INTEGRAL_LIMIT |
        PID_TRAPEZOID_INTEGRAL;
    PID_Init(&s_stCascadeSyncPid);
}

static void vMotorControlCaptureCascadeStartCount(void)
{
    const stDcMotorEncoderDeviceParamTdf *pstEncoder1 =
        c_pstGetEncoderDeviceParam(DC_MOTOR1);
    const stDcMotorEncoderDeviceParamTdf *pstEncoder2 =
        c_pstGetEncoderDeviceParam(DC_MOTOR2);

    s_alCascadeStartCount[DC_MOTOR1] = (pstEncoder1 != NULL) ?
        pstEncoder1->stRunningParam.lCount : 0;
    s_alCascadeStartCount[DC_MOTOR2] = (pstEncoder2 != NULL) ?
        pstEncoder2->stRunningParam.lCount : 0;
}

static void vMotorControlApplyOutput(emDcMotorDevNumTdf emMotor,
    float fOutput)
{
    const stDcMotorDeviceParamTdf *pstMotor =
        c_pstGetDcMotorDeviceParam(emMotor);
    int32_t lPwm;

    if (pstMotor == NULL)
    {
        return;
    }

    lPwm = (int32_t)pstMotor->stStaticParam.usPwmStopValue +
        (int32_t)roundf(fOutput);
    if (lPwm < 0)
    {
        lPwm = 0;
    }
    else if (lPwm > (int32_t)pstMotor->stStaticParam.usPwmMaxValue)
    {
        lPwm = (int32_t)pstMotor->stStaticParam.usPwmMaxValue;
    }
    vDcMotorSetSpeed(emMotor, (uint16_t)lPwm);
}

void vMotorControlInit(void)
{
    vMotorControlInitPid(&s_astMotorPid[DC_MOTOR1],
        MOTOR1_PID_KP, MOTOR1_PID_KI, MOTOR1_PID_KD);
    vMotorControlInitPid(&s_astMotorPid[DC_MOTOR2],
        MOTOR2_PID_KP, MOTOR2_PID_KI, MOTOR2_PID_KD);
    vCascadePidInit(&s_astCascadePid[DC_MOTOR1],
        CASCADE_ROTATE_POSITION_PID_KP,
        CASCADE_ROTATE_POSITION_PID_KI,
        CASCADE_ROTATE_POSITION_PID_KD,
        CASCADE_ROTATE_POSITION_MAX_SPEED_RPM,
        CASCADE_ROTATE_SPEED_PID_KP,
        CASCADE_ROTATE_SPEED_PID_KI,
        CASCADE_ROTATE_SPEED_PID_KD,
        CASCADE_ROTATE_SPEED_OUTPUT_MAX,
        CASCADE_ROTATE_SPEED_INTEGRAL_MAX,
        (float)MOTOR_SAMPLE_TIME / 1000.0f);
    vCascadePidInit(&s_astCascadePid[DC_MOTOR2],
        CASCADE_ROTATE_POSITION_PID_KP,
        CASCADE_ROTATE_POSITION_PID_KI,
        CASCADE_ROTATE_POSITION_PID_KD,
        CASCADE_ROTATE_POSITION_MAX_SPEED_RPM,
        CASCADE_ROTATE_SPEED_PID_KP,
        CASCADE_ROTATE_SPEED_PID_KI,
        CASCADE_ROTATE_SPEED_PID_KD,
        CASCADE_ROTATE_SPEED_OUTPUT_MAX,
        CASCADE_ROTATE_SPEED_INTEGRAL_MAX,
        (float)MOTOR_SAMPLE_TIME / 1000.0f);
    vMotorControlInitCascadeSyncPid();
    s_ucCascadeControlEnable = 0U;
    s_ucCascadeControlComplete = 0U;
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

    if (s_ucControlEnable == 0U)
    {
        return;
    }
    if (s_ucCascadeControlEnable != 0U)
    {
        uint8_t ucAllComplete = 1U;
        float fSyncCorrection = 0.0f;

        /* 每个10ms控制周期比较两侧的完成比例，输出RPM同步修正量。 */
        {
            const stDcMotorEncoderDeviceParamTdf *pstEncoder1 =
                c_pstGetEncoderDeviceParam(DC_MOTOR1);
            const stDcMotorEncoderDeviceParamTdf *pstEncoder2 =
                c_pstGetEncoderDeviceParam(DC_MOTOR2);
            if ((pstEncoder1 != NULL) && (pstEncoder2 != NULL))
            {
                float fProgress1 = fMotorControlProgressRatio(DC_MOTOR1,
                    (float)pstEncoder1->stRunningParam.lCount);
                float fProgress2 = fMotorControlProgressRatio(DC_MOTOR2,
                    (float)pstEncoder2->stRunningParam.lCount);
                fSyncCorrection = PID_Calculate(&s_stCascadeSyncPid,
                    fProgress1, fProgress2);
            }
        }

        for (ucMotor = 0U; ucMotor < DC_MOTOR_DEV_NUM; ucMotor++)
        {
            emDcMotorDevNumTdf emMotor = (emDcMotorDevNumTdf)ucMotor;
            const stDcMotorEncoderDeviceParamTdf *pstEncoder =
                c_pstGetEncoderDeviceParam(emMotor);
            float fPosition = (float)pstEncoder->stRunningParam.lCount;
            float fSpeed = pstEncoder->stRunningParam.fCurrentSpeed;
            float fPositionError =
                (float)s_alCascadeTargetCount[emMotor] - fPosition;
            float fBaseFeedforwardSpeed =
                s_afCascadeFeedforwardRpm[emMotor];
            float fFeedforwardSpeed = s_afCascadeFeedforwardRpm[emMotor];

            if ((s_aucCascadeMotorReached[emMotor] == 0U) &&
                (ucMotorControlCascadeReached(emMotor, fPosition) != 0U))
            {
                s_aucCascadeMotorReached[emMotor] = 1U;
            }

            if (s_aucCascadeMotorReached[emMotor] != 0U)
            {
                fFeedforwardSpeed = 0.0f;
                fBaseFeedforwardSpeed = 0.0f;
            }

            if ((s_aucCascadeMotorReached[emMotor] == 0U) &&
                (fMotorControlAbs(fPositionError) <=
                CASCADE_ROTATE_FEEDFORWARD_DISABLE_COUNTS)
            )
            {
                /* 进入刹车定位区后撤掉恒定前馈，避免持续推动电机越过目标。 */
                fFeedforwardSpeed = 0.0f;
            }
            /* 不再使用位置环，完成比例同步PID直接生成目标RPM。 */
            if (s_aucCascadeMotorReached[emMotor] == 0U)
            {
                fFeedforwardSpeed += fSyncCorrection *
                fMotorControlSign(fBaseFeedforwardSpeed) *
                ((emMotor == DC_MOTOR1) ? 1.0f : -1.0f);
            }
            fFeedforwardSpeed = fMotorControlAbs(fFeedforwardSpeed) >
                CASCADE_ROTATE_TARGET_SPEED_MAX_RPM ?
                fMotorControlSign(fFeedforwardSpeed) *
                CASCADE_ROTATE_TARGET_SPEED_MAX_RPM : fFeedforwardSpeed;
            float fOutput = (s_aucCascadeMotorReached[emMotor] != 0U) ?
                0.0f : fCascadePidCalculateSpeedOnly(
                    &s_astCascadePid[emMotor], fSpeed, fFeedforwardSpeed);

            if ((s_aucCascadeMotorReached[emMotor] != 0U) ||
                ((fMotorControlAbs(fPositionError) <=
                    CASCADE_ROTATE_POSITION_TOLERANCE_COUNTS) &&
                (fMotorControlAbs(fSpeed) <=
                    CASCADE_ROTATE_SPEED_TOLERANCE_RPM)))
            {
                fOutput = 0.0f;
            }
            else
            {
                ucAllComplete = 0U;
            }

            s_afTargetRpm[emMotor] =
                s_astCascadePid[emMotor].fTargetSpeed;
            s_afPidOutput[emMotor] = fOutput;
            if (emMotor == DC_MOTOR1)
            {
                g_fMotor1TargetRpm = s_afTargetRpm[emMotor];
            }
            else
            {
                g_fMotor2TargetRpm = s_afTargetRpm[emMotor];
            }
            vMotorControlApplyOutput(emMotor, fOutput);
        }

        if (ucAllComplete != 0U)
        {
            s_ucCascadeControlEnable = 0U;
            s_ucCascadeControlComplete = 1U;
            s_afTargetRpm[DC_MOTOR1] = 0.0f;
            s_afTargetRpm[DC_MOTOR2] = 0.0f;
            g_fMotor1TargetRpm = 0.0f;
            g_fMotor2TargetRpm = 0.0f;
            s_afPidOutput[DC_MOTOR1] = 0.0f;
            s_afPidOutput[DC_MOTOR2] = 0.0f;
            vCascadePidReset(&s_astCascadePid[DC_MOTOR1]);
            vCascadePidReset(&s_astCascadePid[DC_MOTOR2]);
            vDcMotorStop(DC_MOTOR1);
            vDcMotorStop(DC_MOTOR2);
        }
        return;
    }

    for (ucMotor = 0U; ucMotor < DC_MOTOR_DEV_NUM; ucMotor++)
    {
        emDcMotorDevNumTdf emMotor = (emDcMotorDevNumTdf)ucMotor;
        float fCurrentRpm = c_pstGetEncoderDeviceParam(emMotor)->
            stRunningParam.fCurrentSpeed;
        float fOutput = PID_Calculate(&s_astMotorPid[emMotor],
            fCurrentRpm, s_afTargetRpm[emMotor]);

        s_afPidOutput[emMotor] = fOutput;
        vMotorControlApplyOutput(emMotor, fOutput);
    }
}

void vMotorControlSetEnable(uint8_t ucEnable)
{
    s_ucControlEnable = (ucEnable != 0U) ? 1U : 0U;
    if (s_ucControlEnable == 0U)
    {
        PID_Reset(&s_astMotorPid[DC_MOTOR1]);
        PID_Reset(&s_astMotorPid[DC_MOTOR2]);
        vMotorControlCancelCascadePosition();
    }
}

void vMotorControlStartCascadePosition(int32_t lMotor1TargetCount,
    int32_t lMotor2TargetCount)
{
    vMotorControlStartCascadePositionWithSpeed(lMotor1TargetCount,
        lMotor2TargetCount, CASCADE_ROTATE_POSITION_MAX_SPEED_RPM);
}

void vMotorControlStartCascadePositionWithSpeed(
    int32_t lMotor1TargetCount, int32_t lMotor2TargetCount,
    float fMaxTargetRpm)
{
    if (fMaxTargetRpm <= 0.0f)
    {
        fMaxTargetRpm = CASCADE_ROTATE_POSITION_MAX_SPEED_RPM;
    }
    s_alCascadeTargetCount[DC_MOTOR1] = lMotor1TargetCount;
    s_alCascadeTargetCount[DC_MOTOR2] = lMotor2TargetCount;
    s_aucCascadeMotorReached[DC_MOTOR1] = 0U;
    s_aucCascadeMotorReached[DC_MOTOR2] = 0U;
    vMotorControlCaptureCascadeStartCount();
    s_ucCascadeControlComplete = 0U;
    s_ucCascadeControlEnable = 1U;
    s_ucControlEnable = 1U;
    vCascadePidReset(&s_astCascadePid[DC_MOTOR1]);
    vCascadePidReset(&s_astCascadePid[DC_MOTOR2]);
    vCascadePidSetPositionMaxSpeed(&s_astCascadePid[DC_MOTOR1],
        fMaxTargetRpm);
    vCascadePidSetPositionMaxSpeed(&s_astCascadePid[DC_MOTOR2],
        fMaxTargetRpm);
    vCascadePidSetTargetSpeedMax(&s_astCascadePid[DC_MOTOR1],
        fMaxTargetRpm);
    vCascadePidSetTargetSpeedMax(&s_astCascadePid[DC_MOTOR2],
        fMaxTargetRpm);
    PID_Reset(&s_astMotorPid[DC_MOTOR1]);
    PID_Reset(&s_astMotorPid[DC_MOTOR2]);
    PID_Reset(&s_stCascadeSyncPid);
}

void vMotorControlStartCascadePositionWithFeedforward(
    int32_t lMotor1TargetCount, int32_t lMotor2TargetCount,
    float fMotor1FeedforwardRpm, float fMotor2FeedforwardRpm,
    float fMaxTargetRpm)
{
    if (fMaxTargetRpm <= 0.0f)
    {
        fMaxTargetRpm = CASCADE_ROTATE_TARGET_SPEED_MAX_RPM;
    }
    s_alCascadeTargetCount[DC_MOTOR1] = lMotor1TargetCount;
    s_alCascadeTargetCount[DC_MOTOR2] = lMotor2TargetCount;
    s_aucCascadeMotorReached[DC_MOTOR1] = 0U;
    s_aucCascadeMotorReached[DC_MOTOR2] = 0U;
    vMotorControlCaptureCascadeStartCount();
    s_afCascadeFeedforwardRpm[DC_MOTOR1] = fMotor1FeedforwardRpm;
    s_afCascadeFeedforwardRpm[DC_MOTOR2] = fMotor2FeedforwardRpm;
    s_ucCascadeControlComplete = 0U;
    s_aucCascadeMotorReached[DC_MOTOR1] = 0U;
    s_aucCascadeMotorReached[DC_MOTOR2] = 0U;
    s_ucCascadeControlEnable = 1U;
    s_ucControlEnable = 1U;
    vCascadePidReset(&s_astCascadePid[DC_MOTOR1]);
    vCascadePidReset(&s_astCascadePid[DC_MOTOR2]);
    vCascadePidSetPositionMaxSpeed(&s_astCascadePid[DC_MOTOR1],
        CASCADE_ROTATE_POSITION_MAX_SPEED_RPM);
    vCascadePidSetPositionMaxSpeed(&s_astCascadePid[DC_MOTOR2],
        CASCADE_ROTATE_POSITION_MAX_SPEED_RPM);
    vCascadePidSetTargetSpeedMax(&s_astCascadePid[DC_MOTOR1],
        fMaxTargetRpm);
    vCascadePidSetTargetSpeedMax(&s_astCascadePid[DC_MOTOR2],
        fMaxTargetRpm);
    vCascadePidSetFeedforwardSpeed(&s_astCascadePid[DC_MOTOR1],
        fMotor1FeedforwardRpm);
    vCascadePidSetFeedforwardSpeed(&s_astCascadePid[DC_MOTOR2],
        fMotor2FeedforwardRpm);
    PID_Reset(&s_astMotorPid[DC_MOTOR1]);
    PID_Reset(&s_astMotorPid[DC_MOTOR2]);
    PID_Reset(&s_stCascadeSyncPid);
}

void vMotorControlCancelCascadePosition(void)
{
    s_ucCascadeControlEnable = 0U;
    s_ucCascadeControlComplete = 0U;
    s_afCascadeFeedforwardRpm[DC_MOTOR1] = 0.0f;
    s_afCascadeFeedforwardRpm[DC_MOTOR2] = 0.0f;
    vCascadePidReset(&s_astCascadePid[DC_MOTOR1]);
    vCascadePidReset(&s_astCascadePid[DC_MOTOR2]);
    PID_Reset(&s_stCascadeSyncPid);
}

uint8_t ucMotorControlCascadePositionIsActive(void)
{
    return s_ucCascadeControlEnable;
}

uint8_t ucMotorControlCascadePositionIsComplete(void)
{
    return s_ucCascadeControlComplete;
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

float fMotorControlGetCascadeProgressRatio(emDcMotorDevNumTdf emDevNum)
{
    const stDcMotorEncoderDeviceParamTdf *pstEncoder =
        c_pstGetEncoderDeviceParam(emDevNum);
    if (pstEncoder == NULL)
    {
        return 0.0f;
    }
    return fMotorControlProgressRatio(emDevNum,
        (float)pstEncoder->stRunningParam.lCount);
}

int32_t lMotorControlGetCascadeTargetCount(emDcMotorDevNumTdf emDevNum)
{
    return (emDevNum < emDcMotorDevNumMax) ?
        s_alCascadeTargetCount[emDevNum] : 0;
}

void vMotorControlStop(void)
{
    s_ucCascadeControlEnable = 0U;
    s_ucCascadeControlComplete = 0U;
    s_afTargetRpm[DC_MOTOR1] = 0.0f;
    s_afTargetRpm[DC_MOTOR2] = 0.0f;
    g_fMotor1TargetRpm = 0.0f;
    g_fMotor2TargetRpm = 0.0f;
    s_afPidOutput[DC_MOTOR1] = 0.0f;
    s_afPidOutput[DC_MOTOR2] = 0.0f;
    PID_Reset(&s_astMotorPid[DC_MOTOR1]);
    PID_Reset(&s_astMotorPid[DC_MOTOR2]);
    vCascadePidReset(&s_astCascadePid[DC_MOTOR1]);
    vCascadePidReset(&s_astCascadePid[DC_MOTOR2]);
    vDcMotorStop(DC_MOTOR1);
    vDcMotorStop(DC_MOTOR2);
}
