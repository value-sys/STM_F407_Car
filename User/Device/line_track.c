/**
  * @file       line_track.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      基于灰度位置外环和IMU角速度内环的差速底盘循迹
  */

#include "line_track.h"
#include "GrayscaleSensor.h"
#include "chassis.h"
#include "imu.h"
#include "pid_controller.h"
#include "project_config.h"
#include <stddef.h>
#include <string.h>

#define LINE_TRACK_POSITION_MAX       100.0f
#define LINE_TRACK_PI                 3.14159265358979323846f
#define LINE_TRACK_DEG_TO_RAD         (LINE_TRACK_PI / 180.0f)

/* D1~D4位于左侧，D5~D8位于右侧。 */
static const int8_t s_acLinePosition[8] =
    {-80, -50, -15, -10, 10, 15, 50, 80};

static stLineTrackDeviceParamTdf s_stLineTrackDeviceParam;
static PID_t s_stStraightGrayPid;
static PID_t s_stYawAnglePid;
static PID_t s_stStraightYawRatePid;
static PID_t s_stCurveGrayPid;
static PID_t s_stCurveYawRatePid;
static uint8_t s_ucStraightYawCaptured;
static float s_fLastValidCurveSpeed;

typedef enum
{
    emLineTrackControlDisabled = 0U,
    emLineTrackControlGrayOnly,
    emLineTrackControlGrayImu,
    emLineTrackControlCurveImu,
    emLineTrackControlImuStraight
} emLineTrackControlModeTdf;

static emLineTrackControlModeTdf s_emControlMode;

static float fLineTrackLimit(float fValue, float fLimit)
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

static float fLineTrackRpmToLinearSpeed(float fTargetRpm)
{
    const stChassisDeviceParamTdf *pstChassis =
        c_pstGetChassisDeviceParam();

    return fTargetRpm * 2.0f * LINE_TRACK_PI *
        pstChassis->stStaticParam.fWheelRadius / 60.0f;
}

static float fLineTrackWrapYawError(float fError)
{
    while (fError > 180.0f)
    {
        fError -= 360.0f;
    }
    while (fError < -180.0f)
    {
        fError += 360.0f;
    }
    return fError;
}

static void vLineTrackPidInit(PID_t *pstPid, float fKp, float fKi,
    float fKd, float fMaxOut)
{
    memset(pstPid, 0, sizeof(*pstPid));
    pstPid->fKp = fKp;
    pstPid->fKi = fKi;
    pstPid->fKd = fKd;
    pstPid->fMaxOut = fMaxOut;
    pstPid->fIntegralLimit = fMaxOut;
    pstPid->fControlPeriod = (float)LINE_TRACK_TASK_PERIOD_MS / 1000.0f;
    pstPid->usImprove = PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL;
    PID_Init(pstPid);
}

static void vLineTrackEnterMode(emLineTrackControlModeTdf emMode)
{
    if (s_emControlMode == emMode)
    {
        return;
    }
    PID_Reset(&s_stStraightGrayPid);
    PID_Reset(&s_stYawAnglePid);
    PID_Reset(&s_stStraightYawRatePid);
    PID_Reset(&s_stCurveGrayPid);
    PID_Reset(&s_stCurveYawRatePid);
    s_ucStraightYawCaptured = 0U;
    s_emControlMode = emMode;
}

static void vLineTrackUpdateSensorState(void)
{
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    uint8_t ucDigital = ucGrayscaleSensorGetDigital(GRAYSCALE1);
    int16_t sPositionSum = 0;
    uint8_t ucChannel;

    /* 当前灰度数字量为1表示白色，取反后得到黑线位图。 */
    pstRunning->ucBlackMask = (uint8_t)(~ucDigital);
    pstRunning->ucActiveCount = 0U;
    for (ucChannel = 0U; ucChannel < 8U; ucChannel++)
    {
        if ((pstRunning->ucBlackMask & (uint8_t)(1U << ucChannel)) != 0U)
        {
            sPositionSum += s_acLinePosition[ucChannel];
            pstRunning->ucActiveCount++;
        }
    }

    if (pstRunning->ucActiveCount == 0U)
    {
        pstRunning->emState = emLineTrackStateLost;
        pstRunning->fLinePosition = 0.0f;
        pstRunning->fLineError = 0.0f;
    }
    else if (pstRunning->ucBlackMask == 0xFFU)
    {
        pstRunning->emState = emLineTrackStateIntersection;
        pstRunning->fLinePosition = 0.0f;
        pstRunning->fLineError = 0.0f;
    }
    else
    {
        pstRunning->emState = emLineTrackStateTracking;
        pstRunning->fLinePosition = (float)sPositionSum /
            (float)pstRunning->ucActiveCount;
        pstRunning->fLineError = pstRunning->fLinePosition /
            LINE_TRACK_POSITION_MAX;
        if (pstRunning->fLineError < 0.0f)
        {
            pstRunning->cLastDirection = -1;
        }
        else if (pstRunning->fLineError > 0.0f)
        {
            pstRunning->cLastDirection = 1;
        }
    }
}

static void vLineTrackApplyYawRateControl(float fCommandSpeed,
    float fTargetYawRate, PID_t *pstYawRatePid,
    float fMaxTargetYawRate)
{
    stLineTrackStaticParamTdf *pstStatic =
        &s_stLineTrackDeviceParam.stStaticParam;
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    float fYawRateCorrection = 0.0f;

    pstRunning->fTargetYawRate = fLineTrackLimit(fTargetYawRate,
        fMaxTargetYawRate);
    pstRunning->fActualYawRate = fImuGetGyroZ(IMU_1);

    if (ulImuGetGyroSampleCount(IMU_1) != 0U)
    {
        fYawRateCorrection = PID_Calculate(pstYawRatePid,
            pstRunning->fActualYawRate, pstRunning->fTargetYawRate);
    }
    else
    {
        PID_Reset(pstYawRatePid);
    }

    /* 符号由当前底盘实测方向配置，避免沿用参考工程的相反坐标约定。 */
    pstRunning->fOuterControlOmega = LINE_TRACK_SENSOR_TURN_SIGN *
        pstRunning->fTargetYawRate * LINE_TRACK_DEG_TO_RAD;
    pstRunning->fImuControlOmega = LINE_TRACK_IMU_TURN_SIGN *
        fYawRateCorrection * LINE_TRACK_DEG_TO_RAD;
    pstRunning->fCorrectionOmega = fLineTrackLimit(
        pstStatic->fOuterControlWeight * pstRunning->fOuterControlOmega +
        pstStatic->fImuFeedbackWeight * pstRunning->fImuControlOmega,
        pstStatic->fMaxCorrectionOmega);
    pstRunning->fCommandSpeed = fCommandSpeed;
    vChassisSetSpeed(fCommandSpeed, pstRunning->fCorrectionOmega);
}

const stLineTrackDeviceParamTdf *c_pstGetLineTrackDeviceParam(void)
{
    return &s_stLineTrackDeviceParam;
}

void vLineTrackDeviceInit(const stLineTrackStaticParamTdf *pstInit)
{
    memset(&s_stLineTrackDeviceParam, 0,
        sizeof(s_stLineTrackDeviceParam));
    s_emControlMode = emLineTrackControlDisabled;
    s_ucStraightYawCaptured = 0U;
    s_fLastValidCurveSpeed = 0.0f;
    if (pstInit == NULL)
    {
        return;
    }

    memcpy(&s_stLineTrackDeviceParam.stStaticParam, pstInit,
        sizeof(*pstInit));
    s_stLineTrackDeviceParam.stRunningParam.emState =
        emLineTrackStateDisabled;
    vLineTrackPidInit(&s_stStraightGrayPid,
        pstInit->fStraightGrayPidKp,
        pstInit->fStraightGrayPidKi, pstInit->fStraightGrayPidKd,
        pstInit->fStraightMaxTargetYawRate);
    vLineTrackPidInit(&s_stYawAnglePid, pstInit->fYawAnglePidKp,
        pstInit->fYawAnglePidKi, pstInit->fYawAnglePidKd,
        pstInit->fStraightMaxTargetYawRate);
    vLineTrackPidInit(&s_stStraightYawRatePid,
        pstInit->fStraightYawRatePidKp,
        pstInit->fStraightYawRatePidKi,
        pstInit->fStraightYawRatePidKd,
        pstInit->fStraightMaxTargetYawRate);
    vLineTrackPidInit(&s_stCurveGrayPid,
        pstInit->fCurveGrayPidKp,
        pstInit->fCurveGrayPidKi, pstInit->fCurveGrayPidKd,
        pstInit->fCurveMaxTargetYawRate);
    vLineTrackPidInit(&s_stCurveYawRatePid,
        pstInit->fCurveYawRatePidKp,
        pstInit->fCurveYawRatePidKi,
        pstInit->fCurveYawRatePidKd,
        pstInit->fCurveMaxTargetYawRate);
}

void vLineTrackStart(void)
{
    s_stLineTrackDeviceParam.stRunningParam.ucEnable = 1U;
}

void vLineTrackStop(void)
{
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;

    memset(pstRunning, 0, sizeof(*pstRunning));
    pstRunning->emState = emLineTrackStateDisabled;
    PID_Reset(&s_stStraightGrayPid);
    PID_Reset(&s_stYawAnglePid);
    PID_Reset(&s_stStraightYawRatePid);
    PID_Reset(&s_stCurveGrayPid);
    PID_Reset(&s_stCurveYawRatePid);
    s_emControlMode = emLineTrackControlDisabled;
    s_ucStraightYawCaptured = 0U;
    s_fLastValidCurveSpeed = 0.0f;
    vChassisStop();
}

static void vLineTrackUpdateInternal(float fBaseSpeed,
    uint8_t ucKeepSpeedWhenLost)
{
    stLineTrackStaticParamTdf *pstStatic =
        &s_stLineTrackDeviceParam.stStaticParam;
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;

    if (pstRunning->ucEnable == 0U)
    {
        return;
    }

    vLineTrackEnterMode(emLineTrackControlGrayOnly);
    vLineTrackUpdateSensorState();

    if (pstRunning->emState == emLineTrackStateLost)
    {
        pstRunning->fLineError = 0.0f;
        if (ucKeepSpeedWhenLost != 0U)
        {
            pstRunning->fCommandSpeed = fBaseSpeed;
            pstRunning->fCorrectionOmega = 0.0f;
        }
        else if (pstRunning->cLastDirection == 0)
        {
            pstRunning->fCommandSpeed = 0.0f;
            pstRunning->fCorrectionOmega = 0.0f;
        }
        else
        {
            pstRunning->fCommandSpeed = pstStatic->fBaseSpeed *
                pstStatic->fLostSpeedScale;
            pstRunning->fLineError = (float)pstRunning->cLastDirection;
            pstRunning->fCorrectionOmega = fLineTrackLimit(
                LINE_TRACK_SENSOR_TURN_SIGN * pstStatic->fCorrectionKp *
                    pstRunning->fLineError,
                pstStatic->fMaxCorrectionOmega);
        }
    }
    else if (pstRunning->emState == emLineTrackStateIntersection)
    {
        pstRunning->fLineError = 0.0f;
        pstRunning->fCommandSpeed = 0.0f;
        pstRunning->fCorrectionOmega = 0.0f;
    }
    else
    {
        pstRunning->fCommandSpeed = fBaseSpeed;
        pstRunning->fCorrectionOmega = fLineTrackLimit(
            LINE_TRACK_SENSOR_TURN_SIGN * pstStatic->fCorrectionKp *
                pstRunning->fLineError,
            pstStatic->fMaxCorrectionOmega);
        if (pstRunning->fLineError < 0.0f)
        {
            pstRunning->cLastDirection = -1;
        }
        else if (pstRunning->fLineError > 0.0f)
        {
            pstRunning->cLastDirection = 1;
        }
    }

    pstRunning->fTargetYawRate = 0.0f;
    pstRunning->fActualYawRate = fImuGetGyroZ(IMU_1);
    pstRunning->fOuterControlOmega = pstRunning->fCorrectionOmega;
    pstRunning->fImuControlOmega = 0.0f;
    vChassisSetSpeed(pstRunning->fCommandSpeed,
        pstRunning->fCorrectionOmega);
}

void vLineTrackUpdate(void)
{
    vLineTrackUpdateInternal(s_stLineTrackDeviceParam.stStaticParam.fBaseSpeed,
        0U);
}

void vLineTrackUpdateByTargetRpm(float fTargetRpm)
{
    vLineTrackUpdateInternal(fLineTrackRpmToLinearSpeed(fTargetRpm), 1U);
}

void vLineTrackImuUpdateByTargetRpm(float fTargetRpm)
{
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    float fTargetYawRate = 0.0f;
    float fCommandSpeed;

    if (pstRunning->ucEnable == 0U)
    {
        return;
    }

    vLineTrackEnterMode(emLineTrackControlGrayImu);
    vLineTrackUpdateSensorState();
    fCommandSpeed = fLineTrackRpmToLinearSpeed(fTargetRpm);
    pstRunning->fTargetYaw = 0.0f;
    pstRunning->fYawError = 0.0f;

    if ((pstRunning->emState == emLineTrackStateIntersection) ||
        ((fTargetRpm > -0.001f) && (fTargetRpm < 0.001f)))
    {
        PID_Reset(&s_stStraightGrayPid);
        PID_Reset(&s_stStraightYawRatePid);
        pstRunning->fTargetYawRate = 0.0f;
        pstRunning->fActualYawRate = fImuGetGyroZ(IMU_1);
        pstRunning->fOuterControlOmega = 0.0f;
        pstRunning->fImuControlOmega = 0.0f;
        pstRunning->fCorrectionOmega = 0.0f;
        pstRunning->fCommandSpeed = 0.0f;
        vChassisStop();
        return;
    }

    if (pstRunning->emState == emLineTrackStateLost)
    {
        PID_Reset(&s_stStraightGrayPid);
    }
    else
    {
        fTargetYawRate = PID_Calculate(&s_stStraightGrayPid,
            0.0f, pstRunning->fLineError);
    }
    vLineTrackApplyYawRateControl(fCommandSpeed, fTargetYawRate,
        &s_stStraightYawRatePid,
        s_stLineTrackDeviceParam.stStaticParam.fStraightMaxTargetYawRate);
}

/**
  * @brief      灰度+IMU弯道循迹
  * @note       灰度正常时更新最近一次有效速度；丢线时不重新使用固定
  *             基础速度，而是保持丢线前的有效速度直行，并保留IMU抑制。
  */
void vLineTrackCurveImuUpdateByTargetRpm(float fTargetRpm)
{
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    float fTargetYawRate = 0.0f;
    float fBaseSpeed;
    float fCommandSpeed;

    if (pstRunning->ucEnable == 0U)
    {
        return;
    }

    vLineTrackEnterMode(emLineTrackControlCurveImu);
    vLineTrackUpdateSensorState();
    fBaseSpeed = fLineTrackRpmToLinearSpeed(fTargetRpm);
    if (s_fLastValidCurveSpeed <= 0.0f)
    {
        s_fLastValidCurveSpeed = fBaseSpeed;
    }

    pstRunning->fTargetYaw = 0.0f;
    pstRunning->fYawError = 0.0f;

    if ((pstRunning->emState == emLineTrackStateIntersection) ||
        ((fTargetRpm > -0.001f) && (fTargetRpm < 0.001f)))
    {
        PID_Reset(&s_stCurveGrayPid);
        PID_Reset(&s_stCurveYawRatePid);
        pstRunning->fTargetYawRate = 0.0f;
        pstRunning->fActualYawRate = fImuGetGyroZ(IMU_1);
        pstRunning->fOuterControlOmega = 0.0f;
        pstRunning->fImuControlOmega = 0.0f;
        pstRunning->fCorrectionOmega = 0.0f;
        pstRunning->fCommandSpeed = 0.0f;
        vChassisStop();
        return;
    }

    if (pstRunning->emState == emLineTrackStateLost)
    {
        /* 丢线后沿用最近一次有效速度，不因丢线重新跳到基础速度。 */
        PID_Reset(&s_stCurveGrayPid);
        fCommandSpeed = s_fLastValidCurveSpeed;
    }
    else
    {
        fCommandSpeed = fBaseSpeed;
        s_fLastValidCurveSpeed = fCommandSpeed;
        fTargetYawRate = PID_Calculate(&s_stCurveGrayPid,
            0.0f, pstRunning->fLineError);
    }

    vLineTrackApplyYawRateControl(fCommandSpeed, fTargetYawRate,
        &s_stCurveYawRatePid,
        s_stLineTrackDeviceParam.stStaticParam.fCurveMaxTargetYawRate);
}

void vImuStraightUpdateByTargetRpm(float fTargetRpm)
{
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    float fCurrentYaw;
    float fTargetYawRate;
    float fCommandSpeed;

    if (pstRunning->ucEnable == 0U)
    {
        return;
    }

    vLineTrackEnterMode(emLineTrackControlImuStraight);
    fCommandSpeed = fLineTrackRpmToLinearSpeed(fTargetRpm);
    pstRunning->emState = emLineTrackStateImuStraight;
    pstRunning->ucBlackMask = 0U;
    pstRunning->ucActiveCount = 0U;
    pstRunning->fLinePosition = 0.0f;
    pstRunning->fLineError = 0.0f;

    if ((fTargetRpm > -0.001f) && (fTargetRpm < 0.001f))
    {
        PID_Reset(&s_stYawAnglePid);
        PID_Reset(&s_stStraightYawRatePid);
        vChassisStop();
        return;
    }

    if (ulImuGetGyroSampleCount(IMU_1) == 0U)
    {
        vChassisSetSpeed(fCommandSpeed, 0.0f);
        return;
    }

    fCurrentYaw = fImuGetYaw(IMU_1);
    if (s_ucStraightYawCaptured == 0U)
    {
        pstRunning->fTargetYaw = fCurrentYaw;
        s_ucStraightYawCaptured = 1U;
        PID_Reset(&s_stYawAnglePid);
        PID_Reset(&s_stStraightYawRatePid);
    }
    pstRunning->fYawError = fLineTrackWrapYawError(
        pstRunning->fTargetYaw - fCurrentYaw);
    fTargetYawRate = PID_Calculate(&s_stYawAnglePid,
        0.0f, pstRunning->fYawError);
    vLineTrackApplyYawRateControl(fCommandSpeed, fTargetYawRate,
        &s_stStraightYawRatePid,
        s_stLineTrackDeviceParam.stStaticParam.fStraightMaxTargetYawRate);
}
