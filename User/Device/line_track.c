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

/* D1最靠近左电机，D8最靠近右电机，D4/D5位于中间。 */
/* 第2问直线权重。 */
static const int8_t s_acQ2StraightLinePosition[8] =
    {100, -80, -50, -10, 10, 50, 80, 100};
/* 第2问进入弯道前忽略左侧D1-D4。 */
static const int8_t s_acQ2CurveEntryLinePosition[8] =
    {0, 0, 0, 0, 8, 20, 50, 50};
/* 第2问弯道权重。 */
static const int8_t s_acQ2CurveLinePosition[8] =
    {0, 0, 0, 15, 27, 35,39, 43};
/* 第5/6问权重，初值复制第2问，可独立调节。 */
static const int8_t s_acQ56StraightLinePosition[8] =
    {100, -80, -50, -7, 7, 50, 80, 100};
static const int8_t s_acQ56CurveEntryLinePosition[8] =
    {0, 0, 0, 0, 8, 20, 50, 50};
static const int8_t s_acQ56CurveLinePosition[8] =
    {0, -5, -3, 0, 21, 25, 39, 42};

static stLineTrackDeviceParamTdf s_stLineTrackDeviceParam;
static PID_t s_stStraightGrayPid;
static PID_t s_stYawAnglePid;
static PID_t s_stStraightYawRatePid;
static PID_t s_stCurveGrayPid;
static PID_t s_stCurveYawRatePid;
static uint8_t s_ucStraightYawCaptured;
static uint8_t s_ucCurveEntryConstraint;
static float s_fLastValidCurveSpeed;
static float s_fCurveLineError;
static stLineTrackStaticParamTdf s_stQ2StaticParam;
static const int8_t *s_pcStraightLinePosition =
    s_acQ2StraightLinePosition;
static const int8_t *s_pcCurveEntryLinePosition =
    s_acQ2CurveEntryLinePosition;
static const int8_t *s_pcCurveLinePosition = s_acQ2CurveLinePosition;
static float s_fCurveErrorFilterAlpha = LINE_TRACK_CURVE_ERROR_FILTER_ALPHA;
static float s_fCurveFeedforwardOmega =
    LINE_Q2_TRACK_CURVE_FEEDFORWARD_OMEGA_RAD_S;

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

static uint8_t ucLineTrackCurveSampleIsValid(uint8_t ucBlackMask)
{
    uint8_t ucShiftedMask = ucBlackMask;

    /* Zero or one black channel cannot contain a gap. */
    if ((ucBlackMask == 0U) ||
        ((ucBlackMask & (uint8_t)(ucBlackMask - 1U)) == 0U))
    {
        return 1U;
    }

    while ((ucShiftedMask & 0x01U) == 0U)
    {
        ucShiftedMask >>= 1U;
    }
    while ((ucShiftedMask & 0x01U) != 0U)
    {
        ucShiftedMask >>= 1U;
    }
    return (ucShiftedMask == 0U) ? 1U : 0U;
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

static void vLineTrackInitAllPids(void)
{
    const stLineTrackStaticParamTdf *pstStatic =
        &s_stLineTrackDeviceParam.stStaticParam;

    vLineTrackPidInit(&s_stStraightGrayPid,
        pstStatic->fStraightGrayPidKp, pstStatic->fStraightGrayPidKi,
        pstStatic->fStraightGrayPidKd,
        pstStatic->fStraightMaxTargetYawRate);
    vLineTrackPidInit(&s_stYawAnglePid,
        pstStatic->fYawAnglePidKp, pstStatic->fYawAnglePidKi,
        pstStatic->fYawAnglePidKd,
        pstStatic->fStraightMaxTargetYawRate);
    vLineTrackPidInit(&s_stStraightYawRatePid,
        pstStatic->fStraightYawRatePidKp,
        pstStatic->fStraightYawRatePidKi,
        pstStatic->fStraightYawRatePidKd,
        pstStatic->fStraightMaxTargetYawRate);
    vLineTrackPidInit(&s_stCurveGrayPid,
        pstStatic->fCurveGrayPidKp, pstStatic->fCurveGrayPidKi,
        pstStatic->fCurveGrayPidKd, pstStatic->fCurveMaxTargetYawRate);
    vLineTrackPidInit(&s_stCurveYawRatePid,
        pstStatic->fCurveYawRatePidKp,
        pstStatic->fCurveYawRatePidKi,
        pstStatic->fCurveYawRatePidKd,
        pstStatic->fCurveMaxTargetYawRate);
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
    s_fCurveLineError = 0.0f;
    s_emControlMode = emMode;
}

static void vLineTrackUpdateSensorState(void)
{
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    const int8_t *pcLinePosition;
    uint8_t ucDigital = ucGrayscaleSensorGetDigital(GRAYSCALE1);
    int16_t sPositionSum = 0;
    uint8_t ucChannel;

    if ((s_emControlMode == emLineTrackControlGrayImu) &&
        (s_ucCurveEntryConstraint != 0U))
    {
        pcLinePosition = s_pcCurveEntryLinePosition;
    }
    else if ((s_emControlMode == emLineTrackControlGrayOnly) ||
        (s_emControlMode == emLineTrackControlCurveImu))
    {
        pcLinePosition = s_pcCurveLinePosition;
    }
    else
    {
        pcLinePosition = s_pcStraightLinePosition;
    }

    /* 当前灰度数字量为1表示白色，取反后得到黑线位图。 */
    pstRunning->ucBlackMask = (uint8_t)(~ucDigital);
    pstRunning->ucActiveCount = 0U;
    for (ucChannel = 0U; ucChannel < 8U; ucChannel++)
    {
        if ((pstRunning->ucBlackMask & (uint8_t)(1U << ucChannel)) != 0U)
        {
            sPositionSum += pcLinePosition[ucChannel];
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
    if (s_emControlMode == emLineTrackControlCurveImu)
    {
        pstRunning->fCorrectionOmega = fLineTrackLimit(
            pstRunning->fCorrectionOmega + s_fCurveFeedforwardOmega,
            pstStatic->fMaxCorrectionOmega);
    }
    /* 底盘约定omega为正表示左转；直线末段只允许直行或右转。 */
    if ((s_ucCurveEntryConstraint != 0U) &&
        (pstRunning->fCorrectionOmega > 0.0f))
    {
        pstRunning->fCorrectionOmega = 0.0f;
    }
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
    s_ucCurveEntryConstraint = 0U;
    s_fLastValidCurveSpeed = 0.0f;
    if (pstInit == NULL)
    {
        return;
    }

    memcpy(&s_stLineTrackDeviceParam.stStaticParam, pstInit,
        sizeof(*pstInit));
    s_stLineTrackDeviceParam.stRunningParam.emState =
        emLineTrackStateDisabled;
    memcpy(&s_stQ2StaticParam, pstInit, sizeof(s_stQ2StaticParam));
    vLineTrackInitAllPids();
}

void vLineTrackSelectRouteProfile(emLineTrackRouteProfileTdf emProfile)
{
    s_stLineTrackDeviceParam.stStaticParam = s_stQ2StaticParam;
    if (emProfile == emLineTrackRouteProfileQ56)
    {
        s_stLineTrackDeviceParam.stStaticParam.fCorrectionKp =
            LINE_Q56_TRACK_CORRECTION_KP;
        s_stLineTrackDeviceParam.stStaticParam.fMaxCorrectionOmega =
            LINE_Q56_TRACK_MAX_CORRECTION_OMEGA;
        s_stLineTrackDeviceParam.stStaticParam.fStraightGrayPidKp =
            LINE_Q56_TRACK_STRAIGHT_GRAY_PID_KP;
        s_stLineTrackDeviceParam.stStaticParam.fStraightGrayPidKi =
            LINE_Q56_TRACK_STRAIGHT_GRAY_PID_KI;
        s_stLineTrackDeviceParam.stStaticParam.fStraightGrayPidKd =
            LINE_Q56_TRACK_STRAIGHT_GRAY_PID_KD;
        s_stLineTrackDeviceParam.stStaticParam.fCurveGrayPidKp =
            LINE_Q56_TRACK_CURVE_GRAY_PID_KP;
        s_stLineTrackDeviceParam.stStaticParam.fCurveGrayPidKi =
            LINE_Q56_TRACK_CURVE_GRAY_PID_KI;
        s_stLineTrackDeviceParam.stStaticParam.fCurveGrayPidKd =
            LINE_Q56_TRACK_CURVE_GRAY_PID_KD;
        s_stLineTrackDeviceParam.stStaticParam.fStraightYawRatePidKp =
            LINE_Q56_TRACK_STRAIGHT_YAW_RATE_PID_KP;
        s_stLineTrackDeviceParam.stStaticParam.fStraightYawRatePidKi =
            LINE_Q56_TRACK_STRAIGHT_YAW_RATE_PID_KI;
        s_stLineTrackDeviceParam.stStaticParam.fStraightYawRatePidKd =
            LINE_Q56_TRACK_STRAIGHT_YAW_RATE_PID_KD;
        s_stLineTrackDeviceParam.stStaticParam.fCurveYawRatePidKp =
            LINE_Q56_TRACK_CURVE_YAW_RATE_PID_KP;
        s_stLineTrackDeviceParam.stStaticParam.fCurveYawRatePidKi =
            LINE_Q56_TRACK_CURVE_YAW_RATE_PID_KI;
        s_stLineTrackDeviceParam.stStaticParam.fCurveYawRatePidKd =
            LINE_Q56_TRACK_CURVE_YAW_RATE_PID_KD;
        s_stLineTrackDeviceParam.stStaticParam.fStraightMaxTargetYawRate =
            LINE_Q56_TRACK_STRAIGHT_MAX_TARGET_YAW_RATE;
        s_stLineTrackDeviceParam.stStaticParam.fCurveMaxTargetYawRate =
            LINE_Q56_TRACK_CURVE_MAX_TARGET_YAW_RATE;
        s_stLineTrackDeviceParam.stStaticParam.fOuterControlWeight =
            LINE_Q56_TRACK_OUTER_CONTROL_WEIGHT;
        s_stLineTrackDeviceParam.stStaticParam.fImuFeedbackWeight =
            LINE_Q56_TRACK_IMU_FEEDBACK_WEIGHT;
        s_pcStraightLinePosition = s_acQ56StraightLinePosition;
        s_pcCurveEntryLinePosition = s_acQ56CurveEntryLinePosition;
        s_pcCurveLinePosition = s_acQ56CurveLinePosition;
        s_fCurveErrorFilterAlpha = LINE_Q56_TRACK_CURVE_ERROR_FILTER_ALPHA;
        s_fCurveFeedforwardOmega =
            LINE_Q56_TRACK_CURVE_FEEDFORWARD_OMEGA_RAD_S;
    }
    else
    {
        s_pcStraightLinePosition = s_acQ2StraightLinePosition;
        s_pcCurveEntryLinePosition = s_acQ2CurveEntryLinePosition;
        s_pcCurveLinePosition = s_acQ2CurveLinePosition;
        s_fCurveErrorFilterAlpha = LINE_TRACK_CURVE_ERROR_FILTER_ALPHA;
        s_fCurveFeedforwardOmega =
            LINE_Q2_TRACK_CURVE_FEEDFORWARD_OMEGA_RAD_S;
    }
    vLineTrackInitAllPids();
    s_emControlMode = emLineTrackControlDisabled;
    s_fCurveLineError = 0.0f;
}

void vLineTrackStart(void)
{
    s_ucCurveEntryConstraint = 0U;
    s_stLineTrackDeviceParam.stRunningParam.ucEnable = 1U;
}

void vLineTrackSetCurveEntryConstraint(uint8_t ucEnable)
{
    s_ucCurveEntryConstraint = (ucEnable != 0U) ? 1U : 0U;
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
    s_ucCurveEntryConstraint = 0U;
    s_fLastValidCurveSpeed = 0.0f;
    s_fCurveLineError = 0.0f;
    vLineTrackSelectRouteProfile(emLineTrackRouteProfileQ2);
    vChassisStop();
}

static void vLineTrackUpdateInternal(float fBaseSpeed,
    uint8_t ucKeepSpeedWhenLost, uint8_t ucCurveSearchWhenLost)
{
    stLineTrackStaticParamTdf *pstStatic =
        &s_stLineTrackDeviceParam.stStaticParam;
    stLineTrackRunningParamTdf *pstRunning =
        &s_stLineTrackDeviceParam.stRunningParam;
    float fCorrectionKp = pstStatic->fCorrectionKp;
    float fMaxCorrectionOmega = pstStatic->fMaxCorrectionOmega;
    float fLineError;

    if (pstRunning->ucEnable == 0U)
    {
        return;
    }
    vLineTrackEnterMode(emLineTrackControlGrayOnly);
    vLineTrackUpdateSensorState();
    if (ucLineTrackCurveSampleIsValid(pstRunning->ucBlackMask) == 0U)
    {
        /* 直线或弯道中黑色通道断开，统一视为丢线。 */
        pstRunning->emState = emLineTrackStateLost;
        pstRunning->fLinePosition = 0.0f;
        pstRunning->fLineError = 0.0f;
    }

    if (pstRunning->emState == emLineTrackStateLost)
    {
        pstRunning->fLineError = 0.0f;
        if (ucKeepSpeedWhenLost != 0U)
        {
            pstRunning->fCommandSpeed = fBaseSpeed;
            pstRunning->fCorrectionOmega = 0.0f;
        }
        else if ((ucCurveSearchWhenLost == 0U) &&
                 (pstRunning->cLastDirection == 0))
        {
            pstRunning->fCommandSpeed = 0.0f;
            pstRunning->fCorrectionOmega = 0.0f;
        }
        else
        {
            pstRunning->fCommandSpeed = (ucCurveSearchWhenLost != 0U) ?
                fBaseSpeed : fBaseSpeed * pstStatic->fLostSpeedScale;
            pstRunning->fLineError = (float)pstRunning->cLastDirection;
            pstRunning->fCorrectionOmega = fLineTrackLimit(
                s_fCurveFeedforwardOmega +
                LINE_TRACK_SENSOR_TURN_SIGN * fCorrectionKp *
                    pstRunning->fLineError,
                fMaxCorrectionOmega);
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
        fLineError = pstRunning->fLineError;
        if (ucCurveSearchWhenLost != 0U)
        {
            s_fCurveLineError += s_fCurveErrorFilterAlpha *
                (fLineError - s_fCurveLineError);
            fLineError = s_fCurveLineError;
            pstRunning->fLineError = fLineError;
        }
        pstRunning->fCorrectionOmega = fLineTrackLimit(
            s_fCurveFeedforwardOmega +
            LINE_TRACK_SENSOR_TURN_SIGN * fCorrectionKp * fLineError,
            fMaxCorrectionOmega);
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
        0U, 0U);
}

void vLineTrackUpdateByTargetRpm(float fTargetRpm)
{
    vLineTrackUpdateInternal(fLineTrackRpmToLinearSpeed(fTargetRpm), 1U, 0U);
}

void vLineTrackCurveUpdateByTargetRpm(float fTargetRpm)
{
    vLineTrackUpdateInternal(fLineTrackRpmToLinearSpeed(fTargetRpm),
        0U, 1U);
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
    if (ucLineTrackCurveSampleIsValid(pstRunning->ucBlackMask) == 0U)
    {
        /* 直线中非连续黑色通道视为传感器跳变，并按丢线处理。 */
        pstRunning->emState = emLineTrackStateLost;
        pstRunning->fLinePosition = 0.0f;
        pstRunning->fLineError = 0.0f;
    }
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
    if (ucLineTrackCurveSampleIsValid(pstRunning->ucBlackMask) == 0U)
    {
        /* 弯道中非连续黑色通道视为传感器跳变，并按丢线处理。 */
        pstRunning->emState = emLineTrackStateLost;
        pstRunning->fLinePosition = 0.0f;
        pstRunning->fLineError = 0.0f;
    }
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
