/**
  * @file       line_route.c
  * @version    V1.1.0
  * @date       20260731
  * @brief      第2、5、6问共用的ABCD环形路线循迹状态机
  */

#include "line_route.h"
#include "GrayscaleSensor.h"
#include "chassis.h"
#include "encoder.h"
#include "imu.h"
#include "line_track.h"
#include "project_config.h"

#define LINE_ROUTE_TWO_PI                6.28318530717958647692f

typedef struct
{
    float fStraightTargetRpm;
    float fCurveTargetRpm;
    float fStartAccelRpmPerS;
    float fAccelRpmPerS;
    float fDecelRpmPerS;
    float fStraightDistanceMm;
    float fCurveEntryDistanceMm;
    float fCurveAngleDeg;
    uint32_t ulStartBlindTimeMs;
    uint8_t ucInterferenceBlackCount;
} stLineRouteProfileTdf;

static const stLineRouteProfileTdf s_stLineRouteQ2Profile =
{
    .fStraightTargetRpm = LINE_Q2_STRAIGHT_TARGET_RPM,
    .fCurveTargetRpm = LINE_Q2_CURVE_TARGET_RPM,
    .fStartAccelRpmPerS = LINE_Q2_START_ACCEL_RPM_PER_S,
    .fAccelRpmPerS = LINE_Q2_ACCEL_RPM_PER_S,
    .fDecelRpmPerS = LINE_Q2_DECEL_RPM_PER_S,
    .fStraightDistanceMm = LINE_Q2_STRAIGHT_DISTANCE_MM,
    .fCurveEntryDistanceMm = LINE_Q2_CURVE_ENTRY_DISTANCE_MM,
    .fCurveAngleDeg = LINE_Q2_CURVE_ANGLE_DEG,
    .ulStartBlindTimeMs = LINE_Q2_START_BLIND_TIME_MS,
    .ucInterferenceBlackCount = LINE_Q2_INTERFERENCE_BLACK_COUNT,
};

static const stLineRouteProfileTdf s_stLineRouteQ56Profile =
{
    .fStraightTargetRpm = LINE_Q56_STRAIGHT_TARGET_RPM,
    .fCurveTargetRpm = LINE_Q56_CURVE_TARGET_RPM,
    .fStartAccelRpmPerS = LINE_Q56_START_ACCEL_RPM_PER_S,
    .fAccelRpmPerS = LINE_Q56_ACCEL_RPM_PER_S,
    .fDecelRpmPerS = LINE_Q56_DECEL_RPM_PER_S,
    .fStraightDistanceMm = LINE_Q56_STRAIGHT_DISTANCE_MM,
    .fCurveEntryDistanceMm = LINE_Q56_CURVE_ENTRY_DISTANCE_MM,
    .fCurveAngleDeg = LINE_Q56_CURVE_ANGLE_DEG,
    .ulStartBlindTimeMs = LINE_Q56_START_BLIND_TIME_MS,
    .ucInterferenceBlackCount = LINE_Q56_INTERFERENCE_BLACK_COUNT,
};

volatile emLineRouteStateTdf g_emLineRouteState = emLineRouteIdle;
volatile uint8_t g_ucLineRouteBlackCount;
volatile uint8_t g_ucLineRouteMarkerArmed;
volatile uint8_t g_ucLineRouteMarkerActive;
volatile uint8_t g_ucLineRouteMarkerCount;
volatile float g_fLineRouteSegmentDistanceMm;
volatile float g_fLineRouteCurveAngleDeg;
volatile float g_fLineRouteCommandRpm;
volatile uint32_t g_ulLineRouteFinalStraightElapsedMs;
volatile emLineRouteFinishModeTdf g_emLineRouteFinishMode =
    emLineRouteFinishByStopLine;

static uint8_t s_ucFinishConfirmCount;
static uint8_t s_ucStartAccelerationActive;
static uint32_t s_ulStartBlindElapsedMs;
static int32_t s_lMotor1SegmentStartCount;
static int32_t s_lMotor2SegmentStartCount;
static uint32_t s_ulLastImuSampleCount;
static float s_fLastYawDeg;
static float s_fCurveSignedAngleDeg;
static const stLineRouteProfileTdf *s_pstLineRouteProfile =
    &s_stLineRouteQ2Profile;

static float fLineRouteAbs(float fValue)
{
    return (fValue < 0.0f) ? -fValue : fValue;
}

static float fLineRouteUpdateCommandRpm(float fTargetRpm)
{
    float fRateRpmPerS;
    float fMaxStepRpm;
    float fErrorRpm = fTargetRpm - g_fLineRouteCommandRpm;

    if ((fErrorRpm >= 0.0f) && (s_ucStartAccelerationActive != 0U))
    {
        fRateRpmPerS = s_pstLineRouteProfile->fStartAccelRpmPerS;
    }
    else
    {
        fRateRpmPerS = (fErrorRpm >= 0.0f) ?
            s_pstLineRouteProfile->fAccelRpmPerS :
            s_pstLineRouteProfile->fDecelRpmPerS;
    }
    if (fRateRpmPerS <= 0.0f)
    {
        g_fLineRouteCommandRpm = fTargetRpm;
        s_ucStartAccelerationActive = 0U;
        return g_fLineRouteCommandRpm;
    }

    fMaxStepRpm = fRateRpmPerS *
        (float)LINE_TRACK_TASK_PERIOD_MS / 1000.0f;
    if (fErrorRpm > fMaxStepRpm)
    {
        g_fLineRouteCommandRpm += fMaxStepRpm;
    }
    else if (fErrorRpm < -fMaxStepRpm)
    {
        g_fLineRouteCommandRpm -= fMaxStepRpm;
    }
    else
    {
        g_fLineRouteCommandRpm = fTargetRpm;
        s_ucStartAccelerationActive = 0U;
    }
    return g_fLineRouteCommandRpm;
}

static float fLineRouteWrapYawDelta(float fDeltaDeg)
{
    while (fDeltaDeg > 180.0f)
    {
        fDeltaDeg -= 360.0f;
    }
    while (fDeltaDeg < -180.0f)
    {
        fDeltaDeg += 360.0f;
    }
    return fDeltaDeg;
}

static uint8_t ucLineRouteCountBits(uint8_t ucValue)
{
    uint8_t ucCount = 0U;

    while (ucValue != 0U)
    {
        ucCount += (uint8_t)(ucValue & 0x01U);
        ucValue >>= 1U;
    }
    return ucCount;
}

/**
  * @brief  判断8路灰度中是否存在两个相邻通道同时为黑
  * @param  ucBlackMask D1-D8对应bit0-bit7，1表示黑色
  * @return 1表示存在相邻黑色通道，否则返回0
  */
static uint8_t ucLineRouteHasAdjacentBlack(uint8_t ucBlackMask)
{
    return ((ucBlackMask & (uint8_t)(ucBlackMask >> 1U)) != 0U) ?
        1U : 0U;
}

static void vLineRouteCaptureStraightStart(void)
{
    const stDcMotorEncoderDeviceParamTdf *pstEncoder1 =
        c_pstGetEncoderDeviceParam(DC_MOTOR1);
    const stDcMotorEncoderDeviceParamTdf *pstEncoder2 =
        c_pstGetEncoderDeviceParam(DC_MOTOR2);

    s_lMotor1SegmentStartCount = (pstEncoder1 != NULL) ?
        pstEncoder1->stRunningParam.lCount : 0;
    s_lMotor2SegmentStartCount = (pstEncoder2 != NULL) ?
        pstEncoder2->stRunningParam.lCount : 0;
    g_fLineRouteSegmentDistanceMm = 0.0f;
}

static float fLineRouteWheelDistanceMm(
    const stDcMotorEncoderDeviceParamTdf *pstEncoder,
    int32_t lStartCount, float fWheelRadiusMm)
{
    float fCountsPerWheelRevolution;
    float fDeltaCount;

    if ((pstEncoder == NULL) || (fWheelRadiusMm <= 0.0f))
    {
        return 0.0f;
    }
    fCountsPerWheelRevolution =
        (float)pstEncoder->stStaticParam.usLines *
        (float)pstEncoder->stStaticParam.usReductionRatio *
        (float)pstEncoder->stStaticParam.ucMode;
    if (fCountsPerWheelRevolution <= 0.0f)
    {
        return 0.0f;
    }

    fDeltaCount = (float)(pstEncoder->stRunningParam.lCount - lStartCount);
    return fLineRouteAbs(fDeltaCount) * LINE_ROUTE_TWO_PI *
        fWheelRadiusMm / fCountsPerWheelRevolution;
}

static void vLineRouteUpdateStraightDistance(void)
{
    const stDcMotorEncoderDeviceParamTdf *pstEncoder1 =
        c_pstGetEncoderDeviceParam(DC_MOTOR1);
    const stDcMotorEncoderDeviceParamTdf *pstEncoder2 =
        c_pstGetEncoderDeviceParam(DC_MOTOR2);
    const stChassisDeviceParamTdf *pstChassis = c_pstGetChassisDeviceParam();
    float fMotor1DistanceMm;
    float fMotor2DistanceMm;

    if (pstChassis == NULL)
    {
        g_fLineRouteSegmentDistanceMm = 0.0f;
        return;
    }
    fMotor1DistanceMm = fLineRouteWheelDistanceMm(pstEncoder1,
        s_lMotor1SegmentStartCount, pstChassis->stStaticParam.fWheelRadius);
    fMotor2DistanceMm = fLineRouteWheelDistanceMm(pstEncoder2,
        s_lMotor2SegmentStartCount, pstChassis->stStaticParam.fWheelRadius);
    g_fLineRouteSegmentDistanceMm =
        (fMotor1DistanceMm + fMotor2DistanceMm) * 0.5f;
}

static void vLineRouteResetCurveAngle(void)
{
    s_ulLastImuSampleCount = ulImuGetGyroSampleCount(IMU_1);
    s_fLastYawDeg = fImuGetYaw(IMU_1);
    s_fCurveSignedAngleDeg = 0.0f;
    g_fLineRouteCurveAngleDeg = 0.0f;
}

static void vLineRouteUpdateCurveAngle(void)
{
    uint32_t ulSampleCount = ulImuGetGyroSampleCount(IMU_1);
    float fCurrentYawDeg;
    float fYawDeltaDeg;

    if ((ulSampleCount == 0U) || (ulSampleCount == s_ulLastImuSampleCount))
    {
        return;
    }
    fCurrentYawDeg = fImuGetYaw(IMU_1);
    fYawDeltaDeg = fLineRouteWrapYawDelta(fCurrentYawDeg - s_fLastYawDeg);
    s_fLastYawDeg = fCurrentYawDeg;
    s_ulLastImuSampleCount = ulSampleCount;

    /* 带符号累计后再取绝对值，左右弯均可用，反向摆动也不会虚增角度。 */
    s_fCurveSignedAngleDeg += fYawDeltaDeg;
    g_fLineRouteCurveAngleDeg = fLineRouteAbs(s_fCurveSignedAngleDeg);
}

static void vLineRouteAdvanceState(void)
{
    switch (g_emLineRouteState)
    {
        case emLineRouteStraightAB:
            s_ucStartAccelerationActive = 0U;
            g_emLineRouteState = emLineRouteCurveBC;
            vLineRouteResetCurveAngle();
            break;
        case emLineRouteCurveBC:
            g_emLineRouteState = emLineRouteStraightCD;
            vLineRouteCaptureStraightStart();
            break;
        case emLineRouteStraightCD:
            g_emLineRouteState = emLineRouteCurveDA;
            vLineRouteResetCurveAngle();
            break;
        case emLineRouteCurveDA:
            if (g_emLineRouteFinishMode ==
                emLineRouteFinishByTimedStraight)
            {
                g_emLineRouteState = emLineRouteFinalStraight;
                g_ulLineRouteFinalStraightElapsedMs = 0U;
                vLineRouteCaptureStraightStart();
            }
            else
            {
                g_emLineRouteState = emLineRouteFinishSearch;
                g_ucLineRouteMarkerArmed = 1U;
            }
            break;
        case emLineRouteFinishSearch:
        case emLineRouteFinalStraight:
            g_emLineRouteState = emLineRouteStopped;
            g_ucLineRouteMarkerArmed = 0U;
            g_ucLineRouteMarkerActive = 0U;
            g_fLineRouteCommandRpm = 0.0f;
            vLineTrackStop();
            vChassisStop();
            break;
        case emLineRouteIdle:
        case emLineRouteStopped:
        default:
            return;
    }

    /* 状态切换时先解除直线末段约束，下一直线段再按里程重新开启。 */
    vLineTrackSetCurveEntryConstraint(0U);
    g_ucLineRouteMarkerCount++;
    s_ucFinishConfirmCount = 0U;
}

static void vLineRouteStartInternal(emLineRouteFinishModeTdf emFinishMode)
{
    s_pstLineRouteProfile = (emFinishMode ==
        emLineRouteFinishByTimedStraight) ?
        &s_stLineRouteQ56Profile : &s_stLineRouteQ2Profile;
    vLineTrackSelectRouteProfile((emFinishMode ==
        emLineRouteFinishByTimedStraight) ?
        emLineTrackRouteProfileQ56 : emLineTrackRouteProfileQ2);
    g_emLineRouteState = emLineRouteStraightAB;
    g_emLineRouteFinishMode = emFinishMode;
    g_ucLineRouteBlackCount = 0U;
    g_ucLineRouteMarkerArmed = 0U;
    g_ucLineRouteMarkerActive = 0U;
    g_ucLineRouteMarkerCount = 0U;
    g_fLineRouteCurveAngleDeg = 0.0f;
    g_fLineRouteCommandRpm = 0.0f;
    g_ulLineRouteFinalStraightElapsedMs = 0U;
    s_ucFinishConfirmCount = 0U;
    s_ucStartAccelerationActive = 1U;
    s_ulStartBlindElapsedMs = 0U;
    vLineTrackSetCurveEntryConstraint(0U);
    vLineRouteCaptureStraightStart();
    vLineTrackStart();
}

void vLineRouteStart(void)
{
    vLineRouteStartInternal(emLineRouteFinishByStopLine);
}

void vLineRouteExtendedStart(void)
{
    vLineRouteStartInternal(emLineRouteFinishByTimedStraight);
}

void vLineRouteStop(void)
{
    g_emLineRouteState = emLineRouteIdle;
    g_ucLineRouteBlackCount = 0U;
    g_ucLineRouteMarkerArmed = 0U;
    g_ucLineRouteMarkerActive = 0U;
    g_ucLineRouteMarkerCount = 0U;
    g_fLineRouteSegmentDistanceMm = 0.0f;
    g_fLineRouteCurveAngleDeg = 0.0f;
    g_fLineRouteCommandRpm = 0.0f;
    g_ulLineRouteFinalStraightElapsedMs = 0U;
    g_emLineRouteFinishMode = emLineRouteFinishByStopLine;
    s_pstLineRouteProfile = &s_stLineRouteQ2Profile;
    s_ucFinishConfirmCount = 0U;
    s_ucStartAccelerationActive = 0U;
    s_ulStartBlindElapsedMs = 0U;
    vLineTrackSetCurveEntryConstraint(0U);
    vLineTrackStop();
    vChassisStop();
}

void vLineRouteUpdate(void)
{
    const stGrayscaleSensorDeviceParamTdf *pstGrayscale =
        c_pstGetGrayscaleSensorDeviceParam(GRAYSCALE1);
    uint8_t ucBlackMask;
    uint8_t ucFinishBlackMask;

    if ((g_emLineRouteState == emLineRouteIdle) ||
        (g_emLineRouteState == emLineRouteStopped))
    {
        vChassisStop();
        return;
    }

    /* 起点位于宽黑线上时，先关闭灰度循迹并直行离开起点。 */
    if (s_ulStartBlindElapsedMs <
        s_pstLineRouteProfile->ulStartBlindTimeMs)
    {
        vLineTrackSetCurveEntryConstraint(0U);
        vChassisMoveRpm(fLineRouteUpdateCommandRpm(
            s_pstLineRouteProfile->fStraightTargetRpm));
        vLineRouteUpdateStraightDistance();
        s_ulStartBlindElapsedMs += LINE_TRACK_TASK_PERIOD_MS;
        return;
    }

    if ((pstGrayscale == NULL) ||
        (pstGrayscale->stRunningParam.ucReadyFlag == 0U))
    {
        vChassisStop();
        return;
    }

    ucBlackMask = (uint8_t)(~pstGrayscale->stRunningParam.ucDigitalOutput);
    g_ucLineRouteBlackCount = ucLineRouteCountBits(ucBlackMask);

    switch (g_emLineRouteState)
    {
        case emLineRouteStraightAB:
        case emLineRouteStraightCD:
            vLineRouteUpdateStraightDistance();
            vLineTrackSetCurveEntryConstraint(
                (g_fLineRouteSegmentDistanceMm >=
                 (s_pstLineRouteProfile->fStraightDistanceMm -
                  s_pstLineRouteProfile->fCurveEntryDistanceMm)) ? 1U : 0U);
            if (g_ucLineRouteBlackCount <
                s_pstLineRouteProfile->ucInterferenceBlackCount)
            {
                vLineTrackImuUpdateByTargetRpm(
                    fLineRouteUpdateCommandRpm(
                        s_pstLineRouteProfile->fStraightTargetRpm));
            }
            if (g_fLineRouteSegmentDistanceMm >=
                s_pstLineRouteProfile->fStraightDistanceMm)
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteCurveBC:
            if (g_ucLineRouteBlackCount <
                s_pstLineRouteProfile->ucInterferenceBlackCount)
            {
                vLineTrackCurveUpdateByTargetRpm(
                    fLineRouteUpdateCommandRpm(
                        s_pstLineRouteProfile->fCurveTargetRpm));
            }
            vLineRouteUpdateCurveAngle();
            if (g_fLineRouteCurveAngleDeg >=
                s_pstLineRouteProfile->fCurveAngleDeg)
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteCurveDA:
            if (g_ucLineRouteBlackCount <
                s_pstLineRouteProfile->ucInterferenceBlackCount)
            {
                vLineTrackCurveUpdateByTargetRpm(
                    fLineRouteUpdateCommandRpm(
                        s_pstLineRouteProfile->fCurveTargetRpm));
            }
            vLineRouteUpdateCurveAngle();
            if (((g_emLineRouteFinishMode ==
                 emLineRouteFinishByStopLine) &&
                 (g_fLineRouteCurveAngleDeg >=
                  LINE_Q2_DA_FINISH_SEARCH_ANGLE_DEG)) ||
                ((g_emLineRouteFinishMode ==
                  emLineRouteFinishByTimedStraight) &&
                 (g_fLineRouteCurveAngleDeg >=
                  s_pstLineRouteProfile->fCurveAngleDeg)))
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteFinishSearch:
            /* 完成第二个半圆后继续低速循迹，只在这里识别最终停止线。 */
            ucFinishBlackMask = (uint8_t)(ucBlackMask &
                LINE_Q2_FINISH_SENSOR_MASK);
            g_ucLineRouteMarkerActive =
                ((ucLineRouteCountBits(ucFinishBlackMask) >=
                  LINE_Q2_FINISH_BLACK_COUNT) &&
                 (ucLineRouteHasAdjacentBlack(ucFinishBlackMask) != 0U)) ?
                1U : 0U;
            if (g_ucLineRouteMarkerActive != 0U)
            {
                if (s_ucFinishConfirmCount <
                    LINE_Q2_FINISH_CONFIRM_CYCLES)
                {
                    s_ucFinishConfirmCount++;
                }
                if (s_ucFinishConfirmCount >=
                    LINE_Q2_FINISH_CONFIRM_CYCLES)
                {
                    vLineRouteAdvanceState();
                }
            }
            else
            {
                s_ucFinishConfirmCount = 0U;
                if (g_ucLineRouteBlackCount <
                    s_pstLineRouteProfile->ucInterferenceBlackCount)
                {
                    vLineTrackCurveUpdateByTargetRpm(
                        fLineRouteUpdateCommandRpm(
                            s_pstLineRouteProfile->fCurveTargetRpm));
                }
            }
            break;

        case emLineRouteFinalStraight:
            /* 第5/6问完成DA弯道后，沿直线继续循迹1秒再停车。 */
            vLineRouteUpdateStraightDistance();
            vLineTrackSetCurveEntryConstraint(0U);
            if (g_ucLineRouteBlackCount <
                s_pstLineRouteProfile->ucInterferenceBlackCount)
            {
                vLineTrackImuUpdateByTargetRpm(
                    fLineRouteUpdateCommandRpm(
                        s_pstLineRouteProfile->fStraightTargetRpm));
            }
            if (g_ulLineRouteFinalStraightElapsedMs <
                LINE_Q56_FINAL_STRAIGHT_TIME_MS)
            {
                g_ulLineRouteFinalStraightElapsedMs +=
                    LINE_TRACK_TASK_PERIOD_MS;
            }
            if (g_ulLineRouteFinalStraightElapsedMs >=
                LINE_Q56_FINAL_STRAIGHT_TIME_MS)
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteIdle:
        case emLineRouteStopped:
        default:
            vChassisStop();
            break;
    }
}

emLineRouteStateTdf emLineRouteGetState(void)
{
    return g_emLineRouteState;
}
