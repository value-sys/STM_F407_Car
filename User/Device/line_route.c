/**
  * @file       line_route.c
  * @version    V1.1.0
  * @date       20260731
  * @brief      ABCD环形路线循迹状态机
  */

#include "line_route.h"
#include "GrayscaleSensor.h"
#include "chassis.h"
#include "encoder.h"
#include "imu.h"
#include "line_track.h"
#include "project_config.h"

#define LINE_ROUTE_TWO_PI                6.28318530717958647692f
volatile emLineRouteStateTdf g_emLineRouteState = emLineRouteIdle;
volatile uint8_t g_ucLineRouteBlackCount;
volatile uint8_t g_ucLineRouteMarkerArmed;
volatile uint8_t g_ucLineRouteMarkerActive;
volatile uint8_t g_ucLineRouteMarkerCount;
volatile float g_fLineRouteSegmentDistanceMm;
volatile float g_fLineRouteCurveAngleDeg;

static uint8_t s_ucFinishConfirmCount;
static int32_t s_lMotor1SegmentStartCount;
static int32_t s_lMotor2SegmentStartCount;
static uint32_t s_ulLastImuSampleCount;
static float s_fLastYawDeg;
static float s_fCurveSignedAngleDeg;

static float fLineRouteAbs(float fValue)
{
    return (fValue < 0.0f) ? -fValue : fValue;
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
            g_emLineRouteState = emLineRouteFinishSearch;
            g_ucLineRouteMarkerArmed = 1U;
            break;
        case emLineRouteFinishSearch:
            g_emLineRouteState = emLineRouteStopped;
            g_ucLineRouteMarkerArmed = 0U;
            g_ucLineRouteMarkerActive = 0U;
            vLineTrackStop();
            vChassisStop();
            break;
        case emLineRouteIdle:
        case emLineRouteStopped:
        default:
            return;
    }

    g_ucLineRouteMarkerCount++;
    s_ucFinishConfirmCount = 0U;
}

void vLineRouteStart(void)
{
    g_emLineRouteState = emLineRouteStraightAB;
    g_ucLineRouteBlackCount = 0U;
    g_ucLineRouteMarkerArmed = 0U;
    g_ucLineRouteMarkerActive = 0U;
    g_ucLineRouteMarkerCount = 0U;
    g_fLineRouteCurveAngleDeg = 0.0f;
    s_ucFinishConfirmCount = 0U;
    vLineRouteCaptureStraightStart();
    vLineTrackStart();
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
    s_ucFinishConfirmCount = 0U;
    vLineTrackStop();
    vChassisStop();
}

void vLineRouteUpdate(void)
{
    const stGrayscaleSensorDeviceParamTdf *pstGrayscale =
        c_pstGetGrayscaleSensorDeviceParam(GRAYSCALE1);
    uint8_t ucBlackMask;

    if ((g_emLineRouteState == emLineRouteIdle) ||
        (g_emLineRouteState == emLineRouteStopped))
    {
        vChassisStop();
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
            vLineTrackImuUpdateByTargetRpm(LINE_ROUTE_STRAIGHT_TARGET_RPM);
            vLineRouteUpdateStraightDistance();
            if (g_fLineRouteSegmentDistanceMm >=
                LINE_ROUTE_STRAIGHT_DISTANCE_MM)
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteCurveBC:
            vLineTrackUpdateByTargetRpm(LINE_ROUTE_CURVE_TARGET_RPM);
            vLineRouteUpdateCurveAngle();
            if (g_fLineRouteCurveAngleDeg >= LINE_ROUTE_CURVE_ANGLE_DEG)
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteCurveDA:
            vLineTrackUpdateByTargetRpm(LINE_ROUTE_CURVE_TARGET_RPM);
            vLineRouteUpdateCurveAngle();
            if (g_fLineRouteCurveAngleDeg >=
                LINE_ROUTE_DA_FINISH_SEARCH_ANGLE_DEG)
            {
                vLineRouteAdvanceState();
            }
            break;

        case emLineRouteFinishSearch:
            /* 完成第二个半圆后继续低速循迹，只在这里识别最终停止线。 */
            vLineTrackUpdateByTargetRpm(LINE_ROUTE_CURVE_TARGET_RPM);
            g_ucLineRouteMarkerActive =
                (g_ucLineRouteBlackCount >= LINE_ROUTE_FINISH_BLACK_COUNT) ?
                1U : 0U;
            if (g_ucLineRouteMarkerActive != 0U)
            {
                if (s_ucFinishConfirmCount < LINE_ROUTE_FINISH_CONFIRM_CYCLES)
                {
                    s_ucFinishConfirmCount++;
                }
                if (s_ucFinishConfirmCount >= LINE_ROUTE_FINISH_CONFIRM_CYCLES)
                {
                    vLineRouteAdvanceState();
                }
            }
            else
            {
                s_ucFinishConfirmCount = 0U;
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
