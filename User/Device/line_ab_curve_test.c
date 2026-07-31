/**
  * @file       line_ab_curve_test.c
  * @brief      AB直线后切换弯道循迹并定时停车的独立状态机
  */

#include "line_ab_curve_test.h"
#include "GrayscaleSensor.h"
#include "chassis.h"
#include "encoder.h"
#include "line_track.h"
#include "project_config.h"
#include <stddef.h>

#define LINE_AB_CURVE_TWO_PI 6.28318530717958647692f

volatile emLineAbCurveStateTdf g_emLineAbCurveState = emLineAbCurveIdle;
volatile float g_fLineAbCurveDistanceMm;
volatile float g_fLineAbCurveCommandRpm;
volatile uint32_t g_ulLineAbCurveElapsedMs;

static int32_t s_lMotor1StartCount;
static int32_t s_lMotor2StartCount;

static float fLineAbCurveAbs(float fValue)
{
    return (fValue < 0.0f) ? -fValue : fValue;
}

static uint8_t ucLineAbCurveCountBits(uint8_t ucValue)
{
    uint8_t ucCount = 0U;

    while (ucValue != 0U)
    {
        ucCount += (uint8_t)(ucValue & 0x01U);
        ucValue >>= 1U;
    }
    return ucCount;
}

static float fLineAbCurveUpdateCommandRpm(float fTargetRpm)
{
    float fErrorRpm = fTargetRpm - g_fLineAbCurveCommandRpm;
    float fRateRpmPerS = (fErrorRpm >= 0.0f) ?
        LINE_AB_CURVE_ACCEL_RPM_PER_S : LINE_AB_CURVE_DECEL_RPM_PER_S;
    float fMaxStepRpm;

    if (fRateRpmPerS <= 0.0f)
    {
        g_fLineAbCurveCommandRpm = fTargetRpm;
        return g_fLineAbCurveCommandRpm;
    }
    fMaxStepRpm = fRateRpmPerS *
        (float)LINE_TRACK_TASK_PERIOD_MS / 1000.0f;
    if (fErrorRpm > fMaxStepRpm)
    {
        g_fLineAbCurveCommandRpm += fMaxStepRpm;
    }
    else if (fErrorRpm < -fMaxStepRpm)
    {
        g_fLineAbCurveCommandRpm -= fMaxStepRpm;
    }
    else
    {
        g_fLineAbCurveCommandRpm = fTargetRpm;
    }
    return g_fLineAbCurveCommandRpm;
}

static float fLineAbCurveWheelDistanceMm(
    const stDcMotorEncoderDeviceParamTdf *pstEncoder,
    int32_t lStartCount, float fWheelRadiusMm)
{
    float fCountsPerRevolution;
    float fDeltaCount;

    if ((pstEncoder == NULL) || (fWheelRadiusMm <= 0.0f))
    {
        return 0.0f;
    }
    fCountsPerRevolution =
        (float)pstEncoder->stStaticParam.usLines *
        (float)pstEncoder->stStaticParam.usReductionRatio *
        (float)pstEncoder->stStaticParam.ucMode;
    if (fCountsPerRevolution <= 0.0f)
    {
        return 0.0f;
    }
    fDeltaCount = (float)(pstEncoder->stRunningParam.lCount - lStartCount);
    return fLineAbCurveAbs(fDeltaCount) * LINE_AB_CURVE_TWO_PI *
        fWheelRadiusMm / fCountsPerRevolution;
}

static void vLineAbCurveCaptureStart(void)
{
    const stDcMotorEncoderDeviceParamTdf *pstEncoder1 =
        c_pstGetEncoderDeviceParam(DC_MOTOR1);
    const stDcMotorEncoderDeviceParamTdf *pstEncoder2 =
        c_pstGetEncoderDeviceParam(DC_MOTOR2);

    s_lMotor1StartCount = (pstEncoder1 != NULL) ?
        pstEncoder1->stRunningParam.lCount : 0;
    s_lMotor2StartCount = (pstEncoder2 != NULL) ?
        pstEncoder2->stRunningParam.lCount : 0;
    g_fLineAbCurveDistanceMm = 0.0f;
}

static void vLineAbCurveUpdateDistance(void)
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
        g_fLineAbCurveDistanceMm = 0.0f;
        return;
    }
    fMotor1DistanceMm = fLineAbCurveWheelDistanceMm(pstEncoder1,
        s_lMotor1StartCount, pstChassis->stStaticParam.fWheelRadius);
    fMotor2DistanceMm = fLineAbCurveWheelDistanceMm(pstEncoder2,
        s_lMotor2StartCount, pstChassis->stStaticParam.fWheelRadius);
    g_fLineAbCurveDistanceMm =
        (fMotor1DistanceMm + fMotor2DistanceMm) * 0.5f;
}

void vLineAbCurveStart(void)
{
    g_emLineAbCurveState = emLineAbCurveStraightAB;
    g_fLineAbCurveCommandRpm = LINE_AB_CURVE_STRAIGHT_TARGET_RPM;
    g_ulLineAbCurveElapsedMs = 0U;
    vLineAbCurveCaptureStart();
    vLineTrackStart();
}

void vLineAbCurveStop(void)
{
    g_emLineAbCurveState = emLineAbCurveIdle;
    g_fLineAbCurveDistanceMm = 0.0f;
    g_fLineAbCurveCommandRpm = 0.0f;
    g_ulLineAbCurveElapsedMs = 0U;
    vLineTrackStop();
    vChassisStop();
}

void vLineAbCurveUpdate(void)
{
    const stGrayscaleSensorDeviceParamTdf *pstGrayscale =
        c_pstGetGrayscaleSensorDeviceParam(GRAYSCALE1);
    uint8_t ucBlackCount;

    if ((g_emLineAbCurveState == emLineAbCurveIdle) ||
        (g_emLineAbCurveState == emLineAbCurveStopped))
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
    ucBlackCount = ucLineAbCurveCountBits(
        (uint8_t)(~pstGrayscale->stRunningParam.ucDigitalOutput));

    switch (g_emLineAbCurveState)
    {
        case emLineAbCurveStraightAB:
            if (ucBlackCount < LINE_AB_CURVE_INTERFERENCE_BLACK_COUNT)
            {
                vLineTrackImuUpdateByTargetRpm(
                    fLineAbCurveUpdateCommandRpm(
                        LINE_AB_CURVE_STRAIGHT_TARGET_RPM));
            }
            vLineAbCurveUpdateDistance();
            if (g_fLineAbCurveDistanceMm >=
                LINE_AB_CURVE_STRAIGHT_DISTANCE_MM)
            {
                g_emLineAbCurveState = emLineAbCurveTimedCurve;
                g_ulLineAbCurveElapsedMs = 0U;
            }
            break;

        case emLineAbCurveTimedCurve:
            if (ucBlackCount < LINE_AB_CURVE_INTERFERENCE_BLACK_COUNT)
            {
                vLineTrackCurveUpdateByTargetRpm(
                    fLineAbCurveUpdateCommandRpm(
                        LINE_AB_CURVE_CURVE_TARGET_RPM));
            }
            if (g_ulLineAbCurveElapsedMs < LINE_AB_CURVE_RUN_TIME_MS)
            {
                g_ulLineAbCurveElapsedMs += LINE_TRACK_TASK_PERIOD_MS;
            }
            if (g_ulLineAbCurveElapsedMs >= LINE_AB_CURVE_RUN_TIME_MS)
            {
                g_emLineAbCurveState = emLineAbCurveStopped;
                g_fLineAbCurveCommandRpm = 0.0f;
                vLineTrackStop();
                vChassisStop();
            }
            break;

        case emLineAbCurveIdle:
        case emLineAbCurveStopped:
        default:
            vChassisStop();
            break;
    }
}

emLineAbCurveStateTdf emLineAbCurveGetState(void)
{
    return g_emLineAbCurveState;
}
