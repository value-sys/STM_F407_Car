/**
  * @file       line_track.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      八路灰度与IMU融合循迹模块
  */

#ifndef _LINE_TRACK_H_
#define _LINE_TRACK_H_

#include <stdint.h>

typedef enum
{
    emLineTrackStateDisabled = 0U,
    emLineTrackStateTracking,
    emLineTrackStateLost,
    emLineTrackStateIntersection,
    emLineTrackStateImuStraight
} emLineTrackStateTdf;

typedef struct
{
    float fBaseSpeed;
    float fCorrectionKp;
    float fMaxCorrectionOmega;
    float fLostSpeedScale;
    float fStraightGrayPidKp;
    float fStraightGrayPidKi;
    float fStraightGrayPidKd;
    float fCurveGrayPidKp;
    float fCurveGrayPidKi;
    float fCurveGrayPidKd;
    float fYawAnglePidKp;
    float fYawAnglePidKi;
    float fYawAnglePidKd;
    float fStraightYawRatePidKp;
    float fStraightYawRatePidKi;
    float fStraightYawRatePidKd;
    float fCurveYawRatePidKp;
    float fCurveYawRatePidKi;
    float fCurveYawRatePidKd;
    float fStraightMaxTargetYawRate;
    float fCurveMaxTargetYawRate;
    float fOuterControlWeight;
    float fImuFeedbackWeight;
} stLineTrackStaticParamTdf;

typedef struct
{
    uint8_t ucBlackMask;
    uint8_t ucActiveCount;
    int8_t cLastDirection;
    uint8_t ucEnable;
    float fLinePosition;
    float fLineError;
    float fCorrectionOmega;
    float fCommandSpeed;
    float fTargetYaw;
    float fYawError;
    float fTargetYawRate;
    float fActualYawRate;
    float fOuterControlOmega;
    float fImuControlOmega;
    emLineTrackStateTdf emState;
} stLineTrackRunningParamTdf;

typedef struct
{
    stLineTrackStaticParamTdf stStaticParam;
    stLineTrackRunningParamTdf stRunningParam;
} stLineTrackDeviceParamTdf;

const stLineTrackDeviceParamTdf *c_pstGetLineTrackDeviceParam(void);
void vLineTrackDeviceInit(const stLineTrackStaticParamTdf *pstInit);
void vLineTrackStart(void);
void vLineTrackStop(void);
/**
  * @brief  设置直线末段的弯道入口约束
  * @param  ucEnable 非0时D1-D4权重为0，且不允许底盘向左转
  */
void vLineTrackSetCurveEntryConstraint(uint8_t ucEnable);
void vLineTrackUpdate(void);
void vLineTrackUpdateByTargetRpm(float fTargetRpm);
/// @brief 纯灰度弯道循迹，丢线时按上一次方向保持速度搜索
void vLineTrackCurveUpdateByTargetRpm(float fTargetRpm);
void vLineTrackImuUpdateByTargetRpm(float fTargetRpm);
/// @brief      灰度+IMU弯道循迹，丢线后保持丢线前的有效线速度
void vLineTrackCurveImuUpdateByTargetRpm(float fTargetRpm);
void vImuStraightUpdateByTargetRpm(float fTargetRpm);

#endif
