/**
  * @file       line_route.h
  * @version    V1.0.0
  * @date       20260731
  * @brief      第2、5、6问共用的ABCD环形路线循迹状态机
  */

#ifndef _LINE_ROUTE_H_
#define _LINE_ROUTE_H_

#include <stdint.h>

typedef enum
{
    emLineRouteIdle = 0U,
    emLineRouteStraightAB,
    emLineRouteCurveBC,
    emLineRouteStraightCD,
    emLineRouteCurveDA,
    emLineRouteFinishSearch,
    emLineRouteFinalStraight,
    emLineRouteStopped
} emLineRouteStateTdf;

typedef enum
{
    emLineRouteFinishByStopLine = 0U,
    emLineRouteFinishByTimedStraight
} emLineRouteFinishModeTdf;

/* 供Ozone直接观察的路线状态机运行数据。 */
extern volatile emLineRouteStateTdf g_emLineRouteState;
extern volatile uint8_t g_ucLineRouteBlackCount;
extern volatile uint8_t g_ucLineRouteMarkerArmed;
extern volatile uint8_t g_ucLineRouteMarkerActive;
extern volatile uint8_t g_ucLineRouteMarkerCount;
extern volatile float g_fLineRouteSegmentDistanceMm;
extern volatile float g_fLineRouteCurveAngleDeg;
extern volatile float g_fLineRouteCommandRpm;
extern volatile uint32_t g_ulLineRouteFinalStraightElapsedMs;
extern volatile emLineRouteFinishModeTdf g_emLineRouteFinishMode;

/** @brief 启动第2问整圈循迹，最后通过停止线停车。 */
void vLineRouteStart(void);
/** @brief 启动第5/6问共用循迹，最后转入直线并定时停车。 */
void vLineRouteExtendedStart(void);
void vLineRouteStop(void);
void vLineRouteUpdate(void);
emLineRouteStateTdf emLineRouteGetState(void);

#endif
