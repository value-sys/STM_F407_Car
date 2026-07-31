/**
  * @file       line_route.h
  * @version    V1.0.0
  * @date       20260731
  * @brief      ABCD环形路线循迹状态机
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
    emLineRouteStopped
} emLineRouteStateTdf;

/* 供Ozone直接观察的路线状态机运行数据。 */
extern volatile emLineRouteStateTdf g_emLineRouteState;
extern volatile uint8_t g_ucLineRouteBlackCount;
extern volatile uint8_t g_ucLineRouteMarkerArmed;
extern volatile uint8_t g_ucLineRouteMarkerActive;
extern volatile uint8_t g_ucLineRouteMarkerCount;
extern volatile float g_fLineRouteSegmentDistanceMm;
extern volatile float g_fLineRouteCurveAngleDeg;
extern volatile float g_fLineRouteCommandRpm;

void vLineRouteStart(void);
void vLineRouteStop(void);
void vLineRouteUpdate(void);
emLineRouteStateTdf emLineRouteGetState(void);

#endif
