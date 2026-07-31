/**
  * @file       line_ab_curve_test.h
  * @brief      AB直线后切换弯道循迹并定时停车的独立状态机
  */

#ifndef _LINE_AB_CURVE_TEST_H_
#define _LINE_AB_CURVE_TEST_H_

#include <stdint.h>

typedef enum
{
    emLineAbCurveIdle = 0U,
    emLineAbCurveStraightAB,
    emLineAbCurveTimedCurve,
    emLineAbCurveStopped
} emLineAbCurveStateTdf;

extern volatile emLineAbCurveStateTdf g_emLineAbCurveState;
extern volatile float g_fLineAbCurveDistanceMm;
extern volatile float g_fLineAbCurveCommandRpm;
extern volatile uint32_t g_ulLineAbCurveElapsedMs;

void vLineAbCurveStart(void);
void vLineAbCurveStop(void);
void vLineAbCurveUpdate(void);
emLineAbCurveStateTdf emLineAbCurveGetState(void);

#endif
