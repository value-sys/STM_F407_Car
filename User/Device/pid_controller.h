/**
  * @file       pid_controller.h
  * @version    V1.0.0
  * @date       20260721
  * @brief      从MSPM0工程移植的通用PID控制器头文件
  */

#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include <stdint.h>

typedef enum
{
    PID_IMPROVE_NONE = 0x0000U,
    PID_INTEGRAL_LIMIT = 0x0001U,
    PID_DERIVATIVE_ON_MEASUREMENT = 0x0002U,
    PID_TRAPEZOID_INTEGRAL = 0x0004U,
    PID_PROPORTIONAL_ON_MEASUREMENT = 0x0008U,
    PID_OUTPUT_FILTER = 0x0010U,
    PID_CHANGING_INTEGRATION_RATE = 0x0020U,
    PID_DERIVATIVE_FILTER = 0x0040U,
    PID_ERROR_HANDLE = 0x0080U,
    PID_INCREMENTAL_OUTPUT = 0x0100U,
    PID_FEEDFORWARD_CONTROL = 0x0200U,
    PID_DEADBAND_REMAIN_IOUT = 0x0400U
} emPidImprovementTdf;

typedef enum
{
    PID_ERROR_NONE = 0U,
    PID_ERROR_MOTOR_BLOCKED
} emPidErrorTypeTdf;

typedef struct
{
    uint32_t ulErrorCount;
    emPidErrorTypeTdf emErrorType;
} stPidErrorHandlerTdf;

typedef struct stPidController
{
    float fRef, fKp, fKi, fKd;
    float fFeedforwardJ, fFeedforwardB;
    float fMeasure, fLastMeasure, fEarlierMeasure;
    float fErr, fLastErr, fEarlierErr;
    float fLastIntegralTerm, fPout, fIout, fDout, fIntegralTerm;
    float fFeedforwardOut, fOutput, fLastOutput, fLastDout;
    float fMaxOut, fIntegralLimit, fDeadBand, fControlPeriod;
    float fCoefA, fCoefB, fOutputLpfRc, fDerivativeLpfRc, fDt;
    uint16_t usImprove;
    stPidErrorHandlerTdf stErrorHandler;
    void (*pfUserFunc1)(struct stPidController *pstPid);
    void (*pfUserFunc2)(struct stPidController *pstPid);
} stPidControllerTdf;

typedef stPidControllerTdf PID_t;

void PID_Init(PID_t *pstPid);
float PID_Calculate(PID_t *pstPid, float fMeasure, float fRef);
void PID_Reset(PID_t *pstPid);

#endif
