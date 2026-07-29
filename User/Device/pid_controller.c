/**
  * @file       pid_controller.c
  * @version    V1.0.0
  * @date       20260721
  * @brief      从MSPM0工程移植的通用PID控制器
  * @note       控制任务使用固定周期，因此不依赖芯片专用高精度计时器。
  */

#include "pid_controller.h"
#include <stddef.h>

static float fPidAbs(float fValue)
{
    return (fValue >= 0.0f) ? fValue : -fValue;
}

static void vPidLimit(float *pfValue, float fLimit)
{
    if (*pfValue > fLimit)
    {
        *pfValue = fLimit;
    }
    else if (*pfValue < -fLimit)
    {
        *pfValue = -fLimit;
    }
}

void PID_Reset(PID_t *pstPid)
{
    if (pstPid == NULL)
    {
        return;
    }
    pstPid->fRef = 0.0f;
    pstPid->fMeasure = 0.0f;
    pstPid->fLastMeasure = 0.0f;
    pstPid->fEarlierMeasure = 0.0f;
    pstPid->fErr = 0.0f;
    pstPid->fLastErr = 0.0f;
    pstPid->fEarlierErr = 0.0f;
    pstPid->fLastIntegralTerm = 0.0f;
    pstPid->fPout = 0.0f;
    pstPid->fIout = 0.0f;
    pstPid->fDout = 0.0f;
    pstPid->fIntegralTerm = 0.0f;
    pstPid->fFeedforwardOut = 0.0f;
    pstPid->fOutput = 0.0f;
    pstPid->fLastOutput = 0.0f;
    pstPid->fLastDout = 0.0f;
    pstPid->fDt = pstPid->fControlPeriod;
    pstPid->stErrorHandler.ulErrorCount = 0U;
    pstPid->stErrorHandler.emErrorType = PID_ERROR_NONE;
}

void PID_Init(PID_t *pstPid)
{
    if (pstPid == NULL)
    {
        return;
    }
    if (pstPid->fControlPeriod <= 0.0f)
    {
        pstPid->fControlPeriod = 0.001f;
    }
    PID_Reset(pstPid);
}

float PID_Calculate(PID_t *pstPid, float fMeasure, float fRef)
{
    uint8_t ucIncremental;
    float fTemporaryOutput;
    float fTemporaryIout;

    if (pstPid == NULL)
    {
        return 0.0f;
    }
    pstPid->fDt = pstPid->fControlPeriod;
    pstPid->fMeasure = fMeasure;
    pstPid->fRef = fRef;
    pstPid->fErr = fRef - fMeasure;
    ucIncremental = ((pstPid->usImprove & PID_INCREMENTAL_OUTPUT) != 0U);

    if (pstPid->pfUserFunc1 != NULL)
    {
        pstPid->pfUserFunc1(pstPid);
    }

    if (fPidAbs(pstPid->fErr) > pstPid->fDeadBand)
    {
        if (ucIncremental != 0U)
        {
            pstPid->fPout = pstPid->fKp * (pstPid->fErr - pstPid->fLastErr);
            pstPid->fDout = pstPid->fKd *
                (pstPid->fErr + pstPid->fEarlierErr - 2.0f * pstPid->fLastErr) /
                pstPid->fDt;
        }
        else
        {
            pstPid->fPout = pstPid->fKp * pstPid->fErr;
            pstPid->fDout = pstPid->fKd *
                (pstPid->fErr - pstPid->fLastErr) / pstPid->fDt;
        }
        pstPid->fIntegralTerm = pstPid->fKi * pstPid->fErr * pstPid->fDt;

        if (pstPid->pfUserFunc2 != NULL)
        {
            pstPid->pfUserFunc2(pstPid);
        }
        if ((pstPid->usImprove & PID_TRAPEZOID_INTEGRAL) != 0U)
        {
            pstPid->fIntegralTerm = pstPid->fKi *
                (pstPid->fErr + pstPid->fLastErr) * 0.5f * pstPid->fDt;
        }
        if (((pstPid->usImprove & PID_CHANGING_INTEGRATION_RATE) != 0U) &&
            (pstPid->fErr * pstPid->fIout > 0.0f))
        {
            if (fPidAbs(pstPid->fErr) > pstPid->fCoefA + pstPid->fCoefB)
            {
                pstPid->fIntegralTerm = 0.0f;
            }
            else if ((fPidAbs(pstPid->fErr) > pstPid->fCoefB) &&
                     (pstPid->fCoefA > 0.0f))
            {
                pstPid->fIntegralTerm *=
                    (pstPid->fCoefA - fPidAbs(pstPid->fErr) + pstPid->fCoefB) /
                    pstPid->fCoefA;
            }
        }
        if ((pstPid->usImprove & PID_DERIVATIVE_ON_MEASUREMENT) != 0U)
        {
            pstPid->fDout = (ucIncremental != 0U) ?
                -pstPid->fKd * (pstPid->fMeasure + pstPid->fEarlierMeasure -
                    2.0f * pstPid->fLastMeasure) / pstPid->fDt :
                pstPid->fKd * (pstPid->fLastMeasure - pstPid->fMeasure) /
                    pstPid->fDt;
        }
        if (((pstPid->usImprove & PID_DERIVATIVE_FILTER) != 0U) &&
            (pstPid->fDerivativeLpfRc + pstPid->fDt > 0.0f))
        {
            pstPid->fDout = (pstPid->fDout * pstPid->fDt +
                pstPid->fLastDout * pstPid->fDerivativeLpfRc) /
                (pstPid->fDerivativeLpfRc + pstPid->fDt);
        }
        if ((pstPid->usImprove & PID_INTEGRAL_LIMIT) != 0U)
        {
            fTemporaryOutput = ucIncremental != 0U ?
                pstPid->fPout + pstPid->fIntegralTerm + pstPid->fDout +
                    pstPid->fLastOutput :
                pstPid->fPout + pstPid->fIout + pstPid->fDout;
            if (ucIncremental != 0U)
            {
                if ((fPidAbs(fTemporaryOutput) > pstPid->fMaxOut) &&
                    (pstPid->fErr * fTemporaryOutput > 0.0f))
                {
                    pstPid->fIntegralTerm = 0.0f;
                }
            }
            else
            {
                fTemporaryIout = pstPid->fIout + pstPid->fIntegralTerm;
                if ((fPidAbs(fTemporaryOutput) > pstPid->fMaxOut) &&
                    (pstPid->fErr * pstPid->fIout > 0.0f))
                {
                    pstPid->fIntegralTerm = 0.0f;
                }
                if (fTemporaryIout > pstPid->fIntegralLimit)
                {
                    pstPid->fIout = pstPid->fIntegralLimit;
                    pstPid->fIntegralTerm = 0.0f;
                }
                else if (fTemporaryIout < -pstPid->fIntegralLimit)
                {
                    pstPid->fIout = -pstPid->fIntegralLimit;
                    pstPid->fIntegralTerm = 0.0f;
                }
            }
        }

        if (ucIncremental == 0U)
        {
            pstPid->fIout += pstPid->fIntegralTerm;
        }
        pstPid->fOutput = pstPid->fPout + pstPid->fDout +
            (ucIncremental != 0U ? pstPid->fIntegralTerm + pstPid->fLastOutput :
                                  pstPid->fIout);
        if (((pstPid->usImprove & PID_OUTPUT_FILTER) != 0U) &&
            (pstPid->fOutputLpfRc + pstPid->fDt > 0.0f))
        {
            pstPid->fOutput = (pstPid->fOutput * pstPid->fDt +
                pstPid->fLastOutput * pstPid->fOutputLpfRc) /
                (pstPid->fOutputLpfRc + pstPid->fDt);
        }
        vPidLimit(&pstPid->fOutput, pstPid->fMaxOut);
        vPidLimit(&pstPid->fPout, pstPid->fMaxOut);
    }
    else
    {
        pstPid->fOutput = 0.0f;
        if ((pstPid->usImprove & PID_DEADBAND_REMAIN_IOUT) == 0U)
        {
            pstPid->fIout = 0.0f;
        }
    }

    pstPid->fEarlierMeasure = pstPid->fLastMeasure;
    pstPid->fLastMeasure = pstPid->fMeasure;
    pstPid->fLastDout = pstPid->fDout;
    pstPid->fEarlierErr = pstPid->fLastErr;
    pstPid->fLastErr = pstPid->fErr;
    pstPid->fLastIntegralTerm = pstPid->fIntegralTerm;
    pstPid->fLastOutput = pstPid->fOutput;
    return pstPid->fOutput;
}
