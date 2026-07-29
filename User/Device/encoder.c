/**
  * @file       encoder.c
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32定时器编码器模式测速模块
  */

#include "encoder.h"
#include "project_config.h"
#include <string.h>

static stDcMotorEncoderDeviceParamTdf
    s_astEncoderDeviceParam[DC_MOTOR_DEV_NUM];

const stDcMotorEncoderDeviceParamTdf *c_pstGetEncoderDeviceParam(
    emDcMotorDevNumTdf emDevNum)
{
    return (emDevNum < emDcMotorDevNumMax) ?
        &s_astEncoderDeviceParam[emDevNum] : NULL;
}

void vEncoderInit(const stDcMotorEncoderStaticParamTdf *pstInit,
    emDcMotorDevNumTdf emDevNum)
{
    if ((pstInit == NULL) || (emDevNum >= emDcMotorDevNumMax))
    {
        return;
    }
    memcpy(&s_astEncoderDeviceParam[emDevNum].stStaticParam,
        pstInit, sizeof(stDcMotorEncoderStaticParamTdf));
    s_astEncoderDeviceParam[emDevNum].stRunningParam.fFilterAlpha =
        ENCODER_SPEED_FILTER_ALPHA;
    vEncoderStart(emDevNum);
    vEncoderResetCounter(emDevNum);
}

/// @brief      启动STM32硬件编码器模式
void vEncoderStart(emDcMotorDevNumTdf emDevNum)
{
    stDcMotorEncoderStaticParamTdf *pstStatic;

    if (emDevNum >= emDcMotorDevNumMax)
    {
        return;
    }
    pstStatic = &s_astEncoderDeviceParam[emDevNum].stStaticParam;
    if (pstStatic->pstTimBase != NULL)
    {
        (void)HAL_TIM_Encoder_Start(pstStatic->pstTimBase,
            pstStatic->ulTimChannel);
    }
}

void vEncoderResetCounter(emDcMotorDevNumTdf emDevNum)
{
    stDcMotorEncoderStaticParamTdf *pstStatic;
    stDcMotorEncoderRunningParamTdf *pstRunning;

    if (emDevNum >= emDcMotorDevNumMax)
    {
        return;
    }
    pstStatic = &s_astEncoderDeviceParam[emDevNum].stStaticParam;
    pstRunning = &s_astEncoderDeviceParam[emDevNum].stRunningParam;
    if (pstStatic->pstTimBase == NULL)
    {
        return;
    }
    __HAL_TIM_SET_COUNTER(pstStatic->pstTimBase, 0U);
    pstRunning->lCount = 0;
    pstRunning->lLastCount = 0;
    pstRunning->lCurrentCount = 0;
    pstRunning->lDeltaCount = 0;
    pstRunning->fCurrentSpeed = 0.0f;
    pstRunning->fRawSpeed = 0.0f;
}

/// @brief      更新累计计数和转速
/// @note       依赖当前工程TIM3/TIM4编码器模式；通过ARR自动适配16/32位计数器。
void vEncoderUpdate(emDcMotorDevNumTdf emDevNum)
{
    stDcMotorEncoderStaticParamTdf *pstStatic;
    stDcMotorEncoderRunningParamTdf *pstRunning;
    uint32_t ulCounterPeriod;
    int32_t lDelta;
    float fCountsPerRevolution;
    float fRawSpeed;
    float fAlpha;

    if (emDevNum >= emDcMotorDevNumMax)
    {
        return;
    }
    pstStatic = &s_astEncoderDeviceParam[emDevNum].stStaticParam;
    pstRunning = &s_astEncoderDeviceParam[emDevNum].stRunningParam;
    if ((pstStatic->pstTimBase == NULL) || (pstStatic->usLines == 0U) ||
        (pstStatic->usReductionRatio == 0U) || (pstStatic->ucMode == 0U))
    {
        return;
    }

    pstRunning->lCurrentCount =
        (int32_t)__HAL_TIM_GET_COUNTER(pstStatic->pstTimBase);
    lDelta = pstRunning->lCurrentCount - pstRunning->lLastCount;
    ulCounterPeriod = __HAL_TIM_GET_AUTORELOAD(pstStatic->pstTimBase) + 1U;
    if (lDelta > (int32_t)(ulCounterPeriod / 2U))
    {
        lDelta -= (int32_t)ulCounterPeriod;
    }
    else if (lDelta < -(int32_t)(ulCounterPeriod / 2U))
    {
        lDelta += (int32_t)ulCounterPeriod;
    }
    if (pstStatic->ucReverse != 0U)
    {
        lDelta = -lDelta;
    }

    pstRunning->lDeltaCount = lDelta;
    pstRunning->lCount += lDelta;
    pstRunning->lLastCount = pstRunning->lCurrentCount;

    fCountsPerRevolution = (float)pstStatic->usLines *
        (float)pstStatic->usReductionRatio * (float)pstStatic->ucMode;
    fRawSpeed = (float)lDelta * 60000.0f /
        (fCountsPerRevolution * (float)MOTOR_SAMPLE_TIME);

    if (ENCODER_MAX_SPEED_DELTA_RPM > 0.0f)
    {
        float fDeltaSpeed = fRawSpeed - pstRunning->fCurrentSpeed;
        if (fDeltaSpeed > ENCODER_MAX_SPEED_DELTA_RPM)
        {
            fRawSpeed = pstRunning->fCurrentSpeed +
                ENCODER_MAX_SPEED_DELTA_RPM;
        }
        else if (fDeltaSpeed < -ENCODER_MAX_SPEED_DELTA_RPM)
        {
            fRawSpeed = pstRunning->fCurrentSpeed -
                ENCODER_MAX_SPEED_DELTA_RPM;
        }
    }
    pstRunning->fRawSpeed = fRawSpeed;
    fAlpha = pstRunning->fFilterAlpha;
    if (fAlpha > 1.0f)
    {
        fAlpha = 1.0f;
    }
    else if (fAlpha < 0.0f)
    {
        fAlpha = 0.0f;
    }
    pstRunning->fCurrentSpeed = fAlpha * fRawSpeed +
        (1.0f - fAlpha) * pstRunning->fCurrentSpeed;
}
