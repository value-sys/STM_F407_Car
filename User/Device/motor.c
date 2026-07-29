/**
  * @file       motor.c
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32 HAL直流电机驱动模块
  */

#include "motor.h"
#include <string.h>

static stDcMotorDeviceParamTdf s_astDcMotorDeviceParam[DC_MOTOR_DEV_NUM];

/// @brief      将逻辑PWM转换为硬件比较值
/// @note       反转时以停止值为中心镜像，停止值本身保持不变。
static uint16_t usDcMotorConvertPwm(
    const stDcMotorStaticParamTdf *pstStatic, uint16_t usPwmValue)
{
    int32_t lHardwarePwm = (int32_t)usPwmValue;

    if (pstStatic->ucPwmReverse != 0U)
    {
        lHardwarePwm = (int32_t)pstStatic->usPwmStopValue * 2 -
            (int32_t)usPwmValue;
        if (lHardwarePwm < 0)
        {
            lHardwarePwm = 0;
        }
        else if (lHardwarePwm > (int32_t)pstStatic->usPwmMaxValue)
        {
            lHardwarePwm = (int32_t)pstStatic->usPwmMaxValue;
        }
    }
    return (uint16_t)lHardwarePwm;
}

const stDcMotorDeviceParamTdf *c_pstGetDcMotorDeviceParam(
    emDcMotorDevNumTdf emDevNum)
{
    return (emDevNum < emDcMotorDevNumMax) ?
        &s_astDcMotorDeviceParam[emDevNum] : NULL;
}

/// @brief      初始化电机PWM与使能引脚
/// @note       对应的TIM和GPIO必须已经由CubeMX初始化。
void vDcMotorDeviceInit(const stDcMotorStaticParamTdf *pstInit,
    emDcMotorDevNumTdf emDevNum)
{
    if ((pstInit == NULL) || (emDevNum >= emDcMotorDevNumMax))
    {
        return;
    }

    memcpy(&s_astDcMotorDeviceParam[emDevNum].stStaticParam,
        pstInit, sizeof(stDcMotorStaticParamTdf));
    HAL_TIM_PWM_Start(pstInit->pstTimBase, pstInit->ulTimChannel);
    HAL_GPIO_WritePin(pstInit->pstDirGpioBase0,
        pstInit->usDirGpioPin0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pstInit->pstDirGpioBase1,
        pstInit->usDirGpioPin1, GPIO_PIN_SET);
    vDcMotorStop(emDevNum);
}

void vDcMotorSetSpeed(emDcMotorDevNumTdf emDevNum, uint16_t usPwmValue)
{
    stDcMotorDeviceParamTdf *pstMotor;

    if (emDevNum >= emDcMotorDevNumMax)
    {
        return;
    }
    pstMotor = &s_astDcMotorDeviceParam[emDevNum];
    if (usPwmValue > pstMotor->stStaticParam.usPwmMaxValue)
    {
        usPwmValue = pstMotor->stStaticParam.usPwmMaxValue;
    }
    pstMotor->stRunningParam.usPwmValue = usPwmValue;
    __HAL_TIM_SET_COMPARE(pstMotor->stStaticParam.pstTimBase,
        pstMotor->stStaticParam.ulTimChannel,
        usDcMotorConvertPwm(&pstMotor->stStaticParam, usPwmValue));
}

void vDcMotorStop(emDcMotorDevNumTdf emDevNum)
{
    if (emDevNum < emDcMotorDevNumMax)
    {
        vDcMotorSetSpeed(emDevNum,
            s_astDcMotorDeviceParam[emDevNum].stStaticParam.usPwmStopValue);
    }
}
