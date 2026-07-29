/**
  * @file       motor.h
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32 HAL直流电机驱动模块头文件
  */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f4xx_hal.h"
#include "project_config.h"
#include <stdint.h>

typedef enum
{
    emDcMotorDevNum0 = 0,
    emDcMotorDevNum1,
    emDcMotorDevNumMax
} emDcMotorDevNumTdf;

typedef struct
{
    TIM_HandleTypeDef *pstTimBase;       ///< PWM定时器句柄
    uint32_t ulTimChannel;               ///< PWM比较通道
    GPIO_TypeDef *pstDirGpioBase0;       ///< 电机公共使能端口
    uint16_t usDirGpioPin0;              ///< 电机公共使能引脚
    GPIO_TypeDef *pstDirGpioBase1;       ///< 单路电机使能端口
    uint16_t usDirGpioPin1;              ///< 单路电机使能引脚
    uint16_t usPwmMaxValue;              ///< PWM最大比较值
    uint16_t usPwmStopValue;             ///< 双极性PWM停止值
    uint8_t ucPwmReverse;                ///< PWM方向反转：0-正常，1-反转
} stDcMotorStaticParamTdf;

typedef struct
{
    uint16_t usPwmValue;                 ///< 当前逻辑PWM指令值
} stDcMotorRunningParamTdf;

typedef struct
{
    stDcMotorStaticParamTdf stStaticParam;
    stDcMotorRunningParamTdf stRunningParam;
} stDcMotorDeviceParamTdf;

const stDcMotorDeviceParamTdf *c_pstGetDcMotorDeviceParam(
    emDcMotorDevNumTdf emDevNum);
void vDcMotorDeviceInit(const stDcMotorStaticParamTdf *pstInit,
    emDcMotorDevNumTdf emDevNum);
void vDcMotorSetSpeed(emDcMotorDevNumTdf emDevNum, uint16_t usPwmValue);
void vDcMotorStop(emDcMotorDevNumTdf emDevNum);

#endif
