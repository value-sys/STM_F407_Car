/**
  * @file       encoder.h
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32定时器编码器模式测速模块头文件
  */

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "motor.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *pstTimBase;      ///< 编码器定时器句柄
    uint32_t ulTimChannel;              ///< 固定使用TIM_CHANNEL_ALL
    uint16_t usLines;                   ///< 编码器线数(PPR)
    uint16_t usReductionRatio;          ///< 电机减速比
    uint8_t ucReverse;                  ///< 计数方向反转
    uint8_t ucMode;                     ///< 计数倍频数，通常为4
} stDcMotorEncoderStaticParamTdf;

typedef struct
{
    int32_t lCount;                     ///< 软件累计计数
    int32_t lLastCount;                 ///< 上次硬件计数值
    int32_t lCurrentCount;              ///< 当前硬件计数值
    int32_t lDeltaCount;                ///< 本周期计数变化量
    float fCurrentSpeed;                ///< 滤波后速度(RPM)
    float fRawSpeed;                    ///< 原始速度(RPM)
    float fFilterAlpha;                 ///< 一阶低通滤波系数
} stDcMotorEncoderRunningParamTdf;

typedef struct
{
    stDcMotorEncoderStaticParamTdf stStaticParam;
    stDcMotorEncoderRunningParamTdf stRunningParam;
} stDcMotorEncoderDeviceParamTdf;

const stDcMotorEncoderDeviceParamTdf *c_pstGetEncoderDeviceParam(
    emDcMotorDevNumTdf emDevNum);
void vEncoderInit(const stDcMotorEncoderStaticParamTdf *pstInit,
    emDcMotorDevNumTdf emDevNum);
void vEncoderStart(emDcMotorDevNumTdf emDevNum);
void vEncoderResetCounter(emDcMotorDevNumTdf emDevNum);
void vEncoderUpdate(emDcMotorDevNumTdf emDevNum);

#endif
