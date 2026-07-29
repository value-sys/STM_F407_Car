#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "motor.h"

/// @brief          PID控制器静态参数结构体
/// @note           
typedef struct
{
    float                   fKp;                // 比例增益
    float                   fKi;                // 积分增益
    float                   fKd;                // 微分增益
    
    float                   fOutputMax;         // 输出最大值
    float                   fOutputMin;         // 输出最小值
    
    float                   fIntegralMax;       // 积分最大值（抗积分饱和）
    float                   fIntegralMin;       // 积分最小值（抗积分饱和）
} stPidControllerStaticParamTdf;

/// @brief          PID控制器运行参数结构体
/// @note           
typedef struct
{
    float                   fTarget;            // 目标值
    float                   fMeasure;           // 测量值
    
    float                   fIntegral;          // 积分项
    float                   fErr;               // 当前误差
    float                   fErrLast;           // 上一次误差
    
    float                   fOutput;            // 控制输出
} stPidControllerRunningParamTdf;

/// @brief          PID控制器完整参数结构体
/// @note           
typedef struct
{
    stPidControllerStaticParamTdf   stStaticParam;  // 静态参数
    stPidControllerRunningParamTdf  stRunningParam; // 运行参数
} stPidControllerTdf;

/// @brief      位置PID控制器初始化
/// @param      pstInit    ：PID静态参数初始化指针
/// @param      emDevNum   ：电机设备号
/// @note       必须在电机和编码器初始化后调用
void vPidPositionInit(stPidControllerStaticParamTdf *pstInit, emDcMotorDevNumTdf emDevNum);

/// @brief      位置PID控制器计算
/// @param      emDevNum   ：电机设备号
/// @param      lTarget    ：目标位置（脉冲数）
/// @param      lMeasure   ：当前位置（脉冲数）
/// @return     float      ：控制输出（-999~999）
/// @note       位置式PID算法实现
float fPidControllerCalculate(emDcMotorDevNumTdf emDevNum, float fTarget, float fMeasure);

/// @brief      PID控制器重置
/// @param      pstPid     ：PID控制器结构体指针
/// @note       清空积分项和历史误差
void vPidControllerReset(stPidControllerTdf *pstPid);

#endif /* __PID_H */