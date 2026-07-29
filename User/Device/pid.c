#include "pid.h"
#include <math.h>


stPidControllerTdf astPidController[DC_MOTOR_DEV_NUM]; 


/// @brief      位置PID控制器初始化
/// @param      pstInit    ：PID静态参数初始化指针
/// @param      emDevNum   ：电机设备号
/// @note       
void vPidPositionInit(stPidControllerStaticParamTdf *pstInit, emDcMotorDevNumTdf emDevNum)
{
    // 1. 拷贝PID静态参数（完全对应编码器第1步）
    memcpy(&astPidController[emDevNum].stStaticParam, pstInit, sizeof(stPidControllerStaticParamTdf));
    
    // 2. 重置PID运行参数（完全对应编码器第3步）
    vPidControllerReset(&astPidController[emDevNum]);
}

/// @brief      PID控制器计算
/// @param      emDevNum   ：电机设备号
/// @param      fTarget    ：目标值
/// @param      fMeasure   ：测量值
/// @return     float      ：控制输出
/// @note       位置式PID算法实现
float fPidControllerCalculate(emDcMotorDevNumTdf emDevNum, float fTarget, float fMeasure)
{
    // 1. 更新输入值
    stPidControllerTdf *pstPid = &astPidController[emDevNum];
    pstPid->stRunningParam.fTarget = fTarget;
    pstPid->stRunningParam.fMeasure = fMeasure;

    // 2. 计算当前误差
    pstPid->stRunningParam.fErr = fTarget - fMeasure;

    // 3. 积分项计算
    pstPid->stRunningParam.fIntegral += pstPid->stRunningParam.fErr;

    // 4. 积分限幅（抗积分饱和）
    if (pstPid->stRunningParam.fIntegral > pstPid->stStaticParam.fIntegralMax)
    {
        pstPid->stRunningParam.fIntegral = pstPid->stStaticParam.fIntegralMax;
    }
    if (pstPid->stRunningParam.fIntegral < pstPid->stStaticParam.fIntegralMin)
    {
        pstPid->stRunningParam.fIntegral = pstPid->stStaticParam.fIntegralMin;
    }

    // 5. PID输出计算（位置式）
    pstPid->stRunningParam.fOutput = 
        pstPid->stStaticParam.fKp * pstPid->stRunningParam.fErr +
        pstPid->stStaticParam.fKi * pstPid->stRunningParam.fIntegral +
        pstPid->stStaticParam.fKd * (pstPid->stRunningParam.fErr - pstPid->stRunningParam.fErrLast);

    // 6. 输出限幅
    if (pstPid->stRunningParam.fOutput > pstPid->stStaticParam.fOutputMax)
    {
        pstPid->stRunningParam.fOutput = pstPid->stStaticParam.fOutputMax;
    }
    if (pstPid->stRunningParam.fOutput < pstPid->stStaticParam.fOutputMin)
    {
        pstPid->stRunningParam.fOutput = pstPid->stStaticParam.fOutputMin;
    }

    // 7. 更新历史误差
    pstPid->stRunningParam.fErrLast = pstPid->stRunningParam.fErr;

    // 8. 将PID输出转换为PWM值并设置电机速度
    uint16_t usPwmValue = 500 + (uint16_t)roundf(pstPid->stRunningParam.fOutput);
    vDcMotorSetSpeed(emDevNum, usPwmValue);

    // 9. 返回控制输出
    return pstPid->stRunningParam.fOutput;
}

/// @brief      PID控制器重置
/// @param      pstPid     ：PID控制器结构体指针
/// @note       清空积分项和历史误差
void vPidControllerReset(stPidControllerTdf *pstPid)
{
    pstPid->stRunningParam.fTarget = 0.0f;
    pstPid->stRunningParam.fMeasure = 0.0f;
    pstPid->stRunningParam.fOutput = 0.0f;
    pstPid->stRunningParam.fErr = 0.0f;
    pstPid->stRunningParam.fErrLast = 0.0f;
    pstPid->stRunningParam.fIntegral = 0.0f;
}