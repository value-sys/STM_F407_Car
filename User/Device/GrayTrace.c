/// @file       GrayTrace.c
/// @version    V1.0.0
/// @date       20260627
/// @brief      循迹功能函数

#include "GrayTrace.h"
#include <math.h>

#define GRAY_POSITIONS { -35, -25, -15, -5, 5, 15, 25, 35 }
#define TRACE_KP     1.0f
const int gray_positions[8] = GRAY_POSITIONS;


static const int s_iGrayPositions[8] = { -35, -25, -15, -5, 5, 15, 25, 35 };
#define TRACE_MAX_POSITION_MM     35.0f

// 循迹设备参数数组（与传感器设备一一对应）
static stTraceDeviceParamTdf s_astTraceDeviceParam[GRAYSCALE_SENSOR_DEV_NUM];

// ================= 读取传感器原始数据 =================
uint8_t Gray_ReadRaw(emGrayscaleSensorDevNumTdf emDevNum)
{
    return  ucGrayscaleSensorGetDigital(emDevNum);
}

// ================= 计算黑线位置 =================
float Gray_GetLinePosition(uint8_t raw)
{
    int sum = 0, val = 0;
    for (int i = 0; i < 8; i++)
    {
        int bit = (raw >> i) & 0x01;
        sum += bit;
        val += bit * gray_positions[i];
    }
    return (sum == 0) ? 0.0f : (float)val / sum;
}

// ================= 计算误差 =================
float Gray_GetLineError(uint8_t raw)
{
    return Gray_GetLinePosition(raw) / 35.0f;
}


// ================= 基础循迹逻辑 =================
void Trace_Run(uint8_t raw,float fVy)
{
    float error = Gray_GetLineError(raw);
    int active_sensor_count = 0;
    for (int i = 0; i < 8; i++)
        if (raw & (1 << i)) active_sensor_count++;

    float omega = error * TRACE_KP;

    vChassisSetSpeed( fVy,omega);
}






//加pid的循迹

/// @brief      循迹PID参数初始化
///
/// @param      pstInitParam ：PID静态参数指针
/// @param      emDevNum     ：传感器设备号
///
/// @note   
void vTracePidInit(const stTracePidStaticParamTdf *pstInitParam, emGrayscaleSensorDevNumTdf emDevNum)
{
    stTraceDeviceParamTdf *pstDev = &s_astTraceDeviceParam[emDevNum];
    stTraceRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    
    // 1. 拷贝静态配置参数
    memcpy(&pstDev->stStaticParam, pstInitParam, sizeof(stTracePidStaticParamTdf));
    
    // 2. 运行状态清零初始化
    pstRunning->fIntegralSum = 0.0f;
    pstRunning->fLastError = 0.0f;
    pstRunning->fCurrentError = 0.0f;
    pstRunning->fOutputOmega = 0.0f;
    pstRunning->fLinePosition = 0.0f;
    pstRunning->ucLineLostFlag = 0;
}

/// @brief      计算黑线物理位置
///
/// @param      ucDigitalVal ：8路数字量结果
/// @param      emDevNum     ：传感器设备号
///
/// @return     黑线位置 (mm)
///
/// @note       丢线时保持上一次有效位置，置位丢线标志
float fTraceGetLinePosition(uint8_t ucDigitalVal, emGrayscaleSensorDevNumTdf emDevNum)
{
    stTraceRunningParamTdf *pstRunning = &s_astTraceDeviceParam[emDevNum].stRunningParam;
    int32_t lSumWeight = 0;
    uint8_t ucActiveCount = 0;
    
    // 1. 加权计算黑线中心位置
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((ucDigitalVal >> i) & 0x01)
        {
            lSumWeight += s_iGrayPositions[i];
            ucActiveCount++;
        }
    }
    
    // 2. 丢线保护：无有效探头时保持上一次位置
    if (ucActiveCount == 0)
    {
        pstRunning->ucLineLostFlag = 1;
        return pstRunning->fLinePosition;
    }
    
    // 3. 正常在线，更新位置并清除丢线标志
    pstRunning->ucLineLostFlag = 0;
    pstRunning->fLinePosition = (float)lSumWeight / (float)ucActiveCount;
    
    return pstRunning->fLinePosition;
}

/// @brief      计算归一化线偏差
///
/// @param      emDevNum     ：传感器设备号
///
/// @return     归一化误差，范围 -1 ~ +1
float fTraceGetNormalizedError(emGrayscaleSensorDevNumTdf emDevNum)
{
    stTraceRunningParamTdf *pstRunning = &s_astTraceDeviceParam[emDevNum].stRunningParam;
    
    pstRunning->fCurrentError = pstRunning->fLinePosition / TRACE_MAX_POSITION_MM;
    
    // 限幅在 -1 ~ 1 之间
    if (pstRunning->fCurrentError > 1.0f) pstRunning->fCurrentError = 1.0f;
    if (pstRunning->fCurrentError < -1.0f) pstRunning->fCurrentError = -1.0f;
    
    return pstRunning->fCurrentError;
}

/// @brief      内部静态函数：PID限幅工具
///
/// @param      fValue ：待限幅值
/// @param      fMax   ：上限
/// @param      fMin   ：下限
///
/// @return     限幅后的值
static float fLimitValue(float fValue, float fMax, float fMin)
{
    if (fValue > fMax) return fMax;
    if (fValue < fMin) return fMin;
    return fValue;
}

/// @brief      循迹PID主计算函数
///
/// @param      emDevNum     ：传感器设备号
///
/// @note       内部完成误差计算、PID运算、底盘控制输出
void vTracePidRun(emGrayscaleSensorDevNumTdf emDevNum,float fVy)
{
    stTraceDeviceParamTdf *pstDev = &s_astTraceDeviceParam[emDevNum];
    stTracePidStaticParamTdf *pstStatic = &pstDev->stStaticParam;
    stTraceRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    
    float fPout, fIout, fDout;
    uint8_t ucDigitalVal;
    
    // 1. 获取传感器数字量
    ucDigitalVal = ucGrayscaleSensorGetDigital(emDevNum);
    
    // 2. 计算黑线位置与归一化误差
    fTraceGetLinePosition(ucDigitalVal, emDevNum);
    fTraceGetNormalizedError(emDevNum);
    
    // 3. 丢线处理：清零积分，保持转向输出，避免积分饱和
    if (pstRunning->ucLineLostFlag)
    {
        pstRunning->fIntegralSum = 0.0f;
        // 直接输出上一次角速度，保持转向趋势
        vChassisSetSpeed(fVy, pstRunning->fOutputOmega);
        return;
    }
    
    // 4. 标准位置式PID计算
    // 比例项
    fPout = pstStatic->fKp * pstRunning->fCurrentError;
    
    // 积分项：累加并限幅
    pstRunning->fIntegralSum += pstRunning->fCurrentError;
    pstRunning->fIntegralSum = fLimitValue(pstRunning->fIntegralSum, 
                                           pstStatic->fIntegralMax, 
                                           pstStatic->fIntegralMin);
    fIout = pstStatic->fKi * pstRunning->fIntegralSum;
    
    // 微分项：误差差分
    fDout = pstStatic->fKd * (pstRunning->fCurrentError - pstRunning->fLastError);
    
    // 5. 总输出并限幅
    pstRunning->fOutputOmega = fPout + fIout + fDout;
    pstRunning->fOutputOmega = fLimitValue(pstRunning->fOutputOmega, 
                                           pstStatic->fOutputMax, 
                                           pstStatic->fOutputMin);
    
    // 6. 更新历史误差
    pstRunning->fLastError = pstRunning->fCurrentError;
    
    // 7. 输出到底盘控制：基础前进速度 + PID转向角速度
    vChassisSetSpeed(fVy, pstRunning->fOutputOmega);
}