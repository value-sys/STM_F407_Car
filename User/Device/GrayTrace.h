#include "GrayscaleSensor.h"
#include "project_config.h"
#include "chassis.h"

uint8_t Gray_ReadRaw(emGrayscaleSensorDevNumTdf emDevNum);

float Gray_GetLinePosition(uint8_t raw);

float Gray_GetLineError(uint8_t raw);

void Trace_Run(uint8_t raw,float fVy);

/// @brief          循迹PID静态参数结构体
///
/// @note           PID配置参数，初始化后一般不修改
typedef struct
{
    float       fKp;                // 比例系数
    float       fKi;                // 积分系数
    float       fKd;                // 微分系数
    
    float       fOutputMax;         // 输出角速度上限
    float       fOutputMin;         // 输出角速度下限
    float       fIntegralMax;       // 积分项上限
    float       fIntegralMin;       // 积分项下限
    
    float       fBaseSpeed;         // 循迹基础前进速度 (mm/s)
}
stTracePidStaticParamTdf;

/// @brief          循迹运行状态结构体
///
/// @note           运行时动态更新的状态量
typedef struct
{
    float       fIntegralSum;       // 积分累加值
    float       fLastError;         // 上一次误差值
    float       fCurrentError;      // 当前误差值
    float       fOutputOmega;       // 输出角速度
    
    float       fLinePosition;      // 当前黑线位置 (mm)
    uint8_t     ucLineLostFlag;     // 丢线标志：0-在线上，1-丢线
}
stTraceRunningParamTdf;

/// @brief          循迹完整参数结构体
///
/// @note           
typedef struct
{
    stTracePidStaticParamTdf    stStaticParam;  // PID静态参数
    stTraceRunningParamTdf      stRunningParam; // 运行状态
}
stTraceDeviceParamTdf;

/// @brief      循迹PID参数初始化
///
/// @param      pstInitParam ：PID静态参数指针
/// @param      emDevNum     ：传感器设备号
///
/// @note   
void vTracePidInit(const stTracePidStaticParamTdf *pstInitParam, emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      计算黑线物理位置
///
/// @param      ucDigitalVal ：8路数字量结果
/// @param      emDevNum     ：传感器设备号
///
/// @return     黑线位置 (mm)，范围 -35 ~ +35
///
/// @note       丢线时返回上一次有效位置
float fTraceGetLinePosition(uint8_t ucDigitalVal, emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      计算归一化线偏差
///
/// @param      emDevNum     ：传感器设备号
///
/// @return     归一化误差，范围 -1 ~ +1
float fTraceGetNormalizedError(emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      循迹PID主计算函数
///
/// @param      emDevNum     ：传感器设备号
/// @param      fVy          : 循迹速度
///
/// @note       内部完成误差计算、PID运算、底盘控制输出
void vTracePidRun(emGrayscaleSensorDevNumTdf emDevNum,float fVy);