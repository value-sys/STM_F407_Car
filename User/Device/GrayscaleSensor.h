#ifndef __GRAYSCALESENSOR_H
#define __GRAYSCALESENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "project_config.h"

/// @brief          灰度传感器设备号枚举
///
/// @note 
typedef enum
{
    emGrayscaleSensorDevNum0        = 0,
    emGrayscaleSensorDevNum1,
}
emGrayscaleSensorDevNumTdf;

/// @brief          ADC分辨率枚举
///
/// @note           例程中14位ADC为0
typedef enum
{
    emGrayscaleSensorAdcBits_8      = 0,        // 8位ADC
    emGrayscaleSensorAdcBits_10,                // 10位ADC
    emGrayscaleSensorAdcBits_12,                // 12位ADC
    emGrayscaleSensorAdcBits_14,                // 14位ADC
}
emGrayscaleSensorAdcBitsTdf;

/// @brief          灰度传感器静态参数结构体
///
/// @note           硬件相关配置，初始化后不更改
typedef struct
{
    GPIO_TypeDef            *pstAddrGpioBase0;  // 地址线0 GPIO基址
    uint16_t                usAddrGpioPin0;     // 地址线0 GPIO引脚
    
    GPIO_TypeDef            *pstAddrGpioBase1;  // 地址线1 GPIO基址
    uint16_t                usAddrGpioPin1;     // 地址线1 GPIO引脚
    
    GPIO_TypeDef            *pstAddrGpioBase2;  // 地址线2 GPIO基址
    uint16_t                usAddrGpioPin2;     // 地址线2 GPIO引脚
    
    
    ADC_HandleTypeDef       *pstAdcHandle;      // ADC句柄指针
    uint32_t                ulAdcChannel;       // ADC通道号
    
    emGrayscaleSensorAdcBitsTdf emAdcBits;      // ADC分辨率
    
    uint8_t                 ucDirectionReverse; // 输出方向反转：0-正常，1-反转
    uint8_t                 ucChannelLogicInvert;// 通道地址逻辑取反：0-不取反，1-取反
    uint8_t                 ucSampleTimes;      // 单通道均值滤波采样次数
}
stGrayscaleSensorStaticParamTdf;

/// @brief          灰度传感器运行参数结构体
///
/// @note           运行时动态变化的状态与数据
typedef struct
{
    uint16_t                usCalibratedWhite[8];   // 白色校准基准值
    uint16_t                usCalibratedBlack[8];   // 黑色校准基准值
    
    uint16_t                usGrayWhiteThreshold[8];// 滞回比较白色阈值
    uint16_t                usGrayBlackThreshold[8];// 滞回比较黑色阈值
    
    double                  dNormalFactor[8];       // 归一化系数
    double                  dAdcFullScale;          // ADC满量程值
    
    uint16_t                usAnalogValue[8];       // 原始ADC采样值
    uint16_t                usNormalValue[8];       // 归一化后的值
    
    uint8_t                 ucDigitalOutput;        // 8位数字输出结果
    uint8_t                 ucTick;                 // 时基计数器
    uint8_t                 ucSampleTimeout;        // 采样周期（tick数）
    uint8_t                 ucReadyFlag;            // 就绪标志：0-未校准，1-就绪
}
stGrayscaleSensorRunningParamTdf;

/// @brief          灰度传感器完整参数结构体
///
/// @note           
typedef struct
{
    stGrayscaleSensorStaticParamTdf     stStaticParam;  // 静态参数
    stGrayscaleSensorRunningParamTdf    stRunningParam; // 运行参数
}
stGrayscaleSensorDeviceParamTdf;

/// @brief      获取灰度传感器设备参数
///
/// @param      emDevNum   ：设备号
///
/// @note       返回值是常量指针，指向的内容不可更改
const stGrayscaleSensorDeviceParamTdf *c_pstGetGrayscaleSensorDeviceParam(emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      灰度传感器设备初始化
///
/// @param      pstInit    ：静态参数初始化指针
/// @param      emDevNum   ：设备号
///
/// @note  
void vGrayscaleSensorDeviceInit(stGrayscaleSensorStaticParamTdf *pstInit, emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      设置灰度传感器校准参数
///
/// @param      emDevNum           ：设备号
/// @param      pusCalibratedWhite ：白色校准值数组指针
/// @param      pusCalibratedBlack ：黑色校准值数组指针
///
/// @note       自动计算滞回阈值与归一化系数
void vGrayscaleSensorSetCalibration(emGrayscaleSensorDevNumTdf emDevNum, 
                                    const uint16_t *pusCalibratedWhite, 
                                    const uint16_t *pusCalibratedBlack);

/// @brief      灰度传感器主任务（无时基版本）
///
/// @param      emDevNum   ：设备号
///
/// @note       直接执行采集与处理，无定时控制
void vGrayscaleSensorTask(emGrayscaleSensorDevNumTdf emDevNum);

#ifdef GRAYSCALE_SENSOR_USE_TIMER
/// @brief      灰度传感器主任务（有时基版本）
///
/// @param      emDevNum   ：设备号
///
/// @note       配合时基计数实现定时采样
void vGrayscaleSensorTaskWithTick(emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      灰度传感器时基递增
///
/// @param      emDevNum   ：设备号
///
/// @note       需在1ms定时器中断中周期调用
void vGrayscaleSensorTickInc(emGrayscaleSensorDevNumTdf emDevNum);
#endif

/// @brief      获取灰度传感器数字量输出
///
/// @param      emDevNum   ：设备号
///
/// @note       返回8位数字量，每位对应一路传感器
uint8_t ucGrayscaleSensorGetDigital(emGrayscaleSensorDevNumTdf emDevNum);

/// @brief      获取灰度传感器原始模拟值
///
/// @param      emDevNum   ：设备号
/// @param      pusResult  ：结果存储数组指针
///
/// @retval     0 ：未就绪
/// @retval     1 ：获取成功
uint8_t ucGrayscaleSensorGetAnalogValue(emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult);

/// @brief      获取灰度传感器归一化值
///
/// @param      emDevNum   ：设备号
/// @param      pusResult  ：结果存储数组指针
///
/// @retval     0 ：未就绪
/// @retval     1 ：获取成功
uint8_t ucGrayscaleSensorGetNormalValue(emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult);

#endif /* __GRAYSCALE_SENSOR_H */