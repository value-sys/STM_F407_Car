/**
  * @file       imu.h
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32 HAL IMU驱动模块头文件
  */

#ifndef _IMU_H_
#define _IMU_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "project_config.h"
#include <string.h>
/// @brief          IMU设备号枚举
typedef enum
{
    emImuDevNum0 = 0,
    emImuDevNumMax
}
emImuDevNumTdf;

/// @brief          IMU运行状态枚举
typedef enum
{
    emImuStatus_Idle       = 0,        // 空闲
    emImuStatus_Calibrating,           // 校准中
}
emImuStatusTdf;

/// @brief          IMU静态硬件参数结构体
/// @note           与硬件绑定的配置，初始化时传入
typedef struct
{
    UART_HandleTypeDef      *pstUartHandle;    // 通信串口句柄指针
}
stImuStaticParamTdf;

/// @brief          IMU运行参数结构体
/// @note           实时运行数据与解析状态
typedef struct
{
    volatile float          fYawAngle;         // 当前航向角(°)，范围±180°
    volatile float          fGyroZ;            // 当前Z轴角速度(°/s)
    volatile uint32_t       ulGyroSampleCount; // 有效角速度帧累计数
    
    int16_t                 sRawYaw;           // 角度原始值
    int16_t                 sRawGyroZ;         // 角速度原始值
    
    uint8_t                 ucRxByte;          // 当前接收字节
    uint8_t                 aucRxBuffer[IMU_FRAME_LEN];  // 串口接收缓存
    uint8_t                 ucRxCnt;           // 接收字节计数
    emImuStatusTdf          emStatus;          // 设备运行状态
}
stImuRunningParamTdf;

/// @brief          IMU完整设备参数结构体
typedef struct
{
    stImuStaticParamTdf     stStaticParam;     // 静态硬件参数
    stImuRunningParamTdf    stRunningParam;    // 运行状态与数据
}
stImuDeviceParamTdf;

/// @brief      获取IMU设备参数（只读）
/// @param      emDevNum   ：设备号
/// @return     设备参数常量指针，外部不可修改
const stImuDeviceParamTdf *c_pstGetImuDeviceParam(emImuDevNumTdf emDevNum);

/// @brief      IMU设备初始化
/// @param      pstInit    ：静态硬件参数指针
/// @param      emDevNum   ：设备号
/// @note       需在串口初始化完成后调用
void vImuDeviceInit(const stImuStaticParamTdf *pstInit,
    emImuDevNumTdf emDevNum);

/// @brief      串口单字节数据解析（串口接收中断中调用）
/// @param      emDevNum   ：设备号
/// @param      ucData     ：接收到的单字节数据
/// @note       每收到1个字节调用1次，内部状态机自动解析完整帧
void vImuParseSerialByte(emImuDevNumTdf emDevNum, uint8_t ucData);

/// @brief      获取当前Z轴航向角
/// @param      emDevNum   ：设备号
/// @return     航向角，单位°，范围±180°
float fImuGetYaw(emImuDevNumTdf emDevNum);

/// @brief      获取当前Z轴角速度
/// @param      emDevNum   ：设备号
/// @return     角速度，单位°/s
float fImuGetGyroZ(emImuDevNumTdf emDevNum);

/// @brief      获取已解析的有效Z轴角速度帧数量
/// @note       周期任务通过该计数判断是否收到新数据，避免重复计算角加速度。
uint32_t ulImuGetGyroSampleCount(emImuDevNumTdf emDevNum);

/// @brief      发送Z轴角度归零指令
/// @param      emDevNum   ：设备号
/// @note       执行后当前航向角被设为0°基准，指令自动保存
void vImuSendYawZeroCmd(emImuDevNumTdf emDevNum);

/// @brief      启动自动零偏校准
/// @param      emDevNum   ：设备号
/// @note       校准过程中模组必须保持静止，全程约21秒，阻塞式执行
void vImuStartBiasCalibration(emImuDevNumTdf emDevNum);

#endif
