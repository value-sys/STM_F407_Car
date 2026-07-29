
/**
  * @file       imu.c
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32 HAL IMU驱动模块
  * @note       保留当前工程USART3单字节中断接收方式，只补充有效帧计数供任务使用。
  */

#include "imu.h"
#include <stdio.h>
/* 设备参数数组：文件内私有，外部仅通过接口访问 */
static stImuDeviceParamTdf astImuDeviceParam[IMU_DEV_NUM];

/* ============================== 私有协议指令定义 ============================== */
/* 读帧标识 */
#define IMU_RX_HEADER             0x5A    // 读数据帧头
#define IMU_TYPE_GYRO             0xAA    // 角速度数据类型
#define IMU_TYPE_YAW              0xBB    // 角度数据类型

/* 写指令预定义数组：5字节固定格式 */
static const uint8_t aucImuCmdUnlock[5]    = {0x55, 0xAA, 0x13, 0x8E, 0x5F};  // 解锁指令
static const uint8_t aucImuCmdSave[5]      = {0x55, 0xAA, 0x00, 0x00, 0x00};  // 保存配置指令
static const uint8_t aucImuCmdYawZero[5]   = {0x55, 0xAA, 0x15, 0x00, 0x00};  // Z轴角度归零指令
static const uint8_t aucImuCmdBiasCal[5]   = {0x55, 0xAA, 0x0A, 0x01, 0x00};  // 自动零偏校准指令



// /// @brief      内部通用：发送5字节写指令
// /// @param      emDevNum   ：设备号
// /// @param      ucRegAddr  ：寄存器地址
// /// @param      usValue    ：寄存器写入值
// static void vImuSendWriteCmd(emImuDevNumTdf emDevNum, uint8_t ucRegAddr, uint16_t usValue)
// {
//     uint8_t aucTxBuf[5];
    
//     // 1. 组装写指令帧：0x55 + 0xAA + 地址 + 低字节 + 高字节
//     aucTxBuf[0] = aucCmdHeader[0];
//     aucTxBuf[1] = aucCmdHeader[1];
//     aucTxBuf[2] = ucRegAddr;
//     aucTxBuf[3] = (uint8_t)(usValue & 0xFF);
//     aucTxBuf[4] = (uint8_t)((usValue >> 8) & 0xFF);
    
//     // 2. 串口阻塞发送
//     HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
//                       aucTxBuf, 5, 100);
// }

/// @brief      获取IMU设备参数（只读）
/// @param      emDevNum   ：设备号
/// @return     设备参数常量指针，外部不可修改
const stImuDeviceParamTdf *c_pstGetImuDeviceParam(emImuDevNumTdf emDevNum)
{
    // 入参保护：越界返回第一个设备
    if (emDevNum >= IMU_DEV_NUM)
    {
        emDevNum = emImuDevNum0;
    }
    return &astImuDeviceParam[emDevNum];
}

/// @brief      IMU设备初始化
/// @param      pstInit    ：静态硬件参数指针
/// @param      emDevNum   ：设备号
void vImuDeviceInit(const stImuStaticParamTdf *pstInit,
    emImuDevNumTdf emDevNum)
{
    // 1. 入参保护
    if (pstInit == NULL || emDevNum >= IMU_DEV_NUM)
    {
        return;
    }
    
    // 2. 拷贝静态硬件参数
    memcpy(&astImuDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stImuStaticParamTdf));
    
    // 3. 清空物理量、原始数据、帧计数和协议解析状态
    memset(&astImuDeviceParam[emDevNum].stRunningParam, 0,
        sizeof(stImuRunningParamTdf));
    astImuDeviceParam[emDevNum].stRunningParam.emStatus = emImuStatus_Idle;
    
    //HAL_NVIC_ClearPendingIRQ(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle->Instance->IRQn);    
    // 4. 开启串口接收中断（单字节接收模式）
    HAL_UART_Receive_IT(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                        &astImuDeviceParam[emDevNum].stRunningParam.ucRxByte, 1);
}


/// @brief      串口单字节数据解析
/// @param      emDevNum   ：设备号
/// @param      ucData     ：接收到的单字节数据
/// @note       每收到1个字节调用1次，内部状态机自动解析完整帧
void vImuParseSerialByte(emImuDevNumTdf emDevNum, uint8_t ucData)
{
    if (emDevNum >= emImuDevNumMax)
    {
        return;
    }
    stImuRunningParamTdf *pstRun = &astImuDeviceParam[emDevNum].stRunningParam;

    /* 异常状态保护，防止接收计数损坏后越界写入帧缓存。 */
    if (pstRun->ucRxCnt >= IMU_FRAME_LEN)
    {
        pstRun->ucRxCnt = 0U;
    }
    
    // 1. 存入接收缓存，计数自增
    pstRun->aucRxBuffer[pstRun->ucRxCnt++] = ucData;
    
    // 2. 帧头校验：首字节不是0x5A直接重置，重新同步
    if (pstRun->aucRxBuffer[0] != IMU_RX_HEADER)
    {
        pstRun->ucRxCnt = 0;
        return;
    }
    
    // 3. 未收满一帧，继续等待
    if (pstRun->ucRxCnt < IMU_FRAME_LEN)
    {
        return;
    }

    // 4. 收满5字节，计算校验和（取低8位）
    uint8_t ucSum = pstRun->aucRxBuffer[0] + pstRun->aucRxBuffer[1]
                  + pstRun->aucRxBuffer[2] + pstRun->aucRxBuffer[3];
    
    if (ucSum != pstRun->aucRxBuffer[4])
    {
        // 校验失败，重置接收
        pstRun->ucRxCnt = 0;
        return;
    }
    
    // 5. 按数据类型解析
    switch (pstRun->aucRxBuffer[1])
    {
        case IMU_TYPE_GYRO:
        {
            // 5.1 合成有符号16位原始值（高字节在前，低字节在后）
            int16_t sRaw = (int16_t)((pstRun->aucRxBuffer[3] << 8) | pstRun->aucRxBuffer[2]);
            pstRun->sRawGyroZ = sRaw;
            // 5.2 转换为物理量：原始值 / 32768 * 2000°/s
            pstRun->fGyroZ = (float)sRaw / IMU_RAW_MAX * IMU_GYRO_SCALE;
            /* 最后递增计数，保证任务看到新计数时对应物理量已经更新。 */
            pstRun->ulGyroSampleCount++;
            break;
        }
        
        case IMU_TYPE_YAW:
        {
            // 5.1 合成有符号16位原始值
            int16_t sRaw = (int16_t)((pstRun->aucRxBuffer[3] << 8) | pstRun->aucRxBuffer[2]);
            pstRun->sRawYaw = sRaw;
            // 5.2 转换为物理量：原始值 / 32768 * 180°
            pstRun->fYawAngle = (float)sRaw / IMU_RAW_MAX * IMU_YAW_SCALE;
            break;
        }
        
        default:
            break;
    }
    
    // 6. 解析完成，重置计数器准备下一帧
    pstRun->ucRxCnt = 0;
}

/// @brief      获取当前Z轴航向角
/// @param      emDevNum   ：设备号
/// @return     航向角，单位°，范围±180°
float fImuGetYaw(emImuDevNumTdf emDevNum)
{
    if (emDevNum >= IMU_DEV_NUM)
    {
        return 0.0f;
    }
    return astImuDeviceParam[emDevNum].stRunningParam.fYawAngle;
}

/// @brief      获取当前Z轴角速度
/// @param      emDevNum   ：设备号
/// @return     角速度，单位°/s
float fImuGetGyroZ(emImuDevNumTdf emDevNum)
{
    if (emDevNum >= IMU_DEV_NUM)
    {
        return 0.0f;
    }
    return astImuDeviceParam[emDevNum].stRunningParam.fGyroZ;
}

uint32_t ulImuGetGyroSampleCount(emImuDevNumTdf emDevNum)
{
    if (emDevNum >= IMU_DEV_NUM)
    {
        return 0U;
    }
    return astImuDeviceParam[emDevNum].stRunningParam.ulGyroSampleCount;
}

/// @brief      发送Z轴角度归零指令
/// @param      emDevNum   ：设备号
/// @note       执行后当前航向角被设为0°基准，指令流程与例程完全一致
void vImuSendYawZeroCmd(emImuDevNumTdf emDevNum)
{
    if (emDevNum >= IMU_DEV_NUM)
    {
        return;
    }
    
    // 1. 发送解锁指令
    HAL_StatusTypeDef res;
    res = HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                      (uint8_t *)aucImuCmdUnlock, 5, 100);
    if (res != HAL_OK)    {
        printf("IMU %d Unlock Failed\r\n", emDevNum);
    }
    HAL_Delay(100);
    
    // 2. 发送角度归零指令
    res = HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                      (uint8_t *)aucImuCmdYawZero, 5, 100);
    if (res != HAL_OK)    {
        printf("IMU %d Yaw Zero Failed\r\n", emDevNum);
    }
    HAL_Delay(100);
    
    // 3. 保存配置
    res = HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                      (uint8_t *)aucImuCmdSave, 5, 100);
    if (res != HAL_OK)    {
        printf("IMU %d Save Failed\r\n", emDevNum);
    }
}


/// @brief      启动自动零偏校准
/// @param      emDevNum   ：设备号
/// @note       校准过程中模组必须保持静止，全程约21秒，阻塞式执行，流程与例程完全一致
void vImuStartBiasCalibration(emImuDevNumTdf emDevNum)
{
    if (emDevNum >= IMU_DEV_NUM)
    {
        return;
    }
    
    astImuDeviceParam[emDevNum].stRunningParam.emStatus = emImuStatus_Calibrating;
    
    // 1. 发送解锁指令
    HAL_StatusTypeDef res;
    res = HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                      (uint8_t *)aucImuCmdUnlock, 5, 100);
    if (res != HAL_OK)    {
        printf("IMU %d Unlock Failed\r\n", emDevNum);
    }
    HAL_Delay(100);
    
    // 2. 发送零偏校准指令
    res = HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                      (uint8_t *)aucImuCmdBiasCal, 5, 100);
    if (res != HAL_OK)    {
        printf("IMU %d Bias Calibration Failed\r\n", emDevNum);
    }

    // 3. 等待校准完成（手册要求至少20s，预留1s余量）
    HAL_Delay(21000);
    
    // 4. 保存校准结果
    res = HAL_UART_Transmit(astImuDeviceParam[emDevNum].stStaticParam.pstUartHandle,
                      (uint8_t *)aucImuCmdSave, 5, 100);
    if (res != HAL_OK)    {
        printf("IMU %d Save Failed\r\n", emDevNum);
    }
    astImuDeviceParam[emDevNum].stRunningParam.emStatus = emImuStatus_Idle;
}

/// @brief      串口接收完成回调函数
/// @note       USART3_IRQHandler调用HAL_UART_IRQHandler后由HAL进入本回调。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 匹配IMU通信串口
    if (huart == astImuDeviceParam[emImuDevNum0].stStaticParam.pstUartHandle)
    {
        uint8_t ucData = astImuDeviceParam[emImuDevNum0].stRunningParam.ucRxByte;
        //printf("IMU Raw Byte: 0x%02X\r\n", ucData);
        // 送入解析
        vImuParseSerialByte(emImuDevNum0, ucData);
        // 重新开启下一字节接收中断
        HAL_UART_Receive_IT(huart,
                            &astImuDeviceParam[emImuDevNum0].stRunningParam.ucRxByte, 1);
    }
}
