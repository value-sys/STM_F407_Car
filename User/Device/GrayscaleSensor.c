/// @file       GrayscaleSensor.c
/// @version    V1.0.0
/// @date       20260626
/// @brief      感为无MCU八路灰度传感器驱动模块

#include "GrayscaleSensor.h"
#include <string.h>

stGrayscaleSensorDeviceParamTdf astGrayscaleSensorDeviceParam[GRAYSCALE_SENSOR_DEV_NUM];

/// @brief      内部静态函数：切换传感器通道
///
/// @param      emDevNum   ：设备号
/// @param      ucChannel  ：通道号（0-7）
///
/// @note       内部调用，不对外暴露
static void vSwitchChannel(emGrayscaleSensorDevNumTdf emDevNum, uint8_t ucChannel)
{
    stGrayscaleSensorStaticParamTdf *pstStatic = &astGrayscaleSensorDeviceParam[emDevNum].stStaticParam;
    GPIO_PinState emPinState0, emPinState1, emPinState2;
    
    // 1. 提取通道对应位
    emPinState0 = (ucChannel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    emPinState1 = (ucChannel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    emPinState2 = (ucChannel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    
    // 2. 处理逻辑取反（兼容原例程逻辑）
    if (pstStatic->ucChannelLogicInvert)
    {
        emPinState0 = (emPinState0 == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        emPinState1 = (emPinState1 == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        emPinState2 = (emPinState2 == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    
    // 3. 设置地址线电平
    HAL_GPIO_WritePin(pstStatic->pstAddrGpioBase0, pstStatic->usAddrGpioPin0, emPinState0);
    HAL_GPIO_WritePin(pstStatic->pstAddrGpioBase1, pstStatic->usAddrGpioPin1, emPinState1);
    HAL_GPIO_WritePin(pstStatic->pstAddrGpioBase2, pstStatic->usAddrGpioPin2, emPinState2);
}

/// @brief      内部静态函数：读取一次ADC值
///
/// @param      emDevNum   ：设备号
///
/// @return     ADC采样值
///
/// @note       内部调用，不对外暴露
static uint16_t usReadAdcValue(emGrayscaleSensorDevNumTdf emDevNum)
{
    ADC_HandleTypeDef *pstAdc = astGrayscaleSensorDeviceParam[emDevNum].stStaticParam.pstAdcHandle;
    
    HAL_ADC_Start(pstAdc);
    HAL_ADC_PollForConversion(pstAdc, 1);
    
    return HAL_ADC_GetValue(pstAdc);
}

/// @brief      获取灰度传感器设备参数
///
/// @param      emDevNum   ：设备号
///
/// @note       返回值是 stGrayscaleSensorDeviceParamTdf 型的指针，且指针指向的内容是不可更改的（只读的）
const stGrayscaleSensorDeviceParamTdf *c_pstGetGrayscaleSensorDeviceParam(emGrayscaleSensorDevNumTdf emDevNum)
{
    return &astGrayscaleSensorDeviceParam[emDevNum];
}

/// @brief      灰度传感器设备初始化
///
/// @param      pstInit    ：静态参数初始化指针
/// @param      emDevNum   ：设备号
///
/// @note   
void vGrayscaleSensorDeviceInit(stGrayscaleSensorStaticParamTdf *pstInit, emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorDeviceParamTdf *pstDev = &astGrayscaleSensorDeviceParam[emDevNum];
    stGrayscaleSensorRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    
    // 1. 拷贝静态参数
    memcpy(&pstDev->stStaticParam, pstInit, sizeof(stGrayscaleSensorStaticParamTdf));
    
    // 2. 运行参数清零初始化
    memset(pstRunning->usCalibratedWhite, 0, sizeof(pstRunning->usCalibratedWhite));
    memset(pstRunning->usCalibratedBlack, 0, sizeof(pstRunning->usCalibratedBlack));
    memset(pstRunning->usGrayWhiteThreshold, 0, sizeof(pstRunning->usGrayWhiteThreshold));
    memset(pstRunning->usGrayBlackThreshold, 0, sizeof(pstRunning->usGrayBlackThreshold));
    memset(pstRunning->usAnalogValue, 0, sizeof(pstRunning->usAnalogValue));
    memset(pstRunning->usNormalValue, 0, sizeof(pstRunning->usNormalValue));
    
    // 3. 归一化系数初始化
    for (uint8_t i = 0; i < 8; i++)
    {
        pstRunning->dNormalFactor[i] = 0.0;
    }
    
    // 4. 根据ADC分辨率设置满量程值
    switch (pstInit->emAdcBits)
    {
        case emGrayscaleSensorAdcBits_8:
            pstRunning->dAdcFullScale = 255.0;
            break;
        case emGrayscaleSensorAdcBits_10:
            pstRunning->dAdcFullScale = 1024.0;
            break;
        case emGrayscaleSensorAdcBits_12:
            pstRunning->dAdcFullScale = 4096.0;
            break;
        case emGrayscaleSensorAdcBits_14:
            pstRunning->dAdcFullScale = 16384.0;
            break;
        default:
            pstRunning->dAdcFullScale = 4096.0;
            break;
    }
    
    // 5. 根据传感器版本设置采样超时周期

    pstRunning->ucSampleTimeout = GRAYSCALE_SAMPLE_TIMEOUT;

    // 6. 状态变量初始化
    pstRunning->ucDigitalOutput = 0;
    pstRunning->ucTick = 0;
    pstRunning->ucReadyFlag = 0;
    
    // 7. 使能传感器（低电平使能，对应手册EN引脚规格）
    // HAL_GPIO_WritePin(pstInit->pstEnGpioBase, pstInit->usEnGpioPin, GPIO_PIN_RESET);//默认已经使能
}

/// @brief      设置灰度传感器校准参数
///
/// @param      emDevNum           ：设备号
/// @param      pusCalibratedWhite ：白色校准值数组指针
/// @param      pusCalibratedBlack ：黑色校准值数组指针
///
/// @note       自动计算滞回阈值与归一化系数
void vGrayscaleSensorSetCalibration(emGrayscaleSensorDevNumTdf emDevNum, 
                                    const uint16_t *pusCalibratedWhite, 
                                    const uint16_t *pusCalibratedBlack)
{
    stGrayscaleSensorDeviceParamTdf *pstDev = &astGrayscaleSensorDeviceParam[emDevNum];
    stGrayscaleSensorRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    uint16_t usWhite, usBlack, usTemp;
    
    for (uint8_t i = 0; i < 8; i++)
    {
        usWhite = pusCalibratedWhite[i];
        usBlack = pusCalibratedBlack[i];
        
        // 1. 确保白值大于黑值，异常时自动交换
        if (usBlack >= usWhite)
        {
            usTemp = usWhite;
            usWhite = usBlack;
            usBlack = usTemp;
        }
        
        // 2. 计算滞回比较阈值（与手册公式完全一致：1/3与2/3分界点）
        pstRunning->usGrayWhiteThreshold[i] = (usWhite * 2 + usBlack) / 3;
        pstRunning->usGrayBlackThreshold[i] = (usWhite + usBlack * 2) / 3;
        
        // 3. 保存校准基准值
        pstRunning->usCalibratedWhite[i] = usWhite;
        pstRunning->usCalibratedBlack[i] = usBlack;
        
        // 4. 处理无效校准数据，计算归一化系数
        if ((usWhite == 0 && usBlack == 0) || (usWhite == usBlack))
        {
            pstRunning->dNormalFactor[i] = 0.0;
            continue;
        }
        pstRunning->dNormalFactor[i] = pstRunning->dAdcFullScale / (double)(usWhite - usBlack);
    }
    
    // 5. 标记传感器就绪
    pstRunning->ucReadyFlag = 1;
}

/// @brief      内部静态函数：采集8通道模拟值并均值滤波
///
/// @param      emDevNum   ：设备号
///
/// @note       内部调用，更新运行参数中的原始ADC值
static void vCollectAnalogValue(emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorDeviceParamTdf *pstDev = &astGrayscaleSensorDeviceParam[emDevNum];
    stGrayscaleSensorStaticParamTdf *pstStatic = &pstDev->stStaticParam;
    stGrayscaleSensorRunningParamTdf *pstRunning = &pstDev->stRunningParam;
    uint32_t ulSum = 0;
    uint8_t ucResultIndex;
    
    // 1. 遍历8个通道
    for (uint8_t i = 0; i < 8; i++)
    {
        // 切换当前通道
        vSwitchChannel(emDevNum, i);
        ulSum = 0;
        
        // 多次采样累加
        for (uint8_t j = 0; j < pstStatic->ucSampleTimes; j++)
        {
            ulSum += usReadAdcValue(emDevNum);
        }
        
        // 计算平均值，处理方向反转
        ucResultIndex = pstStatic->ucDirectionReverse ? (7 - i) : i;
        pstRunning->usAnalogValue[ucResultIndex] = (uint16_t)(ulSum / pstStatic->ucSampleTimes);
    }
}

/// @brief      内部静态函数：模拟量转数字量（滞回比较）
///
/// @param      emDevNum   ：设备号
///
/// @note       内部调用，更新运行参数中的数字输出
static void vConvertAnalogToDigital(emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorRunningParamTdf *pstRunning = &astGrayscaleSensorDeviceParam[emDevNum].stRunningParam;
    
    for (uint8_t i = 0; i < 8; i++)
    {
        if (pstRunning->usAnalogValue[i] > pstRunning->usGrayWhiteThreshold[i])
        {
            // 高于白阈值，置1（白色）
            pstRunning->ucDigitalOutput |= (1 << i);
        }
        else if (pstRunning->usAnalogValue[i] < pstRunning->usGrayBlackThreshold[i])
        {
            // 低于黑阈值，置0（黑色）
            pstRunning->ucDigitalOutput &= ~(1 << i);
        }
        // 中间区间保持原有状态，实现滞回特性
    }
}

/// @brief      内部静态函数：ADC值归一化处理
///
/// @param      emDevNum   ：设备号
///
/// @note       内部调用，更新运行参数中的归一化值
static void vNormalizeAnalogValue(emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorRunningParamTdf *pstRunning = &astGrayscaleSensorDeviceParam[emDevNum].stRunningParam;
    uint16_t usNormalized;
    
    for (uint8_t i = 0; i < 8; i++)
    {
        // 计算归一化值（减去黑电平后缩放）
        if (pstRunning->usAnalogValue[i] < pstRunning->usCalibratedBlack[i])
        {
            usNormalized = 0;// 低于黑电平整定为0
        }
        else
        {
            usNormalized = (uint16_t)((pstRunning->usAnalogValue[i] - pstRunning->usCalibratedBlack[i]) 
                                      * pstRunning->dNormalFactor[i]);
        }
        
        // 满幅限幅
        if (usNormalized > (uint16_t)pstRunning->dAdcFullScale)
        {
            usNormalized = (uint16_t)pstRunning->dAdcFullScale;
        }
        
        pstRunning->usNormalValue[i] = usNormalized;
    }
}

/// @brief      灰度传感器主任务（无时基版本）
///
/// @param      emDevNum   ：设备号
///
/// @note       直接执行采集与处理，无定时控制
void vGrayscaleSensorTask(emGrayscaleSensorDevNumTdf emDevNum)
{
    // 1. 采集原始模拟值
    vCollectAnalogValue(emDevNum);
    // 2. 滞回比较转数字量
    vConvertAnalogToDigital(emDevNum);
    // 3. 归一化处理
    vNormalizeAnalogValue(emDevNum);
}

#ifdef GRAYSCALE_SENSOR_USE_TIMER
/// @brief      灰度传感器主任务（有时基版本）
///
/// @param      emDevNum   ：设备号
///
/// @note       配合时基计数实现定时采样
void vGrayscaleSensorTaskWithTick(emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorRunningParamTdf *pstRunning = &astGrayscaleSensorDeviceParam[emDevNum].stRunningParam;
    
    if (pstRunning->ucTick >= pstRunning->ucSampleTimeout)
    {
        // 到达采样周期，执行采集与处理
        vCollectAnalogValue(emDevNum);
        vConvertAnalogToDigital(emDevNum);
        vNormalizeAnalogValue(emDevNum);
        
        // 重置计数器
        pstRunning->ucTick = 0;
    }
}

/// @brief      灰度传感器时基递增
///
/// @param      emDevNum   ：设备号
///
/// @note       需在1ms定时器中断中周期调用
void vGrayscaleSensorTickInc(emGrayscaleSensorDevNumTdf emDevNum)
{
    astGrayscaleSensorDeviceParam[emDevNum].stRunningParam.ucTick++;
}
#endif

/// @brief      获取灰度传感器数字量输出
///
/// @param      emDevNum   ：设备号
///
/// @note       返回8位数字量，每位对应一路传感器
uint8_t ucGrayscaleSensorGetDigital(emGrayscaleSensorDevNumTdf emDevNum)
{
    return astGrayscaleSensorDeviceParam[emDevNum].stRunningParam.ucDigitalOutput;
}

/// @brief      获取灰度传感器原始模拟值
///
/// @param      emDevNum   ：设备号
/// @param      pusResult  ：结果存储数组指针
///
/// @retval     0 ：未就绪
/// @retval     1 ：获取成功
uint8_t ucGrayscaleSensorGetAnalogValue(emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult)
{
    stGrayscaleSensorRunningParamTdf *pstRunning = &astGrayscaleSensorDeviceParam[emDevNum].stRunningParam;
    // 重新采集一次最新数据
    vCollectAnalogValue(emDevNum);
    memcpy(pusResult, pstRunning->usAnalogValue, sizeof(pstRunning->usAnalogValue));    

    // 未就绪返回失败
    if (!pstRunning->ucReadyFlag)
    {
        return 0;
    }
    return 1;
}

/// @brief      获取灰度传感器归一化值
///
/// @param      emDevNum   ：设备号
/// @param      pusResult  ：结果存储数组指针
///
/// @retval     0 ：未就绪
/// @retval     1 ：获取成功
uint8_t ucGrayscaleSensorGetNormalValue(emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult)
{
    stGrayscaleSensorRunningParamTdf *pstRunning = &astGrayscaleSensorDeviceParam[emDevNum].stRunningParam;
    
    // 未就绪返回失败
    if (!pstRunning->ucReadyFlag)
    {
        return 0;
    }
    
    memcpy(pusResult, pstRunning->usNormalValue, sizeof(pstRunning->usNormalValue));
    
    return 1;
}