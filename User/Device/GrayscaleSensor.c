/**
  * @file       GrayscaleSensor.c
  * @version    V1.1.0
  * @date       20260729
  * @brief      八路数字灰度传感器串行驱动
  */

#include "GrayscaleSensor.h"
#include <stddef.h>
#include <string.h>

static stGrayscaleSensorDeviceParamTdf
    s_astGrayscaleSensorDeviceParam[GRAYSCALE_SENSOR_DEV_NUM];

volatile uint8_t g_ucGrayscaleD1;
volatile uint8_t g_ucGrayscaleD2;
volatile uint8_t g_ucGrayscaleD3;
volatile uint8_t g_ucGrayscaleD4;
volatile uint8_t g_ucGrayscaleD5;
volatile uint8_t g_ucGrayscaleD6;
volatile uint8_t g_ucGrayscaleD7;
volatile uint8_t g_ucGrayscaleD8;

/// @brief 产生参考驱动所需的微秒级GPIO建立和保持时间
/// @note  当前STM32F407运行于168MHz，14次空循环沿用参考工程实测参数。
static void vGrayscaleDelayUs(uint32_t ulDelayUs)
{
    ulDelayUs *= 14U;
    while (ulDelayUs > 0U)
    {
        __NOP();
        ulDelayUs--;
    }
}

/// @brief 将一个字节的位顺序反转，用于传感器安装方向修正
static uint8_t ucGrayscaleReverseBits(uint8_t ucValue)
{
    ucValue = (uint8_t)(((ucValue & 0x55U) << 1U) |
        ((ucValue & 0xAAU) >> 1U));
    ucValue = (uint8_t)(((ucValue & 0x33U) << 2U) |
        ((ucValue & 0xCCU) >> 2U));
    return (uint8_t)((ucValue << 4U) | (ucValue >> 4U));
}

/* Map the observed serial bit order to physical channels 1 through 8. */
static uint8_t ucGrayscaleMapChannelOrder(uint8_t ucRaw)
{
    uint8_t ucMapped = (uint8_t)(ucRaw & 0x01U);
    uint8_t ucChannel;

    for (ucChannel = 1U; ucChannel < 8U; ucChannel++)
    {
        ucMapped |= (uint8_t)(((ucRaw >> (8U - ucChannel)) & 0x01U) <<
            ucChannel);
    }
    return ucMapped;
}

/// @brief 按CLK/DAT协议串行读取D1~D8数字状态
/// @note  读取时序与参考Gray_ReadRaw函数保持一致，每个边沿间隔5us。
static uint8_t ucGrayscaleReadRaw(
    const stGrayscaleSensorStaticParamTdf *pstStatic)
{
    uint8_t ucRaw = 0U;
    uint8_t ucIndex;

    HAL_GPIO_WritePin(pstStatic->pstClkGpioBase,
        pstStatic->usClkGpioPin, GPIO_PIN_RESET);
    vGrayscaleDelayUs(5U);

    for (ucIndex = 0U; ucIndex < 8U; ucIndex++)
    {
        HAL_GPIO_WritePin(pstStatic->pstClkGpioBase,
            pstStatic->usClkGpioPin, GPIO_PIN_SET);
        vGrayscaleDelayUs(5U);
        ucRaw <<= 1U;
        if (HAL_GPIO_ReadPin(pstStatic->pstDataGpioBase,
                pstStatic->usDataGpioPin) == GPIO_PIN_SET)
        {
            ucRaw |= 0x01U;
        }
        HAL_GPIO_WritePin(pstStatic->pstClkGpioBase,
            pstStatic->usClkGpioPin, GPIO_PIN_RESET);
        vGrayscaleDelayUs(5U);
    }

    HAL_GPIO_WritePin(pstStatic->pstClkGpioBase,
        pstStatic->usClkGpioPin, GPIO_PIN_SET);
    return ucRaw;
}

const stGrayscaleSensorDeviceParamTdf *c_pstGetGrayscaleSensorDeviceParam(
    emGrayscaleSensorDevNumTdf emDevNum)
{
    return (emDevNum < emGrayscaleSensorDevNumMax) ?
        &s_astGrayscaleSensorDeviceParam[emDevNum] : NULL;
}

void vGrayscaleSensorDeviceInit(
    const stGrayscaleSensorStaticParamTdf *pstInit,
    emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorDeviceParamTdf *pstDevice;

    if ((pstInit == NULL) || (emDevNum >= emGrayscaleSensorDevNumMax))
    {
        return;
    }

    pstDevice = &s_astGrayscaleSensorDeviceParam[emDevNum];
    memset(pstDevice, 0, sizeof(*pstDevice));
    memcpy(&pstDevice->stStaticParam, pstInit, sizeof(*pstInit));

    /* GPIO模式由CubeMX初始化，这里只恢复协议规定的空闲高电平。 */
    HAL_GPIO_WritePin(pstInit->pstClkGpioBase,
        pstInit->usClkGpioPin, GPIO_PIN_SET);
}

void vGrayscaleSensorTask(emGrayscaleSensorDevNumTdf emDevNum)
{
    stGrayscaleSensorDeviceParamTdf *pstDevice;
    uint8_t ucRaw;
    uint8_t ucBlackMask;

    if (emDevNum >= emGrayscaleSensorDevNumMax)
    {
        return;
    }

    pstDevice = &s_astGrayscaleSensorDeviceParam[emDevNum];
    ucRaw = ucGrayscaleReadRaw(&pstDevice->stStaticParam);
    ucRaw = ucGrayscaleMapChannelOrder(ucRaw);
    if (pstDevice->stStaticParam.ucDirectionReverse != 0U)
    {
        ucRaw = ucGrayscaleReverseBits(ucRaw);
    }
    pstDevice->stRunningParam.ucDigitalOutput = ucRaw;
    ucBlackMask = (uint8_t)(~ucRaw);
    g_ucGrayscaleD1 = (uint8_t)((ucBlackMask >> 0U) & 0x01U);
    g_ucGrayscaleD2 = (uint8_t)((ucBlackMask >> 1U) & 0x01U);
    g_ucGrayscaleD3 = (uint8_t)((ucBlackMask >> 2U) & 0x01U);
    g_ucGrayscaleD4 = (uint8_t)((ucBlackMask >> 3U) & 0x01U);
    g_ucGrayscaleD5 = (uint8_t)((ucBlackMask >> 4U) & 0x01U);
    g_ucGrayscaleD6 = (uint8_t)((ucBlackMask >> 5U) & 0x01U);
    g_ucGrayscaleD7 = (uint8_t)((ucBlackMask >> 6U) & 0x01U);
    g_ucGrayscaleD8 = (uint8_t)((ucBlackMask >> 7U) & 0x01U);
    pstDevice->stRunningParam.ucReadyFlag = 1U;
}

uint8_t ucGrayscaleSensorGetDigital(emGrayscaleSensorDevNumTdf emDevNum)
{
    return (emDevNum < emGrayscaleSensorDevNumMax) ?
        s_astGrayscaleSensorDeviceParam[emDevNum].stRunningParam.ucDigitalOutput :
        0U;
}

static uint8_t ucGrayscaleCopyChannels(
    emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult)
{
    const stGrayscaleSensorRunningParamTdf *pstRunning;
    uint8_t ucIndex;

    if ((emDevNum >= emGrayscaleSensorDevNumMax) || (pusResult == NULL))
    {
        return 0U;
    }
    pstRunning =
        &s_astGrayscaleSensorDeviceParam[emDevNum].stRunningParam;
    if (pstRunning->ucReadyFlag == 0U)
    {
        return 0U;
    }
    for (ucIndex = 0U; ucIndex < 8U; ucIndex++)
    {
        pusResult[ucIndex] =
            (uint16_t)((pstRunning->ucDigitalOutput >> ucIndex) & 0x01U);
    }
    return 1U;
}

uint8_t ucGrayscaleSensorGetAnalogValue(
    emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult)
{
    return ucGrayscaleCopyChannels(emDevNum, pusResult);
}

uint8_t ucGrayscaleSensorGetNormalValue(
    emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult)
{
    return ucGrayscaleCopyChannels(emDevNum, pusResult);
}
