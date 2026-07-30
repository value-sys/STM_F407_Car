/**
  * @file       GrayscaleSensor.h
  * @version    V1.1.0
  * @date       20260729
  * @brief      八路数字灰度传感器串行驱动头文件
  */

#ifndef _GRAYSCALE_SENSOR_H_
#define _GRAYSCALE_SENSOR_H_

#include "stm32f4xx_hal.h"
#include "project_config.h"
#include <stdint.h>

typedef enum
{
    emGrayscaleSensorDevNum0 = 0,
    emGrayscaleSensorDevNumMax
} emGrayscaleSensorDevNumTdf;

/// @brief 灰度传感器硬件连接参数
typedef struct
{
    GPIO_TypeDef *pstClkGpioBase;       ///< 串行时钟GPIO端口
    uint16_t usClkGpioPin;              ///< 串行时钟GPIO引脚
    GPIO_TypeDef *pstDataGpioBase;      ///< 串行数据GPIO端口
    uint16_t usDataGpioPin;             ///< 串行数据GPIO引脚
    uint8_t ucDirectionReverse;         ///< 1表示将D1~D8的位顺序反转
} stGrayscaleSensorStaticParamTdf;

/// @brief 灰度传感器运行数据
typedef struct
{
    uint8_t ucDigitalOutput;            ///< D1~D8数字量，bit0对应D1
    uint8_t ucReadyFlag;                ///< 完成至少一次采集后置1
} stGrayscaleSensorRunningParamTdf;

typedef struct
{
    stGrayscaleSensorStaticParamTdf stStaticParam;
    stGrayscaleSensorRunningParamTdf stRunningParam;
} stGrayscaleSensorDeviceParamTdf;

const stGrayscaleSensorDeviceParamTdf *c_pstGetGrayscaleSensorDeviceParam(
    emGrayscaleSensorDevNumTdf emDevNum);

void vGrayscaleSensorDeviceInit(
    const stGrayscaleSensorStaticParamTdf *pstInit,
    emGrayscaleSensorDevNumTdf emDevNum);

/// @brief 按参考时序读取一次完整的8路数字量并更新缓存
void vGrayscaleSensorTask(emGrayscaleSensorDevNumTdf emDevNum);

/// @return 8位数字量，bit0~bit7依次对应D1~D8
uint8_t ucGrayscaleSensorGetDigital(emGrayscaleSensorDevNumTdf emDevNum);

/**
  * @brief      兼容旧调用，将每路数字状态扩展为uint16_t数组
  * @retval     0表示尚未采集，1表示获取成功
  */
uint8_t ucGrayscaleSensorGetAnalogValue(
    emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult);
uint8_t ucGrayscaleSensorGetNormalValue(
    emGrayscaleSensorDevNumTdf emDevNum, uint16_t *pusResult);

#endif
