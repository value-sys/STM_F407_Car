/// @file       function.c
/// @version    V1.0.1
/// @date       20260611
/// @brief      用户自定义功能函数实现

#include "function.h"

// VOFA发送函数
void VOFA_SendFloat(float data1, float data2, float data3)
{
    char buf[128];
    // FireWater协议格式：数据1,数据2,数据3\r\n
    int len = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\r\n",
                       data1, data2, data3);

    if (len > 0 && len < (int)sizeof(buf))
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}
