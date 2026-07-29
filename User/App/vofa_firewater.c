/**
  * @file       vofa_firewater.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      VOFA+ FireWater协议模块
  * @note       保持当前工程USART1发送接口，仅扩展单帧可发送的通道数。
  */

#include "vofa_firewater.h"
#include "usart.h"
#include <stddef.h>
#include <stdio.h>

void vVofaFireWaterSend(const float *pfData, uint8_t ucCount)
{
    char acBuffer[384];
    size_t xUsed = 0U;
    uint8_t ucIndex;

    if ((pfData == NULL) || (ucCount == 0U))
    {
        return;
    }

    for (ucIndex = 0U; ucIndex < ucCount; ucIndex++)
    {
        int iWritten = snprintf(&acBuffer[xUsed],
            sizeof(acBuffer) - xUsed,
            (ucIndex + 1U < ucCount) ? "%.2f," : "%.2f\r\n",
            pfData[ucIndex]);

        if ((iWritten <= 0) ||
            ((size_t)iWritten >= (sizeof(acBuffer) - xUsed)))
        {
            return;
        }
        xUsed += (size_t)iWritten;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)acBuffer,
        (uint16_t)xUsed, 100U);
}
