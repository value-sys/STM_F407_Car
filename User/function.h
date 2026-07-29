#ifndef _FUNCTIONAL_H_
#define _FUNCTIONAL_H_

#include "project_config.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

void VOFA_SendFloat(float data1, float data2, float data3);

#endif
