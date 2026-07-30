/**
  * @file       vofa_firewater.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      VOFA+ FireWater协议发送接口
  */

#ifndef _VOFA_FIREWATER_H_
#define _VOFA_FIREWATER_H_

#include <stdint.h>

/// @brief      将指定数量的浮点通道打包为一帧FireWater文本并发送
void vVofaFireWaterSend(const float *pfData, uint8_t ucCount);

#endif
