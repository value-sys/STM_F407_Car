/**
  * @file       imu_task.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      IMU周期数据处理任务头文件
  */

#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include <stdint.h>

/* 供调试器和VOFA直接观察的Z轴运动数据。 */
extern volatile float g_fImuAngularVelocityZ;
extern volatile float g_fImuAngularAccelerationZ;
extern volatile uint8_t g_ucImuDataValid;

float fImuTaskGetAngularVelocityZ(void);
float fImuTaskGetAngularAccelerationZ(void);
void vImuTask(void *pvParameters);

#endif
