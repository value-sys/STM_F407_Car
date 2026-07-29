/**
  * @file       vofa_task.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      VOFA电机监视任务头文件
  */

#ifndef _VOFA_TASK_H_
#define _VOFA_TASK_H_

void vVofaSendMotorInfo(void);
void vVofaSendImuInfo(void);
void vVofaTask(void *pvParameters);

#endif
