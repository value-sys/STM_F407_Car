/**
  * @file       app_init.h
  * @version    V1.0.0
  * @date       20260729
  * @brief      应用模块初始化头文件
  */

#ifndef _APP_INIT_H_
#define _APP_INIT_H_

/// @brief      初始化电机、编码器、速度环、底盘和IMU模块
/// @note       必须在创建任务和启动调度器之前调用。
void vAppModuleInit(void);

#endif
