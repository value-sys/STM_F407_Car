/**
  * @file       chassis.h
  * @version    V1.1.0
  * @date       20260729
  * @brief      差速底盘运动学模块头文件
  */

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include <stdint.h>

typedef enum
{
    emChassisImuRotateIdle = 0U,
    emChassisImuRotateRunning,
    emChassisImuRotateCompleted,
    emChassisImuRotateNoImu
} emChassisImuRotateStateTdf;

typedef struct
{
    float fWheelRadius;                 ///< 车轮半径(mm)
    float fWheelBase;                   ///< 左右轮中心间距(mm)
    uint8_t ucRightMotorReverse;        ///< 右电机目标RPM是否反向
    float fImuTurnAngleKp;              ///< 航向角外环Kp
    float fImuTurnAngleKi;              ///< 航向角外环Ki
    float fImuTurnAngleKd;              ///< 航向角外环Kd
    float fImuTurnRateKp;               ///< 偏航角速度内环Kp
    float fImuTurnRateKi;               ///< 偏航角速度内环Ki
    float fImuTurnRateKd;               ///< 偏航角速度内环Kd
    float fImuTurnMaxYawRate;           ///< 最大目标偏航角速度(°/s)
    float fImuTurnMaxOmega;             ///< 最大底盘角速度(rad/s)
    float fImuTurnOmegaSlewRate;        ///< 底盘角速度斜坡(rad/s²)
    float fImuTurnAngleTolerance;       ///< 完成角度容差(°)
    float fImuTurnRateTolerance;        ///< 完成角速度容差(°/s)
    uint8_t ucImuTurnStableCycles;      ///< 连续稳定周期数
} stChassisStaticParamTdf;

typedef struct
{
    float vy;                           ///< 前进线速度(mm/s)
    float omega;                        ///< 角速度(rad/s)，左转为正、右转为负
    float fLeftTargetSpeed;             ///< 左轮目标转速(RPM)
    float fRightTargetSpeed;            ///< 右轮目标转速(RPM)
    float fImuTurnTargetYaw;            ///< 定角旋转目标航向角(°)
    float fImuTurnYawError;             ///< 当前航向角误差(°)
    float fImuTurnTargetYawRate;        ///< 外环目标角速度(°/s)
    float fImuTurnActualYawRate;        ///< IMU实测角速度(°/s)
    float fImuTurnOutputOmega;          ///< 内环输出底盘角速度(rad/s)
    uint8_t ucImuTurnStableCount;       ///< 当前连续稳定周期数
    emChassisImuRotateStateTdf emImuRotateState;
} stChassisRunningParamTdf;

typedef struct
{
    stChassisStaticParamTdf stStaticParam;
    stChassisRunningParamTdf stRunningParam;
} stChassisDeviceParamTdf;

const stChassisDeviceParamTdf *c_pstGetChassisDeviceParam(void);
void vChassisDeviceInit(const stChassisStaticParamTdf *pstInit);
void vChassisSetSpeed(float fVy, float fOmega);
void vChassisUpdate(void);
void vChassisStop(void);
void vChassisMove(float fVy);
void vChassisRotate(float fOmega);
void vChassisPivotRotate(float fOmega);

/// @brief      启动一次IMU相对定角旋转
/// @param      fRelativeAngleDeg 相对角度，正值右转，负值左转，限制在±180°。
void vChassisImuRotateStart(float fRelativeAngleDeg);

/// @brief      更新IMU定角旋转双环控制
/// @note       必须由底盘任务以MOTOR_SAMPLE_TIME周期调用。
void vChassisImuRotateUpdate(void);

/// @brief      取消IMU定角旋转并停车
void vChassisImuRotateCancel(void);

emChassisImuRotateStateTdf emChassisImuRotateGetState(void);

#endif
