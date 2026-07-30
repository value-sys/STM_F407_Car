/**
  * @file       project_config.h
  * @version    V1.1.0
  * @date       20260729
  * @brief      STM32F407VET6项目全局配置文件
  */

#ifndef _PROJECT_CONFIG_H_
#define _PROJECT_CONFIG_H_

#include "main.h"
#include "tim.h"
#include "usart.h"

/* 本文件只保存用户可调参数。GPIO、TIM和UART底层配置仍由CubeMX管理。 */

/* ============================== 电机驱动通用配置 ============================== */

#define DC_MOTOR_DEV_NUM                                2U      /* 电机设备数量 */
#define PWM_MAX                                         999U    /* PWM计数最大值 */
#define PWM_STOP_VALUE                                  500U    /* 双极性PWM停止/零速值 */
#define PWM_FREQ                                        21000U  /* PWM频率，单位Hz */

#define DC_MOTOR1                                       emDcMotorDevNum0 /* 左电机设备枚举 */
#define DC_MOTOR2                                       emDcMotorDevNum1 /* 右电机设备枚举 */

#define MOTOR_ENABLE_Port                               MOTOR_ENABLE_GPIO_Port /* 电机总使能GPIO端口 */
#define MOTOR_ENABLE_PIN                                MOTOR_ENABLE_Pin       /* 电机总使能GPIO引脚 */

/* ============================== 电机1硬件参数 ============================== */

#define MOTOR1_PWM_TIM                                  (&htim1)        /* 左电机PWM定时器 */
#define MOTOR1_PWM_CH                                   TIM_CHANNEL_1   /* 左电机PWM通道 */
#define MOTOR1_PWM_REVERSE                              0U              /* 左电机PWM方向是否反转 */
#define MOTOR_1_ENABLE_Port                             MOTOR_1_ENABLE_GPIO_Port /* 左电机使能端口 */
#define MOTOR_1_ENABLE_PIN                              MOTOR_1_ENABLE_Pin       /* 左电机使能引脚 */

#define MOTOR1_ENCODER_TIM                              (&htim4)        /* 左电机编码器定时器 */
#define MOTOR1_ENCODER_CHANNEL                          TIM_CHANNEL_ALL /* 左电机编码器使用全部通道 */
#define MOTOR1_ENCODER_LINES                            500U            /* 编码器每转线数 */
#define MOTOR1_ENCODER_RATIO                            30U             /* 电机到编码器的减速比 */
#define MOTOR1_ENCODER_REVERSE                          1U              /* 左电机编码方向是否反转 */
#define MOTOR1_ENCODER_MODE                             4U              /* 编码器计数倍频，4表示4倍频 */

/* ============================== 电机2硬件参数 ============================== */

#define MOTOR2_PWM_TIM                                  (&htim1)        /* 右电机PWM定时器 */
#define MOTOR2_PWM_CH                                   TIM_CHANNEL_2   /* 右电机PWM通道 */
#define MOTOR2_PWM_REVERSE                              0U              /* 右电机PWM方向是否反转 */
#define MOTOR_2_ENABLE_Port                             MOTOR_2_ENABLE_GPIO_Port /* 右电机使能端口 */
#define MOTOR_2_ENABLE_PIN                              MOTOR_2_ENABLE_Pin       /* 右电机使能引脚 */

#define MOTOR2_ENCODER_TIM                              (&htim3)        /* 右电机编码器定时器 */
#define MOTOR2_ENCODER_CHANNEL                          TIM_CHANNEL_ALL /* 右电机编码器使用全部通道 */
#define MOTOR2_ENCODER_LINES                            500U            /* 编码器每转线数 */
#define MOTOR2_ENCODER_RATIO                            30U             /* 电机到编码器的减速比 */
#define MOTOR2_ENCODER_REVERSE                          1U              /* 右电机编码方向是否反转 */
#define MOTOR2_ENCODER_MODE                             4U              /* 编码器计数倍频，4表示4倍频 */

/* ============================== 电机速度环参数 ============================== */

#define MOTOR_SAMPLE_TIME                               10U     /* 电机控制任务周期，单位ms */

#define MOTOR1_PID_KP                                   0.75f   /* 左电机速度环比例系数 */
#define MOTOR1_PID_KI                                   13.0f   /* 左电机速度环积分系数 */
#define MOTOR1_PID_KD                                   0.0f    /* 左电机速度环微分系数 */
#define MOTOR2_PID_KP                                   0.75f   /* 右电机速度环比例系数 */
#define MOTOR2_PID_KI                                   13.0f   /* 右电机速度环积分系数 */
#define MOTOR2_PID_KD                                   0.0f    /* 右电机速度环微分系数 */

/* 输出围绕双极性PWM停止值变化，保留2个计数的上下边界。 */
#define MOTOR_PID_OUTPUT_MAX                            498.0f  /* 速度PID输出限幅，围绕PWM停止值 */
#define MOTOR_PID_INTEGRAL_MAX                          999.0f  /* 速度PID积分项限幅 */
#define MOTOR_PID_DEADBAND                              0.0f    /* 速度PID误差死区，单位RPM */
#define MOTOR_PID_INTEGRAL_COEF_A                       100.0f  /* 变速积分区间A */
#define MOTOR_PID_INTEGRAL_COEF_B                       20.0f   /* 变速积分区间B */
#define MOTOR_PID_OUTPUT_LPF_RC                         0.0f    /* 输出低通滤波时间常数，0表示关闭 */
#define MOTOR_PID_DERIVATIVE_LPF_RC                     0.0f    /* 微分低通滤波时间常数，0表示关闭 */

/* ============================== 旋转串级PID参数 ============================== */

/* 位置外环：输入编码器计数误差，输出目标转速RPM。 */
#define CASCADE_ROTATE_POSITION_PID_KP                  0.01f   /* 位置环比例系数，单位RPM/count */
#define CASCADE_ROTATE_POSITION_PID_KI                  0.0f    /* 位置环积分系数 */
#define CASCADE_ROTATE_POSITION_PID_KD                  0.0f    /* 位置环微分系数 */
#define CASCADE_ROTATE_POSITION_MAX_SPEED_RPM           80.0f   /* 位置环速度修正限幅，单位RPM */
#define CASCADE_ROTATE_TARGET_SPEED_MAX_RPM             250.0f  /* 前馈加位置修正后的总目标转速限幅 */
#define CASCADE_ROTATE_FEEDFORWARD_OMEGA_RAD_S          0.5f    /* 默认车体旋转角速度，单位rad/s */
/* 半径修正系数 = 当前指令半径 / 实测半径；实际半径偏大时设置为小于1。 */
#define CASCADE_ROTATE_RADIUS_COMMAND_TO_MEASURED_RATIO 1.00f   /* 旋转半径修正比例 */
/* 左右电机理论位移计数修正比例；某侧实际走得过多时将对应系数调小。 */
#define CASCADE_ROTATE_MOTOR1_TARGET_COUNT_RATIO        1.0f    /* 电机1目标计数修正比例 */
#define CASCADE_ROTATE_MOTOR2_TARGET_COUNT_RATIO        1.0f    /* 电机2目标计数修正比例 */
#define CASCADE_ROTATE_SYNC_PID_KP                      80.0f
#define CASCADE_ROTATE_SYNC_PID_KI                       0.0f
#define CASCADE_ROTATE_SYNC_PID_KD                       0.0f
#define CASCADE_ROTATE_SYNC_MAX_CORRECTION_RPM          100.0f

/* 速度内环：输入RPM误差，输出相对PWM停止值的修正量。 */
#define CASCADE_ROTATE_SPEED_PID_KP                     0.75f   /* 旋转速度环比例系数 */
#define CASCADE_ROTATE_SPEED_PID_KI                     13.0f   /* 旋转速度环积分系数 */
#define CASCADE_ROTATE_SPEED_PID_KD                     0.0f    /* 旋转速度环微分系数 */
#define CASCADE_ROTATE_SPEED_OUTPUT_MAX                 498.0f  /* 旋转速度环输出限幅 */
#define CASCADE_ROTATE_SPEED_INTEGRAL_MAX               999.0f  /* 旋转速度环积分限幅 */
#define CASCADE_ROTATE_POSITION_TOLERANCE_COUNTS        100.0f  /* 到位位置误差，单位count */
#define CASCADE_ROTATE_SPEED_TOLERANCE_RPM               3.0f    /* 到位速度误差，单位RPM */
#define CASCADE_ROTATE_FEEDFORWARD_DISABLE_COUNTS      1000.0f  /* 接近目标后关闭前馈，单位count */

/* ============================== 编码器测速参数 ============================== */

#define ENCODER_SPEED_FILTER_ALPHA                      0.25f   /* 编码器速度一阶滤波系数 */
/* 设为0关闭单周期转速变化限幅；调PID时建议先关闭，避免滤波掩盖真实响应。 */
#define ENCODER_MAX_SPEED_DELTA_RPM                     0.0f    /* 单周期最大测速变化，0表示关闭，单位RPM */

/* ============================== 灰度传感器参数 ============================== */

/* 当前板使用一组CLK/DAT串行8路数字灰度传感器。 */
#define GRAYSCALE_SENSOR_DEV_NUM                        1U     /* 灰度传感器设备数量 */
#define GRAYSCALE1                                      emGrayscaleSensorDevNum0 /* 灰度传感器设备枚举 */
#define GRAYSCALE1_CLK_PORT                             CLK_GPIO_Port /* 灰度CLK GPIO端口 */
#define GRAYSCALE1_CLK_PIN                              CLK_Pin       /* 灰度CLK GPIO引脚 */
#define GRAYSCALE1_DATA_PORT                            DAT_GPIO_Port /* 灰度DAT GPIO端口 */
#define GRAYSCALE1_DATA_PIN                             DAT_Pin       /* 灰度DAT GPIO引脚 */
#define GRAYSCALE1_DIRECTION_REVERSE                    0U     /* D1~D8位序是否反转 */
#define GRAYSCALE_SAMPLE_PERIOD_MS                      10U    /* 灰度读取周期，单位ms */

/* ============================== 差速底盘参数 ============================== */

#define CHASSIS_WHEEL_RADIUS                            30.0f   /* 车轮半径，单位mm */
#define CHASSIS_WHEEL_BASE                              225.0f  /* 左右轮中心距，单位mm */
/* 当前电机正方向与车体前进方向相反，统一反转底盘纵向速度。 */
#define CHASSIS_FORWARD_REVERSE                         1U      /* 底盘前进方向是否整体反转 */
/* 右电机镜像安装，底盘前进时需要输出负方向目标RPM。 */
#define CHASSIS_RIGHT_MOTOR_REVERSE                     1U      /* 右电机目标RPM方向是否反转 */

/* ============================== IMU参数 ============================== */

/* IMU底层继续使用当前STM32工程的USART3和HAL单字节接收中断。 */
#define IMU_DEV_NUM                                     1U      /* IMU设备数量 */
#define IMU_1                                           emImuDevNum0 /* IMU设备枚举 */
#define IMU_FRAME_LEN                                   5U      /* IMU协议帧长度，单位字节 */
#define IMU_GYRO_SCALE                                  2000.0f /* 陀螺仪满量程，单位度每秒 */
#define IMU_YAW_SCALE                                   180.0f  /* 航向角原始量程，单位度 */
#define IMU_RAW_MAX                                     32768.0f /* IMU有符号原始数据换算分母 */
#define IMU_TASK_PERIOD_MS                              10U     /* IMU任务周期，单位ms */
#define IMU1_UART_HANDLE                                (&huart3) /* IMU串口句柄 */

/* IMU定角旋转：航向角外环输出目标角速度，角速度内环输出底盘omega。 */
#define CHASSIS_IMU_TURN_ANGLE_PID_KP                   4.0f    /* IMU定角外环比例系数 */
#define CHASSIS_IMU_TURN_ANGLE_PID_KI                   0.0f    /* IMU定角外环积分系数 */
#define CHASSIS_IMU_TURN_ANGLE_PID_KD                   0.0f    /* IMU定角外环微分系数 */
#define CHASSIS_IMU_TURN_RATE_PID_KP                    1.50f   /* IMU角速度内环比例系数 */
#define CHASSIS_IMU_TURN_RATE_PID_KI                    0.15f   /* IMU角速度内环积分系数 */
#define CHASSIS_IMU_TURN_RATE_PID_KD                    0.0f    /* IMU角速度内环微分系数 */
#define CHASSIS_IMU_TURN_MAX_YAW_RATE                   120.0f  /* 定角模式目标角速度限幅，单位度每秒 */
#define CHASSIS_IMU_TURN_MAX_OMEGA                      3.0f    /* 定角模式底盘角速度限幅，单位rad/s */
#define CHASSIS_IMU_TURN_OMEGA_SLEW_RATE                6.0f    /* 定角模式角速度变化率，单位rad/s^2 */
#define CHASSIS_IMU_TURN_ANGLE_TOLERANCE                1.5f    /* 定角完成角度误差，单位度 */
#define CHASSIS_IMU_TURN_RATE_TOLERANCE                 5.0f    /* 定角完成角速度误差，单位度每秒 */
#define CHASSIS_IMU_TURN_STABLE_CYCLES                  10U     /* 连续稳定判定周期数 */

/* ============================== 灰度+IMU循迹参数 ============================== */

#define LINE_TRACK_TASK_PERIOD_MS                       10U     /* 循迹控制周期，单位ms */
#define LINE_TRACK_TARGET_RPM                           100.0f  /* 灰度+IMU直线/弯道测试目标转速 */
#define LINE_TRACK_GRAY_ONLY_TARGET_RPM                 50.0f   /* 纯灰度循迹测试目标转速 */
#define LINE_TRACK_BASE_SPEED                           314.1593f /* 默认线速度，单位mm/s */
#define LINE_TRACK_CORRECTION_KP                        5.0f    /* 传统灰度比例修正系数 */
#define LINE_TRACK_MAX_CORRECTION_OMEGA                 5.0f    /* 底盘循迹角速度限幅，单位rad/s */
#define LINE_TRACK_LOST_SPEED_SCALE                     0.5f    /* 传统灰度丢线时的速度比例 */

/* 直线循迹灰度位置外环，输出目标偏航角速度(°/s)。 */
#define LINE_TRACK_STRAIGHT_GRAY_PID_KP                 60.0f   /* 直线灰度外环比例系数 */
#define LINE_TRACK_STRAIGHT_GRAY_PID_KI                 0.0f    /* 直线灰度外环积分系数 */
#define LINE_TRACK_STRAIGHT_GRAY_PID_KD                 0.0f    /* 直线灰度外环微分系数 */

/* 弯道循迹灰度位置外环，单独调参。 */
#define LINE_TRACK_CURVE_GRAY_PID_KP                    0.1f  /* 弯道灰度外环比例系数 */
#define LINE_TRACK_CURVE_GRAY_PID_KI                    0.0f    /* 弯道灰度外环积分系数 */
#define LINE_TRACK_CURVE_GRAY_PID_KD                    0.0f    /* 弯道灰度外环微分系数 */

/* IMU偏航角外环参数，保留供纯IMU直线模式使用。 */
#define IMU_STRAIGHT_ANGLE_PID_KP                       2.0f    /* 纯IMU直线航向角外环比例系数 */
#define IMU_STRAIGHT_ANGLE_PID_KI                       0.0f    /* 纯IMU直线航向角外环积分系数 */
#define IMU_STRAIGHT_ANGLE_PID_KD                       0.0f    /* 纯IMU直线航向角外环微分系数 */

/* 直线循迹IMU偏航角速度内环，输入输出单位均为度每秒。 */
#define LINE_TRACK_STRAIGHT_YAW_RATE_PID_KP              1.25f   /* 直线IMU角速度内环比例系数 */
#define LINE_TRACK_STRAIGHT_YAW_RATE_PID_KI              0.15f   /* 直线IMU角速度内环积分系数 */
#define LINE_TRACK_STRAIGHT_YAW_RATE_PID_KD              0.0f    /* 直线IMU角速度内环微分系数 */
#define LINE_TRACK_CURVE_YAW_RATE_PID_KP                 0.0f
#define LINE_TRACK_CURVE_YAW_RATE_PID_KI                 0.0f
#define LINE_TRACK_CURVE_YAW_RATE_PID_KD                 0.0f
#define LINE_TRACK_STRAIGHT_MAX_TARGET_YAW_RATE          60.0f
#define LINE_TRACK_CURVE_MAX_TARGET_YAW_RATE             180.0f
#define LINE_TRACK_OUTER_CONTROL_WEIGHT                  1.6f
#define LINE_TRACK_IMU_FEEDBACK_WEIGHT                   0.4f
#define LINE_TRACK_SENSOR_TURN_SIGN                      1.0f
#define LINE_TRACK_IMU_TURN_SIGN                         1.0f

/* ============================== 调试与串口监视参数 ============================== */

#define DEBUG_CHASSIS_TEST_STOP_TIME_MS                 3000U   /* 底盘测试切换前停车时间，单位ms */
#define DEBUG_CHASSIS_TEST_RUN_TIME_MS                  10000U  /* 底盘单方向运行时间，单位ms */
#define DEBUG_CHASSIS_TEST_MOVE_SPEED                   314.1593f /* 底盘测试线速度，单位mm/s */
#define DEBUG_CASCADE_ROTATE_RADIUS_MM                  500.0f  /* 旋转半径，0表示原地旋转，单位mm */
#define DEBUG_CASCADE_ROTATE_ANGLE_DEG                  360.0f  /* 旋转角度，正负表示旋转方向，单位度 */
#define DEBUG_CASCADE_ROTATE_OMEGA_RAD_S                0.6f    /* 旋转前馈车体角速度，单位rad/s */
#define DEBUG_CASCADE_ROTATE_AFTER_TARGET_RPM           50.0f
#define VOFA_SEND_PERIOD_MS                             100U    /* VOFA数据发送周期，单位ms */

#endif
