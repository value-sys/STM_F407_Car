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

#define DC_MOTOR_DEV_NUM                                2U
#define PWM_MAX                                         999U
#define PWM_STOP_VALUE                                  500U
#define PWM_FREQ                                        21000U

#define DC_MOTOR1                                       emDcMotorDevNum0
#define DC_MOTOR2                                       emDcMotorDevNum1

#define MOTOR_ENABLE_Port                               MOTOR_ENABLE_GPIO_Port
#define MOTOR_ENABLE_PIN                                MOTOR_ENABLE_Pin

/* ============================== 电机1硬件参数 ============================== */

#define MOTOR1_PWM_TIM                                  (&htim1)
#define MOTOR1_PWM_CH                                   TIM_CHANNEL_1
#define MOTOR1_PWM_REVERSE                              0U
#define MOTOR_1_ENABLE_Port                             MOTOR_1_ENABLE_GPIO_Port
#define MOTOR_1_ENABLE_PIN                              MOTOR_1_ENABLE_Pin

#define MOTOR1_ENCODER_TIM                              (&htim4)
#define MOTOR1_ENCODER_CHANNEL                          TIM_CHANNEL_ALL
#define MOTOR1_ENCODER_LINES                            500U
#define MOTOR1_ENCODER_RATIO                            30U
#define MOTOR1_ENCODER_REVERSE                          1U
#define MOTOR1_ENCODER_MODE                             4U

/* ============================== 电机2硬件参数 ============================== */

#define MOTOR2_PWM_TIM                                  (&htim1)
#define MOTOR2_PWM_CH                                   TIM_CHANNEL_2
#define MOTOR2_PWM_REVERSE                              0U
#define MOTOR_2_ENABLE_Port                             MOTOR_2_ENABLE_GPIO_Port
#define MOTOR_2_ENABLE_PIN                              MOTOR_2_ENABLE_Pin

#define MOTOR2_ENCODER_TIM                              (&htim3)
#define MOTOR2_ENCODER_CHANNEL                          TIM_CHANNEL_ALL
#define MOTOR2_ENCODER_LINES                            500U
#define MOTOR2_ENCODER_RATIO                            30U
#define MOTOR2_ENCODER_REVERSE                          1U
#define MOTOR2_ENCODER_MODE                             4U

/* ============================== 电机速度环参数 ============================== */

#define MOTOR_SAMPLE_TIME                               10U

#define MOTOR1_PID_KP                                   4.0f
#define MOTOR1_PID_KI                                   150.0f
#define MOTOR1_PID_KD                                   0.0f
#define MOTOR2_PID_KP                                   4.0f
#define MOTOR2_PID_KI                                   150.0f
#define MOTOR2_PID_KD                                   0.0f

/* 输出围绕双极性PWM停止值变化，保留2个计数的上下边界。 */
#define MOTOR_PID_OUTPUT_MAX                            498.0f
#define MOTOR_PID_INTEGRAL_MAX                          999.0f
#define MOTOR_PID_DEADBAND                              0.0f
#define MOTOR_PID_INTEGRAL_COEF_A                       100.0f
#define MOTOR_PID_INTEGRAL_COEF_B                       20.0f
#define MOTOR_PID_OUTPUT_LPF_RC                         0.0f
#define MOTOR_PID_DERIVATIVE_LPF_RC                     0.0f

/* ============================== 编码器测速参数 ============================== */

#define ENCODER_SPEED_FILTER_ALPHA                      0.25f
/* 设为0关闭单周期转速变化限幅；调PID时建议先关闭，避免滤波掩盖真实响应。 */
#define ENCODER_MAX_SPEED_DELTA_RPM                     0.0f

/* ============================== 差速底盘参数 ============================== */

#define CHASSIS_WHEEL_RADIUS                            30.0f
#define CHASSIS_WHEEL_BASE                              150.0f
/* 若右电机的电气正方向与底盘前进方向相反，改为1。 */
#define CHASSIS_RIGHT_MOTOR_REVERSE                     0U

/* ============================== IMU参数 ============================== */

/* IMU底层继续使用当前STM32工程的USART3和HAL单字节接收中断。 */
#define IMU_DEV_NUM                                     1U
#define IMU_1                                           emImuDevNum0
#define IMU_FRAME_LEN                                   5U
#define IMU_GYRO_SCALE                                  2000.0f
#define IMU_YAW_SCALE                                   180.0f
#define IMU_RAW_MAX                                     32768.0f
#define IMU_TASK_PERIOD_MS                              10U
#define IMU1_UART_HANDLE                                (&huart3)

/* IMU定角旋转：航向角外环输出目标角速度，角速度内环输出底盘omega。 */
#define CHASSIS_IMU_TURN_ANGLE_PID_KP                   4.0f
#define CHASSIS_IMU_TURN_ANGLE_PID_KI                   0.0f
#define CHASSIS_IMU_TURN_ANGLE_PID_KD                   0.0f
#define CHASSIS_IMU_TURN_RATE_PID_KP                    1.50f
#define CHASSIS_IMU_TURN_RATE_PID_KI                    0.15f
#define CHASSIS_IMU_TURN_RATE_PID_KD                    0.0f
#define CHASSIS_IMU_TURN_MAX_YAW_RATE                   120.0f
#define CHASSIS_IMU_TURN_MAX_OMEGA                      3.0f
#define CHASSIS_IMU_TURN_OMEGA_SLEW_RATE                6.0f
#define CHASSIS_IMU_TURN_ANGLE_TOLERANCE                1.5f
#define CHASSIS_IMU_TURN_RATE_TOLERANCE                 5.0f
#define CHASSIS_IMU_TURN_STABLE_CYCLES                  10U

/* ============================== 调试与串口监视参数 ============================== */

#define DEBUG_CHASSIS_TEST_STOP_TIME_MS                 3000U
#define DEBUG_CHASSIS_TEST_MOVE_SPEED                   314.1593f
#define VOFA_SEND_PERIOD_MS                             100U

#endif
