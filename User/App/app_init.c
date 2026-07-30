/**
  * @file       app_init.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      STM32F407应用模块集中初始化
  */

#include "app_init.h"
#include "chassis.h"
#include "encoder.h"
#include "GrayscaleSensor.h"
#include "imu.h"
#include "line_track.h"
#include "motor.h"
#include "motor_control.h"
#include "project_config.h"
#include "ui_task.h"
#include "vision_task.h"

/// @brief      初始化应用层使用的全部功能模块
/// @note       GPIO、TIM、UART底层均由CubeMX初始化，本函数只绑定应用设备参数。
void vAppModuleInit(void)
{
    const stDcMotorStaticParamTdf stMotor1 = {
        .pstTimBase = MOTOR1_PWM_TIM,
        .ulTimChannel = MOTOR1_PWM_CH,
        .pstDirGpioBase0 = MOTOR_ENABLE_Port,
        .usDirGpioPin0 = MOTOR_ENABLE_PIN,
        .pstDirGpioBase1 = MOTOR_1_ENABLE_Port,
        .usDirGpioPin1 = MOTOR_1_ENABLE_PIN,
        .usPwmMaxValue = PWM_MAX,
        .usPwmStopValue = PWM_STOP_VALUE,
        .ucPwmReverse = MOTOR1_PWM_REVERSE,
    };
    const stDcMotorStaticParamTdf stMotor2 = {
        .pstTimBase = MOTOR2_PWM_TIM,
        .ulTimChannel = MOTOR2_PWM_CH,
        .pstDirGpioBase0 = MOTOR_ENABLE_Port,
        .usDirGpioPin0 = MOTOR_ENABLE_PIN,
        .pstDirGpioBase1 = MOTOR_2_ENABLE_Port,
        .usDirGpioPin1 = MOTOR_2_ENABLE_PIN,
        .usPwmMaxValue = PWM_MAX,
        .usPwmStopValue = PWM_STOP_VALUE,
        .ucPwmReverse = MOTOR2_PWM_REVERSE,
    };
    const stDcMotorEncoderStaticParamTdf stEncoder1 = {
        .pstTimBase = MOTOR1_ENCODER_TIM,
        .ulTimChannel = MOTOR1_ENCODER_CHANNEL,
        .usLines = MOTOR1_ENCODER_LINES,
        .usReductionRatio = MOTOR1_ENCODER_RATIO,
        .ucReverse = MOTOR1_ENCODER_REVERSE,
        .ucMode = MOTOR1_ENCODER_MODE,
    };
    const stDcMotorEncoderStaticParamTdf stEncoder2 = {
        .pstTimBase = MOTOR2_ENCODER_TIM,
        .ulTimChannel = MOTOR2_ENCODER_CHANNEL,
        .usLines = MOTOR2_ENCODER_LINES,
        .usReductionRatio = MOTOR2_ENCODER_RATIO,
        .ucReverse = MOTOR2_ENCODER_REVERSE,
        .ucMode = MOTOR2_ENCODER_MODE,
    };
    const stChassisStaticParamTdf stChassis = {
        .fWheelRadius = CHASSIS_WHEEL_RADIUS,
        .fWheelBase = CHASSIS_WHEEL_BASE,
        .ucRightMotorReverse = CHASSIS_RIGHT_MOTOR_REVERSE,
        .fImuTurnAngleKp = CHASSIS_IMU_TURN_ANGLE_PID_KP,
        .fImuTurnAngleKi = CHASSIS_IMU_TURN_ANGLE_PID_KI,
        .fImuTurnAngleKd = CHASSIS_IMU_TURN_ANGLE_PID_KD,
        .fImuTurnRateKp = CHASSIS_IMU_TURN_RATE_PID_KP,
        .fImuTurnRateKi = CHASSIS_IMU_TURN_RATE_PID_KI,
        .fImuTurnRateKd = CHASSIS_IMU_TURN_RATE_PID_KD,
        .fImuTurnMaxYawRate = CHASSIS_IMU_TURN_MAX_YAW_RATE,
        .fImuTurnMaxOmega = CHASSIS_IMU_TURN_MAX_OMEGA,
        .fImuTurnOmegaSlewRate = CHASSIS_IMU_TURN_OMEGA_SLEW_RATE,
        .fImuTurnAngleTolerance = CHASSIS_IMU_TURN_ANGLE_TOLERANCE,
        .fImuTurnRateTolerance = CHASSIS_IMU_TURN_RATE_TOLERANCE,
        .ucImuTurnStableCycles = CHASSIS_IMU_TURN_STABLE_CYCLES,
    };
    const stImuStaticParamTdf stImu = {
        .pstUartHandle = IMU1_UART_HANDLE,
    };
    const stGrayscaleSensorStaticParamTdf stGrayscale = {
        .pstClkGpioBase = GRAYSCALE1_CLK_PORT,
        .usClkGpioPin = GRAYSCALE1_CLK_PIN,
        .pstDataGpioBase = GRAYSCALE1_DATA_PORT,
        .usDataGpioPin = GRAYSCALE1_DATA_PIN,
        .ucDirectionReverse = GRAYSCALE1_DIRECTION_REVERSE,
    };
    const stLineTrackStaticParamTdf stLineTrack = {
        .fBaseSpeed = LINE_TRACK_BASE_SPEED,
        .fCorrectionKp = LINE_TRACK_CORRECTION_KP,
        .fMaxCorrectionOmega = LINE_TRACK_MAX_CORRECTION_OMEGA,
        .fLostSpeedScale = LINE_TRACK_LOST_SPEED_SCALE,
        .fStraightGrayPidKp = LINE_TRACK_STRAIGHT_GRAY_PID_KP,
        .fStraightGrayPidKi = LINE_TRACK_STRAIGHT_GRAY_PID_KI,
        .fStraightGrayPidKd = LINE_TRACK_STRAIGHT_GRAY_PID_KD,
        .fCurveGrayPidKp = LINE_TRACK_CURVE_GRAY_PID_KP,
        .fCurveGrayPidKi = LINE_TRACK_CURVE_GRAY_PID_KI,
        .fCurveGrayPidKd = LINE_TRACK_CURVE_GRAY_PID_KD,
        .fYawAnglePidKp = IMU_STRAIGHT_ANGLE_PID_KP,
        .fYawAnglePidKi = IMU_STRAIGHT_ANGLE_PID_KI,
        .fYawAnglePidKd = IMU_STRAIGHT_ANGLE_PID_KD,
        .fStraightYawRatePidKp = LINE_TRACK_STRAIGHT_YAW_RATE_PID_KP,
        .fStraightYawRatePidKi = LINE_TRACK_STRAIGHT_YAW_RATE_PID_KI,
        .fStraightYawRatePidKd = LINE_TRACK_STRAIGHT_YAW_RATE_PID_KD,
        .fCurveYawRatePidKp = LINE_TRACK_CURVE_YAW_RATE_PID_KP,
        .fCurveYawRatePidKi = LINE_TRACK_CURVE_YAW_RATE_PID_KI,
        .fCurveYawRatePidKd = LINE_TRACK_CURVE_YAW_RATE_PID_KD,
        .fStraightMaxTargetYawRate =
            LINE_TRACK_STRAIGHT_MAX_TARGET_YAW_RATE,
        .fCurveMaxTargetYawRate = LINE_TRACK_CURVE_MAX_TARGET_YAW_RATE,
        .fOuterControlWeight = LINE_TRACK_OUTER_CONTROL_WEIGHT,
        .fImuFeedbackWeight = LINE_TRACK_IMU_FEEDBACK_WEIGHT,
    };

    /* 电机控制链的顺序必须保持：PWM -> 编码器 -> PID -> 底盘。 */
    vDcMotorDeviceInit(&stMotor1, DC_MOTOR1);
    vDcMotorDeviceInit(&stMotor2, DC_MOTOR2);
    vEncoderInit(&stEncoder1, DC_MOTOR1);
    vEncoderInit(&stEncoder2, DC_MOTOR2);
    vMotorControlInit();
    vChassisDeviceInit(&stChassis);
    vGrayscaleSensorDeviceInit(&stGrayscale, GRAYSCALE1);
    vLineTrackDeviceInit(&stLineTrack);

    /*
     * IMU继续使用当前工程USART3的HAL单字节接收中断。设备初始化会启动
     * HAL_UART_Receive_IT，随后发送一次航向归零命令。该流程发生在调度器
     * 启动前，因此命令间的短暂HAL_Delay不会阻塞任何FreeRTOS任务。
     */
    vImuDeviceInit(&stImu, IMU_1);
    vImuSendYawZeroCmd(IMU_1);
    vVisionInit();
    vUiInit();
}
