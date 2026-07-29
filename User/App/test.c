#include "project_config.h"
#include "motor.h"
#include "encoder.h"
#include "stdio.h"
#include "function.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "chassis.h"
#include "imu.h"
#include "usart.h"
#include "GrayTrace.h"
#include "GrayscaleSensor.h"

void vMotorInit(void)
{
    // 电机1静态参数配置
    stDcMotorStaticParamTdf stMotor1StaticParam = {
        .pstTimBase = MOTOR1_PWM_TIM,
        .ulTimChannel = MOTOR1_PWM_CH,
        .pstDirGpioBase0 = MOTOR_ENABLE_Port,
        .usDirGpioPin0 = MOTOR_ENABLE_PIN,        
        .pstDirGpioBase1 = MOTOR_1_ENABLE_Port,
        .usDirGpioPin1 = MOTOR_1_ENABLE_PIN,
        .usPwmMaxValue = PWM_MAX,
    };
    vDcMotorDeviceInit(&stMotor1StaticParam, DC_MOTOR1);
    // 电机1编码器参数配置
    stDcMotorEncoderStaticParamTdf stMotor1EncoderStaticParam = {
        .pstTimBase = MOTOR1_ENCODER_TIM,
        .ulTimChannel = MOTOR1_ENCODER_CHANNEL,
        .usLines = MOTOR1_ENCODER_LINES,
        .usReductionRatio = MOTOR1_ENCODER_RATIO,
        .ucReverse = MOTOR1_ENCODER_REVERSE,
        .ucMode = MOTOR1_ENCODER_MODE
    };
    vEncoderInit(&stMotor1EncoderStaticParam, DC_MOTOR1);
    // 电机1PID参数配置
    stPidControllerStaticParamTdf stMotor1PidStaticParam = {
        .fKp = 1.0f,
        .fKi = 0.1f,
        .fKd = 0.01f,
        .fOutputMax = 999.0f,
        .fOutputMin = -999.0f,
        .fIntegralMax = 999.0f,
        .fIntegralMin = -999.0f
    };
    vPidPositionInit(&stMotor1PidStaticParam, DC_MOTOR1);

    // 电机2静态参数配置
    stDcMotorStaticParamTdf stMotor2StaticParam = {
        .pstTimBase = MOTOR2_PWM_TIM,
        .ulTimChannel = MOTOR2_PWM_CH,
        .pstDirGpioBase0 = MOTOR_ENABLE_Port,
        .usDirGpioPin0 = MOTOR_ENABLE_PIN,
        .pstDirGpioBase1 = MOTOR_2_ENABLE_Port,
        .usDirGpioPin1 = MOTOR_2_ENABLE_PIN,
        .usPwmMaxValue = PWM_MAX,
    };
    vDcMotorDeviceInit(&stMotor2StaticParam, DC_MOTOR2);
    // 电机2编码器参数配置
    stDcMotorEncoderStaticParamTdf stMotor2EncoderStaticParam = {
        .pstTimBase = MOTOR2_ENCODER_TIM,
        .ulTimChannel = MOTOR2_ENCODER_CHANNEL,
        .usLines = MOTOR2_ENCODER_LINES, 
        .usReductionRatio = MOTOR2_ENCODER_RATIO,
        .ucReverse = MOTOR2_ENCODER_REVERSE,
        .ucMode = MOTOR2_ENCODER_MODE
    };
    vEncoderInit(&stMotor2EncoderStaticParam, DC_MOTOR2);
    // 电机2PID参数配置
    stPidControllerStaticParamTdf stMotor2PidStaticParam = {
        .fKp = 1.0f,
        .fKi = 0.1f,
        .fKd = 0.01f,
        .fOutputMax = 999.0f,
        .fOutputMin = -999.0f,
        .fIntegralMax = 999.0f,
        .fIntegralMin = -999.0f
    };
    vPidPositionInit(&stMotor2PidStaticParam, DC_MOTOR2);
}

void vchassisInit(void)
{
    // 1. 初始化电机和编码器
    vMotorInit();

    stChassisStaticParamTdf stChassisStaticParam = {
        .fWheelBase = 150.0f, // 轮距30cm
        .fWheelRadius = 30.0f // 轮半径5cm
    };
    vChassisDeviceInit(&stChassisStaticParam);

}    

void vMotorTest(void)
{
    // vDcMotorSetSpeed(DC_MOTOR1, 600); // 设置电机1速度
    // //printf("Motor 1 PWM set to 600\r\n");
    // vDcMotorSetSpeed(DC_MOTOR2, 500); // 设置电机2速度
    // osDelay(100); // 运行2秒
    // 电机1测试：逐渐增加速度
    
    for (uint16_t pwm = 500; pwm <= 700; pwm += 50)
    {
        printf("Setting Motor 1 PWM: %d\r\n", pwm);

        vDcMotorSetSpeed(DC_MOTOR1, pwm);
        osDelay(500); // 每次调整后等待500ms
    }
    // 电机1测试：逐渐减少速度
    for (int16_t pwm = 700; pwm >= 500; pwm -= 50)
    {
        printf("Setting Motor 1 PWM: %d\r\n", pwm);

        vDcMotorSetSpeed(DC_MOTOR1, pwm);
        osDelay(500); // 每次调整后等待500ms
    }
    
    // 电机2测试：逐渐增加速度
    for (uint16_t pwm = 500; pwm <= 700; pwm += 50)
    {
        printf("Setting Motor 2 PWM: %d\r\n", pwm);

        vDcMotorSetSpeed(DC_MOTOR2, pwm);
        osDelay(500); // 每次调整后等待500ms
    }
    // 电机2测试：逐渐减少速度
    for (int16_t pwm = 700; pwm >= 500; pwm -= 50)
    {
        printf("Setting Motor 2 PWM: %d\r\n", pwm);

        vDcMotorSetSpeed(DC_MOTOR2, pwm);
        osDelay(500); // 每次调整后等待500ms
    }
}

void vEncoderTest(void)
{

    const stDcMotorEncoderDeviceParamTdf *pstEnc1 = c_pstGetEncoderDeviceParam(DC_MOTOR1);
    const stDcMotorEncoderDeviceParamTdf *pstEnc2 = c_pstGetEncoderDeviceParam(DC_MOTOR2);
    const stDcMotorDeviceParamTdf *pstMotor1 = c_pstGetDcMotorDeviceParam(DC_MOTOR1);
    const stDcMotorDeviceParamTdf *pstMotor2 = c_pstGetDcMotorDeviceParam(DC_MOTOR2);
    //const stDcMotorEncoderDeviceParamTdf *pstEnc2 = c_pstGetEncoderDeviceParam(DC_MOTOR2);
    // printf("Motor 1 - Count: %ld, Angle: %ld, Speed: %.2f RPM\n", 
    //         pstEnc1->stRunningParam.lCount, 
    //         pstEnc1->stRunningParam.lAngle, 
    //         pstEnc1->stRunningParam.fCurrentSpeed);
    // printf("Motor 2 - Count: %ld, Angle: %ld, Speed: %.2f RPM\n", 
    //         pstEnc2->stRunningParam.lCount, 
    //         pstEnc2->stRunningParam.lAngle, 
    //         pstEnc2->stRunningParam.fCurrentSpeed);
    //HAL_Delay(10); // 每次读取后等待500ms

    VOFA_SendFloat(pstEnc1->stRunningParam.fCurrentSpeed, pstEnc1->stRunningParam.lCount, pstMotor1->stRunningParam.usPwmValue); // 通过VOFA发送速度数据
    VOFA_SendFloat(pstEnc2->stRunningParam.fCurrentSpeed, pstEnc2->stRunningParam.lCount, pstMotor2->stRunningParam.usPwmValue); // 通过VOFA发送速度数据
}


void vImuInit()
{
    stImuStaticParamTdf stInitParam;
    
    // 1. 配置设备0参数：使用USART3
    stInitParam.pstUartHandle = &huart3;
    vImuDeviceInit(&stInitParam, IMU_1);
    printf("开始IMU校准...\r\n");
    //vImuStartBiasCalibration(IMU_1); // 启动自动零偏校准
    vImuSendYawZeroCmd(IMU_1); // 发送Z轴角度归零指令
    printf("IMU校准完成！\r\n");
}

void vImuTest()
{

    float fYawAngle = fImuGetYaw(IMU_1);
    float fGyroZ = fImuGetGyroZ(IMU_1);

    printf("Yaw: %7.2f °  |  GyroZ: %7.2f °/s\r\n", fYawAngle, fGyroZ);
    osDelay(1000);
}

void vGrayTraceInit(void)
{
    // 灰度传感器静态参数初始化
    stGrayscaleSensorStaticParamTdf stGraySensorInit = {
        .pstAddrGpioBase0     = GRAYSCALE1_ADDR0_PORT,
        .usAddrGpioPin0       = GRAYSCALE1_ADDR0_PIN,
        .pstAddrGpioBase1     = GRAYSCALE1_ADDR1_PORT,
        .usAddrGpioPin1       = GRAYSCALE1_ADDR1_PIN,
        .pstAddrGpioBase2     = GRAYSCALE1_ADDR2_PORT,
        .usAddrGpioPin2       = GRAYSCALE1_ADDR2_PIN,
        
        // .pstEnGpioBase        = GRAYSCALE1_EN_PORT,
        // .usEnGpioPin          = GRAYSCALE1_EN_PIN,
        
        .pstAdcHandle         = GRAYSCALE1_ADC_HANDLE,
        .ulAdcChannel         = GRAYSCALE1_ADC_CHANNEL,
        
        .emAdcBits            = GRAYSCALE_ADC_BITS,
        .ucDirectionReverse   = GRAYSCALE_DIRECTION_REVERSE,
        .ucChannelLogicInvert = GRAYSCALE_CHANNEL_LOGIC_INVERT,
        .ucSampleTimes        = GRAYSCALE_SAMPLE_TIMES,
    };
    vGrayscaleSensorDeviceInit(&stGraySensorInit, GRAYSCALE1);

    vGrayscaleSensorTask(GRAYSCALE1);//灰度传感器主任务
    unsigned short Anolog[8]={0};
    unsigned short white[8]={1600,1600,1600,1600,1600,1600,1600,1600};
    unsigned short black[8]={100,100,100,100,100,100,100,100};
    ucGrayscaleSensorGetAnalogValue(GRAYSCALE1,Anolog);//获取灰度传感器原始模拟值
    //此时打印的ADC的采样值，可用通过这个ADC作为黑白值的校准
    printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);

    vGrayscaleSensorSetCalibration(GRAYSCALE1,white,black);
}


void vGrayTraceTest(void)
{
    vGrayscaleSensorTask(GRAYSCALE1);
    unsigned char Digtal = ucGrayscaleSensorGetDigital(GRAYSCALE1);//获取传感器数字量结果
    printf("Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);

    unsigned short Anolog[8]={0};
    //获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
    if(ucGrayscaleSensorGetAnalogValue(GRAYSCALE1,Anolog))
    {
        printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
    }

    unsigned short Normal[8]={0};
    //获取传感器归一化结果
    if(ucGrayscaleSensorGetNormalValue(GRAYSCALE1, Normal))
    {
        printf("Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
    }
    
}