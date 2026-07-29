/**
  * @file       debug_task.c
  * @version    V1.0.0
  * @date       20260729
  * @brief      通过调试器变量选择功能的非阻塞调试任务
  */

#include "debug_task.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "motor_control.h"
#include "project_config.h"

/* 当前固件用于底盘前进测试：上电先停车保护，再进入直线前进。 */
volatile emDebugModeTdf g_emDebugMode = emDebugModeChassis;
volatile float g_fDebugMotor1TargetRpm = 100.0f;
volatile float g_fDebugMotor2TargetRpm = 100.0f;
volatile uint16_t g_usDebugMotor1Pwm = 550U;
volatile uint16_t g_usDebugMotor2Pwm = 550U;
volatile float g_fDebugChassisVy = DEBUG_CHASSIS_TEST_MOVE_SPEED;
volatile float g_fDebugChassisOmega = 0.0f;
volatile float g_fDebugImuRotateAngleDeg = 90.0f;
volatile uint8_t g_ucDebugChassisTestState = 0U;

static void vDebugModeExit(void)
{
    /* 先取消可能仍在运行的定角旋转，再清空速度环历史输出。 */
    vChassisImuRotateCancel();
    vMotorControlSetEnable(1U);
    vMotorControlStop();
}

/// @brief      更新底盘直线前进测试
/// @note       上电或重新进入该模式后先停车指定时间，再持续写入前进目标。
static void vDebugChassisForwardTestUpdate(uint32_t ulStateStartTick)
{
    uint32_t ulNow = osKernelGetTickCount();
    uint32_t ulElapsed = ulNow - ulStateStartTick;

    if (ulElapsed < DEBUG_CHASSIS_TEST_STOP_TIME_MS)
    {
        vChassisStop();
        g_ucDebugChassisTestState = 0U;
    }
    else
    {
        vChassisSetSpeed(g_fDebugChassisVy, g_fDebugChassisOmega);
        g_ucDebugChassisTestState = 1U;
    }
}

void vDebugTask(void *pvParameters)
{
    emDebugModeTdf emLastMode = emDebugModeNone;
    uint32_t ulStateStartTick = osKernelGetTickCount();
    uint32_t ulWakeTick = osKernelGetTickCount();

    (void)pvParameters;
    for (;;)
    {
        emDebugModeTdf emMode = g_emDebugMode;

        if (emMode != emLastMode)
        {
            vDebugModeExit();
            ulStateStartTick = osKernelGetTickCount();
            if (emMode == emDebugModeImuRotate)
            {
                vChassisImuRotateStart(g_fDebugImuRotateAngleDeg);
            }
            emLastMode = emMode;
        }
        switch (emMode)
        {
            case emDebugModeMotorOpenLoop:
                vMotorControlSetEnable(0U);
                vDcMotorSetSpeed(DC_MOTOR1, g_usDebugMotor1Pwm);
                vDcMotorSetSpeed(DC_MOTOR2, g_usDebugMotor2Pwm);
                break;
            case emDebugModeMotorSpeedLoop:
                vMotorControlSetEnable(1U);
                vMotorControlSetTargetRpm(DC_MOTOR1,
                    g_fDebugMotor1TargetRpm);
                vMotorControlSetTargetRpm(DC_MOTOR2,
                    g_fDebugMotor2TargetRpm);
                break;
            case emDebugModeChassis:
                vDebugChassisForwardTestUpdate(ulStateStartTick);
                break;
            case emDebugModeImuRotate:
                /* 若进入模式时IMU尚未收到数据，收到首帧后自动重新尝试。 */
                if (emChassisImuRotateGetState() == emChassisImuRotateNoImu)
                {
                    vChassisImuRotateStart(g_fDebugImuRotateAngleDeg);
                }
                break;
            case emDebugModeEncoder:
            case emDebugModeImu:
            case emDebugModeNone:
            default:
                break;
        }
        ulWakeTick += MOTOR_SAMPLE_TIME;
        (void)osDelayUntil(ulWakeTick);
    }
}
