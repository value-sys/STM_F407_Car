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
#include "line_track.h"
#include "project_config.h"
#include "ui_task.h"

/* 当前固件默认用于位置-速度串级PID旋转测试。 */
volatile emDebugModeTdf g_emDebugMode = emDebugModeGrayLineTrack;
volatile float g_fDebugMotor1TargetRpm = 100.0f;
volatile float g_fDebugMotor2TargetRpm = 100.0f;
volatile uint16_t g_usDebugMotor1Pwm = 550U;
volatile uint16_t g_usDebugMotor2Pwm = 550U;
volatile float g_fDebugChassisVy = DEBUG_CHASSIS_TEST_MOVE_SPEED;
volatile float g_fDebugChassisOmega = 0.0f;
volatile float g_fDebugImuRotateAngleDeg = 90.0f;
volatile float g_fDebugCascadeRotateRadiusMm =
    DEBUG_CASCADE_ROTATE_RADIUS_MM;
volatile float g_fDebugCascadeRotateAngleDeg =
    DEBUG_CASCADE_ROTATE_ANGLE_DEG;
volatile float g_fDebugCascadeRotateOmegaRadS =
    DEBUG_CASCADE_ROTATE_OMEGA_RAD_S;
volatile uint8_t g_ucDebugChassisTestState = 0U;

static void vDebugModeExit(void)
{
    /* 先取消可能仍在运行的定角旋转，再清空速度环历史输出。 */
    vChassisImuRotateCancel();
    vChassisCascadeRotateCancel();
    vLineTrackStop();
    vMotorControlSetEnable(1U);
    vMotorControlStop();
}

/// @brief      更新底盘直线前进测试
/// @note       上电或重新进入该模式后先停车指定时间，再持续写入前进目标。
typedef enum
{
    emChassisTestStopBeforeForward = 0U,
    emChassisTestForward,
    emChassisTestStopBeforeBackward,
    emChassisTestBackward,
    emChassisTestStateMax
} emChassisTestStateTdf;

/// @brief      更新底盘前进/停车/后退测试状态机
/// @note       每次切换方向前先停车，避免直接反向给电机造成冲击。
static void vDebugChassisTestUpdate(uint32_t *pulStateStartTick,
    emChassisTestStateTdf *pemState)
{
    uint32_t ulNow = osKernelGetTickCount();
    uint32_t ulElapsed = ulNow - *pulStateStartTick;

    if (((*pemState == emChassisTestStopBeforeForward) ||
         (*pemState == emChassisTestStopBeforeBackward)) &&
        (ulElapsed >= DEBUG_CHASSIS_TEST_STOP_TIME_MS))
    {
        *pulStateStartTick = ulNow;
        *pemState = (emChassisTestStateTdf)((uint8_t)(*pemState) + 1U);
    }
    else if (((*pemState == emChassisTestForward) ||
              (*pemState == emChassisTestBackward)) &&
             (ulElapsed >= DEBUG_CHASSIS_TEST_RUN_TIME_MS))
    {
        *pulStateStartTick = ulNow;
        *pemState = (emChassisTestStateTdf)((uint8_t)(*pemState) + 1U);
        if (*pemState >= emChassisTestStateMax)
        {
            *pemState = emChassisTestStopBeforeForward;
        }
    }

    switch (*pemState)
    {
        case emChassisTestForward:
            vChassisMove(DEBUG_CHASSIS_TEST_MOVE_SPEED);
            break;

        case emChassisTestBackward:
            vChassisMove(-DEBUG_CHASSIS_TEST_MOVE_SPEED);
            break;

        case emChassisTestStopBeforeForward:
        case emChassisTestStopBeforeBackward:
        default:
            vChassisStop();
            break;
    }

    /* 0/2表示停车阶段，1/3表示前进/后退阶段。 */
    g_ucDebugChassisTestState = (uint8_t)(*pemState);
}

void vDebugTask(void *pvParameters)
{
    emDebugModeTdf emLastMode = emDebugModeNone;
    emChassisTestStateTdf emChassisState =
        emChassisTestStopBeforeForward;
    uint32_t ulStateStartTick = osKernelGetTickCount();
    uint32_t ulWakeTick = osKernelGetTickCount();

    (void)pvParameters;
    for (;;)
    {
        if (ucUiRunEnabled() == 0U)
        {
            g_emDebugMode = emDebugModeNone;
            vDebugModeExit();
            ulWakeTick += MOTOR_SAMPLE_TIME;
            (void)osDelayUntil(ulWakeTick);
            continue;
        }

        /* KEY1 starts grayscale-only line tracking in LINE 1 CIRCLE mode. */
        emDebugModeTdf emMode = eUiGetMode() == UI_MODE_LINE_LAP
            ? emDebugModeGrayLineTrack
            : emDebugModeNone;
        g_emDebugMode = emMode;

        if (emMode != emLastMode)
        {
            vDebugModeExit();
            ulStateStartTick = osKernelGetTickCount();
            emChassisState = emChassisTestStopBeforeForward;
            g_ucDebugChassisTestState = (uint8_t)emChassisState;
            if (emMode == emDebugModeImuRotate)
            {
                vChassisImuRotateStart(g_fDebugImuRotateAngleDeg);
            }
            else if (emMode == emDebugModeCascadeRotate)
            {
                vChassisCascadeRotateStartWithSpeed(
                    g_fDebugCascadeRotateRadiusMm,
                    g_fDebugCascadeRotateAngleDeg,
                    g_fDebugCascadeRotateOmegaRadS);
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
                vDebugChassisTestUpdate(&ulStateStartTick, &emChassisState);
                break;
            case emDebugModeLineTrackImu:
                vLineTrackStart();
                break;
            case emDebugModeCurveLineTrackImu:
                vLineTrackStart();
                break;
            case emDebugModeGrayLineTrack:
                vLineTrackStart();
                break;
            case emDebugModeCascadeRotate:
                /* 旋转目标在进入模式时计算一次，完成后由串级PID自动停车。 */
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
            case emDebugModeGrayscale:
            case emDebugModeNone:
            default:
                break;
        }
        ulWakeTick += MOTOR_SAMPLE_TIME;
        (void)osDelayUntil(ulWakeTick);
    }
}
