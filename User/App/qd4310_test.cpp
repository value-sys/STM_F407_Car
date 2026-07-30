#include "qd4310_test.h"

#include "Qd4310Motor.hpp"
#include "cmsis_os2.h"
#include "usart.h"

volatile uint8_t g_qd4310_test_motion_authorized = 0U;
volatile uint8_t g_qd4310_test_action = QD4310_TEST_ACTION_NONE;
volatile float g_qd4310_test_angle_command_deg = 0.0f;
volatile float g_qd4310_test_step_command_deg = 1.0f;

volatile uint8_t g_qd4310_test_online = 0U;
volatile uint8_t g_qd4310_test_enabled = 0U;
volatile uint8_t g_qd4310_test_state_raw = 0U;
volatile float g_qd4310_test_angle_deg = 0.0f;
volatile float g_qd4310_test_speed_rpm = 0.0f;
volatile float g_qd4310_test_current_a = 0.0f;
volatile uint8_t g_qd4310_test_last_status = 0xFFU;
volatile uint8_t g_qd4310_test_last_action = QD4310_TEST_ACTION_NONE;
volatile uint32_t g_qd4310_test_ok_count = 0U;
volatile uint32_t g_qd4310_test_error_count = 0U;

namespace
{
    yuntai::motor::Qd4310Motor g_qd4310_motor;

    void vPublishFeedback()
    {
        const yuntai::motor::Qd4310Feedback &feedback =
            g_qd4310_motor.feedback();

        g_qd4310_test_online = 1U;
        g_qd4310_test_enabled = feedback.enabled ? 1U : 0U;
        g_qd4310_test_state_raw = feedback.state_raw;
        g_qd4310_test_angle_deg = feedback.angle_deg;
        g_qd4310_test_speed_rpm = feedback.speed_rpm;
        g_qd4310_test_current_a = feedback.current_a;
        ++g_qd4310_test_ok_count;
    }

    void vPublishFailure(HAL_StatusTypeDef status)
    {
        g_qd4310_test_online = 0U;
        g_qd4310_test_last_status = static_cast<uint8_t>(status);
        ++g_qd4310_test_error_count;
    }

    void vRecordResult(HAL_StatusTypeDef status, uint8_t action)
    {
        g_qd4310_test_last_action = action;
        g_qd4310_test_last_status = static_cast<uint8_t>(status);
        if (status == HAL_OK)
        {
            vPublishFeedback();
        }
        else
        {
            vPublishFailure(status);
        }
    }

    bool bMotionAuthorized()
    {
        return g_qd4310_test_motion_authorized != 0U;
    }

    HAL_StatusTypeDef eExecuteAction(uint8_t action)
    {
        using yuntai::motor::Qd4310Command;

        switch (action)
        {
            case QD4310_TEST_ACTION_ENABLE:
                return bMotionAuthorized() ? g_qd4310_motor.enable() : HAL_ERROR;
            case QD4310_TEST_ACTION_DISABLE:
                return g_qd4310_motor.disable();
            case QD4310_TEST_ACTION_STOP:
                return g_qd4310_motor.stop();
            case QD4310_TEST_ACTION_SET_ANGLE:
                return bMotionAuthorized()
                    ? g_qd4310_motor.setAngleDeg(g_qd4310_test_angle_command_deg)
                    : HAL_ERROR;
            case QD4310_TEST_ACTION_STEP_POSITIVE:
                return bMotionAuthorized()
                    ? g_qd4310_motor.setStepAngleDeg(g_qd4310_test_step_command_deg)
                    : HAL_ERROR;
            case QD4310_TEST_ACTION_STEP_NEGATIVE:
                return bMotionAuthorized()
                    ? g_qd4310_motor.setStepAngleDeg(-g_qd4310_test_step_command_deg)
                    : HAL_ERROR;
            case QD4310_TEST_ACTION_CLEAR_ERROR:
                return g_qd4310_motor.clearError();
            case QD4310_TEST_ACTION_SET_ZERO:
                return bMotionAuthorized() ? g_qd4310_motor.setZeroPos() : HAL_ERROR;
            case QD4310_TEST_ACTION_NONE:
            default:
                return g_qd4310_motor.sendRaw(Qd4310Command::Nop, 0);
        }
    }
}

void vQd4310TestTask(void *argument)
{
    uint8_t action;
    bool motion_authorized_last = false;

    (void)argument;
    g_qd4310_motor.attach(&huart2, QD4310_TEST_MOTOR_ID);

    /* Force a safe state after an MCU reset before accepting Ozone commands. */
    (void)g_qd4310_motor.stop();
    (void)g_qd4310_motor.disable();

    for (;;)
    {
        const bool motion_authorized = bMotionAuthorized();

        /* Removing authorization is an active safety command, not just a flag change. */
        if (!motion_authorized && motion_authorized_last)
        {
            (void)g_qd4310_motor.stop();
            (void)g_qd4310_motor.disable();
        }
        motion_authorized_last = motion_authorized;

        action = g_qd4310_test_action;
        g_qd4310_test_action = QD4310_TEST_ACTION_NONE;

        const HAL_StatusTypeDef status = eExecuteAction(action);
        vRecordResult(status, action);

        osDelay(QD4310_TEST_PERIOD_MS);
    }
}
