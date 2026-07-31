#include "qd4310_test.h"

#include "cmsis_os2.h"
#include "qd4310.h"
#include "ui_task.h"
#include "usart.h"
#include "vision_task.h"

volatile uint8_t g_qd4310_test_motion_authorized = 0U;
volatile uint8_t g_qd4310_test_manual_mode = QD4310_TEST_MANUAL_MODE_DEFAULT;
volatile uint8_t g_qd4310_test_balance_enabled = 1U;
volatile uint8_t g_qd4310_test_stop_request = 0U;
volatile uint8_t g_qd4310_test_action = QD4310_TEST_ACTION_NONE;
volatile float g_qd4310_test_target_position_mm = QD4310_TEST_TARGET_POSITION_MM;
volatile float g_qd4310_test_initial_angle_deg = QD4310_TEST_INITIAL_ANGLE_DEG;
volatile float g_qd4310_test_step_angle_deg = QD4310_TEST_STEP_ANGLE_DEG;
volatile float g_qd4310_test_deadband_mm = QD4310_TEST_DEADBAND_MM;
volatile int8_t g_qd4310_test_direction_sign = 1;

volatile uint8_t g_qd4310_test_online = 0U;
volatile uint8_t g_qd4310_test_enabled = 0U;
volatile uint8_t g_qd4310_test_state_raw = 0U;
volatile float g_qd4310_test_angle_deg = 0.0f;
volatile float g_qd4310_test_speed_rpm = 0.0f;
volatile float g_qd4310_test_current_a = 0.0f;
volatile float g_qd4310_test_actual_position_mm = 0.0f;
volatile float g_qd4310_test_position_error_mm = 0.0f;
volatile float g_qd4310_test_last_step_deg = 0.0f;
volatile uint8_t g_qd4310_test_angle_limit_blocked = 0U;
volatile uint32_t g_qd4310_test_last_vision_sequence = 0U;
volatile uint32_t g_qd4310_test_last_vision_age_ms = 0U;
volatile uint8_t g_qd4310_test_last_vision_valid = 0U;
volatile uint8_t g_qd4310_test_last_vision_confidence = 0U;
volatile uint8_t g_qd4310_test_state = QD4310_TEST_STATE_WAIT_AUTH;
volatile uint8_t g_qd4310_test_stop_reason = QD4310_TEST_STOP_REASON_NONE;
volatile uint8_t g_qd4310_test_last_status = HAL_OK;
volatile uint8_t g_qd4310_test_last_action = QD4310_TEST_ACTION_NONE;
volatile uint32_t g_qd4310_test_ok_count = 0U;
volatile uint32_t g_qd4310_test_error_count = 0U;
volatile uint32_t g_qd4310_test_step_count = 0U;
volatile uint32_t g_qd4310_test_tx_count = 0U;
volatile uint8_t g_qd4310_test_last_tx_frame[5] = {0U};

namespace
{
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    qd4310_t g_qd4310_motor;
    uint32_t g_last_control_frame_count;
    uint8_t g_have_control_frame;
    uint8_t g_previous_authorized;

    float fAbs(float value)
    {
        return value < 0.0f ? -value : value;
    }

    int8_t cDirectionSign()
    {
        return g_qd4310_test_direction_sign < 0 ? -1 : 1;
    }

    HAL_StatusTypeDef eSendStep(float step_deg);
    void vRecordSend(HAL_StatusTypeDef status, uint8_t action);

    bool bPrepareStep(float requested_deg, float *applied_deg)
    {
        if (applied_deg == nullptr || requested_deg != requested_deg)
        {
            return false;
        }

        if (g_qd4310_test_angle_deg < QD4310_TEST_MIN_ANGLE_DEG)
        {
            g_qd4310_test_angle_deg = QD4310_TEST_MIN_ANGLE_DEG;
        }
        else if (g_qd4310_test_angle_deg > QD4310_TEST_MAX_ANGLE_DEG)
        {
            g_qd4310_test_angle_deg = QD4310_TEST_MAX_ANGLE_DEG;
        }

        float step_deg = requested_deg;
        if (step_deg > 360.0f)
        {
            step_deg = 360.0f;
        }
        else if (step_deg < -360.0f)
        {
            step_deg = -360.0f;
        }

        const float next_angle_deg = g_qd4310_test_angle_deg + step_deg;
        if (next_angle_deg > QD4310_TEST_MAX_ANGLE_DEG)
        {
            step_deg = QD4310_TEST_MAX_ANGLE_DEG -
                       g_qd4310_test_angle_deg;
        }
        else if (next_angle_deg < QD4310_TEST_MIN_ANGLE_DEG)
        {
            step_deg = QD4310_TEST_MIN_ANGLE_DEG -
                       g_qd4310_test_angle_deg;
        }

        if (fAbs(step_deg) < 0.0001f)
        {
            *applied_deg = 0.0f;
            return false;
        }

        *applied_deg = step_deg;
        return true;
    }

    bool bSendLimitedStep(float requested_deg, uint8_t action)
    {
        float applied_deg = 0.0f;
        g_qd4310_test_angle_limit_blocked = 0U;

        if (!bPrepareStep(requested_deg, &applied_deg))
        {
            g_qd4310_test_angle_limit_blocked = 1U;
            g_qd4310_test_last_step_deg = 0.0f;
            g_qd4310_test_last_action = action;
            g_qd4310_test_last_status = HAL_OK;
            return false;
        }

        const HAL_StatusTypeDef status = eSendStep(applied_deg);
        g_qd4310_test_last_step_deg = applied_deg;
        if (status == HAL_OK)
        {
            g_qd4310_test_angle_deg += applied_deg;
            ++g_qd4310_test_step_count;
            vRecordSend(status, action);
            return true;
        }

        vRecordSend(status, QD4310_TEST_ACTION_NONE);
        return false;
    }

    void vCopyTxDiagnostics()
    {
        for (uint8_t i = 0U; i < 5U; ++i)
        {
            g_qd4310_test_last_tx_frame[i] = g_qd4310_motor.last_tx_frame[i];
        }
        g_qd4310_test_tx_count = g_qd4310_motor.tx_count;
    }

    HAL_StatusTypeDef eSendEnable()
    {
        return qd4310_send_only(&g_qd4310_motor, QD4310_CMD_ENABLE, 0);
    }

    HAL_StatusTypeDef eSendSpeed(float speed_rpm)
    {
        return qd4310_set_speed_only(&g_qd4310_motor, speed_rpm);
    }

    HAL_StatusTypeDef eSendDisable()
    {
        return qd4310_send_only(&g_qd4310_motor, QD4310_CMD_DISABLE, 0);
    }

    void vStopMotor(uint8_t state, uint8_t reason)
    {
        /* A step-angle command is asynchronous. Clear the active speed
         * command first, then remove torque so the last move is interrupted. */
        const HAL_StatusTypeDef speed_status = eSendSpeed(0.0f);
        const HAL_StatusTypeDef disable_status = eSendDisable();
        const HAL_StatusTypeDef status =
            speed_status != HAL_OK ? speed_status : disable_status;

        g_qd4310_test_motion_authorized = 0U;
        g_qd4310_test_balance_enabled = 0U;
        g_qd4310_test_enabled = 0U;
        g_qd4310_test_last_step_deg = 0.0f;
        g_qd4310_test_state = state;
        if (reason != QD4310_TEST_STOP_REASON_AUTH_LOST ||
            g_qd4310_test_stop_reason == QD4310_TEST_STOP_REASON_NONE)
        {
            g_qd4310_test_stop_reason = reason;
        }
        vRecordSend(status, QD4310_TEST_ACTION_DISABLE);
    }

    HAL_StatusTypeDef eSendStep(float step_deg)
    {
        return qd4310_set_step_angle_only(&g_qd4310_motor,
                                          step_deg * kDegToRad);
    }

    void vRecordSend(HAL_StatusTypeDef status, uint8_t action)
    {
        g_qd4310_test_last_status = static_cast<uint8_t>(status);
        g_qd4310_test_last_action = action;
        vCopyTxDiagnostics();
        if (status == HAL_OK)
        {
            g_qd4310_test_online = 1U;
            ++g_qd4310_test_ok_count;
        }
        else
        {
            g_qd4310_test_online = 0U;
            ++g_qd4310_test_error_count;
            g_qd4310_test_state = QD4310_TEST_STATE_ERROR;
        }
    }

    void vRefreshVisionDiagnostics(const stVisionBallData &data,
                                   uint32_t now)
    {
        g_qd4310_test_actual_position_mm =
            static_cast<float>(data.ball_position_mm);
        g_qd4310_test_position_error_mm =
            g_qd4310_test_target_position_mm -
            g_qd4310_test_actual_position_mm;
        g_qd4310_test_last_vision_sequence = data.sequence;
        g_qd4310_test_last_vision_age_ms = now - data.receive_tick_ms;
        g_qd4310_test_last_vision_valid = data.valid;
        g_qd4310_test_last_vision_confidence = data.confidence;
    }

    void vInitializeMotor()
    {
        /* No motor feedback is used. Initialization therefore starts a new
         * software coordinate system and assumes the mechanism is physically
         * at its mechanical 0 deg (lowest position). */
        g_qd4310_test_angle_deg = QD4310_TEST_MIN_ANGLE_DEG;
        g_qd4310_test_angle_limit_blocked = 0U;
        g_qd4310_test_stop_reason = QD4310_TEST_STOP_REASON_NONE;
        g_have_control_frame = 0U;
        g_last_control_frame_count = 0U;

        if (g_qd4310_test_motion_authorized == 0U)
        {
            return;
        }

        const HAL_StatusTypeDef enable_status = eSendEnable();
        vRecordSend(enable_status, QD4310_TEST_ACTION_ENABLE);
        if (enable_status != HAL_OK)
        {
            return;
        }

        osDelay(QD4310_TEST_PERIOD_MS);

        if (g_qd4310_test_motion_authorized == 0U)
        {
            vStopMotor(QD4310_TEST_STATE_STOPPED,
                       QD4310_TEST_STOP_REASON_AUTH_LOST);
            return;
        }

        float init_angle = fAbs(g_qd4310_test_initial_angle_deg);
        if (init_angle > QD4310_TEST_MAX_ANGLE_DEG)
        {
            init_angle = QD4310_TEST_MAX_ANGLE_DEG;
        }

        /* Mechanical zero is defined as 0 deg. Initialization always moves
         * counterclockwise, independent of the visual correction sign. */
        if (bSendLimitedStep(init_angle, QD4310_TEST_ACTION_RESTART_INIT))
        {
            if (g_qd4310_test_motion_authorized != 0U)
            {
                g_qd4310_test_enabled = 1U;
                g_qd4310_test_state = QD4310_TEST_STATE_WAIT_VISION;
            }
            else
            {
                vStopMotor(QD4310_TEST_STATE_STOPPED,
                           QD4310_TEST_STOP_REASON_AUTH_LOST);
            }
        }
    }

    void vRunBalanceControl()
    {
        stVisionBallData data;
        const uint32_t now = HAL_GetTick();
        const uint8_t have_vision_data = ucVisionGetLatest(&data);

        /* Expose the last received coordinate before applying control gates. */
        if (have_vision_data != 0U)
        {
            vRefreshVisionDiagnostics(data, now);
        }

        /* During bring-up, every CRC-valid frame is a control sample. */
        if (have_vision_data == 0U)
        {
            g_qd4310_test_last_step_deg = 0.0f;
            g_qd4310_test_state = QD4310_TEST_STATE_WAIT_VISION;
            return;
        }

        if (g_have_control_frame != 0U &&
            data.receive_count == g_last_control_frame_count)
        {
            return;
        }

        g_last_control_frame_count = data.receive_count;
        g_have_control_frame = 1U;

        const float error = g_qd4310_test_position_error_mm;
        const float deadband = fAbs(g_qd4310_test_deadband_mm);
        float step_deg = 0.0f;

        if (error > deadband)
        {
            step_deg = static_cast<float>(cDirectionSign()) *
                       fAbs(g_qd4310_test_step_angle_deg);
        }
        else if (error < -deadband)
        {
            step_deg = -static_cast<float>(cDirectionSign()) *
                       fAbs(g_qd4310_test_step_angle_deg);
        }

        if (step_deg == 0.0f)
        {
            g_qd4310_test_last_step_deg = 0.0f;
            g_qd4310_test_angle_limit_blocked = 0U;
            g_qd4310_test_state = QD4310_TEST_STATE_BALANCE_RUN;
            return;
        }

        const uint8_t action = step_deg > 0.0f
                                   ? QD4310_TEST_ACTION_STEP_POSITIVE
                                   : QD4310_TEST_ACTION_STEP_NEGATIVE;
        (void)bSendLimitedStep(step_deg, action);
        if (g_qd4310_test_state != QD4310_TEST_STATE_ERROR)
        {
            g_qd4310_test_state = QD4310_TEST_STATE_BALANCE_RUN;
        }
    }

    void vExecuteAction(uint8_t action)
    {
        HAL_StatusTypeDef status = HAL_OK;

        switch (action)
        {
            case QD4310_TEST_ACTION_ENABLE:
                if (g_qd4310_test_motion_authorized == 0U)
                {
                    vRecordSend(HAL_ERROR, action);
                    return;
                }
                status = eSendEnable();
                if (status == HAL_OK)
                {
                    g_qd4310_test_enabled = 1U;
                }
                break;
            case QD4310_TEST_ACTION_DISABLE:
                vStopMotor(QD4310_TEST_STATE_STOPPED,
                           QD4310_TEST_STOP_REASON_ACTION);
                return;
            case QD4310_TEST_ACTION_STEP_POSITIVE:
                if (g_qd4310_test_motion_authorized == 0U)
                {
                    vRecordSend(HAL_ERROR, action);
                    return;
                }
                (void)bSendLimitedStep(fAbs(g_qd4310_test_step_angle_deg),
                                        action);
                return;
            case QD4310_TEST_ACTION_STEP_NEGATIVE:
                if (g_qd4310_test_motion_authorized == 0U)
                {
                    vRecordSend(HAL_ERROR, action);
                    return;
                }
                (void)bSendLimitedStep(-fAbs(g_qd4310_test_step_angle_deg),
                                        action);
                return;
            case QD4310_TEST_ACTION_RESTART_INIT:
                if (g_qd4310_test_motion_authorized != 0U)
                {
                    vInitializeMotor();
                    return;
                }
                status = HAL_ERROR;
                break;
            case QD4310_TEST_ACTION_RESET_ESTIMATE:
                g_qd4310_test_angle_deg = 0.0f;
                g_have_control_frame = 0U;
                g_last_control_frame_count = 0U;
                break;
            case QD4310_TEST_ACTION_NONE:
            default:
                return;
        }

        vRecordSend(status, action);
    }
}

extern "C" void vQd4310TestRequestStop(void)
{
    /* UI only posts the request. UART access remains owned by the QD4310
     * task, which prevents two RTOS tasks from interleaving frames. */
    const uint8_t was_active =
        (g_qd4310_test_motion_authorized != 0U) ||
        (g_qd4310_test_balance_enabled != 0U) ||
        (g_qd4310_test_enabled != 0U) ||
        (g_qd4310_test_stop_request != 0U);

    g_qd4310_test_motion_authorized = 0U;
    g_qd4310_test_balance_enabled = 0U;
    g_qd4310_test_stop_reason = QD4310_TEST_STOP_REASON_UI;
    if (was_active != 0U)
    {
        g_qd4310_test_stop_request = 1U;
    }
}

void vQd4310TestTask(void *argument)
{
    (void)argument;
    qd4310_init(&g_qd4310_motor, &huart2, QD4310_TEST_MOTOR_ID);
    g_qd4310_test_state = QD4310_TEST_STATE_WAIT_AUTH;

    for (;;)
    {
        stVisionBallData latest_vision_data;
        if (ucVisionGetLatest(&latest_vision_data) != 0U)
        {
            vRefreshVisionDiagnostics(latest_vision_data, HAL_GetTick());
        }

        const uint8_t authorized = g_qd4310_test_motion_authorized;
        const uint8_t action = g_qd4310_test_action;
        const uint8_t stop_requested = g_qd4310_test_stop_request;
        uint8_t action_handled = 0U;

        /* Stop has priority over a queued movement command. */
        if (authorized == 0U || stop_requested != 0U)
        {
            if (action != QD4310_TEST_ACTION_NONE)
            {
                g_qd4310_test_action = QD4310_TEST_ACTION_NONE;
            }
            g_qd4310_test_stop_request = 0U;

            if (g_previous_authorized != 0U ||
                g_qd4310_test_enabled != 0U ||
                stop_requested != 0U)
            {
                vStopMotor(QD4310_TEST_STATE_STOPPED,
                           stop_requested != 0U
                               ? QD4310_TEST_STOP_REASON_UI
                               : QD4310_TEST_STOP_REASON_AUTH_LOST);
            }
            else
            {
                g_qd4310_test_balance_enabled = 0U;
                if (g_qd4310_test_state != QD4310_TEST_STATE_VISION_LOST &&
                    g_qd4310_test_state != QD4310_TEST_STATE_STOPPED)
                {
                    g_qd4310_test_state = QD4310_TEST_STATE_WAIT_AUTH;
                }
            }
            g_previous_authorized = 0U;
        }
        else
        {
            if (action != QD4310_TEST_ACTION_NONE)
            {
                g_qd4310_test_action = QD4310_TEST_ACTION_NONE;
                vExecuteAction(action);
                action_handled = 1U;
            }

            /* vExecuteAction() can revoke authorization, for example for a
             * manual DISABLE action. Re-check it before initialization. */
            if (g_qd4310_test_motion_authorized == 0U)
            {
                g_previous_authorized = 0U;
                osDelay(QD4310_TEST_PERIOD_MS);
                continue;
            }

            if (g_qd4310_test_manual_mode != 0U)
            {
                /* Manual mode deliberately does not initialize or read
                 * vision. Every motor movement must be an explicit action. */
                g_qd4310_test_balance_enabled = 0U;
                if (g_qd4310_test_state != QD4310_TEST_STATE_ERROR)
                {
                    g_qd4310_test_state = QD4310_TEST_STATE_MANUAL_TEST;
                }
            }
            else if (g_previous_authorized == 0U)
            {
                g_qd4310_test_balance_enabled = 1U;
                g_qd4310_test_state = QD4310_TEST_STATE_INITIALIZING;
                vInitializeMotor();
                action_handled = 1U;
            }
            g_previous_authorized =
                g_qd4310_test_motion_authorized != 0U ? 1U : 0U;

            if (g_qd4310_test_manual_mode == 0U &&
                action_handled == 0U &&
                g_qd4310_test_balance_enabled != 0U &&
                g_qd4310_test_state != QD4310_TEST_STATE_INITIALIZING &&
                g_qd4310_test_state != QD4310_TEST_STATE_ERROR)
            {
                vRunBalanceControl();
            }
        }

        osDelay(QD4310_TEST_PERIOD_MS);
    }
}
