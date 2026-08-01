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
volatile uint8_t g_qd4310_test_manual_angle_follow = 1U;
volatile uint8_t g_qd4310_test_init_ramp_enabled =
    QD4310_TEST_INIT_RAMP_DEFAULT;
volatile uint8_t g_qd4310_test_balance_map_enabled =
    QD4310_TEST_BALANCE_MAP_DEFAULT;
volatile float g_qd4310_test_manual_target_angle_deg =
    QD4310_TEST_MANUAL_TARGET_ANGLE_DEG;
volatile float g_qd4310_test_init_ramp_step_deg =
    QD4310_TEST_INIT_RAMP_STEP_DEG;
volatile uint32_t g_qd4310_test_init_ramp_delay_ms =
    QD4310_TEST_INIT_RAMP_DELAY_MS;
volatile float g_qd4310_test_balance_map_position_mm[QD4310_TEST_BALANCE_MAP_SIZE] =
{
    25.0f, 50.0f, 75.0f, 100.0f, 125.0f,
    150.0f, 175.0f, 200.0f, 225.0f,
};
volatile float g_qd4310_test_balance_map_angle_deg[QD4310_TEST_BALANCE_MAP_SIZE] =
{
    5.0f, 5.0f, 5.0f, 5.0f, 5.0f,
    5.0f, 5.0f, 5.0f, 5.0f,
};
volatile float g_qd4310_test_target_position_mm = QD4310_TEST_TARGET_POSITION_MM;
volatile uint8_t g_qd4310_test_position_sequence_enabled = 0U;
volatile uint8_t g_qd4310_test_position_sequence_phase = 0U;
volatile uint8_t g_qd4310_test_position_sequence_wait_center_enabled = 0U;
volatile float g_qd4310_test_position_sequence_center_target_mm =
    QD4310_TEST_POSITION_SEQUENCE_CENTER_TARGET_MM;
volatile float g_qd4310_test_position_sequence_first_target_mm =
    QD4310_TEST_POSITION_SEQUENCE_FIRST_TARGET_MM;
volatile float g_qd4310_test_position_sequence_second_target_mm =
    QD4310_TEST_POSITION_SEQUENCE_SECOND_TARGET_MM;
volatile float g_qd4310_test_position_sequence_tolerance_mm =
    QD4310_TEST_POSITION_SEQUENCE_TOLERANCE_MM;
volatile uint32_t g_qd4310_test_position_sequence_confirm_frames =
    QD4310_TEST_POSITION_SEQUENCE_CONFIRM_FRAMES;
volatile uint32_t g_qd4310_test_position_sequence_confirm_count = 0U;
volatile uint8_t g_qd4310_test_position_sequence_in_range = 0U;
volatile uint8_t g_qd4310_test_position_sequence_first_target_reached = 0U;
volatile uint8_t g_qd4310_test_position_sequence_timer_started = 0U;
volatile uint8_t g_qd4310_test_position_sequence_completed = 0U;
volatile float g_qd4310_test_initial_angle_deg = QD4310_TEST_INITIAL_ANGLE_DEG;
volatile float g_qd4310_test_step_angle_deg = QD4310_TEST_STEP_ANGLE_DEG;
volatile float g_qd4310_test_deadband_mm = QD4310_TEST_DEADBAND_MM;
volatile float g_qd4310_test_kp_deg_per_mm = QD4310_TEST_KP_DEG_PER_MM;
volatile float g_qd4310_test_kd_deg_per_mm_s = QD4310_TEST_KD_DEG_PER_MM_S;
volatile float g_qd4310_test_ki_deg_per_mm_s = QD4310_TEST_KI_DEG_PER_MM_S;
volatile float g_qd4310_test_max_integral_deg =
    QD4310_TEST_MAX_INTEGRAL_DEG;
volatile float g_qd4310_test_integral_velocity_limit_mm_s =
    QD4310_TEST_INTEGRAL_VEL_MM_S;
volatile float g_qd4310_test_max_correction_deg = QD4310_TEST_MAX_CORRECTION_DEG;
volatile float g_qd4310_test_max_angle_delta_deg = QD4310_TEST_MAX_ANGLE_DELTA_DEG;
volatile float g_qd4310_test_max_i_angle_delta_deg =
    QD4310_TEST_MAX_I_ANGLE_DELTA_DEG;
volatile float g_qd4310_test_velocity_lpf_alpha = QD4310_TEST_VELOCITY_LPF_ALPHA;
volatile int8_t g_qd4310_test_direction_sign = 1;
volatile float g_qd4310_test_stuck_error_mm = QD4310_TEST_STUCK_ERROR_MM;
volatile float g_qd4310_test_stuck_velocity_mm_s =
    QD4310_TEST_STUCK_VELOCITY_MM_S;
volatile float g_qd4310_test_stuck_release_velocity_mm_s =
    QD4310_TEST_STUCK_RELEASE_VELOCITY_MM_S;
volatile uint32_t g_qd4310_test_stuck_confirm_ms =
    QD4310_TEST_STUCK_CONFIRM_MS;
volatile uint32_t g_qd4310_test_i_pulse_duration_ms =
    QD4310_TEST_I_PULSE_DURATION_MS;
volatile uint8_t g_qd4310_test_feedback_poll_enabled =
    QD4310_TEST_FEEDBACK_POLL_DEFAULT;

volatile uint8_t g_qd4310_test_online = 0U;
volatile uint8_t g_qd4310_test_enabled = 0U;
volatile uint8_t g_qd4310_test_feedback_enabled = 0U;
volatile uint8_t g_qd4310_test_state_raw = 0U;
volatile uint8_t g_qd4310_test_feedback_valid = 0U;
volatile float g_qd4310_test_feedback_angle_deg = 0.0f;
volatile uint32_t g_qd4310_test_feedback_count = 0U;
volatile uint32_t g_qd4310_test_feedback_error_count = 0U;
volatile uint32_t g_qd4310_test_last_feedback_tick_ms = 0U;
volatile float g_qd4310_test_angle_deg = 0.0f;
volatile float g_qd4310_test_speed_rpm = 0.0f;
volatile float g_qd4310_test_current_a = 0.0f;
volatile float g_qd4310_test_actual_position_mm = 0.0f;
volatile float g_qd4310_test_position_error_mm = 0.0f;
volatile float g_qd4310_test_effective_error_mm = 0.0f;
volatile float g_qd4310_test_ball_velocity_mm_s = 0.0f;
volatile float g_qd4310_test_filtered_velocity_mm_s = 0.0f;
volatile float g_qd4310_test_p_output_deg = 0.0f;
volatile float g_qd4310_test_d_output_deg = 0.0f;
volatile float g_qd4310_test_i_output_deg = 0.0f;
volatile float g_qd4310_test_pd_output_deg = 0.0f;
volatile float g_qd4310_test_pid_output_deg = 0.0f;
volatile uint8_t g_qd4310_test_integral_enabled = 0U;
volatile uint8_t g_qd4310_test_stuck_active = 0U;
volatile uint8_t g_qd4310_test_i_pulse_active = 0U;
volatile uint8_t g_qd4310_test_pd_delta_limit_blocked = 0U;
volatile uint8_t g_qd4310_test_i_delta_limit_blocked = 0U;
volatile uint32_t g_qd4310_test_i_pulse_remaining_ms = 0U;
volatile uint32_t g_qd4310_test_i_pulse_trigger_count = 0U;
volatile float g_qd4310_test_balance_base_angle_deg =
    QD4310_TEST_INITIAL_ANGLE_DEG;
volatile float g_qd4310_test_unclamped_target_angle_deg = 0.0f;
volatile float g_qd4310_test_raw_target_angle_deg = 0.0f;
volatile float g_qd4310_test_target_angle_deg = 0.0f;
volatile float g_qd4310_test_motor_command_angle_deg = 0.0f;
volatile float g_qd4310_test_last_angle_delta_deg = 0.0f;
volatile float g_qd4310_test_last_step_deg = 0.0f;
volatile uint8_t g_qd4310_test_angle_limit_blocked = 0U;
volatile uint8_t g_qd4310_test_deadband_active = 0U;
volatile uint8_t g_qd4310_test_output_limit_blocked = 0U;
volatile uint8_t g_qd4310_test_delta_limit_blocked = 0U;
volatile uint32_t g_qd4310_test_control_dt_ms = 0U;
volatile uint32_t g_qd4310_test_control_sample_count = 0U;
volatile uint32_t g_qd4310_test_wait_vision_count = 0U;
volatile uint32_t g_qd4310_test_same_frame_skip_count = 0U;
volatile uint32_t g_qd4310_test_last_control_tick_ms = 0U;
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
    uint8_t g_have_velocity_sample;
    float g_last_position_mm;
    uint32_t g_last_position_tick_ms;
    uint8_t g_previous_authorized;
    uint8_t g_previous_position_sequence_enabled;
    uint8_t g_have_manual_angle_command;
    float g_last_manual_angle_command_deg;
    uint32_t g_stuck_since_tick_ms;
    uint32_t g_i_pulse_until_tick_ms;
    int8_t g_i_pulse_direction_sign;

    float fAbs(float value)
    {
        return value < 0.0f ? -value : value;
    }

    int8_t cDirectionSign()
    {
        return g_qd4310_test_direction_sign < 0 ? -1 : 1;
    }

    HAL_StatusTypeDef eSendStep(float step_deg);
    HAL_StatusTypeDef eSendAngle(float angle_deg);
    void vRecordSend(HAL_StatusTypeDef status, uint8_t action);

    void vCopyFeedbackDiagnostics()
    {
        g_qd4310_test_feedback_enabled =
            g_qd4310_motor.enabled ? 1U : 0U;
        g_qd4310_test_enabled = g_qd4310_test_feedback_enabled;
        g_qd4310_test_state_raw = g_qd4310_motor.state_raw;
        g_qd4310_test_current_a = g_qd4310_motor.current_a;
        g_qd4310_test_speed_rpm = g_qd4310_motor.speed_rpm;
        g_qd4310_test_feedback_angle_deg =
            g_qd4310_motor.angle_rad * 180.0f / 3.14159265358979323846f;
        g_qd4310_test_feedback_valid = 1U;
        ++g_qd4310_test_feedback_count;
        g_qd4310_test_last_feedback_tick_ms = HAL_GetTick();
    }

    float fClamp(float value, float min_value, float max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }
        if (value > max_value)
        {
            return max_value;
        }
        return value;
    }

    float fWrap360(float value)
    {
        while (value >= 360.0f)
        {
            value -= 360.0f;
        }
        while (value < 0.0f)
        {
            value += 360.0f;
        }
        return value;
    }

    float fLookupBalanceAngle(float position_mm)
    {
        if (QD4310_TEST_BALANCE_MAP_SIZE == 0U)
        {
            return g_qd4310_test_initial_angle_deg;
        }

        float left_position_mm = g_qd4310_test_balance_map_position_mm[0U];
        float left_angle_deg = g_qd4310_test_balance_map_angle_deg[0U];

        if (position_mm <= left_position_mm)
        {
            return left_angle_deg;
        }

        for (uint8_t i = 1U; i < QD4310_TEST_BALANCE_MAP_SIZE; ++i)
        {
            const float right_position_mm =
                g_qd4310_test_balance_map_position_mm[i];
            const float right_angle_deg =
                g_qd4310_test_balance_map_angle_deg[i];

            if (position_mm <= right_position_mm)
            {
                const float span_mm = right_position_mm - left_position_mm;
                if (fAbs(span_mm) < 0.001f)
                {
                    return right_angle_deg;
                }

                const float ratio =
                    (position_mm - left_position_mm) / span_mm;
                return left_angle_deg +
                       ratio * (right_angle_deg - left_angle_deg);
            }

            left_position_mm = right_position_mm;
            left_angle_deg = right_angle_deg;
        }

        return left_angle_deg;
    }

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

    bool bSendLimitedAngle(float requested_deg, uint8_t action)
    {
        if (requested_deg != requested_deg)
        {
            return false;
        }

        const float next_angle_deg = fClamp(requested_deg,
                                            QD4310_TEST_MIN_ANGLE_DEG,
                                            QD4310_TEST_MAX_ANGLE_DEG);
        const float angle_delta = next_angle_deg - g_qd4310_test_angle_deg;
        g_qd4310_test_raw_target_angle_deg = requested_deg;
        g_qd4310_test_target_angle_deg = next_angle_deg;
        g_qd4310_test_last_angle_delta_deg = angle_delta;
        g_qd4310_test_last_step_deg = angle_delta;
        g_qd4310_test_angle_limit_blocked =
            next_angle_deg != requested_deg ? 1U : 0U;

        const HAL_StatusTypeDef status = eSendAngle(next_angle_deg);
        if (status == HAL_OK)
        {
            g_qd4310_test_angle_deg = next_angle_deg;
            ++g_qd4310_test_step_count;
            vRecordSend(status, action);
            return true;
        }

        vRecordSend(status, QD4310_TEST_ACTION_NONE);
        return false;
    }

    bool bRampToAngle(float target_angle_deg, uint8_t action)
    {
        const float target_deg = fClamp(target_angle_deg,
                                        QD4310_TEST_MIN_ANGLE_DEG,
                                        QD4310_TEST_MAX_ANGLE_DEG);
        float step_deg = fAbs(g_qd4310_test_init_ramp_step_deg);
        uint32_t delay_ms = g_qd4310_test_init_ramp_delay_ms;

        if (step_deg < 0.1f)
        {
            step_deg = 0.1f;
        }
        else if (step_deg > 20.0f)
        {
            step_deg = 20.0f;
        }

        if (delay_ms > 500U)
        {
            delay_ms = 500U;
        }

        while (g_qd4310_test_motion_authorized != 0U &&
               g_qd4310_test_state != QD4310_TEST_STATE_ERROR &&
               fAbs(target_deg - g_qd4310_test_angle_deg) >
                   QD4310_TEST_MANUAL_ANGLE_EPS_DEG)
        {
            const float delta_deg =
                fClamp(target_deg - g_qd4310_test_angle_deg,
                       -step_deg,
                       step_deg);
            if (!bSendLimitedAngle(g_qd4310_test_angle_deg + delta_deg,
                                   action))
            {
                return false;
            }

            if (delay_ms > 0U &&
                fAbs(target_deg - g_qd4310_test_angle_deg) >
                    QD4310_TEST_MANUAL_ANGLE_EPS_DEG)
            {
                osDelay(delay_ms);
            }
        }

        return g_qd4310_test_motion_authorized != 0U &&
               g_qd4310_test_state != QD4310_TEST_STATE_ERROR;
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
        return g_qd4310_test_feedback_poll_enabled != 0U
                   ? qd4310_enable(&g_qd4310_motor)
                   : qd4310_send_only(&g_qd4310_motor, QD4310_CMD_ENABLE, 0);
    }

    HAL_StatusTypeDef eSendSpeed(float speed_rpm)
    {
        return g_qd4310_test_feedback_poll_enabled != 0U
                   ? qd4310_set_speed(&g_qd4310_motor, speed_rpm)
                   : qd4310_set_speed_only(&g_qd4310_motor, speed_rpm);
    }

    HAL_StatusTypeDef eSendDisable()
    {
        return g_qd4310_test_feedback_poll_enabled != 0U
                   ? qd4310_disable(&g_qd4310_motor)
                   : qd4310_send_only(&g_qd4310_motor, QD4310_CMD_DISABLE, 0);
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
        g_qd4310_test_i_output_deg = 0.0f;
        g_qd4310_test_pid_output_deg = 0.0f;
        g_qd4310_test_integral_enabled = 0U;
        g_qd4310_test_stuck_active = 0U;
        g_qd4310_test_i_pulse_active = 0U;
        g_qd4310_test_i_pulse_remaining_ms = 0U;
        g_qd4310_test_position_sequence_phase = 0U;
        g_qd4310_test_position_sequence_confirm_count = 0U;
        g_qd4310_test_position_sequence_in_range = 0U;
        g_qd4310_test_position_sequence_first_target_reached = 0U;
        g_qd4310_test_position_sequence_timer_started = 0U;
        g_qd4310_test_position_sequence_completed = 0U;
        g_previous_position_sequence_enabled = 0U;
        g_stuck_since_tick_ms = 0U;
        g_i_pulse_until_tick_ms = 0U;
        g_i_pulse_direction_sign = 0;
        g_qd4310_test_state = state;
        g_have_manual_angle_command = 0U;
        if (reason != QD4310_TEST_STOP_REASON_AUTH_LOST ||
            g_qd4310_test_stop_reason == QD4310_TEST_STOP_REASON_NONE)
        {
            g_qd4310_test_stop_reason = reason;
        }
        vRecordSend(status, QD4310_TEST_ACTION_DISABLE);
    }

    HAL_StatusTypeDef eSendStep(float step_deg)
    {
        return g_qd4310_test_feedback_poll_enabled != 0U
                   ? qd4310_set_step_angle(&g_qd4310_motor,
                                           step_deg * kDegToRad)
                   : qd4310_set_step_angle_only(&g_qd4310_motor,
                                                step_deg * kDegToRad);
    }

    HAL_StatusTypeDef eSendAngle(float angle_deg)
    {
        const float motor_angle_deg = fWrap360(angle_deg);
        g_qd4310_test_motor_command_angle_deg = motor_angle_deg;
        return g_qd4310_test_feedback_poll_enabled != 0U
                   ? qd4310_set_angle(&g_qd4310_motor,
                                      motor_angle_deg * kDegToRad)
                   : qd4310_set_angle_only(&g_qd4310_motor,
                                           motor_angle_deg * kDegToRad);
    }

    void vRecordSend(HAL_StatusTypeDef status, uint8_t action)
    {
        g_qd4310_test_last_status = static_cast<uint8_t>(status);
        g_qd4310_test_last_action = action;
        if (g_qd4310_test_feedback_poll_enabled != 0U)
        {
            if (status == HAL_OK)
            {
                vCopyFeedbackDiagnostics();
            }
            else
            {
                g_qd4310_test_feedback_valid = 0U;
                ++g_qd4310_test_feedback_error_count;
            }
        }
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

    void vUpdatePositionSequenceSwitch()
    {
        const uint8_t enabled =
            g_qd4310_test_position_sequence_enabled != 0U ? 1U : 0U;

        if (enabled != 0U &&
            (g_previous_position_sequence_enabled == 0U ||
             g_qd4310_test_position_sequence_phase == 0U))
        {
            g_qd4310_test_position_sequence_phase =
                g_qd4310_test_position_sequence_wait_center_enabled != 0U
                    ? 3U
                    : 1U;
            g_qd4310_test_position_sequence_confirm_count = 0U;
            g_qd4310_test_position_sequence_in_range = 0U;
            g_qd4310_test_position_sequence_first_target_reached = 0U;
            g_qd4310_test_position_sequence_timer_started = 0U;
            g_qd4310_test_position_sequence_completed = 0U;
            g_qd4310_test_target_position_mm =
                g_qd4310_test_position_sequence_phase == 3U
                    ? g_qd4310_test_position_sequence_center_target_mm
                    : g_qd4310_test_position_sequence_first_target_mm;
            g_qd4310_test_position_error_mm =
                g_qd4310_test_target_position_mm -
                g_qd4310_test_actual_position_mm;
        }
        else if (enabled == 0U &&
                 g_previous_position_sequence_enabled != 0U)
        {
            g_qd4310_test_position_sequence_phase = 0U;
            g_qd4310_test_position_sequence_confirm_count = 0U;
            g_qd4310_test_position_sequence_in_range = 0U;
            g_qd4310_test_position_sequence_first_target_reached = 0U;
            g_qd4310_test_position_sequence_timer_started = 0U;
            g_qd4310_test_position_sequence_completed = 0U;
        }

        g_previous_position_sequence_enabled = enabled;
    }

    void vUpdatePositionSequenceFrame(float actual_position_mm)
    {
        if (g_qd4310_test_position_sequence_enabled == 0U ||
            g_qd4310_test_position_sequence_phase == 0U)
        {
            return;
        }

        const float tolerance = fAbs(
            g_qd4310_test_position_sequence_tolerance_mm);
        const uint8_t phase = g_qd4310_test_position_sequence_phase;
        const float target =
            phase == 3U
                ? g_qd4310_test_position_sequence_center_target_mm
                : (phase == 1U
                       ? g_qd4310_test_position_sequence_first_target_mm
                       : g_qd4310_test_position_sequence_second_target_mm);
        g_qd4310_test_target_position_mm = target;
        const bool in_range = fAbs(actual_position_mm - target) <= tolerance;
        g_qd4310_test_position_sequence_in_range = in_range ? 1U : 0U;

        if (phase == 3U)
        {
            if (in_range)
            {
                g_qd4310_test_position_sequence_phase = 1U;
                g_qd4310_test_position_sequence_timer_started = 1U;
                g_qd4310_test_position_sequence_confirm_count = 0U;
                g_qd4310_test_position_sequence_in_range = 0U;
                g_qd4310_test_target_position_mm =
                    g_qd4310_test_position_sequence_first_target_mm;
            }
        }
        else if (phase == 1U || phase == 2U)
        {
            if (in_range)
            {
                ++g_qd4310_test_position_sequence_confirm_count;
            }
            else
            {
                g_qd4310_test_position_sequence_confirm_count = 0U;
            }

            uint32_t required_frames =
                g_qd4310_test_position_sequence_confirm_frames;
            if (required_frames == 0U)
            {
                required_frames = 1U;
            }

            if (g_qd4310_test_position_sequence_confirm_count >=
                required_frames)
            {
                if (phase == 1U)
                {
                    g_qd4310_test_position_sequence_phase = 2U;
                    g_qd4310_test_position_sequence_first_target_reached =
                        1U;
                    g_qd4310_test_position_sequence_confirm_count = 0U;
                    g_qd4310_test_position_sequence_in_range = 0U;
                    g_qd4310_test_target_position_mm =
                        g_qd4310_test_position_sequence_second_target_mm;
                }
                else
                {
                    g_qd4310_test_position_sequence_phase = 4U;
                    g_qd4310_test_position_sequence_completed = 1U;
                    g_qd4310_test_position_sequence_in_range = 1U;
                }
            }
        }
        else if (phase == 4U)
        {
            g_qd4310_test_target_position_mm =
                g_qd4310_test_position_sequence_second_target_mm;
        }

        g_qd4310_test_position_error_mm =
            g_qd4310_test_target_position_mm - actual_position_mm;
    }

    void vInitializeMotor()
    {
        /* No motor feedback is used. Initialization therefore starts a new
         * software coordinate system and assumes the mechanism is physically
         * at its mechanical 0 deg (lowest position). */
        g_qd4310_test_angle_deg = QD4310_TEST_BOOT_ANGLE_DEG;
        g_qd4310_test_angle_limit_blocked = 0U;
        g_qd4310_test_stop_reason = QD4310_TEST_STOP_REASON_NONE;
        g_qd4310_test_effective_error_mm = 0.0f;
        g_qd4310_test_ball_velocity_mm_s = 0.0f;
        g_qd4310_test_filtered_velocity_mm_s = 0.0f;
        g_qd4310_test_p_output_deg = 0.0f;
        g_qd4310_test_d_output_deg = 0.0f;
        g_qd4310_test_i_output_deg = 0.0f;
        g_qd4310_test_pd_output_deg = 0.0f;
        g_qd4310_test_pid_output_deg = 0.0f;
        g_qd4310_test_integral_enabled = 0U;
        g_qd4310_test_stuck_active = 0U;
        g_qd4310_test_i_pulse_active = 0U;
        g_qd4310_test_i_pulse_remaining_ms = 0U;
        g_qd4310_test_pd_delta_limit_blocked = 0U;
        g_qd4310_test_i_delta_limit_blocked = 0U;
        g_stuck_since_tick_ms = 0U;
        g_i_pulse_until_tick_ms = 0U;
        g_i_pulse_direction_sign = 0;
        g_qd4310_test_balance_base_angle_deg =
            g_qd4310_test_initial_angle_deg;
        g_qd4310_test_unclamped_target_angle_deg =
            g_qd4310_test_initial_angle_deg;
        g_qd4310_test_raw_target_angle_deg = g_qd4310_test_initial_angle_deg;
        g_qd4310_test_target_angle_deg = g_qd4310_test_initial_angle_deg;
        g_qd4310_test_last_angle_delta_deg = 0.0f;
        g_qd4310_test_deadband_active = 0U;
        g_qd4310_test_output_limit_blocked = 0U;
        g_qd4310_test_delta_limit_blocked = 0U;
        g_qd4310_test_control_dt_ms = 0U;
        g_qd4310_test_control_sample_count = 0U;
        g_qd4310_test_wait_vision_count = 0U;
        g_qd4310_test_same_frame_skip_count = 0U;
        g_qd4310_test_last_control_tick_ms = 0U;
        g_have_control_frame = 0U;
        g_last_control_frame_count = 0U;
        g_have_velocity_sample = 0U;
        g_last_position_mm = 0.0f;
        g_last_position_tick_ms = 0U;

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
        g_qd4310_test_enabled = 1U;

        osDelay(QD4310_TEST_PERIOD_MS);

        if (g_qd4310_test_motion_authorized == 0U)
        {
            vStopMotor(QD4310_TEST_STATE_STOPPED,
                       QD4310_TEST_STOP_REASON_AUTH_LOST);
            return;
        }

        const float init_angle = fClamp(g_qd4310_test_initial_angle_deg,
                                        QD4310_TEST_MIN_ANGLE_DEG,
                                        QD4310_TEST_MAX_ANGLE_DEG);
        const bool init_ok =
            g_qd4310_test_init_ramp_enabled != 0U
                ? bRampToAngle(init_angle, QD4310_TEST_ACTION_RESTART_INIT)
                : bSendLimitedAngle(init_angle,
                                    QD4310_TEST_ACTION_RESTART_INIT);

        if (init_ok)
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
        else if (g_qd4310_test_motion_authorized == 0U)
        {
            vStopMotor(QD4310_TEST_STATE_STOPPED,
                       QD4310_TEST_STOP_REASON_AUTH_LOST);
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
            ++g_qd4310_test_wait_vision_count;
            g_qd4310_test_state = QD4310_TEST_STATE_WAIT_VISION;
            return;
        }

        if (g_have_control_frame != 0U &&
            data.receive_count == g_last_control_frame_count)
        {
            ++g_qd4310_test_same_frame_skip_count;
            return;
        }

        g_last_control_frame_count = data.receive_count;
        g_have_control_frame = 1U;
        ++g_qd4310_test_control_sample_count;
        g_qd4310_test_last_control_tick_ms = now;

        const uint8_t sequence_phase_before =
            g_qd4310_test_position_sequence_phase;
        vUpdatePositionSequenceFrame(g_qd4310_test_actual_position_mm);

        /* Static-position mode starts from the calibrated 5 deg angle. Do
         * not steer the ball toward center while waiting for the operator's
         * initial placement at 125 mm. */
        if (sequence_phase_before == 3U &&
            g_qd4310_test_position_sequence_phase == 3U)
        {
            g_qd4310_test_p_output_deg = 0.0f;
            g_qd4310_test_d_output_deg = 0.0f;
            g_qd4310_test_i_output_deg = 0.0f;
            g_qd4310_test_pd_output_deg = 0.0f;
            g_qd4310_test_pid_output_deg = 0.0f;
            g_qd4310_test_integral_enabled = 0U;
            g_qd4310_test_target_angle_deg =
                g_qd4310_test_angle_deg;
            g_qd4310_test_motor_command_angle_deg =
                g_qd4310_test_angle_deg;
            g_qd4310_test_last_angle_delta_deg = 0.0f;
            g_qd4310_test_last_step_deg = 0.0f;
            return;
        }

        const float error = g_qd4310_test_position_error_mm;
        const float deadband = fAbs(g_qd4310_test_deadband_mm);
        const float actual_position_mm = g_qd4310_test_actual_position_mm;
        float effective_error = 0.0f;
        float velocity_mm_s = 0.0f;
        uint32_t dt_ms = QD4310_TEST_PERIOD_MS;
        const uint32_t sample_tick = data.receive_tick_ms;

        if (error > deadband)
        {
            effective_error = error - deadband;
        }
        else if (error < -deadband)
        {
            effective_error = error + deadband;
        }
        g_qd4310_test_effective_error_mm = effective_error;
        g_qd4310_test_deadband_active =
            fAbs(error) <= deadband ? 1U : 0U;

        if (g_have_velocity_sample != 0U &&
            sample_tick != g_last_position_tick_ms)
        {
            dt_ms = sample_tick - g_last_position_tick_ms;
            velocity_mm_s = (actual_position_mm - g_last_position_mm) *
                            1000.0f / static_cast<float>(dt_ms);
        }
        g_have_velocity_sample = 1U;
        g_qd4310_test_control_dt_ms = dt_ms;
        g_last_position_mm = actual_position_mm;
        g_last_position_tick_ms = sample_tick;

        float alpha = g_qd4310_test_velocity_lpf_alpha;
        alpha = fClamp(alpha, 0.0f, 0.98f);
        g_qd4310_test_ball_velocity_mm_s = velocity_mm_s;
        g_qd4310_test_filtered_velocity_mm_s =
            alpha * g_qd4310_test_filtered_velocity_mm_s +
            (1.0f - alpha) * velocity_mm_s;

        const float control_sign = static_cast<float>(cDirectionSign());
        g_qd4310_test_p_output_deg =
            control_sign * g_qd4310_test_kp_deg_per_mm * effective_error;
        g_qd4310_test_d_output_deg =
            -control_sign * g_qd4310_test_kd_deg_per_mm_s *
            g_qd4310_test_filtered_velocity_mm_s;
        g_qd4310_test_pd_output_deg =
            g_qd4310_test_p_output_deg + g_qd4310_test_d_output_deg;

        /* I is used as a finite breakaway pulse, not as a continuously
         * accumulating integral. PD remains the normal smooth controller. */
        const float stuck_error_limit = fAbs(g_qd4310_test_stuck_error_mm);
        const float stuck_velocity_limit =
            fAbs(g_qd4310_test_stuck_velocity_mm_s);
        const float release_velocity_limit = fAbs(
            g_qd4310_test_stuck_release_velocity_mm_s);
        const bool error_is_large = fAbs(error) > stuck_error_limit;
        const bool velocity_is_low =
            fAbs(g_qd4310_test_filtered_velocity_mm_s) <=
            stuck_velocity_limit;

        if (error_is_large && velocity_is_low)
        {
            if (g_stuck_since_tick_ms == 0U)
            {
                g_stuck_since_tick_ms = now;
            }

            g_qd4310_test_stuck_active =
                (now - g_stuck_since_tick_ms >=
                 g_qd4310_test_stuck_confirm_ms)
                    ? 1U
                    : 0U;
        }
        else
        {
            g_stuck_since_tick_ms = 0U;
            g_qd4310_test_stuck_active = 0U;
        }

        if (g_qd4310_test_i_pulse_active != 0U)
        {
            const bool pulse_expired =
                static_cast<int32_t>(now - g_i_pulse_until_tick_ms) >= 0;
            const bool ball_has_started =
                fAbs(g_qd4310_test_filtered_velocity_mm_s) >=
                release_velocity_limit;
            const bool error_reversed =
                (g_i_pulse_direction_sign != 0) &&
                (error * static_cast<float>(g_i_pulse_direction_sign) <
                 0.0f);

            if (pulse_expired || ball_has_started || error_reversed)
            {
                g_qd4310_test_i_pulse_active = 0U;
                g_i_pulse_until_tick_ms = 0U;
                /* Require another full confirmation interval before a
                 * second pulse after any release condition. */
                g_stuck_since_tick_ms = now;
                g_qd4310_test_stuck_active = 0U;
            }
        }

        if (g_qd4310_test_i_pulse_active == 0U &&
            g_qd4310_test_stuck_active != 0U)
        {
            g_i_pulse_direction_sign =
                effective_error >= 0.0f ? 1 : -1;
            g_qd4310_test_i_pulse_active = 1U;
            g_i_pulse_until_tick_ms =
                now + g_qd4310_test_i_pulse_duration_ms;
            ++g_qd4310_test_i_pulse_trigger_count;
        }

        const float max_integral = fAbs(g_qd4310_test_max_integral_deg);
        const float i_pulse_target_deg =
            g_qd4310_test_i_pulse_active != 0U
                ? control_sign *
                      static_cast<float>(g_i_pulse_direction_sign) *
                      max_integral
                : 0.0f;
        const float max_i_delta =
            fAbs(g_qd4310_test_max_i_angle_delta_deg);
        const float requested_i_delta =
            i_pulse_target_deg - g_qd4310_test_i_output_deg;
        const float i_delta = fClamp(requested_i_delta,
                                     -max_i_delta,
                                     max_i_delta);
        g_qd4310_test_i_delta_limit_blocked =
            fAbs(i_delta - requested_i_delta) > 0.0001f ? 1U : 0U;
        g_qd4310_test_i_output_deg += i_delta;
        g_qd4310_test_integral_enabled =
            g_qd4310_test_i_pulse_active != 0U ||
            fAbs(g_qd4310_test_i_output_deg) > 0.0001f ? 1U : 0U;

        if (g_qd4310_test_i_pulse_active != 0U &&
            now < g_i_pulse_until_tick_ms)
        {
            g_qd4310_test_i_pulse_remaining_ms =
                g_i_pulse_until_tick_ms - now;
        }
        else
        {
            g_qd4310_test_i_pulse_remaining_ms = 0U;
        }

        const float unclamped_control_output =
            g_qd4310_test_pd_output_deg + g_qd4310_test_i_output_deg;
        float control_output = unclamped_control_output;
        const float max_correction = fAbs(g_qd4310_test_max_correction_deg);
        control_output = fClamp(control_output,
                                -max_correction,
                                max_correction);
        g_qd4310_test_pid_output_deg = control_output;
        g_qd4310_test_output_limit_blocked =
            fAbs(control_output - unclamped_control_output) > 0.0001f
                ? 1U
                : 0U;

        if (g_qd4310_test_balance_map_enabled != 0U)
        {
            g_qd4310_test_balance_base_angle_deg =
                fLookupBalanceAngle(actual_position_mm);
        }
        else
        {
            g_qd4310_test_balance_base_angle_deg =
                g_qd4310_test_initial_angle_deg;
        }

        const float unclamped_target_angle_deg =
            g_qd4310_test_balance_base_angle_deg + control_output;
        g_qd4310_test_unclamped_target_angle_deg =
            unclamped_target_angle_deg;
        g_qd4310_test_raw_target_angle_deg =
            fClamp(unclamped_target_angle_deg,
                   QD4310_TEST_MIN_ANGLE_DEG,
                   QD4310_TEST_MAX_ANGLE_DEG);

        /* Apply two independent angle paths. The PD path remains limited to
         * the normal smooth step; the I pulse has its own larger limit. */
        const float pd_target_angle_deg =
            fClamp(g_qd4310_test_balance_base_angle_deg +
                       g_qd4310_test_pd_output_deg,
                   QD4310_TEST_MIN_ANGLE_DEG,
                   QD4310_TEST_MAX_ANGLE_DEG);
        const float max_pd_delta =
            fAbs(g_qd4310_test_max_angle_delta_deg);
        const float requested_pd_delta =
            pd_target_angle_deg - g_qd4310_test_angle_deg;
        const float pd_delta = fClamp(requested_pd_delta,
                                      -max_pd_delta,
                                      max_pd_delta);
        g_qd4310_test_pd_delta_limit_blocked =
            fAbs(pd_delta - requested_pd_delta) > 0.0001f ? 1U : 0U;

        const float next_angle_unclamped =
            g_qd4310_test_angle_deg + pd_delta + i_delta;
        const float next_angle_deg = fClamp(next_angle_unclamped,
                                            QD4310_TEST_MIN_ANGLE_DEG,
                                            QD4310_TEST_MAX_ANGLE_DEG);
        const float angle_delta = next_angle_deg - g_qd4310_test_angle_deg;
        g_qd4310_test_delta_limit_blocked =
            g_qd4310_test_pd_delta_limit_blocked != 0U ||
            g_qd4310_test_i_delta_limit_blocked != 0U;
        g_qd4310_test_last_angle_delta_deg = angle_delta;
        g_qd4310_test_last_step_deg = angle_delta;
        g_qd4310_test_target_angle_deg = next_angle_deg;
        g_qd4310_test_angle_limit_blocked =
            (g_qd4310_test_raw_target_angle_deg !=
             unclamped_target_angle_deg) ||
            (next_angle_deg != next_angle_unclamped) ? 1U : 0U;

        const HAL_StatusTypeDef status =
            eSendAngle(g_qd4310_test_target_angle_deg);
        if (status == HAL_OK)
        {
            g_qd4310_test_angle_deg = g_qd4310_test_target_angle_deg;
            ++g_qd4310_test_step_count;
        }
        vRecordSend(status,
                    angle_delta >= 0.0f
                        ? QD4310_TEST_ACTION_STEP_POSITIVE
                        : QD4310_TEST_ACTION_STEP_NEGATIVE);
        if (g_qd4310_test_state != QD4310_TEST_STATE_ERROR)
        {
            g_qd4310_test_state = QD4310_TEST_STATE_BALANCE_RUN;
        }
    }

    void vRunManualAngleFollow()
    {
        if (g_qd4310_test_manual_angle_follow == 0U)
        {
            return;
        }

        if (g_qd4310_test_enabled == 0U)
        {
            const HAL_StatusTypeDef enable_status = eSendEnable();
            if (enable_status == HAL_OK)
            {
                g_qd4310_test_enabled = 1U;
            }
            vRecordSend(enable_status, QD4310_TEST_ACTION_ENABLE);
            return;
        }

        const float requested_angle_deg =
            g_qd4310_test_manual_target_angle_deg;
        const float limited_angle_deg =
            fClamp(requested_angle_deg,
                   QD4310_TEST_MIN_ANGLE_DEG,
                   QD4310_TEST_MAX_ANGLE_DEG);

        if (g_have_manual_angle_command != 0U &&
            fAbs(limited_angle_deg - g_last_manual_angle_command_deg) <
                QD4310_TEST_MANUAL_ANGLE_EPS_DEG)
        {
            return;
        }

        if (bSendLimitedAngle(requested_angle_deg,
                              QD4310_TEST_ACTION_SET_ANGLE))
        {
            g_have_manual_angle_command = 1U;
            g_last_manual_angle_command_deg = g_qd4310_test_angle_deg;
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
            case QD4310_TEST_ACTION_SET_ANGLE:
                if (g_qd4310_test_motion_authorized == 0U)
                {
                    vRecordSend(HAL_ERROR, action);
                    return;
                }
                if (g_qd4310_test_enabled == 0U)
                {
                    status = eSendEnable();
                    if (status == HAL_OK)
                    {
                        g_qd4310_test_enabled = 1U;
                    }
                    vRecordSend(status, QD4310_TEST_ACTION_ENABLE);
                    if (status != HAL_OK)
                    {
                        return;
                    }
                }
                if (bSendLimitedAngle(g_qd4310_test_manual_target_angle_deg,
                                      action))
                {
                    g_have_manual_angle_command = 1U;
                    g_last_manual_angle_command_deg =
                        g_qd4310_test_angle_deg;
                }
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
                g_qd4310_test_angle_deg = QD4310_TEST_BOOT_ANGLE_DEG;
                g_qd4310_test_target_angle_deg = QD4310_TEST_BOOT_ANGLE_DEG;
                g_qd4310_test_motor_command_angle_deg = QD4310_TEST_BOOT_ANGLE_DEG;
                g_qd4310_test_balance_base_angle_deg =
                    g_qd4310_test_initial_angle_deg;
                g_qd4310_test_last_angle_delta_deg = 0.0f;
                g_qd4310_test_ball_velocity_mm_s = 0.0f;
                g_qd4310_test_filtered_velocity_mm_s = 0.0f;
                g_qd4310_test_effective_error_mm = 0.0f;
                g_qd4310_test_i_output_deg = 0.0f;
                g_qd4310_test_pid_output_deg = 0.0f;
                g_qd4310_test_integral_enabled = 0U;
                g_qd4310_test_stuck_active = 0U;
                g_qd4310_test_i_pulse_active = 0U;
                g_qd4310_test_i_pulse_remaining_ms = 0U;
                g_qd4310_test_pd_delta_limit_blocked = 0U;
                g_qd4310_test_i_delta_limit_blocked = 0U;
                g_stuck_since_tick_ms = 0U;
                g_i_pulse_until_tick_ms = 0U;
                g_i_pulse_direction_sign = 0;
                g_qd4310_test_unclamped_target_angle_deg =
                    g_qd4310_test_initial_angle_deg;
                g_qd4310_test_deadband_active = 0U;
                g_qd4310_test_output_limit_blocked = 0U;
                g_qd4310_test_delta_limit_blocked = 0U;
                g_qd4310_test_control_dt_ms = 0U;
                g_qd4310_test_control_sample_count = 0U;
                g_qd4310_test_wait_vision_count = 0U;
                g_qd4310_test_same_frame_skip_count = 0U;
                g_qd4310_test_last_control_tick_ms = 0U;
                g_have_control_frame = 0U;
                g_last_control_frame_count = 0U;
                g_have_velocity_sample = 0U;
                g_last_position_mm = 0.0f;
                g_last_position_tick_ms = 0U;
                g_have_manual_angle_command = 0U;
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
        vUpdatePositionSequenceSwitch();

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
                /* Manual mode skips the vision loop. Ozone can either post a
                 * one-shot action or edit the target angle directly. */
                g_qd4310_test_balance_enabled = 0U;
                vRunManualAngleFollow();
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
