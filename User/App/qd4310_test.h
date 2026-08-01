#ifndef QD4310_TEST_H
#define QD4310_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The motor ID must match the QD4310 configuration. */
#define QD4310_TEST_MOTOR_ID             0U
#define QD4310_TEST_PERIOD_MS            20U
#define QD4310_TEST_TARGET_POSITION_MM   75.0f
#define QD4310_TEST_MIN_ANGLE_DEG        (-10.0f)
#define QD4310_TEST_MAX_ANGLE_DEG        103.0f
#define QD4310_TEST_BOOT_ANGLE_DEG       0.0f
#define QD4310_TEST_INITIAL_ANGLE_DEG    5.0f
#define QD4310_TEST_MANUAL_TARGET_ANGLE_DEG QD4310_TEST_INITIAL_ANGLE_DEG
#define QD4310_TEST_MANUAL_ANGLE_EPS_DEG 0.05f
#define QD4310_TEST_INIT_RAMP_DEFAULT    1U
#define QD4310_TEST_INIT_RAMP_STEP_DEG   1.0f
#define QD4310_TEST_INIT_RAMP_DELAY_MS   80U
#define QD4310_TEST_BALANCE_MAP_SIZE     9U
#define QD4310_TEST_BALANCE_MAP_DEFAULT  1U
#define QD4310_TEST_STEP_ANGLE_DEG       0.8f
#define QD4310_TEST_DEADBAND_MM          10.0f
#define QD4310_TEST_KP_DEG_PER_MM        0.10f
#define QD4310_TEST_KD_DEG_PER_MM_S      0.023f
#define QD4310_TEST_KI_DEG_PER_MM_S      1000.0f
#define QD4310_TEST_MAX_INTEGRAL_DEG     6.0f
#define QD4310_TEST_INTEGRAL_VEL_MM_S    15.0f
#define QD4310_TEST_MAX_CORRECTION_DEG   10.0f
#define QD4310_TEST_MAX_ANGLE_DELTA_DEG  0.4f
#define QD4310_TEST_MAX_I_ANGLE_DELTA_DEG 2.0f
#define QD4310_TEST_VELOCITY_LPF_ALPHA   0.70f
#define QD4310_TEST_STUCK_ERROR_MM       10.0f
#define QD4310_TEST_STUCK_VELOCITY_MM_S  20.0f
#define QD4310_TEST_STUCK_RELEASE_VELOCITY_MM_S 30.0f
#define QD4310_TEST_STUCK_CONFIRM_MS     200U
#define QD4310_TEST_I_PULSE_DURATION_MS  200U
#define QD4310_TEST_FEEDBACK_POLL_DEFAULT 0U
#define QD4310_TEST_MANUAL_MODE_DEFAULT  1U
#define QD4310_TEST_POSITION_SEQUENCE_FIRST_TARGET_MM  175.0f
#define QD4310_TEST_POSITION_SEQUENCE_SECOND_TARGET_MM 75.0f
#define QD4310_TEST_POSITION_SEQUENCE_CENTER_TARGET_MM 125.0f
#define QD4310_TEST_POSITION_SEQUENCE_TOLERANCE_MM     10.0f
#define QD4310_TEST_POSITION_SEQUENCE_CONFIRM_FRAMES  5U

#define QD4310_TEST_STOP_REASON_NONE       0U
#define QD4310_TEST_STOP_REASON_UI         1U
#define QD4310_TEST_STOP_REASON_VISION     2U
#define QD4310_TEST_STOP_REASON_ACTION     3U
#define QD4310_TEST_STOP_REASON_AUTH_LOST  4U

typedef enum
{
    QD4310_TEST_ACTION_NONE = 0U,
    QD4310_TEST_ACTION_ENABLE = 1U,
    QD4310_TEST_ACTION_DISABLE = 2U,
    QD4310_TEST_ACTION_STEP_POSITIVE = 3U,
    QD4310_TEST_ACTION_STEP_NEGATIVE = 4U,
    QD4310_TEST_ACTION_RESTART_INIT = 5U,
    QD4310_TEST_ACTION_RESET_ESTIMATE = 6U,
    QD4310_TEST_ACTION_SET_ANGLE = 7U
} eQd4310TestAction;

typedef enum
{
    QD4310_TEST_STATE_WAIT_AUTH = 0U,
    QD4310_TEST_STATE_INITIALIZING,
    QD4310_TEST_STATE_WAIT_VISION,
    QD4310_TEST_STATE_BALANCE_RUN,
    QD4310_TEST_STATE_VISION_LOST,
    QD4310_TEST_STATE_STOPPED,
    QD4310_TEST_STATE_MANUAL_TEST,
    QD4310_TEST_STATE_ERROR
} eQd4310TestState;

/* Ozone controls. Set authorization to 1 to enable the motor and start init. */
extern volatile uint8_t g_qd4310_test_motion_authorized;
/* 1: Ozone manual command test; 0: initialization and vision control. */
extern volatile uint8_t g_qd4310_test_manual_mode;
extern volatile uint8_t g_qd4310_test_balance_enabled;
extern volatile uint8_t g_qd4310_test_stop_request;
extern volatile uint8_t g_qd4310_test_action;
extern volatile uint8_t g_qd4310_test_manual_angle_follow;
extern volatile uint8_t g_qd4310_test_init_ramp_enabled;
extern volatile uint8_t g_qd4310_test_balance_map_enabled;
extern volatile float g_qd4310_test_manual_target_angle_deg;
extern volatile float g_qd4310_test_init_ramp_step_deg;
extern volatile uint32_t g_qd4310_test_init_ramp_delay_ms;
extern volatile float g_qd4310_test_balance_map_position_mm[QD4310_TEST_BALANCE_MAP_SIZE];
extern volatile float g_qd4310_test_balance_map_angle_deg[QD4310_TEST_BALANCE_MAP_SIZE];
extern volatile float g_qd4310_test_target_position_mm;
/* Optional visual-position sequence. Phase 1: first target, phase 2: second
 * target, phase 3: waiting for center, phase 4: completed. */
extern volatile uint8_t g_qd4310_test_position_sequence_enabled;
extern volatile uint8_t g_qd4310_test_position_sequence_phase;
extern volatile uint8_t g_qd4310_test_position_sequence_wait_center_enabled;
extern volatile float g_qd4310_test_position_sequence_center_target_mm;
extern volatile float g_qd4310_test_position_sequence_first_target_mm;
extern volatile float g_qd4310_test_position_sequence_second_target_mm;
extern volatile float g_qd4310_test_position_sequence_tolerance_mm;
extern volatile uint32_t g_qd4310_test_position_sequence_confirm_frames;
extern volatile uint32_t g_qd4310_test_position_sequence_confirm_count;
extern volatile uint8_t g_qd4310_test_position_sequence_in_range;
extern volatile uint8_t g_qd4310_test_position_sequence_first_target_reached;
extern volatile uint8_t g_qd4310_test_position_sequence_timer_started;
extern volatile uint8_t g_qd4310_test_position_sequence_completed;
extern volatile float g_qd4310_test_initial_angle_deg;
extern volatile float g_qd4310_test_step_angle_deg;
extern volatile float g_qd4310_test_deadband_mm;
extern volatile float g_qd4310_test_kp_deg_per_mm;
extern volatile float g_qd4310_test_kd_deg_per_mm_s;
extern volatile float g_qd4310_test_ki_deg_per_mm_s;
extern volatile float g_qd4310_test_max_integral_deg;
extern volatile float g_qd4310_test_integral_velocity_limit_mm_s;
extern volatile float g_qd4310_test_max_correction_deg;
extern volatile float g_qd4310_test_max_angle_delta_deg;
extern volatile float g_qd4310_test_max_i_angle_delta_deg;
extern volatile float g_qd4310_test_velocity_lpf_alpha;
extern volatile int8_t g_qd4310_test_direction_sign;
extern volatile float g_qd4310_test_stuck_error_mm;
extern volatile float g_qd4310_test_stuck_velocity_mm_s;
extern volatile float g_qd4310_test_stuck_release_velocity_mm_s;
extern volatile uint32_t g_qd4310_test_stuck_confirm_ms;
extern volatile uint32_t g_qd4310_test_i_pulse_duration_ms;
/* 0: keep send-only mode; 1: wait for and decode QD4310 feedback frames. */
extern volatile uint8_t g_qd4310_test_feedback_poll_enabled;

/* Ozone diagnostics. Angle is a software estimate because no motor feedback is used. */
extern volatile uint8_t g_qd4310_test_online;
extern volatile uint8_t g_qd4310_test_enabled;
extern volatile uint8_t g_qd4310_test_feedback_enabled;
extern volatile uint8_t g_qd4310_test_state_raw;
extern volatile uint8_t g_qd4310_test_feedback_valid;
extern volatile float g_qd4310_test_feedback_angle_deg;
extern volatile uint32_t g_qd4310_test_feedback_count;
extern volatile uint32_t g_qd4310_test_feedback_error_count;
extern volatile uint32_t g_qd4310_test_last_feedback_tick_ms;
extern volatile float g_qd4310_test_angle_deg;
extern volatile float g_qd4310_test_speed_rpm;
extern volatile float g_qd4310_test_current_a;
extern volatile float g_qd4310_test_actual_position_mm;
extern volatile float g_qd4310_test_position_error_mm;
extern volatile float g_qd4310_test_effective_error_mm;
extern volatile float g_qd4310_test_ball_velocity_mm_s;
extern volatile float g_qd4310_test_filtered_velocity_mm_s;
extern volatile float g_qd4310_test_p_output_deg;
extern volatile float g_qd4310_test_d_output_deg;
extern volatile float g_qd4310_test_i_output_deg;
extern volatile float g_qd4310_test_pd_output_deg;
extern volatile float g_qd4310_test_pid_output_deg;
extern volatile uint8_t g_qd4310_test_integral_enabled;
extern volatile uint8_t g_qd4310_test_stuck_active;
extern volatile uint8_t g_qd4310_test_i_pulse_active;
extern volatile uint8_t g_qd4310_test_pd_delta_limit_blocked;
extern volatile uint8_t g_qd4310_test_i_delta_limit_blocked;
extern volatile uint32_t g_qd4310_test_i_pulse_remaining_ms;
extern volatile uint32_t g_qd4310_test_i_pulse_trigger_count;
extern volatile float g_qd4310_test_balance_base_angle_deg;
extern volatile float g_qd4310_test_unclamped_target_angle_deg;
extern volatile float g_qd4310_test_raw_target_angle_deg;
extern volatile float g_qd4310_test_target_angle_deg;
extern volatile float g_qd4310_test_motor_command_angle_deg;
extern volatile float g_qd4310_test_last_angle_delta_deg;
extern volatile float g_qd4310_test_last_step_deg;
extern volatile uint8_t g_qd4310_test_angle_limit_blocked;
extern volatile uint8_t g_qd4310_test_deadband_active;
extern volatile uint8_t g_qd4310_test_output_limit_blocked;
extern volatile uint8_t g_qd4310_test_delta_limit_blocked;
extern volatile uint32_t g_qd4310_test_control_dt_ms;
extern volatile uint32_t g_qd4310_test_control_sample_count;
extern volatile uint32_t g_qd4310_test_wait_vision_count;
extern volatile uint32_t g_qd4310_test_same_frame_skip_count;
extern volatile uint32_t g_qd4310_test_last_control_tick_ms;
extern volatile uint32_t g_qd4310_test_last_vision_sequence;
extern volatile uint32_t g_qd4310_test_last_vision_age_ms;
extern volatile uint8_t g_qd4310_test_last_vision_valid;
extern volatile uint8_t g_qd4310_test_last_vision_confidence;
extern volatile uint8_t g_qd4310_test_state;
extern volatile uint8_t g_qd4310_test_stop_reason;
extern volatile uint8_t g_qd4310_test_last_status;
extern volatile uint8_t g_qd4310_test_last_action;
extern volatile uint32_t g_qd4310_test_ok_count;
extern volatile uint32_t g_qd4310_test_error_count;
extern volatile uint32_t g_qd4310_test_step_count;
extern volatile uint32_t g_qd4310_test_tx_count;
extern volatile uint8_t g_qd4310_test_last_tx_frame[5];

void vQd4310TestTask(void *argument);
void vQd4310TestRequestStop(void);

#ifdef __cplusplus
}
#endif

#endif
