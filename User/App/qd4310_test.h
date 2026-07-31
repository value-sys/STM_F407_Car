#ifndef QD4310_TEST_H
#define QD4310_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The motor ID must match the QD4310 configuration. */
#define QD4310_TEST_MOTOR_ID             0U
#define QD4310_TEST_PERIOD_MS            20U
#define QD4310_TEST_TARGET_POSITION_MM   125.0f
#define QD4310_TEST_MIN_ANGLE_DEG        0.0f
#define QD4310_TEST_MAX_ANGLE_DEG        103.0f
#define QD4310_TEST_INITIAL_ANGLE_DEG    31.5f
#define QD4310_TEST_STEP_ANGLE_DEG       0.8f
#define QD4310_TEST_DEADBAND_MM          5.0f
#define QD4310_TEST_MANUAL_MODE_DEFAULT  1U

#define QD4310_TEST_STOP_REASON_NONE       0U
#define QD4310_TEST_STOP_REASON_UI         1U
#define QD4310_TEST_STOP_REASON_VISION     2U
#define QD4310_TEST_STOP_REASON_ACTION     3U
#define QD4310_TEST_STOP_REASON_AUTH_LOST  4U

typedef enum
{
    QD4310_TEST_ACTION_NONE = 0U,
    QD4310_TEST_ACTION_ENABLE,
    QD4310_TEST_ACTION_DISABLE,
    QD4310_TEST_ACTION_STEP_POSITIVE,
    QD4310_TEST_ACTION_STEP_NEGATIVE,
    QD4310_TEST_ACTION_RESTART_INIT,
    QD4310_TEST_ACTION_RESET_ESTIMATE
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
extern volatile float g_qd4310_test_target_position_mm;
extern volatile float g_qd4310_test_initial_angle_deg;
extern volatile float g_qd4310_test_step_angle_deg;
extern volatile float g_qd4310_test_deadband_mm;
extern volatile int8_t g_qd4310_test_direction_sign;

/* Ozone diagnostics. Angle is a software estimate because no motor feedback is used. */
extern volatile uint8_t g_qd4310_test_online;
extern volatile uint8_t g_qd4310_test_enabled;
extern volatile uint8_t g_qd4310_test_state_raw;
extern volatile float g_qd4310_test_angle_deg;
extern volatile float g_qd4310_test_speed_rpm;
extern volatile float g_qd4310_test_current_a;
extern volatile float g_qd4310_test_actual_position_mm;
extern volatile float g_qd4310_test_position_error_mm;
extern volatile float g_qd4310_test_last_step_deg;
extern volatile uint8_t g_qd4310_test_angle_limit_blocked;
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
