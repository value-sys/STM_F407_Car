#ifndef QD4310_TEST_H
#define QD4310_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Set this to the ID configured in the QD4310 USB host tool. */
#define QD4310_TEST_MOTOR_ID             1U
#define QD4310_TEST_PERIOD_MS             50U

typedef enum
{
    QD4310_TEST_ACTION_NONE = 0U,
    QD4310_TEST_ACTION_ENABLE,
    QD4310_TEST_ACTION_DISABLE,
    QD4310_TEST_ACTION_STOP,
    QD4310_TEST_ACTION_SET_ANGLE,
    QD4310_TEST_ACTION_STEP_POSITIVE,
    QD4310_TEST_ACTION_STEP_NEGATIVE,
    QD4310_TEST_ACTION_CLEAR_ERROR,
    QD4310_TEST_ACTION_SET_ZERO
} eQd4310TestAction;

/* Ozone controls. Motion actions are ignored until authorization is 1. */
extern volatile uint8_t g_qd4310_test_motion_authorized;
extern volatile uint8_t g_qd4310_test_action;
extern volatile float g_qd4310_test_angle_command_deg;
extern volatile float g_qd4310_test_step_command_deg;

/* Ozone feedback and diagnostics. */
extern volatile uint8_t g_qd4310_test_online;
extern volatile uint8_t g_qd4310_test_enabled;
extern volatile uint8_t g_qd4310_test_state_raw;
extern volatile float g_qd4310_test_angle_deg;
extern volatile float g_qd4310_test_speed_rpm;
extern volatile float g_qd4310_test_current_a;
extern volatile uint8_t g_qd4310_test_last_status;
extern volatile uint8_t g_qd4310_test_last_action;
extern volatile uint32_t g_qd4310_test_ok_count;
extern volatile uint32_t g_qd4310_test_error_count;

void vQd4310TestTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
