#ifndef QD4310_H
#define QD4310_H

#include <stdbool.h>
#include <stdint.h>

#include "usart.h"
#ifdef __cplusplus
#include "Qd4310Motor.hpp"
extern "C" {
#endif

typedef enum {
    QD4310_CMD_NOP = 0x00,
    QD4310_CMD_ENABLE = 0x01,
    QD4310_CMD_DISABLE = 0x02,
    QD4310_CMD_CURRENT = 0x03,
    QD4310_CMD_SPEED = 0x04,
    QD4310_CMD_ANGLE = 0x05,
    QD4310_CMD_LOW_SPEED = 0x06,
    QD4310_CMD_STEP_ANGLE = 0x07,
    QD4310_CMD_CLEAR_ERROR = 0xFB,
    QD4310_CMD_REBOOT = 0xFF,
    QD4310_CMD_SET_ZERO_POS = 0xFE,
} qd4310_command_t;

typedef struct {
    bool enabled;
    uint8_t id;
    float speed_rpm;
    float angle_rad;
    float current_a;
    UART_HandleTypeDef *huart;
    uint8_t last_tx_frame[5];
    uint32_t tx_count;
} qd4310_t;

#define QD4310_PI 3.14159265358979323846f
#define QD4310_TWO_PI (2.0f * QD4310_PI)
#define QD4310_MAX_SPEED_RPM 1000.0f
#define QD4310_MIN_SPEED_RPM (-1000.0f)
#define QD4310_MAX_CURRENT_A 10.0f
#define QD4310_MIN_CURRENT_A (-10.0f)
#define QD4310_MAX_STEP_ANGLE_RAD QD4310_TWO_PI
#define QD4310_MIN_STEP_ANGLE_RAD (-QD4310_TWO_PI)

void qd4310_init(qd4310_t *motor, UART_HandleTypeDef *huart, uint8_t id);
HAL_StatusTypeDef qd4310_send_only(qd4310_t *motor,
                                   qd4310_command_t cmd,
                                   int16_t value);
HAL_StatusTypeDef qd4310_send_raw(qd4310_t *motor, qd4310_command_t cmd, int16_t value);
void qd4310_update(qd4310_t *motor, const uint8_t feedback[8]);

HAL_StatusTypeDef qd4310_enable(qd4310_t *motor);
HAL_StatusTypeDef qd4310_disable(qd4310_t *motor);
HAL_StatusTypeDef qd4310_reboot(qd4310_t *motor);
HAL_StatusTypeDef qd4310_set_zero_pos(qd4310_t *motor);
HAL_StatusTypeDef qd4310_set_angle(qd4310_t *motor, float angle_rad);
HAL_StatusTypeDef qd4310_set_angle_only(qd4310_t *motor, float angle_rad);
HAL_StatusTypeDef qd4310_set_step_angle_only(qd4310_t *motor,
                                             float step_angle_rad);
HAL_StatusTypeDef qd4310_set_speed_only(qd4310_t *motor, float speed_rpm);
HAL_StatusTypeDef qd4310_set_step_angle(qd4310_t *motor, float step_angle_rad);
HAL_StatusTypeDef qd4310_set_speed(qd4310_t *motor, float speed_rpm);
HAL_StatusTypeDef qd4310_set_low_speed(qd4310_t *motor, float speed_rpm);
HAL_StatusTypeDef qd4310_set_current(qd4310_t *motor, float current_a);

#ifdef __cplusplus
}
#endif

#endif
