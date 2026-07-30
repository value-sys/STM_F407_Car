#include "qd4310.h"

#include <limits.h>
#include <string.h>

static uint8_t qd4310_reverse_bits(uint8_t data)
{
    data = ((data & 0x55u) << 1) | ((data & 0xAAu) >> 1);
    data = ((data & 0x33u) << 2) | ((data & 0xCCu) >> 2);
    data = ((data & 0x0Fu) << 4) | ((data & 0xF0u) >> 4);
    return data;
}

static uint8_t qd4310_crc8(const uint8_t *data,
                           uint32_t len,
                           uint8_t polynomial,
                           uint8_t init,
                           uint8_t xor_out,
                           bool input_invert,
                           bool output_invert)
{
    uint8_t crc = init;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    while (len-- > 0U) {
        crc ^= input_invert ? qd4310_reverse_bits(*data++) : *data++;
        for (uint8_t i = 0U; i < 8U; ++i) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ polynomial) : (uint8_t)(crc << 1);
        }
    }

    crc ^= xor_out;
    return output_invert ? qd4310_reverse_bits(crc) : crc;
}

static float qd4310_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void qd4310_init(qd4310_t *motor, UART_HandleTypeDef *huart, uint8_t id)
{
    if (motor == NULL) {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->huart = huart;
    motor->id = id;
}

HAL_StatusTypeDef qd4310_send_raw(qd4310_t *motor, qd4310_command_t cmd, int16_t value)
{
    uint8_t tx_buffer[5];
    uint8_t rx_buffer[10];
    HAL_StatusTypeDef status;

    if (motor == NULL || motor->huart == NULL) {
        return HAL_ERROR;
    }

    tx_buffer[0] = motor->id;
    tx_buffer[1] = (uint8_t)cmd;
    tx_buffer[2] = (uint8_t)(value & 0xFF);
    tx_buffer[3] = (uint8_t)((uint16_t)value >> 8);
    tx_buffer[4] = qd4310_crc8(tx_buffer, 4, 0x07, 0x00, 0x00, false, false);

    status = HAL_UART_Transmit(motor->huart, tx_buffer, sizeof(tx_buffer), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UART_Receive(motor->huart, rx_buffer, sizeof(rx_buffer), 5);
    if (status != HAL_OK) {
        return status;
    }

    if (rx_buffer[0] != motor->id) {
        return HAL_ERROR;
    }

    if (qd4310_crc8(rx_buffer, 9, 0x07, 0x00, 0x00, false, false) != rx_buffer[9]) {
        return HAL_ERROR;
    }

    qd4310_update(motor, &rx_buffer[1]);
    return HAL_OK;
}

void qd4310_update(qd4310_t *motor, const uint8_t feedback[8])
{
    int16_t current_raw;
    int16_t speed_raw;
    uint16_t angle_raw;

    if (motor == NULL || feedback == NULL) {
        return;
    }

    motor->enabled = (feedback[0] & 0x01u) != 0u;

    current_raw = (int16_t)(((uint16_t)feedback[3] << 8) | feedback[2]);
    speed_raw = (int16_t)(((uint16_t)feedback[5] << 8) | feedback[4]);
    angle_raw = (uint16_t)(((uint16_t)feedback[7] << 8) | feedback[6]);

    motor->current_a = ((float)current_raw * QD4310_MAX_CURRENT_A) / (float)INT16_MAX;
    motor->speed_rpm = ((float)speed_raw * QD4310_MAX_SPEED_RPM) / 32767.0f;
    motor->angle_rad = ((float)angle_raw * QD4310_TWO_PI) / 65535.0f;
}

HAL_StatusTypeDef qd4310_enable(qd4310_t *motor)
{
    return qd4310_send_raw(motor, QD4310_CMD_ENABLE, 0);
}

HAL_StatusTypeDef qd4310_disable(qd4310_t *motor)
{
    return qd4310_send_raw(motor, QD4310_CMD_DISABLE, 0);
}

HAL_StatusTypeDef qd4310_reboot(qd4310_t *motor)
{
    return qd4310_send_raw(motor, QD4310_CMD_REBOOT, 0);
}

HAL_StatusTypeDef qd4310_set_zero_pos(qd4310_t *motor)
{
    return qd4310_send_raw(motor, QD4310_CMD_SET_ZERO_POS, 0);
}

HAL_StatusTypeDef qd4310_set_angle(qd4310_t *motor, float angle_rad)
{
    uint16_t raw;
    angle_rad = qd4310_clamp(angle_rad, 0.0f, QD4310_TWO_PI);
    raw = (uint16_t)((angle_rad / QD4310_TWO_PI) * 65535.0f);
    return qd4310_send_raw(motor, QD4310_CMD_ANGLE, (int16_t)raw);
}

HAL_StatusTypeDef qd4310_set_step_angle(qd4310_t *motor, float step_angle_rad)
{
    int16_t raw;
    step_angle_rad = qd4310_clamp(step_angle_rad, QD4310_MIN_STEP_ANGLE_RAD, QD4310_MAX_STEP_ANGLE_RAD);
    raw = (int16_t)((step_angle_rad / QD4310_MAX_STEP_ANGLE_RAD) * (float)INT16_MAX);
    return qd4310_send_raw(motor, QD4310_CMD_STEP_ANGLE, raw);
}

HAL_StatusTypeDef qd4310_set_speed(qd4310_t *motor, float speed_rpm)
{
    int16_t raw;
    speed_rpm = qd4310_clamp(speed_rpm, QD4310_MIN_SPEED_RPM, QD4310_MAX_SPEED_RPM);
    raw = (int16_t)((speed_rpm / QD4310_MAX_SPEED_RPM) * (float)INT16_MAX);
    return qd4310_send_raw(motor, QD4310_CMD_SPEED, raw);
}

HAL_StatusTypeDef qd4310_set_low_speed(qd4310_t *motor, float speed_rpm)
{
    int16_t raw;
    speed_rpm = qd4310_clamp(speed_rpm, QD4310_MIN_SPEED_RPM, QD4310_MAX_SPEED_RPM);
    raw = (int16_t)((speed_rpm / QD4310_MAX_SPEED_RPM) * (float)INT16_MAX);
    return qd4310_send_raw(motor, QD4310_CMD_LOW_SPEED, raw);
}

HAL_StatusTypeDef qd4310_set_current(qd4310_t *motor, float current_a)
{
    int16_t raw;
    current_a = qd4310_clamp(current_a, QD4310_MIN_CURRENT_A, QD4310_MAX_CURRENT_A);
    raw = (int16_t)((current_a / QD4310_MAX_CURRENT_A) * (float)INT16_MAX);
    return qd4310_send_raw(motor, QD4310_CMD_CURRENT, raw);
}
