#include "Qd4310Motor.hpp"

#include <climits>

namespace yuntai::motor {

namespace {

constexpr float kDegToRawAngle = 65535.0f / 360.0f;
constexpr float kRawToDegAngle = 360.0f / 65535.0f;

float clampScalar(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

}  // namespace

Qd4310Motor::Qd4310Motor(UART_HandleTypeDef *huart, uint8_t id) {
    attach(huart, id);
}

void Qd4310Motor::attach(UART_HandleTypeDef *huart, uint8_t id) {
    huart_ = huart;
    id_ = id;
    feedback_ = {};
    feedback_.id = id;
}

void Qd4310Motor::setLimits(const Qd4310Limits &limits) {
    limits_ = limits;
}

HAL_StatusTypeDef Qd4310Motor::enable() {
    return sendRaw(Qd4310Command::Enable, 0);
}

HAL_StatusTypeDef Qd4310Motor::disable() {
    return sendRaw(Qd4310Command::Disable, 0);
}

HAL_StatusTypeDef Qd4310Motor::reboot() {
    return sendRaw(Qd4310Command::Reboot, 0);
}

HAL_StatusTypeDef Qd4310Motor::clearError() {
    return sendRaw(Qd4310Command::ClearError, 0);
}

HAL_StatusTypeDef Qd4310Motor::setZeroPos() {
    return sendRaw(Qd4310Command::SetZeroPos, 0);
}

HAL_StatusTypeDef Qd4310Motor::setSpeedRpm(float speed_rpm) {
    const float clamped = clampValue(speed_rpm, limits_.min_speed_rpm, limits_.max_speed_rpm);
    const int16_t raw = static_cast<int16_t>((clamped / limits_.max_speed_rpm) * static_cast<float>(INT16_MAX));
    return sendRaw(Qd4310Command::Speed, raw);
}

HAL_StatusTypeDef Qd4310Motor::setLowSpeedRpm(float speed_rpm) {
    const float clamped = clampValue(speed_rpm, limits_.min_speed_rpm, limits_.max_speed_rpm);
    const int16_t raw = static_cast<int16_t>((clamped / limits_.max_speed_rpm) * static_cast<float>(INT16_MAX));
    return sendRaw(Qd4310Command::LowSpeed, raw);
}

HAL_StatusTypeDef Qd4310Motor::setAngleDeg(float angle_deg) {
    const float clamped = clampValue(angle_deg, limits_.min_angle_deg, limits_.max_angle_deg);
    const uint16_t raw = static_cast<uint16_t>(clamped * kDegToRawAngle);
    return sendRaw(Qd4310Command::Angle, static_cast<int16_t>(raw));
}

HAL_StatusTypeDef Qd4310Motor::setStepAngleDeg(float step_angle_deg) {
    const float clamped = clampValue(step_angle_deg, limits_.min_step_angle_deg, limits_.max_step_angle_deg);
    const int16_t raw = static_cast<int16_t>((clamped / limits_.max_step_angle_deg) * static_cast<float>(INT16_MAX));
    return sendRaw(Qd4310Command::StepAngle, raw);
}

HAL_StatusTypeDef Qd4310Motor::setCurrentA(float current_a) {
    const float clamped = clampValue(current_a, limits_.min_current_a, limits_.max_current_a);
    const int16_t raw = static_cast<int16_t>((clamped / limits_.max_current_a) * static_cast<float>(INT16_MAX));
    return sendRaw(Qd4310Command::Current, raw);
}

HAL_StatusTypeDef Qd4310Motor::stop() {
    return setSpeedRpm(0.0f);
}

HAL_StatusTypeDef Qd4310Motor::sendRaw(Qd4310Command cmd, int16_t value) {
    uint8_t tx_buffer[5];
    uint8_t rx_buffer[10];
    HAL_StatusTypeDef status;

    if (!isAttached()) {
        return HAL_ERROR;
    }

    tx_buffer[0] = id_;
    tx_buffer[1] = static_cast<uint8_t>(cmd);
    tx_buffer[2] = static_cast<uint8_t>(value & 0xFF);
    tx_buffer[3] = static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8);
    tx_buffer[4] = crc8(tx_buffer, 4);

    status = HAL_UART_Transmit(huart_, tx_buffer, sizeof(tx_buffer), HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UART_Receive(huart_, rx_buffer, sizeof(rx_buffer), 5);
    if (status != HAL_OK) {
        return status;
    }

    if (rx_buffer[0] != id_ || crc8(rx_buffer, 9) != rx_buffer[9]) {
        return HAL_ERROR;
    }

    updateFeedback(&rx_buffer[1]);
    return HAL_OK;
}

void Qd4310Motor::updateFeedback(const uint8_t feedback[8]) {
    const int16_t current_raw = static_cast<int16_t>((static_cast<uint16_t>(feedback[3]) << 8) | feedback[2]);
    const int16_t speed_raw = static_cast<int16_t>((static_cast<uint16_t>(feedback[5]) << 8) | feedback[4]);
    const uint16_t angle_raw = static_cast<uint16_t>((static_cast<uint16_t>(feedback[7]) << 8) | feedback[6]);

    feedback_.id = id_;
    feedback_.state_raw = feedback[0];
    feedback_.enabled = (feedback[0] & 0x01u) != 0u;
    feedback_.current_a = (static_cast<float>(current_raw) * limits_.max_current_a) / static_cast<float>(INT16_MAX);
    feedback_.speed_rpm = (static_cast<float>(speed_raw) * limits_.max_speed_rpm) / 32767.0f;
    feedback_.angle_deg = static_cast<float>(angle_raw) * kRawToDegAngle;
}

const Qd4310Feedback &Qd4310Motor::feedback() const {
    return feedback_;
}

bool Qd4310Motor::isAttached() const {
    return huart_ != nullptr;
}

float Qd4310Motor::clampValue(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

uint8_t Qd4310Motor::crc8(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0x00;

    if (data == nullptr || len == 0U) {
        return 0U;
    }

    while (len-- > 0U) {
        crc ^= *data++;
        for (uint8_t i = 0U; i < 8U; ++i) {
            crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U) : static_cast<uint8_t>(crc << 1);
        }
    }

    return crc;
}

uint8_t Qd4310Motor::reverseBits(uint8_t data) {
    data = ((data & 0x55u) << 1) | ((data & 0xAAu) >> 1);
    data = ((data & 0x33u) << 2) | ((data & 0xCCu) >> 2);
    data = ((data & 0x0Fu) << 4) | ((data & 0xF0u) >> 4);
    return data;
}

Qd4310Servo::Qd4310Servo(UART_HandleTypeDef *huart, uint8_t id) : motor_(huart, id) {}

void Qd4310Servo::attach(UART_HandleTypeDef *huart, uint8_t id) {
    motor_.attach(huart, id);
}

void Qd4310Servo::setMotorLimits(const Qd4310Limits &limits) {
    motor_.setLimits(limits);
}

void Qd4310Servo::setConfig(const Qd4310ServoConfig &config) {
    config_ = config;
}

HAL_StatusTypeDef Qd4310Servo::enable() {
    return motor_.enable();
}

HAL_StatusTypeDef Qd4310Servo::disable() {
    return motor_.disable();
}

HAL_StatusTypeDef Qd4310Servo::clearError() {
    return motor_.clearError();
}

HAL_StatusTypeDef Qd4310Servo::reboot() {
    return motor_.reboot();
}

HAL_StatusTypeDef Qd4310Servo::stop() {
    return motor_.stop();
}

HAL_StatusTypeDef Qd4310Servo::setOutputAngleDeg(float output_angle_deg) {
    return motor_.setAngleDeg(outputToMotorAngleDeg(output_angle_deg));
}

HAL_StatusTypeDef Qd4310Servo::jogOutputAngleDeg(float delta_output_deg) {
    const float target_output_deg = currentOutputAngleDeg() + delta_output_deg;
    return setOutputAngleDeg(target_output_deg);
}

HAL_StatusTypeDef Qd4310Servo::setOutputSpeedRpm(float speed_rpm) {
    const float clamped_speed = clampScalar(speed_rpm, -config_.max_output_speed_rpm, config_.max_output_speed_rpm);
    const float motor_speed = config_.reversed ? -clamped_speed : clamped_speed;
    return motor_.setSpeedRpm(motor_speed);
}

HAL_StatusTypeDef Qd4310Servo::holdCurrentOutputAngle() {
    return setOutputAngleDeg(currentOutputAngleDeg());
}

void Qd4310Servo::setZeroOffsetDeg(float zero_offset_deg) {
    config_.zero_offset_deg = wrap360(zero_offset_deg);
}

void Qd4310Servo::alignZeroOffsetToCurrentPosition(float current_output_deg) {
    const float output_sign = config_.reversed ? -1.0f : 1.0f;
    config_.zero_offset_deg = wrap360(motor_.feedback().angle_deg - current_output_deg * output_sign);
}

float Qd4310Servo::currentOutputAngleDeg() const {
    const float motor_relative_deg = wrap360(motor_.feedback().angle_deg - config_.zero_offset_deg);
    const float signed_output_deg = config_.reversed ? -motor_relative_deg : motor_relative_deg;

    if (signed_output_deg > 180.0f) {
        return signed_output_deg - 360.0f;
    }
    if (signed_output_deg < -180.0f) {
        return signed_output_deg + 360.0f;
    }
    return signed_output_deg;
}

float Qd4310Servo::clampOutputAngleDeg(float output_angle_deg) const {
    return clampScalar(output_angle_deg, config_.min_output_deg, config_.max_output_deg);
}

float Qd4310Servo::outputToMotorAngleDeg(float output_angle_deg) const {
    const float clamped_output_deg = clampOutputAngleDeg(output_angle_deg);
    const float output_sign = config_.reversed ? -1.0f : 1.0f;
    return wrap360(config_.zero_offset_deg + clamped_output_deg * output_sign);
}

const Qd4310ServoConfig &Qd4310Servo::config() const {
    return config_;
}

Qd4310ServoFeedback Qd4310Servo::feedback() const {
    const auto &motor_feedback = motor_.feedback();
    return {
        motor_feedback.enabled,
        currentOutputAngleDeg(),
        motor_feedback.angle_deg,
        motor_feedback.speed_rpm,
        motor_feedback.current_a,
    };
}

Qd4310Motor &Qd4310Servo::motor() {
    return motor_;
}

const Qd4310Motor &Qd4310Servo::motor() const {
    return motor_;
}

float Qd4310Servo::wrap360(float angle_deg) {
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

}  // namespace yuntai::motor
