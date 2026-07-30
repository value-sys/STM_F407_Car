#include "Y42Motor.hpp"

#include <cmath>
#include <limits>

namespace yuntai::motor {

namespace {

constexpr float kXFeedbackAngleUnitDeg = 0.1f;
constexpr float kXFeedbackErrorUnitDeg = 0.01f;
constexpr float kXSpeedUnitRpm = 0.1f;

uint16_t roundToU16(float value) {
    return static_cast<uint16_t>(value + 0.5f);
}

uint32_t roundToU32(float value) {
    return static_cast<uint32_t>(value + 0.5f);
}

}  // namespace

Y42Motor::Y42Motor(UART_HandleTypeDef *huart, const Y42Config &config) {
    attach(huart, config);
}

void Y42Motor::attach(UART_HandleTypeDef *huart, const Y42Config &config) {
    huart_ = huart;
    config_ = config;
    feedback_ = {};
    rx_size_ = 0U;
    rx_expected_size_ = 0U;
}

const Y42Config &Y42Motor::config() const {
    return config_;
}

const Y42Feedback &Y42Motor::feedback() const {
    return feedback_;
}

bool Y42Motor::isAttached() const {
    return huart_ != nullptr && config_.address != 0U &&
           finitePositive(config_.command_angle_unit_deg);
}

HAL_StatusTypeDef Y42Motor::enable(bool enabled) {
    return sendFrame(Y42Protocol::enable(config_.address, enabled));
}

HAL_StatusTypeDef Y42Motor::stop() {
    return sendFrame(Y42Protocol::stop(config_.address));
}

HAL_StatusTypeDef Y42Motor::clearPosition() {
    return sendFrame(Y42Protocol::clearPosition(config_.address));
}

HAL_StatusTypeDef Y42Motor::clearProtection() {
    return sendFrame(Y42Protocol::clearProtection(config_.address));
}

HAL_StatusTypeDef Y42Motor::setSpeedRpm(float speed_rpm, float acceleration_rpm_per_sec) {
    if (!isAttached() || !std::isfinite(speed_rpm) ||
        !std::isfinite(acceleration_rpm_per_sec) || acceleration_rpm_per_sec < 0.0f) {
        return HAL_ERROR;
    }

    const float speed_raw = absoluteValue(speed_rpm) / kXSpeedUnitRpm;
    if (speed_raw > static_cast<float>(Y42Protocol::kMaxSpeedRaw) ||
        acceleration_rpm_per_sec > static_cast<float>(Y42Protocol::kMaxAccelerationRpmPerSec)) {
        return HAL_ERROR;
    }

    return sendFrame(Y42Protocol::speed(config_.address, directionFor(speed_rpm),
                                         roundToU16(acceleration_rpm_per_sec),
                                         roundToU16(speed_raw)));
}

HAL_StatusTypeDef Y42Motor::moveRelativeDeg(float angle_deg, float max_speed_rpm,
                                            float acceleration_rpm_per_sec,
                                            float deceleration_rpm_per_sec) {
    return moveDeg(angle_deg, max_speed_rpm, acceleration_rpm_per_sec,
                   deceleration_rpm_per_sec, Y42PositionMode::RelativeToLastTarget);
}

HAL_StatusTypeDef Y42Motor::moveAbsoluteDeg(float angle_deg, float max_speed_rpm,
                                            float acceleration_rpm_per_sec,
                                            float deceleration_rpm_per_sec) {
    return moveDeg(angle_deg, max_speed_rpm, acceleration_rpm_per_sec,
                   deceleration_rpm_per_sec, Y42PositionMode::Absolute);
}

HAL_StatusTypeDef Y42Motor::moveFromCurrentDeg(float angle_deg, float max_speed_rpm,
                                               float acceleration_rpm_per_sec,
                                               float deceleration_rpm_per_sec) {
    return moveDeg(angle_deg, max_speed_rpm, acceleration_rpm_per_sec,
                   deceleration_rpm_per_sec, Y42PositionMode::RelativeToCurrent);
}

HAL_StatusTypeDef Y42Motor::requestSpeed() {
    return sendFrame(Y42Protocol::readSpeed(config_.address));
}

HAL_StatusTypeDef Y42Motor::requestPosition() {
    return sendFrame(Y42Protocol::readPosition(config_.address));
}

HAL_StatusTypeDef Y42Motor::requestPositionError() {
    return sendFrame(Y42Protocol::readPositionError(config_.address));
}

HAL_StatusTypeDef Y42Motor::requestStatus() {
    return sendFrame(Y42Protocol::readStatus(config_.address));
}

bool Y42Motor::feedRxByte(uint8_t byte) {
    if (rx_size_ == 0U) {
        if (byte != config_.address) {
            return false;
        }
        rx_frame_[rx_size_++] = byte;
        return false;
    }

    if (rx_size_ == 1U) {
        rx_expected_size_ = expectedFrameSize(byte);
        if (rx_expected_size_ == 0U) {
            rx_size_ = (byte == config_.address) ? 1U : 0U;
            return false;
        }
        rx_frame_[rx_size_++] = byte;
        return false;
    }

    rx_frame_[rx_size_++] = byte;
    if (rx_size_ < rx_expected_size_) {
        return false;
    }

    const bool parsed = parseFrame(rx_frame_, rx_expected_size_);
    rx_size_ = 0U;
    rx_expected_size_ = 0U;
    return parsed;
}

HAL_StatusTypeDef Y42Motor::sendFrame(const Y42Frame &frame) {
    if (!isAttached() || frame.size == 0U || frame.size > sizeof(frame.data)) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(huart_, const_cast<uint8_t *>(frame.data), frame.size,
                             config_.uart_timeout_ms);
}

HAL_StatusTypeDef Y42Motor::moveDeg(float angle_deg, float max_speed_rpm,
                                    float acceleration_rpm_per_sec,
                                    float deceleration_rpm_per_sec, Y42PositionMode mode) {
    if (!isAttached() || !std::isfinite(angle_deg) || !finitePositive(max_speed_rpm) ||
        !std::isfinite(acceleration_rpm_per_sec) ||
        !std::isfinite(deceleration_rpm_per_sec) || acceleration_rpm_per_sec < 0.0f ||
        deceleration_rpm_per_sec < 0.0f) {
        return HAL_ERROR;
    }

    const float angle_raw = absoluteValue(angle_deg) / config_.command_angle_unit_deg;
    const float speed_raw = max_speed_rpm / kXSpeedUnitRpm;

    if (angle_raw > static_cast<float>(std::numeric_limits<uint32_t>::max()) ||
        speed_raw > static_cast<float>(Y42Protocol::kMaxSpeedRaw) ||
        acceleration_rpm_per_sec > static_cast<float>(Y42Protocol::kMaxAccelerationRpmPerSec) ||
        deceleration_rpm_per_sec > static_cast<float>(Y42Protocol::kMaxAccelerationRpmPerSec)) {
        return HAL_ERROR;
    }

    return sendFrame(Y42Protocol::trapezoidPosition(
        config_.address, directionFor(angle_deg), roundToU16(acceleration_rpm_per_sec),
        roundToU16(deceleration_rpm_per_sec), roundToU16(speed_raw),
        roundToU32(angle_raw), mode));
}

bool Y42Motor::parseFrame(const uint8_t *frame, uint8_t size) {
    if (frame == nullptr || size < 4U || frame[0] != config_.address ||
        frame[size - 1U] != Y42Protocol::kFixedChecksum) {
        ++feedback_.invalid_frame_count;
        return false;
    }

    const uint8_t function = frame[1];
    feedback_.last_function = function;

    if (function == 0x35U && size == 6U) {
        float sign = frame[2] == 0x01U ? -1.0f : 1.0f;
        if (config_.reversed) {
            sign = -sign;
        }
        feedback_.speed_rpm = sign * static_cast<float>(readU16Be(&frame[3])) * kXSpeedUnitRpm;
    } else if (function == 0x36U && size == 8U) {
        float sign = frame[2] == 0x01U ? -1.0f : 1.0f;
        if (config_.reversed) {
            sign = -sign;
        }
        feedback_.position_deg =
            sign * static_cast<float>(readU32Be(&frame[3])) * kXFeedbackAngleUnitDeg;
    } else if (function == 0x37U && size == 8U) {
        float sign = frame[2] == 0x01U ? -1.0f : 1.0f;
        if (config_.reversed) {
            sign = -sign;
        }
        feedback_.position_error_deg =
            sign * static_cast<float>(readU32Be(&frame[3])) * kXFeedbackErrorUnitDeg;
    } else if (function == 0x3AU && size == 4U) {
        feedback_.status_flags = frame[2];
        feedback_.enabled = (frame[2] & 0x01U) != 0U;
        feedback_.position_reached = (frame[2] & 0x02U) != 0U;
        feedback_.stalled = (frame[2] & 0x04U) != 0U;
        feedback_.stall_protected = (frame[2] & 0x08U) != 0U;
        feedback_.left_limit_active = (frame[2] & 0x10U) != 0U;
        feedback_.right_limit_active = (frame[2] & 0x20U) != 0U;
    } else if (size == 4U) {
        feedback_.last_response = static_cast<Y42Response>(frame[2]);
        if (frame[2] != static_cast<uint8_t>(Y42Response::Accepted) &&
            frame[2] != static_cast<uint8_t>(Y42Response::MotionCompleted) &&
            frame[2] != static_cast<uint8_t>(Y42Response::ParameterError) &&
            frame[2] != static_cast<uint8_t>(Y42Response::FormatError)) {
            ++feedback_.invalid_frame_count;
            return false;
        }
    } else {
        ++feedback_.invalid_frame_count;
        return false;
    }

    ++feedback_.valid_frame_count;
    return true;
}

uint8_t Y42Motor::expectedFrameSize(uint8_t function) const {
    switch (function) {
        case 0x35U:
            return 6U;
        case 0x36U:
        case 0x37U:
            return 8U;
        case 0x0AU:
        case 0x0EU:
        case 0x3AU:
        case 0xF3U:
        case 0xF6U:
        case 0xFDU:
        case 0xFEU:
            return 4U;
        default:
            return 0U;
    }
}

Y42Direction Y42Motor::directionFor(float signed_value) const {
    bool cw = signed_value >= 0.0f;
    if (config_.reversed) {
        cw = !cw;
    }
    return cw ? Y42Direction::Cw : Y42Direction::Ccw;
}

uint16_t Y42Motor::readU16Be(const uint8_t *src) {
    return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8U) | src[1]);
}

uint32_t Y42Motor::readU32Be(const uint8_t *src) {
    return (static_cast<uint32_t>(src[0]) << 24U) |
           (static_cast<uint32_t>(src[1]) << 16U) |
           (static_cast<uint32_t>(src[2]) << 8U) | static_cast<uint32_t>(src[3]);
}

float Y42Motor::absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

bool Y42Motor::finitePositive(float value) {
    return std::isfinite(value) && value > 0.0f;
}

}  // namespace yuntai::motor
