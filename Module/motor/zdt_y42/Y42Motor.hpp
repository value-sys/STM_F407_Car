#ifndef Y42_MOTOR_HPP
#define Y42_MOTOR_HPP

#include <stdint.h>

extern "C" {
#include "usart.h"
}

#include "Y42Protocol.hpp"

namespace yuntai::motor {

enum class Y42Response : uint8_t {
    None = 0x00U,
    Accepted = 0x02U,
    ParameterError = 0xE2U,
    FormatError = 0xEEU,
    MotionCompleted = 0x9FU,
};

struct Y42Config {
    uint8_t address{1U};
    float command_angle_unit_deg{0.1f};
    bool reversed{false};
    uint32_t uart_timeout_ms{10U};
};

struct Y42Feedback {
    float speed_rpm{0.0f};
    float position_deg{0.0f};
    float position_error_deg{0.0f};
    bool enabled{false};
    bool position_reached{false};
    bool stalled{false};
    bool stall_protected{false};
    bool left_limit_active{false};
    bool right_limit_active{false};
    uint8_t status_flags{0U};
    uint8_t last_function{0U};
    Y42Response last_response{Y42Response::None};
    uint32_t valid_frame_count{0U};
    uint32_t invalid_frame_count{0U};
};

class Y42Motor {
public:
    Y42Motor() = default;
    Y42Motor(UART_HandleTypeDef *huart, const Y42Config &config);

    void attach(UART_HandleTypeDef *huart, const Y42Config &config);
    const Y42Config &config() const;
    const Y42Feedback &feedback() const;
    bool isAttached() const;

    HAL_StatusTypeDef enable(bool enabled);
    HAL_StatusTypeDef stop();
    HAL_StatusTypeDef clearPosition();
    HAL_StatusTypeDef clearProtection();

    HAL_StatusTypeDef setSpeedRpm(float speed_rpm, float acceleration_rpm_per_sec);
    HAL_StatusTypeDef moveRelativeDeg(float angle_deg, float max_speed_rpm,
                                      float acceleration_rpm_per_sec,
                                      float deceleration_rpm_per_sec);
    HAL_StatusTypeDef moveAbsoluteDeg(float angle_deg, float max_speed_rpm,
                                      float acceleration_rpm_per_sec,
                                      float deceleration_rpm_per_sec);
    HAL_StatusTypeDef moveFromCurrentDeg(float angle_deg, float max_speed_rpm,
                                         float acceleration_rpm_per_sec,
                                         float deceleration_rpm_per_sec);

    HAL_StatusTypeDef requestSpeed();
    HAL_StatusTypeDef requestPosition();
    HAL_StatusTypeDef requestPositionError();
    HAL_StatusTypeDef requestStatus();

    bool feedRxByte(uint8_t byte);
    HAL_StatusTypeDef sendFrame(const Y42Frame &frame);

private:
    HAL_StatusTypeDef moveDeg(float angle_deg, float max_speed_rpm,
                              float acceleration_rpm_per_sec,
                              float deceleration_rpm_per_sec, Y42PositionMode mode);
    bool parseFrame(const uint8_t *frame, uint8_t size);
    uint8_t expectedFrameSize(uint8_t function) const;
    Y42Direction directionFor(float signed_value) const;
    static uint16_t readU16Be(const uint8_t *src);
    static uint32_t readU32Be(const uint8_t *src);
    static float absoluteValue(float value);
    static bool finitePositive(float value);

    UART_HandleTypeDef *huart_{nullptr};
    Y42Config config_{};
    Y42Feedback feedback_{};
    uint8_t rx_frame_[8]{};
    uint8_t rx_size_{0U};
    uint8_t rx_expected_size_{0U};
};

}  // namespace yuntai::motor

#endif
