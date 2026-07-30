#ifndef QD4310_MOTOR_HPP
#define QD4310_MOTOR_HPP

#include <stdint.h>

extern "C" {
#include "usart.h"
}

namespace yuntai::motor {

enum class Qd4310Command : uint8_t {
    Nop = 0x00,
    Enable = 0x01,
    Disable = 0x02,
    Current = 0x03,
    Speed = 0x04,
    Angle = 0x05,
    LowSpeed = 0x06,
    StepAngle = 0x07,
    ClearError = 0xFB,
    SetZeroPos = 0xFE,
    Reboot = 0xFF,
};

struct Qd4310Feedback {
    bool enabled{false};
    uint8_t id{0U};
    uint8_t state_raw{0U};
    float speed_rpm{0.0f};
    float angle_deg{0.0f};
    float current_a{0.0f};
};

struct Qd4310Limits {
    float max_speed_rpm{1000.0f};
    float min_speed_rpm{-1000.0f};
    float max_current_a{10.0f};
    float min_current_a{-10.0f};
    float max_step_angle_deg{360.0f};
    float min_step_angle_deg{-360.0f};
    float min_angle_deg{0.0f};
    float max_angle_deg{360.0f};
};

struct Qd4310ServoConfig {
    float zero_offset_deg{0.0f};
    float min_output_deg{-180.0f};
    float max_output_deg{180.0f};
    float max_output_speed_rpm{300.0f};
    bool reversed{false};
};

struct Qd4310ServoFeedback {
    bool enabled{false};
    float output_angle_deg{0.0f};
    float motor_angle_deg{0.0f};
    float speed_rpm{0.0f};
    float current_a{0.0f};
};

class Qd4310Motor {
public:
    Qd4310Motor() = default;
    Qd4310Motor(UART_HandleTypeDef *huart, uint8_t id);

    void attach(UART_HandleTypeDef *huart, uint8_t id);
    void setLimits(const Qd4310Limits &limits);

    HAL_StatusTypeDef enable();
    HAL_StatusTypeDef disable();
    HAL_StatusTypeDef reboot();
    HAL_StatusTypeDef clearError();
    HAL_StatusTypeDef setZeroPos();
    HAL_StatusTypeDef setSpeedRpm(float speed_rpm);
    HAL_StatusTypeDef setLowSpeedRpm(float speed_rpm);
    HAL_StatusTypeDef setAngleDeg(float angle_deg);
    HAL_StatusTypeDef setStepAngleDeg(float step_angle_deg);
    HAL_StatusTypeDef setCurrentA(float current_a);
    HAL_StatusTypeDef stop();

    HAL_StatusTypeDef sendRaw(Qd4310Command cmd, int16_t value);
    void updateFeedback(const uint8_t feedback[8]);

    const Qd4310Feedback &feedback() const;
    bool isAttached() const;

    static constexpr float kPi = 3.14159265358979323846f;

private:
    static float clampValue(float value, float min_value, float max_value);
    static uint8_t crc8(const uint8_t *data, uint32_t len);
    static uint8_t reverseBits(uint8_t data);

    UART_HandleTypeDef *huart_{nullptr};
    uint8_t id_{0U};
    Qd4310Limits limits_{};
    Qd4310Feedback feedback_{};
};

class Qd4310Servo {
public:
    Qd4310Servo() = default;
    Qd4310Servo(UART_HandleTypeDef *huart, uint8_t id);

    void attach(UART_HandleTypeDef *huart, uint8_t id);
    void setMotorLimits(const Qd4310Limits &limits);
    void setConfig(const Qd4310ServoConfig &config);

    HAL_StatusTypeDef enable();
    HAL_StatusTypeDef disable();
    HAL_StatusTypeDef clearError();
    HAL_StatusTypeDef reboot();
    HAL_StatusTypeDef stop();

    HAL_StatusTypeDef setOutputAngleDeg(float output_angle_deg);
    HAL_StatusTypeDef jogOutputAngleDeg(float delta_output_deg);
    HAL_StatusTypeDef setOutputSpeedRpm(float speed_rpm);
    HAL_StatusTypeDef holdCurrentOutputAngle();

    void setZeroOffsetDeg(float zero_offset_deg);
    void alignZeroOffsetToCurrentPosition(float current_output_deg = 0.0f);

    float currentOutputAngleDeg() const;
    float clampOutputAngleDeg(float output_angle_deg) const;
    float outputToMotorAngleDeg(float output_angle_deg) const;

    const Qd4310ServoConfig &config() const;
    Qd4310ServoFeedback feedback() const;
    Qd4310Motor &motor();
    const Qd4310Motor &motor() const;

private:
    static float wrap360(float angle_deg);

    Qd4310Motor motor_{};
    Qd4310ServoConfig config_{};
};

}  // namespace yuntai::motor

#endif
