#include "Y42Protocol.hpp"

namespace yuntai::motor {

Y42Frame Y42Protocol::enable(uint8_t address, bool enabled, bool synchronized) {
    Y42Frame frame{};
    frame.size = 6U;
    frame.data[0] = address;
    frame.data[1] = 0xF3U;
    frame.data[2] = 0xABU;
    frame.data[3] = enabled ? 0x01U : 0x00U;
    frame.data[4] = synchronized ? 0x01U : 0x00U;
    frame.data[5] = kFixedChecksum;
    return frame;
}

Y42Frame Y42Protocol::speed(uint8_t address, Y42Direction direction,
                            uint16_t acceleration_rpm_per_sec, uint16_t speed_raw,
                            bool synchronized) {
    Y42Frame frame{};
    frame.size = 9U;
    frame.data[0] = address;
    frame.data[1] = 0xF6U;
    frame.data[2] = static_cast<uint8_t>(direction);
    putU16Be(&frame.data[3], acceleration_rpm_per_sec);
    putU16Be(&frame.data[5], speed_raw);
    frame.data[7] = synchronized ? 0x01U : 0x00U;
    frame.data[8] = kFixedChecksum;
    return frame;
}

Y42Frame Y42Protocol::trapezoidPosition(uint8_t address, Y42Direction direction,
                                        uint16_t acceleration_rpm_per_sec,
                                        uint16_t deceleration_rpm_per_sec,
                                        uint16_t max_speed_raw, uint32_t angle_raw,
                                        Y42PositionMode mode, bool synchronized) {
    Y42Frame frame{};
    frame.size = 16U;
    frame.data[0] = address;
    frame.data[1] = 0xFDU;
    frame.data[2] = static_cast<uint8_t>(direction);
    putU16Be(&frame.data[3], acceleration_rpm_per_sec);
    putU16Be(&frame.data[5], deceleration_rpm_per_sec);
    putU16Be(&frame.data[7], max_speed_raw);
    putU32Be(&frame.data[9], angle_raw);
    frame.data[13] = static_cast<uint8_t>(mode);
    frame.data[14] = synchronized ? 0x01U : 0x00U;
    frame.data[15] = kFixedChecksum;
    return frame;
}

Y42Frame Y42Protocol::stop(uint8_t address, bool synchronized) {
    Y42Frame frame{};
    frame.size = 5U;
    frame.data[0] = address;
    frame.data[1] = 0xFEU;
    frame.data[2] = 0x98U;
    frame.data[3] = synchronized ? 0x01U : 0x00U;
    frame.data[4] = kFixedChecksum;
    return frame;
}

Y42Frame Y42Protocol::clearPosition(uint8_t address) {
    Y42Frame frame{};
    frame.size = 4U;
    frame.data[0] = address;
    frame.data[1] = 0x0AU;
    frame.data[2] = 0x6DU;
    frame.data[3] = kFixedChecksum;
    return frame;
}

Y42Frame Y42Protocol::clearProtection(uint8_t address) {
    Y42Frame frame{};
    frame.size = 4U;
    frame.data[0] = address;
    frame.data[1] = 0x0EU;
    frame.data[2] = 0x52U;
    frame.data[3] = kFixedChecksum;
    return frame;
}

Y42Frame Y42Protocol::readSpeed(uint8_t address) {
    return Y42Frame{{address, 0x35U, kFixedChecksum}, 3U};
}

Y42Frame Y42Protocol::readPosition(uint8_t address) {
    return Y42Frame{{address, 0x36U, kFixedChecksum}, 3U};
}

Y42Frame Y42Protocol::readPositionError(uint8_t address) {
    return Y42Frame{{address, 0x37U, kFixedChecksum}, 3U};
}

Y42Frame Y42Protocol::readStatus(uint8_t address) {
    return Y42Frame{{address, 0x3AU, kFixedChecksum}, 3U};
}

void Y42Protocol::putU16Be(uint8_t *dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value >> 8U);
    dst[1] = static_cast<uint8_t>(value);
}

void Y42Protocol::putU32Be(uint8_t *dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>(value >> 24U);
    dst[1] = static_cast<uint8_t>(value >> 16U);
    dst[2] = static_cast<uint8_t>(value >> 8U);
    dst[3] = static_cast<uint8_t>(value);
}

}  // namespace yuntai::motor
