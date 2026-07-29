#ifndef Y42_PROTOCOL_HPP
#define Y42_PROTOCOL_HPP

#include <stdint.h>

namespace yuntai::motor {

enum class Y42Direction : uint8_t {
    Cw = 0x00U,
    Ccw = 0x01U,
};

enum class Y42PositionMode : uint8_t {
    RelativeToLastTarget = 0x00U,
    Absolute = 0x01U,
    RelativeToCurrent = 0x02U,
};

struct Y42Frame {
    uint8_t data[16]{};
    uint8_t size{0U};
};

class Y42Protocol {
public:
    static constexpr uint8_t kFixedChecksum = 0x6BU;
    static constexpr uint16_t kMaxAccelerationRpmPerSec = 65535U;
    static constexpr uint16_t kMaxSpeedRaw = 30000U;

    static Y42Frame enable(uint8_t address, bool enabled, bool synchronized = false);
    static Y42Frame speed(uint8_t address, Y42Direction direction,
                          uint16_t acceleration_rpm_per_sec, uint16_t speed_raw,
                          bool synchronized = false);
    static Y42Frame trapezoidPosition(uint8_t address, Y42Direction direction,
                                      uint16_t acceleration_rpm_per_sec,
                                      uint16_t deceleration_rpm_per_sec,
                                      uint16_t max_speed_raw, uint32_t angle_raw,
                                      Y42PositionMode mode, bool synchronized = false);
    static Y42Frame stop(uint8_t address, bool synchronized = false);
    static Y42Frame clearPosition(uint8_t address);
    static Y42Frame clearProtection(uint8_t address);
    static Y42Frame readSpeed(uint8_t address);
    static Y42Frame readPosition(uint8_t address);
    static Y42Frame readPositionError(uint8_t address);
    static Y42Frame readStatus(uint8_t address);

private:
    static void putU16Be(uint8_t *dst, uint16_t value);
    static void putU32Be(uint8_t *dst, uint32_t value);
};

}  // namespace yuntai::motor

#endif
