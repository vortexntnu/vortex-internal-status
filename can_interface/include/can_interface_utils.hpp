#ifndef CAN_INTERFACE_UTILS_HPP
#define CAN_INTERFACE_UTILS_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

std::vector<double> convert_angles_to_radians(const uint8_t* can_data);

static constexpr std::uint16_t joy_to_pwm(std::uint16_t pwm_idle,
                                          std::uint16_t pwm_gain,
                                          const double joy_value) {
    return static_cast<std::uint16_t>(pwm_idle + pwm_gain * joy_value);
}

static constexpr std::array<std::uint8_t, 2> pwm_to_can_data(
    std::uint16_t pwm) {
    return {static_cast<std::uint8_t>((pwm >> 8) & 0xFF),
            static_cast<std::uint8_t>(pwm & 0xFF)};
}

static constexpr std::uint16_t can_to_encoder_angles(
    std::array<std::uint8_t, 2> data) {
    return (static_cast<std::uint16_t>(data[0]) << 8) | data[1];
}

static constexpr double raw_angle_to_radians(std::uint16_t raw_angle) {
    return (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * M_PI);
}

#endif  // !CAN_INTERFACE_UTILS_HPP
