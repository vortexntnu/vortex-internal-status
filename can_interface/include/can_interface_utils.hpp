#ifndef CAN_INTERFACE_UTILS_HPP
#define CAN_INTERFACE_UTILS_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#define NUM_ANGLES 3

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

static constexpr double raw_angle_to_radians(std::uint16_t raw_angle) {
    return (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * M_PI);
}

/**
 * @brief Converts from raw angle to radians
 * @param raw angle data
 * @param output array
 **/
static constexpr void convert_angles_to_radians(
    const uint8_t* can_data,
    std::vector<double>& encoder_angles) {
    for (size_t i = 0; i < NUM_ANGLES; ++i) {
        uint16_t raw_angle = (static_cast<uint16_t>(can_data[2 * i]) << 8) |
                             static_cast<uint16_t>(can_data[2 * i + 1]);
        encoder_angles.push_back(raw_angle_to_radians(raw_angle));
    }
}

#endif  // !CAN_INTERFACE_UTILS_HPP
