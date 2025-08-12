

#include "can_interface_message_handler.hpp"
#include <sys/types.h>
#include <cstdint>
#include <ctime>
#include "can_interface_driver.h"
#include "can_interface_ros.hpp"

#define VOLTAGE_SCALE 11.236
#define VOLTAGE_RANGE 6.144
#define DIODE_LOSS 0.8
// #define CURRENT_OFFSET      0.595
#define CURRENT_OFFSET 0.6004
#define CURRENT_SENSITIVITY 0.0255

static constexpr void psm_unit_conversion(const uint8_t* raw_data,
                                          double& current,
                                          double& voltage) {
    int16_t raw_voltage = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    voltage =
        ((raw_voltage * VOLTAGE_RANGE) / 32768.0) * VOLTAGE_SCALE + DIODE_LOSS;

    int16_t raw_current = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    current = (CURRENT_OFFSET - ((raw_current * VOLTAGE_RANGE) / 32768.0)) /
              CURRENT_SENSITIVITY;
}

void encoder_angles_handler(
    const CANFD_Message& msg,
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub,
    rclcpp::Clock::SharedPtr clock) {
    std::vector<double> encoder_angles;
    convert_angles_to_radians(msg.data, encoder_angles);

    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = clock->now();
    joint_state_msg.name = {"shoulder", "wrist", "grip"};
    joint_state_msg.position = encoder_angles;

    joint_state_pub->publish(joint_state_msg);
}

void psm_handler(
    const CANFD_Message& msg,
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_,
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_) {
    double current, voltage;
    psm_unit_conversion(msg.data, current, voltage);
    auto voltage_msg = std_msgs::msg::Float64();
    voltage_msg.data = voltage;
    voltage_pub_->publish(voltage_msg);
    auto current_msg = std_msgs::msg::Float64();
    current_msg.data = current;
    current_pub_->publish(current_msg);
}
