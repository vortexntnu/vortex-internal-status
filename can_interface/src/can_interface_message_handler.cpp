

#include "can_interface_message_handler.hpp"
#include <ctime>

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
