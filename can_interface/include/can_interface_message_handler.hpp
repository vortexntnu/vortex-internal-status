#ifndef CAN_INTERFACE_MESSAGE_HANDLER_HPP
#define CAN_INTERFACE_MESSAGE_HANDLER_HPP

#include <spdlog/spdlog.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include "can_interface_ros.hpp"
#include <string>
#include <thread>
#include <vector>
#include "can_interface_driver.h"
#include "can_interface_utils.hpp"

void encoder_angles_handler(
    const CANFD_Message& msg,
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub,
    rclcpp::Clock::SharedPtr clock);
void pressure_handler(const CANFD_Message& msg);
void temp_handler(const CANFD_Message& msg);

void psm_handler(
    const CANFD_Message& msg,
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_,
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_);

#endif
