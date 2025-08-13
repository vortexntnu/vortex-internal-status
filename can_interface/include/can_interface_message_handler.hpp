#ifndef CAN_INTERFACE_MESSAGE_HANDLER_HPP
#define CAN_INTERFACE_MESSAGE_HANDLER_HPP

#include <linux/can.h>
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
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <thread>
#include <vector>
#include "can_interface_driver.h"
#include "can_interface_utils.hpp"

void encoder_angles_handler(
    const struct canfd_frame& msg,
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub,
    rclcpp::Clock::SharedPtr clock);
void pressure_handler(const struct canfd_frame& msg);
void temp_handler(const struct canfd_frame& msg);

void psm_handler(
    const struct canfd_frame& msg,
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_,
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_);

#endif
