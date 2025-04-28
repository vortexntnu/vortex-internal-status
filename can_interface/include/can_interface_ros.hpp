#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

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
#include <string>
#include <thread>
#include <vector>
#include "can_interface_driver.h"
#include "can_interface_utils.hpp"

class CANInterface : public rclcpp::Node {
   public:
    CANInterface();

    ~CANInterface();

   private:
    /**
     * @brief Extract parameters from the config file.
     */
    void extract_parameters();

    void set_publisher_and_subsribers();

    /**
     * @brief Callback function for the joystick message.
     * @param msg The joystick message.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void can_receive_loop();
    void on_can_message(const CANFD_Message& msg);
    /**
     * @brief Convert a vector of PWM values to a ROS message.
     * @param vec The vector of PWM values.
     * @return The ROS message.
     */
    std_msgs::msg::Int16MultiArray array_to_msg(
        std::array<std::uint16_t, 3> arr);

    std::string joy_topic_;
    std::string pwm_topic_;
    int pwm_gain_;
    int pwm_idle_;
    std::string joint_state_topic_;
    std::string can_interface_;
    std::thread can_thread_;
    bool running_;

    CANFD_Message canMsg;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_msg_time_;
};

#endif  // CAN_INTERFACE_HPP
