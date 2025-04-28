#ifndef PSM_ORIN_ROS_HPP
#define PSM_ORIN_ROS_HPP

#include <chrono>
#include <ctime>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"

class PSMOrinNode : public rclcpp::Node {
   public:
    explicit PSMOrinNode();

   private:
    void set_subscribers_and_publishers();
    void publish_voltage();
    void publish_current();
    void publish_pressure();
    void read_ads_callback();
    void read_pressure_callback();

    rclcpp::TimerBase::SharedPtr read_psm_timer_;
    rclcpp::TimerBase::SharedPtr read_pressure_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_pub_;
    rclcpp::Time last_msg_time_;

    double voltage = 0.0;
    double current = 0.0;
    double pressure = 0.0;
};

#endif  // PSM_ORIN_ROS_HPP
