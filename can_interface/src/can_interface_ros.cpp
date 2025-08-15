#include "can_interface_ros.hpp"
#include <linux/can.h>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <array>
#include <cstdint>
#include "can_interface_driver.h"
#include "can_interface_message_handler.hpp"
#include "can_interface_utils.hpp"

CANInterface::CANInterface() : Node("can_interface_node") {
    extract_parameters();
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10,
        std::bind(&CANInterface::joy_callback, this, std::placeholders::_1));
    pwm_pub_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(pwm_topic_, 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10);

    can_thread_ = std::thread(&CANInterface::can_receive_loop, this);

    last_msg_time_ = this->now();
    canfd_init(can_interface_.c_str());
    spdlog::info("CAN interface node started");
}
CANInterface::~CANInterface() {
    running_ = false;
    if (can_thread_.joinable()) {
        can_thread_.join();
    }
}

void CANInterface::extract_parameters() {
    this->declare_parameter<std::string>("topics.joy");
    this->declare_parameter<std::string>("topics.pwm");
    this->declare_parameter<std::string>("topics.joint_state");
    this->declare_parameter<int>("pwm.gain");
    this->declare_parameter<int>("pwm.idle");
    this->declare_parameter<std::string>("can.interface");

    this->joy_topic_ = this->get_parameter("topics.joy").as_string();
    this->pwm_idle_ = this->get_parameter("pwm.idle").as_int();
    this->joint_state_topic_ =
        this->get_parameter("topics.joint_state").as_string();
    this->pwm_topic_ = this->get_parameter("topics.pwm").as_string();
    this->pwm_gain_ = this->get_parameter("pwm.gain").as_int();
    this->can_interface_ = this->get_parameter("can.interface").as_string();
}

void CANInterface::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::array<uint16_t, 3> pwm_values;
    double shoulder_value = msg->axes[1];
    double wrist_value = msg->axes[0];
    double grip_value = msg->axes[3];

    pwm_values[0] = joy_to_pwm(pwm_idle_, pwm_gain_, shoulder_value);
    pwm_values[1] = joy_to_pwm(pwm_idle_, pwm_gain_, wrist_value);
    pwm_values[2] = joy_to_pwm(pwm_idle_, pwm_gain_, grip_value);
    
    struct canfd_frame frame;
    
    frame.can_id = SET_GRIPPER_PWM;

    pwm_to_can_data(frame.data, pwm_values);

    frame.len = pwm_values.size() * 2;

    std_msgs::msg::Int16MultiArray pwm_msg = array_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);

    canfd_send(&frame);

    if (msg->buttons[0]) {
        frame.can_id = STOP_GRIPPER;
        frame.data[0] = 0;
        frame.len = 1;
        canfd_send(&frame);

    } else if (msg->buttons[1]) {
        frame.can_id = START_GRIPPER;
        frame.data[0] = 0;
        frame.len = 1;
        canfd_send(&frame);
    }
}

std_msgs::msg::Int16MultiArray CANInterface::array_to_msg(
    std::array<std::uint16_t, 3> arr) {
    std_msgs::msg::Int16MultiArray msg;
    std::ranges::copy(arr, std::back_inserter(msg.data));
    return msg;
}
void CANInterface::can_receive_loop() {
    while (running_) {
        struct canfd_frame msg;
        int ret = canfd_recieve(&msg, 1000);
        if (ret == 0) {
            on_can_message(msg);
        }
    }
}

void CANInterface::on_can_message(const struct canfd_frame& msg) {
    switch (msg.can_id) {
        case ENCODER_ANGLES:
            encoder_angles_handler(msg, joint_state_pub_, this->get_clock());
            break;
        case PRESSURE_INTERNAL:
            break;
        case PRESSURE_EXTERNAL:
            break;
        case TEMP_INTERNAL:
            break;
        case PSM:
            psm_handler(msg, voltage_pub_, current_pub_);
            break;
        default:
            break;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANInterface>());
    rclcpp::shutdown();
    return 0;
}
