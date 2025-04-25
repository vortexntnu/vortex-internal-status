#include "can_interface_ros.hpp"
#include <spdlog/common.h>
#include <spdlog/spdlog.h>

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

    pwm_values.at(0) = joy_to_pwm(pwm_idle_, pwm_gain_, shoulder_value);
    pwm_values.at(1) = joy_to_pwm(pwm_idle_, pwm_gain_, wrist_value);
    pwm_values.at(2) = joy_to_pwm(pwm_idle_, pwm_gain_, grip_value);

    canMsg.id = SET_GRIPPER_PWM;
    canMsg.is_extended = false;
    canMsg.is_fd = true;

    auto joined_bytes = pwm_values |
                        std::views::transform([](std::uint16_t pwm) {
                            return pwm_to_can_data(pwm);
                        }) |
                        std::views::join;

    std::ranges::copy(joined_bytes, canMsg.data);

    canMsg.length = pwm_values.size() * 2;

    std_msgs::msg::Int16MultiArray pwm_msg = array_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);

    canfd_send(&canMsg);

    if (msg->buttons[0]) {
        canMsg.id = STOP_GRIPPER;
        canMsg.data[0] = 0;
        canMsg.length = 1;
        canfd_send(&canMsg);

    } else if (msg->buttons[1]) {
        canMsg.id = START_GRIPPER;
        canMsg.data[0] = 0;
        canMsg.length = 1;
        canfd_send(&canMsg);
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
        CANFD_Message msg;
        int ret = canfd_recieve(&msg, 1000);
        if (ret == 0) {
            on_can_message(msg);
        }
    }
}

void CANInterface::on_can_message(const CANFD_Message& msg) {
    switch (msg.id) {
        case ENCODER_ANGLES:
            const uint8_t encoder_status = msg.data[6];
            std::vector<double> encoder_angles;
            convert_angles_to_radians(msg.data, encoder_angles);

            for (int i = 0; i < NUM_ANGLES; i++) {
                if (encoder_status & (1 << i)) {
                    encoder_angles.at(i) = -1;
                    spdlog::error("Reading encoder {} failed", i);
                }
            }

            auto joint_state_msg = sensor_msgs::msg::JointState();

            joint_state_msg.header.stamp = this->now();
            joint_state_msg.name = {"shoulder", "wrist", "grip"};
            joint_state_msg.position = encoder_angles;

            joint_state_pub_->publish(joint_state_msg);
            break;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANInterface>());
    rclcpp::shutdown();
    return 0;
}
