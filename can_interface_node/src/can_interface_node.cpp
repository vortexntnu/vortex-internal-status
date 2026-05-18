#include "can_interface_node.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "can_decode.hpp"

CanInterfaceNode::CanInterfaceNode(const rclcpp::NodeOptions& options)
    : Node("can_interface_node", options) {
    can_interface_name_ =
        declare_parameter<std::string>("can_interface", "vcan0");
    publish_decoded_ = declare_parameter<bool>("publish_decoded", true);
    start_bms_on_startup_ =
        declare_parameter<bool>("start_bms_on_startup", true);

    init_registry();


    const can_status status = can_.init(can_interface_name_.c_str());
    if (status != can_status::OK) {
        throw std::runtime_error("Failed to init CAN interface: " +
                                 can_interface_name_);
    }

    if (publish_decoded_) {
        decoded_pub_ =
            create_publisher<std_msgs::msg::String>("can/decoded", 10);
    }

    if (start_bms_on_startup_) {
        uint8_t dummy = 0;
        can_.send(0x215, &dummy, 1);
        RCLCPP_INFO(get_logger(), "Sent BMS start command");
    }

    running_.store(true);
    receive_thread_ = std::thread(&CanInterfaceNode::receive_loop, this);

    RCLCPP_INFO(get_logger(), "CAN interface node listening on %s",
                can_interface_name_.c_str());
}

CanInterfaceNode::~CanInterfaceNode() {
    running_.store(false);

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (start_bms_on_startup_) {
        uint8_t dummy = 0;
        can_.send(0x216, &dummy, 1);
        RCLCPP_INFO(get_logger(), "Sent BMS stop command");
    }

    RCLCPP_INFO(get_logger(), "CAN interface node stopped");
}

void CanInterfaceNode::init_registry() {
    registry_.add({0x46D, "Gripper Encoder angles", decode_encoder_angles});
    registry_.add({0x45A, "Motor Controller Frame", decode_motor_frames});
    registry_.add({CAN_VOLTAGE_ID, "BMS cell voltages", decode_voltage});
    registry_.add({CAN_CURRENT_ID, "BMS current measurement", decode_current});
    registry_.add({CAN_ALERT_PFA_1_ID, "BMS alert PFA1", decode_alert_pfa_1});
    registry_.add({CAN_ALERT_PFA_2_ID, "BMS alert PFA2", decode_alert_pfa_2});
    registry_.add({CAN_ALERT_SSA_ID, "BMS alert SSA", decode_alert_ssa});
    registry_.add({CAN_TEMP_ID, "BMS Temperature", decode_temp});
    registry_.add({0x780, "Pressure Sample", decode_pressure_sample});
    registry_.add({0x100, "Leakage Alarm", decode_leakage_alarm});
}

uint32_t CanInterfaceNode::get_can_id(const canfd_frame& frame) {
    if (frame.can_id & CAN_EFF_FLAG) {
        return frame.can_id & CAN_EFF_MASK;
    }

    return frame.can_id & CAN_SFF_MASK;
}

uint64_t CanInterfaceNode::steady_time_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void CanInterfaceNode::receive_loop() {
    while (rclcpp::ok() && running_.load()) {
        canfd_frame frame{};

        const can_status status = can_.receive(frame, 1000);

        if (status == can_status::OK) {
            handle_frame(frame);
        } else if (status == can_status::ERR_RECEIVE) {
            continue;
        } else {
            RCLCPP_ERROR(get_logger(), "CAN receive error");
            break;
        }
    }
}

void CanInterfaceNode::handle_frame(const canfd_frame& frame) {
    const uint32_t id = get_can_id(frame);
    const uint64_t ts_us = steady_time_us();


    const CanMessageDef* def = registry_.find(id);

    std::ostringstream oss;

    if (def) {
        const std::string decoded = def->decode(frame.data, frame.len);

        oss << "ID=0x" << std::hex << id << std::dec
            << " LEN=" << static_cast<int>(frame.len) << " " << def->name
            << " | " << decoded;
    } else {
        oss << "ID=0x" << std::hex << id << std::dec
            << " LEN=" << static_cast<int>(frame.len) << " UNKNOWN";
    }

    const std::string output = oss.str();

    if (publish_decoded_ && decoded_pub_) {
        std_msgs::msg::String msg;
        msg.data = output;
        decoded_pub_->publish(msg);
    }
}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
