#pragma once

#include <linux/can.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <vortex_msgs/msg/operation_mode.hpp>

#include "can_interface.hpp"
#include "can_registry.hpp"

class CanInterfaceNode : public rclcpp::Node {
   public:
    explicit CanInterfaceNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CanInterfaceNode() override;

   private:
    void init_registry();

    std::string make_log_filename() const;

    static uint32_t get_can_id(const canfd_frame& frame);

    void receive_loop();
    void handle_frame(const canfd_frame& frame);

    void handle_bms_cell_voltages(const canfd_frame& frame);
    void handle_bms_current(const canfd_frame& frame);
    void handle_bms_temperatures(const canfd_frame& frame);

    void handle_bms_alert_ssa(const canfd_frame& frame);
    void handle_bms_alert_pfa1(const canfd_frame& frame);
    void handle_bms_alert_pfa2(const canfd_frame& frame);

    void handle_pressure_sample(const canfd_frame& frame);
    void handle_leakage_alarm(const canfd_frame& frame);
    void handle_pi_status(const canfd_frame& frame);

    void operation_mode_callback(
        const vortex_msgs::msg::OperationMode::SharedPtr msg);

   private:
    std::string can_interface_name_;
    std::string log_directory_;

    bool print_enabled_{true};
    bool publish_decoded_{true};
    bool start_bms_on_startup_{true};

    std::atomic<bool> running_{false};
    std::thread receive_thread_;

    CanRegistry registry_;
    can_interface can_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decoded_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        bms_cell_voltages_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bms_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bms_current_counts_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        bms_temperatures_pub_;

    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
        bms_alert_ssa_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
        bms_alert_pfa1_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr bms_alert_pfa2_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        pressure_temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr leakage_alarm_pub_;

    rclcpp::Subscription<vortex_msgs::msg::OperationMode>::SharedPtr
        operation_mode_sub_;

    uint8_t current_operation_mode_ = 255;
};
