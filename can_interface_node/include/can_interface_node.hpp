#pragma once

#include <linux/can.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "can_interface.hpp"
#include "can_registry.hpp"

class CanInterfaceNode : public rclcpp::Node
{
public:
    explicit CanInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CanInterfaceNode() override;

private:
    void init_registry();

    static uint32_t get_can_id(const canfd_frame& frame);
    static uint64_t steady_time_us();

    void receive_loop();
    void handle_frame(const canfd_frame& frame);

private:
    std::string can_interface_name_;

    bool publish_decoded_{true};
    bool start_bms_on_startup_{true};

    std::atomic<bool> running_{false};
    std::thread receive_thread_;

    CanRegistry registry_;
    can_interface can_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decoded_pub_;
};
