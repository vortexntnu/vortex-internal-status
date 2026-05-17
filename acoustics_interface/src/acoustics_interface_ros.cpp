#include "acoustics_interface_ros.hpp"

#include <spdlog/spdlog.h>
#include <functional>
#include <memory>

AcousticsRosNode::AcousticsRosNode() : Node("acoustics_ros_node") {
    publisher_ = this->create_publisher<vortex_msgs::msg::BearingMeasurement>(
        "acoustics/bearing_measurement", 10);

    can_status status = driver_.init_can();
    if (status != can_status::OK) {
        spdlog::error("Failed to initialize acoustics CAN driver");
        return;
    }

    status = driver_.start_async_read(
        std::bind(&AcousticsRosNode::acoustics_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (status != can_status::OK) {
        spdlog::error("Failed to start async acoustics read");
        return;
    }

    spdlog::info("Acoustics ROS node started");
}

void AcousticsRosNode::acoustics_callback(const AcousticsData& data,
                                          can_status status) {
    if (status != can_status::OK) {
        spdlog::warn("Failed to read acoustics data from CAN");
        return;
    }

    vortex_msgs::msg::BearingMeasurement msg;

    msg.bearing.header.stamp = this->now();
    msg.bearing.header.frame_id = "acoustics";

    msg.bearing.vector.x = static_cast<double>(data.x);
    msg.bearing.vector.y = static_cast<double>(data.y);
    msg.bearing.vector.z = static_cast<double>(data.z);

    msg.weight = static_cast<double>(data.weight);
    msg.target_id = 0;

    publisher_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AcousticsRosNode>();

    spdlog::info("Spinning acoustics ROS node...");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
