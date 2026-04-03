#include "acoustics_ros_node.hpp"

#include <memory>
#include <stdexcept>

AcousticsRosNode::AcousticsRosNode()
: Node("acoustics_ros_node")
{
    publisher_ = this->create_publisher<acoustics_msgs::msg::AcousticsReading>(
        "acoustics/data", 10);

    can_status status = driver_.init_can();
    if (status != can_status::OK) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize acoustics CAN driver");
    }

    status = driver_.start_async_read(
        std::bind(&AcousticsRosNode::acoustics_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (status != can_status::OK) {
        RCLCPP_FATAL(this->get_logger(), "Failed to start async acoustics read");
    }

    RCLCPP_INFO(this->get_logger(), "Acoustics ROS node started");
}

void AcousticsRosNode::acoustics_callback(const AcousticsData& data, can_status status)
{
    if (status != can_status::OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to read acoustics data from CAN");
        return;
    }

    acoustics_msgs::msg::AcousticsReading msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "acoustics";
    msg.x = data.x;
    msg.y = data.y;
    msg.z = data.z;
    msg.weight = data.weight;

    publisher_->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcousticsRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
