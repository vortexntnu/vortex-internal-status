#ifndef ACOUSTICS_ROS_NODE_HPP_
#define ACOUSTICS_ROS_NODE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "acoustics_interface_driver.hpp"
#include "acoustics_msgs/msg/acoustics_reading.hpp"

class AcousticsRosNode : public rclcpp::Node {
public:
    AcousticsRosNode();

private:
    void acoustics_callback(const AcousticsData& data, can_status status);

    AcousticsInterfaceDriver driver_;
    rclcpp::Publisher<acoustics_msgs::msg::AcousticsReading>::SharedPtr publisher_;
};

#endif
