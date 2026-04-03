#ifndef ACOUSTICS_ROS_NODE_HPP_
#define ACOUSTICS_ROS_NODE_HPP_



#include "rclcpp/rclcpp.hpp"
#include "acoustics_interface_driver.hpp"
#include "custom_msgs/msg/bearing_measurement.hpp"

class AcousticsRosNode : public rclcpp::Node {
public:
    AcousticsRosNode();

private:
    void acoustics_callback(const AcousticsData& data, can_status status);

    AcousticsInterfaceDriver driver_;
    rclcpp::Publisher<vortex_msgs::msg::BearingMeasurement>::SharedPtr publisher_;
};

#endif
