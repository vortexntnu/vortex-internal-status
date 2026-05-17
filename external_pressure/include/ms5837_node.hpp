#ifndef MS5837_NODE_HPP_
#define MS5837_NODE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float32.hpp"

#include "ms5837.hpp"

#include <string>

class MS5837Node : public rclcpp::Node
{
public:
    explicit MS5837Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void update();

    MS5837 sensor_;

    double fluid_density_;
    double rate_hz_;
    bool publish_depth_;
    bool publish_altitude_;
    std::string frame_id_;

    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // !MS5837_NODE_HPP_
