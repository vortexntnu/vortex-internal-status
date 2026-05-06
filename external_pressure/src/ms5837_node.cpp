#include "ms5837_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

MS5837Node::MS5837Node(const rclcpp::NodeOptions& options)
    : Node("ms5837_node", options),
      sensor_(declare_parameter<std::string>("i2c_device", "/dev/i2c-1"),
              static_cast<uint8_t>(declare_parameter<int>("i2c_address", 0x76)))
{
    fluid_density_ = declare_parameter<double>("fluid_density", 1029.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "ms5837_link");
    publish_depth_ = declare_parameter<bool>("publish_depth", true);
    publish_altitude_ = declare_parameter<bool>("publish_altitude", false);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);

    sensor_.setFluidDensity(static_cast<float>(fluid_density_));

    const int forced_model = declare_parameter<int>("model", 0);
    if (forced_model == 1) {
        sensor_.setModel(MS5837::MODEL_02BA);
    } else if (forced_model == 2) {
        sensor_.setModel(MS5837::MODEL_30BA);
    }

    if (!sensor_.init()) {
        RCLCPP_FATAL(get_logger(), "Failed to initialize MS5837 sensor");
        throw std::runtime_error("MS5837 init failed");
    }

    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("pressure", 10);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);

    if (publish_depth_) {
        depth_pub_ = create_publisher<std_msgs::msg::Float32>("depth", 10);
    }

    if (publish_altitude_) {
        altitude_pub_ = create_publisher<std_msgs::msg::Float32>("altitude", 10);
    }

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MS5837Node::update, this));

    RCLCPP_INFO(get_logger(), "MS5837 node started");
}

void MS5837Node::update()
{
    if (!sensor_.read()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MS5837 read failed");
        return;
    }

    const auto now = get_clock()->now();

    sensor_msgs::msg::FluidPressure pressure_msg;
    pressure_msg.header.stamp = now;
    pressure_msg.header.frame_id = frame_id_;
    pressure_msg.fluid_pressure = sensor_.pressure(PressureUnit::Pa);
    pressure_msg.variance = 0.0;
    pressure_pub_->publish(pressure_msg);

    sensor_msgs::msg::Temperature temp_msg;
    temp_msg.header.stamp = now;
    temp_msg.header.frame_id = frame_id_;
    temp_msg.temperature = sensor_.temperature();
    temp_msg.variance = 0.0;
    temp_pub_->publish(temp_msg);

    if (publish_depth_ && depth_pub_) {
        std_msgs::msg::Float32 depth_msg;
        depth_msg.data = sensor_.depth();
        depth_pub_->publish(depth_msg);
    }

    if (publish_altitude_ && altitude_pub_) {
        std_msgs::msg::Float32 altitude_msg;
        altitude_msg.data = sensor_.altitude();
        altitude_pub_->publish(altitude_msg);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MS5837Node>());
    rclcpp::shutdown();
    return 0;
}
