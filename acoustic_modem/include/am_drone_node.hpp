#ifndef AM_ROS_HPP
#define AM_ROS_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "am_driver.hpp"

/**
 * Creating two nodes, one for each system
 * Initially implementing uni-directional communication:
 * drone -> base station
 *
 * this one should have subscriber and then through acoustic channel should send
 *data to base station
 *
 *(send_msg)
 */

class DroneNode : public rclcpp::Node {
   public:
    DroneNode();
    ~DroneNode();
   private:
    /**
     * initialize the connection by creating AcousticModemDriver object
     */
    void init_connection();
    void setup_tdma();
    /**
     * Create the subscriber to the topic
     */
    void set_subscriber();
    // needed to publish data received in a particular topic, for now for the drone not needed
    void set_publisher();
    void poll_modem();
    void tx_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr persistent_pub_;
    std::unique_ptr<AcousticModemDriver> driver_;
    std::unique_ptr<TDMAManager> tdma_;
    std::unique_ptr<TDMALink> link_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string latest_;
};

#endif