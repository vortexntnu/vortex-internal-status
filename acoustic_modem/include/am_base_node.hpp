#ifndef AM_BASE_NODE_HPP
#define AM_BASE_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "am_driver.hpp"
#include "tdma_link.hpp"
#include "am_driver_split.hpp"
#include "am_driver_iface.hpp"

/**
 * Creating two nodes, one for each system
 * Initially implementing uni-directional communication:
 * drone -> base station
 *
 * this one should have publisher and then through acoustic channel should read
 * data
 *
 * (read_packet)
 */

class BaseNode : public rclcpp::Node {
   public:
    explicit BaseNode();
    ~BaseNode();

   private:
    /**
     * initialize the connection by creating AcousticModemDriver object
     */
    void init_connection();
    void setup_tdma();

    void set_publishers();

    void set_subscribers();

    void persistent_callback(const std_msgs::msg::UInt16::SharedPtr msg);

    void poll_and_publish_rx();

    void sync_callback(const std_msgs::msg::UInt8::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_1_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_2_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_3_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_0_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr persistent_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sync_sub_;
    std::unique_ptr<IAcousticModemDriver> driver_;
    std::unique_ptr<TDMAManager> tdma_;
    std::unique_ptr<TDMALink> link_;
    // std::string latest_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
