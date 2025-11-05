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
    explicit DroneNode();

   private:
    /**
     * initialize the connection by creating AcousticModemDriver object
     */
    void init_connection();

    /**
     * Create the subscriber to the topic
     */
    void set_subscriber();

    /**
     * Called every # second to send different messages to the other modem
     * through acoustic communication
     *
     * TODO: fix logic, probably seconds not correct
     */
    void acoustic_callback();

    /**
     * the idea is this but it depends on what the data are and how we get them
     * from the topic:
     *
     * we save te last string received and then send it using the timer with an
     * interval
     *
     * To fix because we will probably lose information
     *
     * idea: queue for each new message in the topic and send them in order
     * by giving each of them a sending time based on the length of the message
     * ?
     */
    void data_callback();

    /**
     * subscriber to the data-topic, the type is now string so that
     * it's easier with send_msg(string), can be changed
     */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::unique_ptr<AcousticModemDriver> drone_modem_;
    std::string latest_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
