#ifndef AM_BASE_NODE.HPP
#define AM_BASE_NODE.HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include "am_driver.hpp"

/**
 * Creating two nodes, one for each system
 * Initially implementing uni-directional communication:
 * drone -> base station
 * 
 * this one should have publisher and then through acoustic channel should read data 
 * 
 * (read_packet)
 */


class BaseNode : public rclcpp::Node {
    
    public:
    explicit BaseNode();

    private:
    /**
     * initialize the connection by creating AcousticModemDriver object
     */
    void init_connection();


    void set_publishers();

    /**
     * receive message from acoustic modem 
     */ 
    void receive_message_timer();

    /**
     * build message based on order given by header
     */
    void rebuild_message();
    
    /**
     * publish message in correct topic based by header type
     */
    void publish();
    
    /**
     * need method to extract header from each packet, organize order and data type and reconstruct message
     * 
     * then publish it in the correct topic
     */
    


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_3_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_4_;
    std::unique_ptr<AcousticModemDriver> base_modem_;
    //std::string latest_;
    rclcpp::TimerBase::SharedPtr timer_;
}


#endif