#ifndef AM_BASE_NODE.HPP
#define AM_BASE_NODE.HPP

#include <rclcpp/rclcpp.hpp>

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
    void set_sub_and_pub();


    /**
     * read_modem_data_callback, will read data from the modem and publish them in the
     * correct topic  
     */ 
    void read_modem_data_callback;
    /**
     * used in read_modem_data_callback to actually publishing in the topic
     */
    void publish_x;
    
    /**
     * need a function to actually 
     */
    
}


#endif