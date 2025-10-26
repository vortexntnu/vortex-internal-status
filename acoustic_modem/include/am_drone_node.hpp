#ifndef AM_ROS_HPP
#define AM_ROS_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * Creating two nodes, one for each system
 * Initially implementing uni-directional communication:
 * drone -> base station
 * 
 * this one should have subscriber and then through acoustic channel should send data to base station
 * 
 *(send_msg)
 */

class DroneNode : public rclcpp::Node {
    
    public:
    explicit DroneNode();

    private:
    //void set_sub_and_pub();


    
    
    
}


#endif