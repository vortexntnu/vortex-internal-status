#include "am_drone_node.hpp"



DroneNode::DroneNode() : Node("drone_node"){
    //TODO
    set_subscriber();
    init_connection();

    timer_ = this->create_wall_timer(5000ms, std::bind(&DroneNode::acoustic_callback, this));
}

void DroneNode::init_connection(){
    this->declare_parameter<std::string>("device");
    this->declare_parameter<int>("baudrate",9600);
    this->declare_parameter<int>("channel",1);
    this->declare_parameter<int>("level",4);
    this->declare_parameter<bool>("diagnostic",false);
    this->declare_parameter<double>("timeout",0.5);

    std::string device=this->get_parameter("device").as_string();
    int baudrate=this->get_parameter("baudrate").as_int();
    int channel=this->get_parameter("channel").as_int();
    int level=this->get_parameter("level").as_int();
    bool diagnostic=this->get_parameter("diagnostic").as_bool();
    float timeout=static_cast<float>(this->get_parameter("timeout").as_double());

    drone_modem_=AcousticModemDriver(device,baudrate,channel,level,diagnostic,timeout);
}

void DroneNode::set_subscriber(){
    // Depends on the topic in which we will read the data
    subscription_=this->create_subscription<std_msgs::msg::String>("data_topic", 10,
         std::bind(&DroneNode::data_callback),this,std::placeholders::_1);
}


void DroneNode::acoustic_callback(){
    //TODO
    if(!latest_.empty()){
        drone_modem_.send_msg(latest_);
    }
}


void DroneNode::data_callback(const std_msgs::msg::String::SharedPtr msg) const{
    //TODO
    latest_=msg->data;
}
