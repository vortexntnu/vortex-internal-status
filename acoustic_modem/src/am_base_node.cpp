#include "am_driver.hpp"
#include "am_drone_node.hpp"

BaseNode::BaseNode() : Node("base_node") {
    // TODO
    set_publishers();
    init_connection();

    /**
     * possible future idea:
     * one timer for filling the map with data (higher frequency)
     * one timer for re-building the message (lower frequency)
     *
     * present idea: one timer for both filling the map and re-building the
     * message
     */

    timer_ = this->create_wall_timer(5000ms, std::bind(, this));
}

void BaseNode::init_connection() {
    this->declare_parameter<std::string>("device");
    this->declare_parameter<int>("baudrate", 9600);
    this->declare_parameter<int>("channel", 1);
    this->declare_parameter<int>("level", 4);
    this->declare_parameter<bool>("diagnostic", false);
    this->declare_parameter<double>("timeout", 0.5);

    std::string device = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int channel = this->get_parameter("channel").as_int();
    int level = this->get_parameter("level").as_int();
    bool diagnostic = this->get_parameter("diagnostic").as_bool();
    float timeout =
        static_cast<float>(this->get_parameter("timeout").as_double());

    base_modem_ = AcousticModemDriver(device, baudrate, channel, level,
                                      diagnostic, timeout);
}

void BaseNode::set_publishers() {
    // we reserved 2 bit for the type of data so we'll have 4 types of data
    data_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10);
    data_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
    data_3_ = this->create_publisher<std_msgs::msg::String>("topic_3", 10);
    data_4_ = this->create_publisher<std_msgs::msg::String>("topic_4", 10);
}

void BaseNode::rebuild_message() {}
