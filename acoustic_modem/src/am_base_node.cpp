#include "am_base_node.hpp"

BaseNode::BaseNode() : Node("base_node") {
    // TODO
    set_publishers();
    init_connection();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&BaseNode::poll_and_publish_rx, this));
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

    base_modem_ = std::make_unique<AcousticModemDriver>(device, baudrate, channel, level,
                                      diagnostic, timeout);
}

void BaseNode::set_publishers() {
    // we reserved 2 bit for the type of data so we'll have 4 types of data
    data_0_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_0", 10);
    data_1_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_1", 10);
    data_2_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_2", 10);
    data_3_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_3", 10);
}


void BaseNode::poll_and_publish_rx() {
    AcousticModemDriver::DecodedMessage msg;
    while (base_modem_->try_pop_decoded(msg)) {
        std_msgs::msg::Float32MultiArray out;
        for (std::uint8_t i = 0; i < msg.n_floats; ++i) {
            out.data.push_back(msg.floats[i]);
        }
        switch(msg.type) {
            case MsgType::Type_1: 
                data_1_->publish(out); 
                break;
            case MsgType::Type_2: 
                data_2_->publish(out); 
                break;
            case MsgType::Type_3: 
                data_3_->publish(out); 
                break;
            default:
                data_0_->publish(out); 
                break;
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}