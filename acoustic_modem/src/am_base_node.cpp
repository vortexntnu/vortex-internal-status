#include "am_driver.hpp"
#include "am_drone_node.hpp"

BaseNode::BaseNode() : Node("base_node") {
    // TODO
    set_publishers();
    init_connection();

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&BaseNode::poll_and_publish_rx(), this));
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
    data_0_ = this->create_publisher<std_msgs::msg::String>("topic_0", 10);
    data_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10);
    data_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
    data_3_ = this->create_publisher<std_msgs::msg::String>("topic_3", 10);
}


void BaseNode::poll_and_publish_rx() {
    DecodedMessage msg;
    while (base_modem_.try_pop_decoded(msg)) {
        switch(msg.type) {
            case MsgType::Type_1: data_1_->publish(msg.floats); break;
            case MsgType::Type_2: data_2_->publish(msg.floats); break;
            case MsgType::Type_3: data_3_->publish(msg.floats); break;
            default:              data_4_->publish(msg.floats); break;
        }
    }
}



// void BaseNode::publish_in_correct_topic() {
//     std::optional<std::vector<uint8_t>> message = base_modem_.process_packet();

//     if (!message.has_value()) {
//         return;
//     }
//     uint8_t type = extract_type_and_shift(message);

//     switch (type) {
//         case 0:
//             data_0_->publish(message);
//         case 1:
//             data_1_->publish(message);
//         case 2:
//             data_2_->publish(message);
//         case 3:
//             data_3_->publish(message);
//     }
// }

// remove the type bit and shifts the message as before
// uint8_t BaseNode::extract_type_and_shift(std::vector<uint8_t>& msg) {
//     uint8_t header = msg[0];
//     uint8_t type = (header & 0xC0) >> 6;

//     uint8_t carry = 0;
//     for (size_t i = 0; i < msg.size(); ++i) {
//         uint8_t current = msg[i];
//         msg[i] = static_cast<uint8_t>((current << 2) | (carry >> 6));
//         carry = current;
//     }
//     // remove non necessary bits at the end of the messages
//     msg.pop_back();

//     return type();
// }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}