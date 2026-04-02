#include "am_base_node.hpp"

BaseNode::BaseNode() : Node("base_node") {
    init_connection();
    setup_tdma();
    set_publishers();
    set_subscribers();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&BaseNode::poll_and_publish_rx, this));

    link_->start();

    RCLCPP_INFO(this->get_logger(), "BaseNode started");
}

BaseNode::~BaseNode() {
    if (link_) {
        link_->stop();
    }
}


void BaseNode::init_connection() {
    this->declare_parameter<std::string>("device", "");
    this->declare_parameter<std::string>("tx_device", "");
    this->declare_parameter<std::string>("rx_device", "");
    this->declare_parameter<int>("baudrate", 9600);
    this->declare_parameter<int>("channel", 1);
    this->declare_parameter<int>("level", 4);
    this->declare_parameter<bool>("diagnostic", false);
    this->declare_parameter<double>("timeout", 0.5);
    this->declare_parameter<bool>("split_mode",false);

    int baudrate = this->get_parameter("baudrate").as_int();
    int channel = this->get_parameter("channel").as_int();
    int level = this->get_parameter("level").as_int();
    bool diagnostic = this->get_parameter("diagnostic").as_bool();
    float timeout =
        static_cast<float>(this->get_parameter("timeout").as_double());
    bool split=this->get_parameter("split_mode").as_bool();
    
    if (!split) {
        std::string device = this->get_parameter("device").as_string();

        driver_ = std::make_unique<AcousticModemDriver>(
            device, baudrate, channel, level, diagnostic, timeout);
    } else {
        std::string tx_device = this->get_parameter("tx_device").as_string();
        std::string rx_device = this->get_parameter("rx_device").as_string();

        driver_ = std::make_unique<AcousticModemDriverSplit>(
            tx_device, rx_device, baudrate, channel, level, diagnostic, timeout);   
    }
}

void BaseNode::setup_tdma(){
    this->declare_parameter<int>("num_slots", 2);
    this->declare_parameter<int>("my_slot", 1);
    this->declare_parameter<int>("slot_duration_sec", 25);
    this->declare_parameter<int>("guard_ms", 1000);

    TDMAConfig cfg;
    cfg.num_slots = static_cast<std::uint8_t>(this->get_parameter("num_slots").as_int());
    cfg.my_slot = static_cast<std::uint8_t>(this->get_parameter("my_slot").as_int());
    cfg.slot_duration = std::chrono::seconds(this->get_parameter("slot_duration_sec").as_int());
    cfg.guard = std::chrono::milliseconds(this->get_parameter("guard_ms").as_int());

    cfg.t0 = std::chrono::steady_clock::now() + std::chrono::seconds(5);

    tdma_=std::make_unique<TDMAManager>(cfg);
    link_=std::make_unique<TDMALink>(*driver_,*tdma_);

}

void BaseNode::set_publishers() {
    // we reserved 2 bit for the type of data so we'll have 4 types of data
    data_0_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_0", 10);
    data_1_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_1", 10);
    data_2_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_2", 10);
    data_3_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_3", 10);
}

void BaseNode::set_subscribers() {
    persistent_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "persistent",
        10,
        std::bind(&BaseNode::persistent_callback, this, std::placeholders::_1));
}

void BaseNode::persistent_callback(const std_msgs::msg::UInt16::SharedPtr msg) {
    std::uint16_t value = msg->data;

    if (value == 0) {
        link_->stop_persistent_command();
        RCLCPP_INFO(this->get_logger(), "Stopped persistent command");
        return;
    }

    PersistentCmd cmd = static_cast<PersistentCmd>(value);

    link_->start_persistent_command(cmd);

    RCLCPP_INFO(this->get_logger(),
                "Sent persistent command: %u",
                static_cast<std::uint16_t>(cmd));
}

void BaseNode::poll_and_publish_rx() {
    DecodedMessage msg;
    while (driver_->try_pop_decoded(msg)) {
        link_->on_data_received(msg.type,msg.msg_id);
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
    Ack ack;
    while(driver_->try_pop_ack(ack)){
        link_->on_ack_received(ack.type,ack.msg_id);

        RCLCPP_INFO(this->get_logger(),"ACK received, msg_id=%u", ack.msg_id);
    }

    PersistentCmd cmd;
    if(driver_->consume_persistent(cmd)){
        //std_msgs::msg::UInt16 out_cmd;
        //out_cmd.data = static_cast<std::uint16_t>(cmd);

        //persistent_pub_->publish(out_cmd);

        RCLCPP_INFO(this->get_logger(),"Received persistent command: %u", static_cast<std::uint16_t>(cmd));
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}