#include "nodes/am_drone_node.hpp"

DroneNode::DroneNode() : Node("drone_node") {

    init_connection();
    setup_tdma();
    set_subscriber();
    set_publisher();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&DroneNode::poll_modem, this));

    link_->start();

    RCLCPP_INFO(this->get_logger(), "DroneNode started");
}

DroneNode::~DroneNode() {
    if (link_) {
        link_->stop();
    }
}

void DroneNode::init_connection() {
    this->declare_parameter<std::string>("device", "");
    this->declare_parameter<std::string>("tx_device", "");
    this->declare_parameter<std::string>("rx_device", "");
    this->declare_parameter<int>("baudrate", 9600);
    this->declare_parameter<int>("channel", 1); 
    this->declare_parameter<int>("level", 4);
    this->declare_parameter<bool>("diagnostic", false);
    this->declare_parameter<double>("timeout", 0.5);
    this->declare_parameter<bool>("split_mode",false);

    std::string device = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int channel = this->get_parameter("channel").as_int();
    int level = this->get_parameter("level").as_int();
    bool diagnostic = this->get_parameter("diagnostic").as_bool();
    float timeout =static_cast<float>(this->get_parameter("timeout").as_double());    
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

void DroneNode::setup_tdma(){
    this->declare_parameter<int>("num_slots", 2);
    this->declare_parameter<int>("my_slot", 0);
    this->declare_parameter<int>("slot_duration_sec", 25);
    this->declare_parameter<int>("guard_ms", 1000);
    this->declare_parameter<int>("sync_delay", 5);
    this->declare_parameter<int>("estimated_prop_delay", 0);

    TDMAConfig cfg;
    cfg.num_slots = static_cast<std::uint8_t>(this->get_parameter("num_slots").as_int());
    cfg.my_slot = static_cast<std::uint8_t>(this->get_parameter("my_slot").as_int());
    cfg.slot_duration = std::chrono::seconds(this->get_parameter("slot_duration_sec").as_int());
    cfg.guard = std::chrono::milliseconds(this->get_parameter("guard_ms").as_int());
    cfg.sync_delay=std::chrono::seconds(this->get_parameter("sync_delay").as_int());
    cfg.estimated_prop_delay=std::chrono::milliseconds(this->get_parameter("estimated_prop_delay").as_int());

    cfg.t0 = std::chrono::steady_clock::time_point{};

    tdma_=std::make_unique<TDMAManager>(cfg);
    link_=std::make_unique<TDMALink>(*driver_,*tdma_);

}

void DroneNode::set_subscriber() {
    // Depends on the topic in which we will read the data
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "data_topic", 10, std::bind(&DroneNode::tx_callback, this,
        std::placeholders::_1));
}
void DroneNode::set_publisher(){
    persistent_pub_ = this->create_publisher<std_msgs::msg::UInt16>("persistent_cmd_topic", 10);
    publisher_=this->create_publisher<std_msgs::msg::Float32MultiArray>("not_necessary_topic",10);
}

void DroneNode::tx_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    std::vector<float> payload(msg->data.begin(), msg->data.end());

    MsgType type;

    switch (payload.size()) {
        case 2:
            type = MsgType::Type_1;
            break;

        case 4:
            type = MsgType::Type_2;
            break;

        case 5:
            type = MsgType::Type_3;
            break;

        default:
            RCLCPP_WARN(this->get_logger(),
                        "Unsupported payload size: %zu floats",
                        payload.size());
            return;
    }


    link_->enqueue(type, payload);
    RCLCPP_INFO(this->get_logger(),"Enqueued outgoing acoustic message with %zu floats",payload.size());

}


/*
 * Persistent commands are received as standalone 16-bit control words and are
 * decoded directly by the driver. In poll_modem(), we check whether a new
 * persistent command has been received and, if so, publish it immediately on
 * a dedicated ROS topic so the drone control logic can react without waiting
 * for normal payload handling.
 */
void DroneNode::poll_modem(){

    std::chrono::steady_clock::time_point sync_time;
    if(driver_->try_tdma_sync_event(sync_time)){
        link_->on_tdma_sync_received();
        RCLCPP_INFO(this->get_logger(),"TDMA SYNC processed in Drone Node");
    }

    DecodedMessage msg;
    while(driver_->try_pop_decoded(msg)){
        link_->on_data_received(msg.type,msg.msg_id);

        std_msgs::msg::Float32MultiArray out;
        for (std::uint8_t i = 0; i < msg.n_floats; ++i) {
            out.data.push_back(msg.floats[i]);
        }
        // TODO: here i can publish the data in a topic or i can just print them out or whatever
        // Let's publish them for now
        publisher_->publish(out);
         RCLCPP_INFO(this->get_logger(),"Published received acoustic message, msg_id=%u", msg.msg_id);
    }

    Ack ack;
    while(driver_->try_pop_ack(ack)){
        link_->on_ack_received(ack.type,ack.msg_id);

        RCLCPP_INFO(this->get_logger(),"ACK received, msg_id=%u", ack.msg_id);
    }

    PersistentCmd cmd;
    if(driver_->consume_persistent(cmd)){
        std_msgs::msg::UInt16 out_cmd;
        out_cmd.data = static_cast<std::uint16_t>(cmd);

        persistent_pub_->publish(out_cmd);

        RCLCPP_INFO(this->get_logger(),"Received persistent command: %u", static_cast<std::uint16_t>(cmd));
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
