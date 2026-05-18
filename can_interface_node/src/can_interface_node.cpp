#include "can_interface_node.hpp"

#include <chrono>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>

#include "can_decode.hpp"


CanInterfaceNode::CanInterfaceNode(const rclcpp::NodeOptions& options)
    : Node("can_interface_node", options)
{
    can_interface_name_ =
        declare_parameter<std::string>("can_interface", "vcan0");

    start_bms_on_startup_ =
        declare_parameter<bool>("start_bms_on_startup", true);

    // 1. Create all publishers first
    bms_cell_voltages_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>(
            "bms/cell_voltages", 10);

    bms_current_pub_ =
        create_publisher<std_msgs::msg::Float32>(
            "bms/current", 10);

    bms_current_counts_pub_ =
        create_publisher<std_msgs::msg::Int32>(
            "bms/current_counts", 10);

    bms_temperatures_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>(
            "bms/temperatures", 10);

    bms_alert_ssa_pub_ =
        create_publisher<std_msgs::msg::UInt16MultiArray>(
            "bms/alerts/ssa", 10);

    bms_alert_pfa1_pub_ =
        create_publisher<std_msgs::msg::UInt16MultiArray>(
            "bms/alerts/pfa1", 10);

    bms_alert_pfa2_pub_ =
        create_publisher<std_msgs::msg::UInt16>(
            "bms/alerts/pfa2", 10);

    pressure_pub_ =
        create_publisher<std_msgs::msg::Float64>(
            "pressure/pressure", 10);

    leakage_alarm_pub_ =
        create_publisher<std_msgs::msg::Bool>(
            "leakage/alarm", 10);

    // 2. Register handlers
    init_registry();

    // 3. Init CAN
    const can_status status = can_.init(can_interface_name_.c_str());
    if (status != can_status::OK) {
        throw std::runtime_error(
            "Failed to init CAN interface: " + can_interface_name_);
    }

    // 4. Optional startup command
    if (start_bms_on_startup_) {
        uint8_t dummy = 0;
        can_.send(0x215, &dummy, 1);
        RCLCPP_INFO(get_logger(), "Sent BMS start command");
    }

    // 5. Only now start the receive thread
    running_.store(true);
    receive_thread_ = std::thread(&CanInterfaceNode::receive_loop, this);

    RCLCPP_INFO(
        get_logger(),
        "CAN interface node listening on %s",
        can_interface_name_.c_str());
}

CanInterfaceNode::~CanInterfaceNode() {
    running_.store(false);

    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    if (start_bms_on_startup_) {
        uint8_t dummy = 0;
        can_.send(0x216, &dummy, 1);
        RCLCPP_INFO(get_logger(), "Sent BMS stop command");
    }

    RCLCPP_INFO(get_logger(), "CAN interface node stopped");
}

void CanInterfaceNode::init_registry() {
    registry_.add(
        {CAN_VOLTAGE_ID, "BMS cell voltages", [this](const canfd_frame& frame) {
             handle_bms_cell_voltages(frame);
         }});

    registry_.add(
        {CAN_CURRENT_ID, "BMS current measurement",
         [this](const canfd_frame& frame) { handle_bms_current(frame); }});

    registry_.add(
        {CAN_TEMP_ID, "BMS Temperature",
         [this](const canfd_frame& frame) { handle_bms_temperatures(frame); }});

    registry_.add(
        {CAN_ALERT_SSA_ID, "BMS alert SSA",
         [this](const canfd_frame& frame) { handle_bms_alert_ssa(frame); }});

    registry_.add(
        {CAN_ALERT_PFA_1_ID, "BMS alert PFA1",
         [this](const canfd_frame& frame) { handle_bms_alert_pfa1(frame); }});

    registry_.add(
        {CAN_ALERT_PFA_2_ID, "BMS alert PFA2",
         [this](const canfd_frame& frame) { handle_bms_alert_pfa2(frame); }});

    registry_.add(
        {CAN_PRESSURE_ID, "Pressure Sample",
         [this](const canfd_frame& frame) { handle_pressure_sample(frame); }});
}

uint32_t CanInterfaceNode::get_can_id(const canfd_frame& frame) {
    if (frame.can_id & CAN_EFF_FLAG) {
        return frame.can_id & CAN_EFF_MASK;
    }

    return frame.can_id & CAN_SFF_MASK;
}


//
// void CanInterfaceNode::receive_loop() {
//     while (rclcpp::ok() && running_.load()) {
//         canfd_frame frame{};
//
//         const can_status status = can_.receive(frame, 1000);
//
//         if (status == can_status::OK) {
//             handle_frame(frame);
//         } else if (status == can_status::ERR_RECEIVE) {
//             continue;
//         } else {
//             RCLCPP_ERROR(get_logger(), "CAN receive error");
//             break;
//         }
//     }
// }
void CanInterfaceNode::receive_loop()
{
    RCLCPP_INFO(get_logger(), "[RX LOOP] started");

    while (rclcpp::ok() && running_.load()) {
        canfd_frame frame{};

        RCLCPP_INFO(get_logger(), "[RX LOOP] waiting for CAN frame");

        const can_status status = can_.receive(frame, 1000);

        RCLCPP_INFO(get_logger(), "[RX LOOP] receive returned status=%d",
                    static_cast<int>(status));

        if (status == can_status::OK) {
            RCLCPP_INFO(get_logger(),
                        "[RX LOOP] received frame raw_can_id=0x%X len=%u",
                        frame.can_id,
                        static_cast<unsigned int>(frame.len));

            handle_frame(frame);

            RCLCPP_INFO(get_logger(), "[RX LOOP] handle_frame returned");
        } else if (status == can_status::ERR_RECEIVE) {
            RCLCPP_INFO(get_logger(), "[RX LOOP] receive timeout/no frame");
            continue;
        } else {
            RCLCPP_ERROR(get_logger(), "[RX LOOP] CAN receive error");
            break;
        }
    }

    RCLCPP_INFO(get_logger(), "[RX LOOP] exiting");
}

void CanInterfaceNode::handle_frame(const canfd_frame& frame) {
    const uint32_t id = get_can_id(frame);

    const CanMessageDef* def = registry_.find(id);

    if (def->handle) {
        def->handle(frame);
    }
}

void CanInterfaceNode::handle_bms_cell_voltages(const canfd_frame& frame) {
    std::cout << "bms cell" << std::endl;
    const auto parsed = parse_voltage(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid BMS cell voltage frame");
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(parsed->cell_voltages_mV.size());

    for (const uint16_t cell_mV : parsed->cell_voltages_mV) {
        msg.data.push_back(static_cast<float>(cell_mV) / 1000.0f);
    }

    bms_cell_voltages_pub_->publish(msg);
}

void CanInterfaceNode::handle_bms_current(const canfd_frame& frame) {
    std::cout << "bms current" << std::endl;
    const auto parsed = parse_current(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid BMS current frame");
        return;
    }

    std_msgs::msg::Float32 current_msg;
    current_msg.data = static_cast<float>(parsed->current_mA) / 1000.0f;
    bms_current_pub_->publish(current_msg);

    std_msgs::msg::Int32 counts_msg;
    counts_msg.data = parsed->current_counts;
    bms_current_counts_pub_->publish(counts_msg);
}

void CanInterfaceNode::handle_pressure_sample(const canfd_frame& frame)
{
    RCLCPP_INFO(get_logger(), "Pressure handler called");

    if (!pressure_pub_) {
        RCLCPP_ERROR(get_logger(), "pressure_pub_ is null");
        return;
    }

    const auto parsed = parse_pressure_sample(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid pressure sample frame");
        return;
    }

    std_msgs::msg::Float64 msg;
    msg.data = parsed->pressure_hPa;

    pressure_pub_->publish(msg);
}

void CanInterfaceNode::handle_bms_temperatures(const canfd_frame& frame) {
    const auto parsed = parse_temp(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid BMS temperature frame");
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(parsed->temperatures_dC.size());

    for (const int16_t temperature_dC : parsed->temperatures_dC) {
        msg.data.push_back(static_cast<float>(temperature_dC) / 10.0f);
    }

    bms_temperatures_pub_->publish(msg);
}

void CanInterfaceNode::handle_bms_alert_ssa(const canfd_frame& frame) {
    const auto parsed = parse_alert_ssa(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid BMS SSA alert frame");
        return;
    }

    std_msgs::msg::UInt16MultiArray msg;
    msg.data = {parsed->alarm, parsed->ssa, parsed->ssb, parsed->ssc};

    bms_alert_ssa_pub_->publish(msg);
}

void CanInterfaceNode::handle_bms_alert_pfa1(const canfd_frame& frame) {

    RCLCPP_INFO(get_logger(), "bms alert pfa1");
    const auto parsed = parse_alert_pfa_1(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid BMS PFA1 alert frame");
        return;
    }

    std_msgs::msg::UInt16MultiArray msg;
    msg.data = {parsed->pfa, parsed->pfb, parsed->pfc, parsed->pfd};

    bms_alert_pfa1_pub_->publish(msg);
}

void CanInterfaceNode::handle_bms_alert_pfa2(const canfd_frame& frame)
{
    RCLCPP_INFO(
        get_logger(),
        "[PFA2] handler entered, len=%u",
        frame.len
    );

    if (!bms_alert_pfa2_pub_) {
        RCLCPP_ERROR(get_logger(), "[PFA2] bms_alert_pfa2_pub_ is null");
        return;
    }

    RCLCPP_INFO(get_logger(), "[PFA2] publisher exists");

    const auto parsed = parse_alert_pfa_2(frame.data, frame.len);

    RCLCPP_INFO(get_logger(), "[PFA2] parser returned");

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "[PFA2] invalid BMS PFA2 alert frame");
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "[PFA2] parsed fet=0x%04X",
        parsed->fet
    );

    std_msgs::msg::UInt16 msg;
    msg.data = parsed->fet;

    RCLCPP_INFO(get_logger(), "[PFA2] publishing");

    bms_alert_pfa2_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "[PFA2] publish done");
}

void CanInterfaceNode::handle_leakage_alarm(const canfd_frame& frame) {
    const auto parsed = parse_leakage_alarm(frame.data, frame.len);

    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Invalid leakage alarm frame");
        return;
    }

    std_msgs::msg::Bool msg;
    msg.data = parsed->active;

    leakage_alarm_pub_->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
