#include <linux/can.h>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "can_decode.hpp"
#include "can_interface.hpp"
#include "can_logger.hpp"
#include "can_registry.hpp"

static volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}

static void init_registry(CanRegistry& registry) {
    registry.add({0x46D, "Gripper Encoder angles", decode_encoder_angles});
    registry.add({0x45A, "Motor Controler Frame", decode_motor_frames});
    registry.add({CAN_VOLTAGE_ID, "BMS cell voltages", decode_voltage});
    registry.add({CAN_CURRENT_ID, "BMS current measurement", decode_current});
    registry.add({CAN_ALERT_PFA_1_ID, "BMS alert PFA1", decode_alert_pfa_1});
    registry.add({CAN_ALERT_PFA_2_ID, "BMS alert PFA2", decode_alert_pfa_2});
    registry.add({CAN_ALERT_SSA_ID, "BMS alert SSA", decode_alert_ssa});
    registry.add({CAN_TEMP_ID, "BMS Temparture", decode_temp});
    registry.add({0x780, "Pressure Sample", decode_pressure_sample});
    registry.add({0x100, "Leakage Alarm", decode_leakage_alarm});
}

static void handle_frame(const canfd_frame& frame,
                         FastCsvLogger& logger,
                         const CanRegistry& registry) {
    uint32_t id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                : (frame.can_id & CAN_SFF_MASK);

    // Placeholder timestamp for now.
    // Replace later with socket timestamp if you add recvmsg().
    uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();

    logger.log(ts_us, id, frame.len, frame.data);

    const CanMessageDef* def = registry.find(id);
    if (def) {
        std::string decoded = def->decode(frame.data, frame.len);

        std::cout << "ID=0x" << std::hex << id << std::dec
                  << " LEN=" << static_cast<int>(frame.len) << " " << def->name
                  << " | " << decoded << '\n';
    }
}

int main() {
    std::signal(SIGINT, signal_handler);

    CanRegistry registry;
    init_registry(registry);

    FastCsvLogger logger("can_log.csv", 4096);

    can_interface can;
    can_status status = can.init("can0");
    if (status != can_status::OK) {
        std::cerr << "Failed to init can0\n";
        return 1;
    }

    uint8_t dummy = 0;
    // start bms send
    can.send(0x215, &dummy, 1);

    std::cout << "Listening on can0. Press Ctrl+C to stop.\n";

    while (g_running) {
        canfd_frame frame{};

        status = can.receive(frame, 1000);
        if (status == can_status::OK) {
            handle_frame(frame, logger, registry);
        } else if (status == can_status::ERR_RECEIVE) {
            continue;
        } else {
            std::cerr << "Receive error\n";
            break;
        }
    }

    // stop bms send
    can.send(0x216, &dummy, 1);

    logger.flush();
    std::cout << "Stopped. Log written to can_log.csv\n";
    return 0;
}
