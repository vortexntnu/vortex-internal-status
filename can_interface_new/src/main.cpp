#include <linux/can.h>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include "can_interface.h"
#include <unordered_map>



volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}


struct CanMessageDef {
    uint32_t id;
    const char* name;
    std::function<std::string(const uint8_t* data, size_t len)> decode;
};

std::unordered_map<uint32_t, CanMessageDef> map;

#define NUM_ANGLES 2

std::string decode_encoder_angles(const uint8_t* data, size_t len) {
    if (len < 2 * NUM_ANGLES) {
        return "invalid length";
    }

    char buffer[256];
    int offset = 0;

    for (size_t i = 0; i < NUM_ANGLES; ++i) {
        uint16_t raw_angle = (static_cast<uint16_t>(data[2 * i + 1]) << 8) |
                             static_cast<uint16_t>(data[2 * i]);

        double angle = (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * 3.1415926f);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "A%zu=%.3f%s", i, angle,
                                (i < NUM_ANGLES - 1) ? ", " : "");
    }

    return std::string(buffer);
}

void can_map_init() {
    CanMessageDef decode_encoder{};
    decode_encoder.id = 0x46D;
    decode_encoder.name = "Gripper Encoder angles";
    decode_encoder.decode = decode_encoder_angles;
    map[0x46D] = decode_encoder;
}

void print_raw_frame(const canfd_frame& frame) {
    uint32_t raw_id = frame.can_id;
    uint32_t id = (raw_id & CAN_EFF_FLAG) ? (raw_id & CAN_EFF_MASK)
                                          : (raw_id & CAN_SFF_MASK);

    std::cout << "ID=0x" << std::hex << std::uppercase << id << std::dec
              << " LEN=" << static_cast<int>(frame.len)
              << " DATA=";

    for (uint8_t i = 0; i < frame.len; ++i) {
        std::printf("%02X", frame.data[i]);
        if (i + 1 < frame.len) {
            std::printf(" ");
        }
    }
}

void handle_frame(const canfd_frame& frame) {
    auto now = std::chrono::steady_clock::now();
    uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         now.time_since_epoch())
                         .count();

    uint32_t id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                : (frame.can_id & CAN_SFF_MASK);

    std::cout << "[" << ts_us << " us] ";
    print_raw_frame(frame);

    auto it = map.find(id);
    if (it != map.end()) {
        const CanMessageDef& def = it->second;
        std::string decoded = def.decode(frame.data, frame.len);
        std::cout << " | " << def.name << " | " << decoded;
    }

    std::cout << '\n';
}


int main() {
    std::signal(SIGINT, signal_handler);

    can_map_init();

    can_interface can;

    can_status status = can.init("can0");
    if (status != can_status::OK) {
        std::cerr << "Failed to initialize CAN interface\n";
        return 1;
    }

    std::cout << "Listening on " << can.get_interface_name() << '\n';
    std::cout << "Press Ctrl+C to stop\n";

    while (g_running) {
        canfd_frame frame{};
        status = can.receive(frame, 1000);  // 1000 ms timeout

        if (status == can_status::OK) {
            handle_frame(frame);
        } else if (status == can_status::ERR_RECEIVE) {
            // ignore timeout-style receive failures if your implementation uses
            // ERR_RECEIVE for timeout
            continue;
        } else {
            std::cerr << "Receive error\n";
            break;
        }
    }

    std::cout << "Exiting\n";
    return 0;
}
