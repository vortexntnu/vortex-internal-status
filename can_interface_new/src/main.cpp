#include <csignal>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <linux/can.h>

#include "can_interface.hpp"
#include "can_logger.hpp"
#include "can_registry.hpp"

static volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}

#define NUM_ANGLES 2

std::string decode_encoder_angles(const uint8_t* data, size_t len) {
    if (len < 2 * NUM_ANGLES) {
        return "invalid length";
    }

    char buffer[256];
    int offset = 0;

    for (size_t i = 0; i < NUM_ANGLES; ++i) {
        uint16_t raw_angle =
            (static_cast<uint16_t>(data[2 * i + 1]) << 8) |
            static_cast<uint16_t>(data[2 * i]);

        raw_angle &= 0x3FFF;

        double angle = (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * 3.1415926535897932384626433f);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "A%zu=%.3f%s",
                                i,
                                angle,
                                (i < NUM_ANGLES - 1) ? ", " : "");
    }

    return std::string(buffer);
}

std::string decode_pt_sample(const uint8_t* data, size_t len) {
    if (len < 2 * sizeof(float)) {
        return "invalid length";
    }

    float temperature_c = 0.0f;
    float pressure_pa = 0.0f;

    std::memcpy(&temperature_c, data, sizeof(float));
    std::memcpy(&pressure_pa, data + sizeof(float), sizeof(float));

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "T=%.3f C, P=%.3f Pa", temperature_c, pressure_pa);
    return std::string(buffer);
}

std::string decode_leakage_alarm(const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    return "LEAKAGE ALARM";
}

static void init_registry(CanRegistry& registry) {
    registry.add({0x46D, "Gripper Encoder angles", decode_encoder_angles});
    registry.add({0x333, "Internal PT Sample", decode_pt_sample});
    registry.add({0x100, "Leakage Alarm", decode_leakage_alarm});
}

static void handle_frame(const canfd_frame& frame,
                         FastCsvLogger& logger,
                         const CanRegistry& registry) {
    uint32_t id = (frame.can_id & CAN_EFF_FLAG)
                    ? (frame.can_id & CAN_EFF_MASK)
                    : (frame.can_id & CAN_SFF_MASK);

    // Placeholder timestamp for now.
    // Replace later with socket timestamp if you add recvmsg().
    uint64_t ts_us = 0;

    logger.log(ts_us, id, frame.len, frame.data);

    const CanMessageDef* def = registry.find(id);
    if (def) {
        std::string decoded = def->decode(frame.data, frame.len);

        std::cout << "ID=0x" << std::hex << id << std::dec
                  << " LEN=" << static_cast<int>(frame.len)
                  << " " << def->name
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

    logger.flush();
    std::cout << "Stopped. Log written to can_log.csv\n";
    return 0;
}
