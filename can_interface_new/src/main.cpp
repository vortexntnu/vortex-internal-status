#include <linux/can.h>
#include <csignal>
#include <cstdint>
#include <iostream>

#include "can_interface.hpp"
#include "can_logger.hpp"
#include "can_registry.hpp"

static volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}

#define NUM_ANGLES 2
#define CELLS_COUNT        6
#define CELLS_PAYLOAD_LEN  12
#define CAN_ID_BOTHOFF_CMD 0x200  //EXAMPLE VALUE
#define BOTHOFF_CMD_BYTE 0xA5
#define CAN_TEMP_ID 0x100         //EXAMPLE VALUE
// #define CAN_VOLTAGE_ID 0x101      //EXAMPLE VALUE

//legg til func for current, pressure, standbymode, reset mcu

#define CAN_PRESSURE_ID 0x102      //EXAMPLE VALUE
#define CAN_STANDBYMODE_ID 0x103     //EXAMPLE VALUE
#define CAN_RST_MCU 0x104     //EXAMPLE VALUE
// #define CAN_CURRENT_ID 0x105     //EXAMPLE VALUE

#define CAN_ALERT_SSA_ID   0x200u
#define CAN_ALERT_PFA_1_ID 0x201u
#define CAN_ALERT_PFA_2_ID 0x202u
#define CAN_CURRENT_ID     0x203u
#define CAN_VOLTAGE_ID 0x204      //EXAMPLE VALUE

std::string decode_encoder_angles(const uint8_t* data, size_t len) {
    if (len < 2 * NUM_ANGLES) {
        return "invalid length";
    }

    char buffer[256];
    int offset = 0;

    for (size_t i = 0; i < NUM_ANGLES; ++i) {
        uint16_t raw_angle = (static_cast<uint16_t>(data[2 * i + 1]) << 8) |
                             static_cast<uint16_t>(data[2 * i]);

        raw_angle &= 0x3FFF;

        double angle = (static_cast<double>(raw_angle) / 0x3FFF) *
                       (2.0 * 3.1415926535897932384626433f);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "A%zu=%.3f%s", i, angle,
                                (i < NUM_ANGLES - 1) ? ", " : "");
    }

    return std::string(buffer);
}

std::string decode_set_gripper_pwm(const uint8_t* data, size_t len) {
    uint16_t duty_cycle[NUM_ANGLES] = {0};

    for (size_t i = 0; i < NUM_ANGLES; ++i) {
        uint16_t raw_angle = (static_cast<uint16_t>(data[2 * i + 1]) << 8) |
                             static_cast<uint16_t>(data[2 * i]);
    }
    return "";
}

std::string decode_gripper_start(const uint8_t* data, size_t len) {
    return "Starting gripper";
}

std::string decode_gripper_stop(const uint8_t* data, size_t len) {
    return "Stopping gripper";
}

std::string decode_alert_ssa(const uint8_t* data, size_t len) {
    if (len < 8) {
        return "invalid length";
    }

    uint16_t alarm = (static_cast<uint16_t>(data[1]) << 8) |
                     static_cast<uint16_t>(data[0]);

    uint16_t ssa = (static_cast<uint16_t>(data[3]) << 8) |
                   static_cast<uint16_t>(data[2]);

    uint16_t ssb = (static_cast<uint16_t>(data[5]) << 8) |
                   static_cast<uint16_t>(data[4]);

    uint16_t ssc = (static_cast<uint16_t>(data[7]) << 8) |
                   static_cast<uint16_t>(data[6]);

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer),
                  "alert=0x%04X,ssa=0x%04X,ssb=0x%04X,ssc=0x%04X",
                  static_cast<unsigned int>(alarm),
                  static_cast<unsigned int>(ssa),
                  static_cast<unsigned int>(ssb),
                  static_cast<unsigned int>(ssc));

    return std::string(buffer);
}

std::string decode_alert_pfa_1(const uint8_t* data, size_t len) {
    if (len < 8) {
        return "invalid length";
    }

    uint16_t pfa = (static_cast<uint16_t>(data[1]) << 8) |
                   static_cast<uint16_t>(data[0]);

    uint16_t pfb = (static_cast<uint16_t>(data[3]) << 8) |
                   static_cast<uint16_t>(data[2]);

    uint16_t pfc = (static_cast<uint16_t>(data[5]) << 8) |
                   static_cast<uint16_t>(data[4]);

    uint16_t pfd = (static_cast<uint16_t>(data[7]) << 8) |
                   static_cast<uint16_t>(data[6]);

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer),
                  "pfa=0x%04X,pfb=0x%04X,pfc=0x%04X,pfd=0x%04X",
                  static_cast<unsigned int>(pfa),
                  static_cast<unsigned int>(pfb),
                  static_cast<unsigned int>(pfc),
                  static_cast<unsigned int>(pfd));

    return std::string(buffer);
}

std::string decode_alert_pfa_2(const uint8_t* data, size_t len) {
    if (len < 2) {
        return "invalid length";
    }

    uint16_t fet = (static_cast<uint16_t>(data[1]) << 8) |
                   static_cast<uint16_t>(data[0]);

    char buffer[64];
    std::snprintf(buffer, sizeof(buffer),
                  "fet=0x%04X",
                  static_cast<unsigned int>(fet));

    return std::string(buffer);
}

std::string decode_current(const uint8_t* data, size_t len) {
    if (len < 2) {
        return "invalid length";
    }

    uint16_t raw = (static_cast<uint16_t>(data[1]) << 8) |
                   static_cast<uint16_t>(data[0]);

    int16_t current = static_cast<int16_t>(raw);

    char buffer[64];
    std::snprintf(buffer, sizeof(buffer),
                  "current=%d mA",
                  static_cast<int>(current));

    return std::string(buffer);
}


std::string decode_temp(const uint8_t* data, size_t len) {
    if (len < 6) {
        return "invalid length";
    }

    int16_t t1 = static_cast<int16_t>(
        (static_cast<uint16_t>(data[1]) << 8) |
         static_cast<uint16_t>(data[0]));

    int16_t t2 = static_cast<int16_t>(
        (static_cast<uint16_t>(data[3]) << 8) |
         static_cast<uint16_t>(data[2]));

    int16_t t3 = static_cast<int16_t>(
        (static_cast<uint16_t>(data[5]) << 8) |
         static_cast<uint16_t>(data[4]));

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer),
                  "t1=%d dC,t2=%d dC,t3=%d dC",
                  static_cast<int>(t1),
                  static_cast<int>(t2),
                  static_cast<int>(t3));

    return std::string(buffer);
}

std::string decode_voltage(const uint8_t* data, size_t len) {
    if (len < 2 * CELLS_COUNT) {
        return "invalid length";
    }

    char buffer[256];
    int offset = 0;

    for (size_t i = 0; i < CELLS_COUNT; ++i) {
        uint16_t cell_mV = (static_cast<uint16_t>(data[2 * i + 1]) << 8) |
                            static_cast<uint16_t>(data[2 * i]);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "cell%zu=%u mV%s",
                                i + 1,
                                static_cast<unsigned int>(cell_mV),
                                (i < CELLS_COUNT - 1) ? ", " : "");
    }

    return std::string(buffer);
}


static void init_registry(CanRegistry& registry) {
    registry.add({0x46D, "Gripper Encoder angles", decode_encoder_angles});
    registry.add({CAN_VOLTAGE_ID, "BMS cell voltages", decode_voltage});
    registry.add({CAN_CURRENT_ID, "BMS current measurement", decode_current});
    registry.add({CAN_ALERT_PFA_1_ID, "BMS alert PFA1", decode_alert_pfa_1});
    registry.add({CAN_ALERT_PFA_2_ID, "BMS alert PFA2", decode_alert_pfa_2});
    registry.add({CAN_ALERT_SSA_ID, "BMS alert SSA", decode_alert_ssa});
}

static void handle_frame(const canfd_frame& frame,
                         FastCsvLogger& logger,
                         const CanRegistry& registry) {
    uint32_t id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                                : (frame.can_id & CAN_SFF_MASK);

    // Placeholder timestamp for now.
    // Replace later with socket timestamp if you add recvmsg().
    uint64_t ts_us = 0;

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
