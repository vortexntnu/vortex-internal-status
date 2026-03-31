#include <linux/can.h>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <string>

#include "can_decode.hpp"
#include "can_interface.hpp"
#include "can_logger.hpp"
#include "can_registry.hpp"

static volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}

struct ProgramOptions {
    bool print = false;
    std::string can_interface_name = "can0";
};

static ProgramOptions parse_args(int argc, char* argv[]) {
    ProgramOptions opts;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--print" || arg == "-p") {
            opts.print = true;
        } else if ((arg == "--interface" || arg == "-i") && i + 1 < argc) {
            opts.can_interface_name = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: " << argv[0] << " [--print] [--interface can0]\n"
                << "  --print, -p        Print decoded frames to stdout\n"
                << "  --interface, -i    CAN interface name (default: can0)\n"
                << "  --help, -h         Show this help\n";
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::cerr << "Use --help for usage.\n";
            std::exit(1);
        }
    }

    return opts;
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



std::string make_log_filename() {
    const std::string log_dir = "/home/vortex/can_logger/logs/";

    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&time, &tm);

    std::ostringstream oss;
    oss << log_dir
        << "can_log_"
        << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
        << ".csv";

    return oss.str();
}

static void handle_frame(const canfd_frame& frame,
                         FastCsvLogger& logger,
                         const CanRegistry& registry,
                         bool print_enabled) {
    uint32_t id = (frame.can_id & CAN_EFF_FLAG)
                      ? (frame.can_id & CAN_EFF_MASK)
                      : (frame.can_id & CAN_SFF_MASK);

    uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();

    // Always log
    logger.log(ts_us, id, frame.len, frame.data);

    // Only print when enabled
    if (!print_enabled) {
        return;
    }

    const CanMessageDef* def = registry.find(id);
    if (def) {
        std::string decoded = def->decode(frame.data, frame.len);

        std::cout << "ID=0x" << std::hex << id << std::dec
                  << " LEN=" << static_cast<int>(frame.len) << " "
                  << def->name << " | " << decoded << '\n';
    } else {
        std::cout << "ID=0x" << std::hex << id << std::dec
                  << " LEN=" << static_cast<int>(frame.len)
                  << " UNKNOWN\n";
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    ProgramOptions opts = parse_args(argc, argv);

    CanRegistry registry;
    init_registry(registry);

    FastCsvLogger logger(make_log_filename().c_str(), 4096);

    can_interface can;
    can_status status = can.init(opts.can_interface_name.c_str());
    if (status != can_status::OK) {
        std::cerr << "Failed to init " << opts.can_interface_name << "\n";
        return 1;
    }

    uint8_t dummy = 0;

    // Start BMS send
    can.send(0x215, &dummy, 1);

    if (opts.print) {
        std::cout << "Listening on " << opts.can_interface_name
                  << ". Press Ctrl+C to stop.\n";
    }

    while (g_running) {
        canfd_frame frame{};

        status = can.receive(frame, 1000);
        if (status == can_status::OK) {
            handle_frame(frame, logger, registry, opts.print);
        } else if (status == can_status::ERR_RECEIVE) {
            continue;
        } else {
            std::cerr << "Receive error\n";
            break;
        }
    }

    // Stop BMS send
    can.send(0x216, &dummy, 1);

    logger.flush();

    if (opts.print) {
        std::cout << "Stopped.\n";
    }

    return 0;
}
