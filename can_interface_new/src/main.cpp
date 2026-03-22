#include <linux/can.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <csignal>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include "can_interface.h"
#include <unordered_map>

struct CanMessageDef {
    uint32_t id;
    const char* name;
    std::function<std::string(const uint8_t* data, size_t len)> decode;
};

std::unordered_map<uint32_t, CanMessageDef> map;

#define NUM_ANGLES 2

class FastCsvLogger {
   public:
    explicit FastCsvLogger(const char* path, size_t flush_threshold = 64 * 1024)
        : flush_threshold_(flush_threshold) {
        file_ = std::fopen(path, "wb");
        if (!file_) {
            std::perror("fopen");
            return;
        }

        buffer_.reserve(flush_threshold_ * 2);
        buffer_ += "timestamp_us,id,dlc,data\n";
    }

    ~FastCsvLogger() {
        flush();
        if (file_) {
            std::fclose(file_);
        }
    }

    void log(uint64_t ts_us, uint32_t id, uint8_t dlc, const uint8_t* data) {
        if (!file_) {
            return;
        }

        char line[128];
        int n = std::snprintf(line, sizeof(line), "%llu,0x%X,%u,",
                              static_cast<unsigned long long>(ts_us), id,
                              static_cast<unsigned>(dlc));

        if (n > 0) {
            buffer_.append(line, static_cast<size_t>(n));
        }

        static constexpr char hex[] = "0123456789ABCDEF";
        for (uint8_t i = 0; i < dlc; ++i) {
            buffer_.push_back(hex[(data[i] >> 4) & 0x0F]);
            buffer_.push_back(hex[data[i] & 0x0F]);
        }
        buffer_.push_back('\n');

        if (buffer_.size() >= flush_threshold_) {
            flush();
        }
    }

    void flush() {
        if (file_ && !buffer_.empty()) {
            std::fwrite(buffer_.data(), 1, buffer_.size(), file_);
            buffer_.clear();
        }
    }

   private:
    FILE* file_ = nullptr;
    std::string buffer_;
    size_t flush_threshold_;
};

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

        double angle = (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * 3.14159265358979323846f);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "A%zu=%.3f%s",
                                i,
                                angle,
                                (i < NUM_ANGLES - 1) ? ", " : "");
    }

    return std::string(buffer);
}

void can_map_init() {
    CanMessageDef decode_encoder{};
    decode_encoder.id = 0x46D;
    decode_encoder.name = "Gripper Encoder angles";
    decode_encoder.decode = decode_encoder_angles;
    map[decode_encoder.id] = decode_encoder;
}

static volatile std::sig_atomic_t g_running = 1;

void signal_handler(int) {
    g_running = 0;
}

void handle_frame(const struct canfd_frame& frame, FastCsvLogger& logger) {
    auto now = std::chrono::steady_clock::now();
    uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         now.time_since_epoch())
                         .count();

    uint32_t id;
    if (frame.can_id & CAN_EFF_FLAG) {
        id = frame.can_id & CAN_EFF_MASK;
    } else {
        id = frame.can_id & CAN_SFF_MASK;
    }

    logger.log(ts_us, id, frame.len, frame.data);

    auto it = map.find(id);
    if (it != map.end()) {
        const CanMessageDef& can_decode = it->second;
        std::string decoded = can_decode.decode(frame.data, frame.len);

        std::cout << "ID=0x" << std::hex << id << std::dec
                  << " LEN=" << static_cast<int>(frame.len)
                  << " " << can_decode.name
                  << " | " << decoded << '\n';
    }
}

int main() {
    std::signal(SIGINT, signal_handler);

    can_map_init();

    FastCsvLogger logger("can_log.csv", 4096);

    can_interface can;
    can_status status = can.init("can0");
    if (status != can_status::OK) {
        std::cerr << "Failed to init can0\n";
        return 1;
    }

    std::cout << "Listening on can0. Press Ctrl+C to stop.\n";

    while (g_running) {
        struct canfd_frame frame{};

        status = can.receive(frame, 1000);
        if (status == can_status::OK) {
            handle_frame(frame, logger);
        } else if (status == can_status::ERR_RECEIVE) {
            // ignore timeout/no-frame if your receive(timeout) uses ERR_RECEIVE for that
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
