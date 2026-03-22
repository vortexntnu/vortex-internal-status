#include <linux/can.h>
#include <math.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>


class FastCsvLogger {
   public:
    explicit FastCsvLogger(const char* path, size_t flush_threshold = 64 * 1024)
        : flush_threshold_(flush_threshold) {
        file_ = std::fopen(path, "wb");
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
        char line[128];
        int n = std::snprintf(line, sizeof(line), "%llu,0x%03X,%u,",
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

        double angle = (static_cast<double>(raw_angle) / 0x3FFF) * (2.0 * M_PI);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "A%zu=%.3f%s", i, angle,
                                (i < NUM_ANGLES - 1) ? ", " : "");
    }

    return std::string(buffer);
}


void can_map_init(){
    CanMessageDef decode_encoder{};
    decode_encoder.id = 0x46D;
    decode_encoder.name = "Gripper Encoder angles";
    decode_encoder.decode = decode_encoder_angles;
    map[0x46D] = decode_encoder;

}

void handle_frame(struct canfd_frame& frame) {
    auto now = std::chrono::steady_clock::now();
    uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         now.time_since_epoch())
                         .count();

    logger.log(ts_us, frame.can_id, frame.len, frame.data);
#ifdef DEBUG
    if (debug) {
        CanMessageDef can_decode = map.find(frame.can_id);
        can_decode.decode(frame.data, frame.len);
    }
#endif
}
