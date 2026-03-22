#include "can_logger.hpp"
#include <cstdio>

FastCsvLogger::FastCsvLogger(const char* path, size_t flush_threshold)
    : flush_threshold_(flush_threshold) {
    file_ = std::fopen(path, "wb");
    if (!file_) {
        std::perror("fopen");
        return;
    }

    buffer_.reserve(flush_threshold_ * 2);
    buffer_ += "timestamp_us,id,dlc,data\n";
}

FastCsvLogger::~FastCsvLogger() {
    flush();
    if (file_) {
        std::fclose(file_);
    }
}

void FastCsvLogger::log(uint64_t ts_us, uint32_t id, uint8_t dlc, const uint8_t* data) {
    if (!file_) {
        return;
    }

    char line[128];
    int n = std::snprintf(line, sizeof(line), "%llu,0x%03X,%u,",
                          static_cast<unsigned long long>(ts_us),
                          id,
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

void FastCsvLogger::flush() {
    if (file_ && !buffer_.empty()) {
        std::fwrite(buffer_.data(), 1, buffer_.size(), file_);
        buffer_.clear();
    }
}


