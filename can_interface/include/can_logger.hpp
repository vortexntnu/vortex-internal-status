#ifndef CAN_LOGGER_
#define CAN_LOGGER_

#include <cstdint>
#include <cstdio>
#include <string>

class FastCsvLogger {
   public:
    explicit FastCsvLogger(const char* path,
                           size_t flush_threshold = 64 * 1024);
    ~FastCsvLogger();

    void log(uint64_t ts_us, uint32_t id, uint8_t dlc, const uint8_t* data);
    void flush();

   private:
    FILE* file_ = nullptr;
    std::string buffer_;
    size_t flush_threshold_;
};

#endif  // !CAN_LOGGER_
