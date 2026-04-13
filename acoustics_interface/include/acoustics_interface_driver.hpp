#ifndef ACOUSTCS_INTERFACE_DRIVER_HPP_
#define ACOUSTCS_INTERFACE_DRIVER_HPP_

#include <asio.hpp>
#include <functional>
#include <linux/can.h>

struct AcousticsData {
    float x;
    float y;
    float z;
    float weight;
};

enum class can_status {
    OK,
    ERR_NOT_INITIALIZED,
    ERR_RECEIVE
};

class AcousticsInterfaceDriver {
public:
    AcousticsInterfaceDriver();

    can_status init_can();
    can_status read_acoustics(AcousticsData& data);

    can_status start_async_read(
        std::function<void(const AcousticsData&, can_status)> callback);

private:
    bool decode_frame(const struct canfd_frame& frame, AcousticsData& data);
    bool parse_serial_frame(const std::string& line, struct canfd_frame& frame);

    void do_async_read();

private:
    asio::io_context io_;
    asio::serial_port serial_{io_};
    asio::streambuf buffer_;

    std::function<void(const AcousticsData&, can_status)> callback_;

    bool initialized_{false};
};

#endif

