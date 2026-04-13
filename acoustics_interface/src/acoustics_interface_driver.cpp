

#include "acoustics_interface_driver.hpp"

#include <cstring>
#include <iostream>
#include <thread>

#define ACOUSTICS_CAN_ID 0x200

AcousticsInterfaceDriver::AcousticsInterfaceDriver() {}

can_status AcousticsInterfaceDriver::init_can() {
    try {
        serial_.open("/dev/ttyACM0");
        serial_.set_option(asio::serial_port_base::baud_rate(115200));
        serial_.set_option(asio::serial_port_base::character_size(8));
        serial_.set_option(asio::serial_port_base::parity(
            asio::serial_port_base::parity::none));
        serial_.set_option(asio::serial_port_base::stop_bits(
            asio::serial_port_base::stop_bits::one));

        initialized_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Serial init failed: " << e.what() << std::endl;
        return can_status::ERR_NOT_INITIALIZED;
    }

    return can_status::OK;
}


bool AcousticsInterfaceDriver::parse_serial_frame(
    const std::string& line,
    struct canfd_frame& frame)
{
    // Minimum length: 1 (prefix) + 3 (id) + 32 (payload)
    if (line.size() < 36) {
        std::cerr << "Invalid line size: " << line.size() << std::endl;
        return false;
    }

    try {
        // Optional: validate prefix
        if (line[0] != 'b') {
            std::cerr << "Invalid frame prefix: " << line[0] << std::endl;
            return false;
        }

        // Extract CAN ID (skip prefix)
        std::string id_str = line.substr(1, 3);
        frame.can_id = std::stoul(id_str, nullptr, 16);

        // Extract payload
        std::string payload = line.substr(5, 34);

        if (payload.size() != 33) {
            std::cerr << "Invalid payload size: " << payload.size() << std::endl;
            return false;
        }

        frame.len = 16;

        //std::cout << line << std::endl;
        for (int i = 0; i < 16; ++i) {
            std::string byte_str = payload.substr(i * 2, 2);
            frame.data[i] =
                static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16));
          //      std::cout << "Byte " << i << ": " << byte_str << " -> "
            //              << static_cast<int>(frame.data[i]) << std::endl;
        }

    } catch (...) {
        std::cerr << "Exception while parsing frame: " << line << std::endl;
        return false;
    }

    return true;
}

bool AcousticsInterfaceDriver::decode_frame(
    const struct canfd_frame& frame,
    AcousticsData& data)
{
    if ((frame.can_id & CAN_SFF_MASK) != ACOUSTICS_CAN_ID) {
        return false;
    }

    if (frame.len != 16) {
        return false;
    }

    std::memcpy(&data.x,      &frame.data[0],  4);
    std::memcpy(&data.y,      &frame.data[4],  4);
    std::memcpy(&data.z,      &frame.data[8],  4);
    std::memcpy(&data.weight, &frame.data[12], 4);

    return true;
}

can_status AcousticsInterfaceDriver::read_acoustics(AcousticsData& data) {
    if (!initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    try {
        asio::read_until(serial_, buffer_, '\r');

        std::istream is(&buffer_);
        std::string line;
        std::getline(is, line);

        struct canfd_frame frame{};

        if (!parse_serial_frame(line, frame)) {
            return can_status::ERR_RECEIVE;
        }

        if (!decode_frame(frame, data)) {
            return can_status::ERR_RECEIVE;
        }

    } catch (...) {
        return can_status::ERR_RECEIVE;
    }

    return can_status::OK;
}


void AcousticsInterfaceDriver::do_async_read() {
    asio::async_read_until(serial_, buffer_, '\r',
        [this](std::error_code ec, std::size_t) {
            if (ec) {
                if (callback_) {
                    callback_(AcousticsData{}, can_status::ERR_RECEIVE);
                }
                return;
            }

            std::istream is(&buffer_);
            std::string line;
            std::getline(is, line);

            struct canfd_frame frame{};
            AcousticsData data{};

            if (!parse_serial_frame(line, frame) ||
                !decode_frame(frame, data)) {

                if (callback_) {
                    callback_(AcousticsData{}, can_status::ERR_RECEIVE);
                }
            } else {
                if (callback_) {
                    callback_(data, can_status::OK);
                }
            }

            do_async_read(); // continue loop
        });
}

can_status AcousticsInterfaceDriver::start_async_read(
    std::function<void(const AcousticsData&, can_status)> callback)
{
    if (!initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    callback_ = callback;

    do_async_read();

    // Run IO in background thread
    std::thread([this]() {
        io_.run();
    }).detach();

    return can_status::OK;
}