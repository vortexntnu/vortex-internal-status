#include <array>
#include <asio.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using asio::io_context;
using asio::serial_port;

static constexpr uint8_t UART_START_BYTE = 0xAA;
static constexpr std::size_t READ_CHUNK_SIZE = 256;
static constexpr std::size_t MAX_PAYLOAD_SIZE = 255;

// Set this to whatever MSG_CURRENT_MEASUREMENTS is in your embedded code.
static constexpr uint8_t MSG_FLT_EVENT = 0x10;
static constexpr uint8_t MSG_PGOOD_EVENT = 0x11;
static constexpr uint8_t MSG_KILLSWITCH_EVENT = 0x12;
static constexpr uint8_t MSG_CURRENT_MEASUREMENTS = 0x13;

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>

struct ChannelEvent {
    uint8_t channel = 0;
    uint8_t code = 0;
};

static std::string make_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) %
                    1000;

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3)
        << std::setfill('0') << ms.count();
    return oss.str();
}

static void log_info(const std::string& message) {
    std::cout << "[" << make_timestamp() << "] [INFO] " << message << "\n";
}

static void log_error(const std::string& message) {
    std::cerr << "[" << make_timestamp() << "] [ERROR] " << message << "\n";
}

struct Frame {
    uint8_t msg_id = 0;
    uint8_t length = 0;
    std::vector<uint8_t> payload;
    uint8_t checksum = 0;
};

static uint8_t compute_checksum(uint8_t msg_id,
                                uint8_t length,
                                const uint8_t* payload) {
    uint8_t csum = msg_id ^ length;
    for (uint8_t i = 0; i < length; ++i) {
        csum ^= payload[i];
    }
    return csum;
}

static bool decode_current_measurements(const Frame& frame,
                                        std::array<float, 8>& currents) {
    if (frame.msg_id != MSG_CURRENT_MEASUREMENTS) {
        return false;
    }

    if (frame.length != 8 * sizeof(float)) {
        log_error("Unexpected payload length for current measurements: " +
                  std::to_string(static_cast<int>(frame.length)));
        return false;
    }

    for (std::size_t i = 0; i < 8; ++i) {
        std::memcpy(&currents[i], &frame.payload[i * sizeof(float)],
                    sizeof(float));
    }

    return true;
}

// static bool decode_current_measurements(const Frame& frame, std::array<float,
// 8>& currents) {
//     if (frame.msg_id != MSG_CURRENT_MEASUREMENTS) {
//         return false;
//     }
//
//     if (frame.length != 8 * sizeof(float)) {
//         return false;
//     }
//
//     for (std::size_t i = 0; i < 8; ++i) {
//         std::memcpy(&currents[i],
//                     &frame.payload[i * sizeof(float)],
//                     sizeof(float));
//     }
//
//     return true;
// }

static bool decode_flt_event(const Frame& frame, ChannelEvent& event) {
    if (frame.msg_id != MSG_FLT_EVENT) {
        return false;
    }

    if (frame.length != 2 || frame.payload.size() != 2) {
        return false;
    }

    event.channel = frame.payload[0];
    event.code = frame.payload[1];

    return event.code == 0x01;
}

static bool decode_pgood_event(const Frame& frame, ChannelEvent& event) {
    if (frame.msg_id != MSG_PGOOD_EVENT) {
        return false;
    }

    if (frame.length != 2 || frame.payload.size() != 2) {
        return false;
    }

    event.channel = frame.payload[0];
    event.code = frame.payload[1];

    return event.code == 0x02;
}

static bool decode_killswitch_event(const Frame& frame) {
    if (frame.msg_id != MSG_KILLSWITCH_EVENT) {
        return false;
    }

    return frame.length == 0 && frame.payload.empty();
}

class SerialFrameDecoder {
   public:
    void append(const uint8_t* data, std::size_t length) {
        buffer_.insert(buffer_.end(), data, data + length);
        process_buffer();
    }

   private:
    void process_buffer() {
        while (true) {
            // Need at least: start + msg_id + length + checksum
            if (buffer_.size() < 4) {
                return;
            }

            // Find start byte
            auto start_it =
                std::find(buffer_.begin(), buffer_.end(), UART_START_BYTE);
            if (start_it == buffer_.end()) {
                buffer_.clear();
                return;
            }

            // Drop garbage before start byte
            if (start_it != buffer_.begin()) {
                buffer_.erase(buffer_.begin(), start_it);
            }

            if (buffer_.size() < 4) {
                return;
            }

            const uint8_t start = buffer_[0];
            const uint8_t msg_id = buffer_[1];
            const uint8_t length = buffer_[2];

            if (start != UART_START_BYTE) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            if (length > MAX_PAYLOAD_SIZE) {
                log_error("Invalid length: " +
                          std::to_string(static_cast<int>(length)));
                buffer_.erase(buffer_.begin());
                continue;
            }

            const std::size_t full_frame_size =
                4u + static_cast<std::size_t>(length);
            if (buffer_.size() < full_frame_size) {
                return;
            }

            Frame frame;
            frame.msg_id = msg_id;
            frame.length = length;
            frame.payload.assign(buffer_.begin() + 3,
                                 buffer_.begin() + 3 + length);
            frame.checksum = buffer_[3 + length];

            const uint8_t expected = compute_checksum(
                frame.msg_id, frame.length, frame.payload.data());

            if (frame.checksum != expected) {
                std::ostringstream oss;
                oss << "Checksum error. Received: 0x" << std::hex
                    << std::setw(2) << std::setfill('0')
                    << static_cast<int>(frame.checksum) << ", expected: 0x"
                    << std::setw(2) << static_cast<int>(expected);
                log_error(oss.str());

                // Resync by discarding just the start byte and trying again
                buffer_.erase(buffer_.begin());
                continue;
            }

            handle_frame(frame);

            // Remove processed frame
            buffer_.erase(buffer_.begin(), buffer_.begin() + full_frame_size);
        }
    }
    //
    // void handle_frame(const Frame& frame) {
    //     {
    //         std::ostringstream oss;
    //         oss << "Valid frame received: msg_id=0x"
    //             << std::hex << std::setw(2) << std::setfill('0')
    //             << static_cast<int>(frame.msg_id)
    //             << std::dec
    //             << ", length=" << static_cast<int>(frame.length);
    //         log_info(oss.str());
    //     }
    //
    //     if (frame.msg_id == MSG_CURRENT_MEASUREMENTS) {
    //         std::array<float, 8> currents{};
    //         if (decode_current_measurements(frame, currents)) {
    //             for (std::size_t i = 0; i < currents.size(); ++i) {
    //                 std::ostringstream oss;
    //                 oss << "Current measurement I[" << i << "] = " <<
    //                 currents[i]; log_info(oss.str());
    //             }
    //         }
    //     } else {
    //         std::ostringstream oss;
    //         oss << "Payload bytes:";
    //         for (uint8_t b : frame.payload) {
    //             oss << " 0x"
    //                 << std::hex << std::setw(2) << std::setfill('0')
    //                 << static_cast<int>(b);
    //         }
    //         log_info(oss.str());
    //     }
    // }
    void handle_frame(const Frame& frame) {
        {
            std::ostringstream oss;
            oss << "Valid frame received: msg_id=0x" << std::hex << std::setw(2)
                << std::setfill('0') << static_cast<int>(frame.msg_id)
                << std::dec << ", length=" << static_cast<int>(frame.length);
            log_info(oss.str());
        }

        switch (frame.msg_id) {
            case MSG_FLT_EVENT: {
                ChannelEvent event{};
                if (decode_flt_event(frame, event)) {
                    std::ostringstream oss;
                    oss << "FLT event: channel="
                        << static_cast<int>(event.channel) << ", code=0x"
                        << std::hex << std::setw(2) << std::setfill('0')
                        << static_cast<int>(event.code);
                    log_info(oss.str());
                } else {
                    log_info("Invalid FLT event frame");
                }
                break;
            }

            case MSG_PGOOD_EVENT: {
                ChannelEvent event{};
                if (decode_pgood_event(frame, event)) {
                    std::ostringstream oss;
                    oss << "PGOOD event: channel="
                        << static_cast<int>(event.channel) << ", code=0x"
                        << std::hex << std::setw(2) << std::setfill('0')
                        << static_cast<int>(event.code);
                    log_info(oss.str());
                } else {
                    log_info("Invalid PGOOD event frame");
                }
                break;
            }

            case MSG_KILLSWITCH_EVENT: {
                if (decode_killswitch_event(frame)) {
                    log_info("Killswitch event received");
                } else {
                    log_info("Invalid killswitch event frame");
                }
                break;
            }

            case MSG_CURRENT_MEASUREMENTS: {
                std::array<float, 8> currents{};
                if (decode_current_measurements(frame, currents)) {
                    for (std::size_t i = 0; i < currents.size(); ++i) {
                        std::ostringstream oss;
                        oss << "Current measurement I[" << i
                            << "] = " << currents[i];
                        log_info(oss.str());
                    }
                } else {
                    log_info("Invalid current measurements frame");
                }
                break;
            }

            default: {
                std::ostringstream oss;
                oss << "Unknown message ID 0x" << std::hex << std::setw(2)
                    << std::setfill('0') << static_cast<int>(frame.msg_id)
                    << ", payload bytes:";
                for (uint8_t b : frame.payload) {
                    oss << " 0x" << std::hex << std::setw(2)
                        << std::setfill('0') << static_cast<int>(b);
                }
                log_info(oss.str());
                break;
            }
        }
    }

    std::vector<uint8_t> buffer_;
};

class SerialReceiver {
   public:
    SerialReceiver(io_context& io,
                   const std::string& port_name,
                   unsigned int baud_rate)
        : serial_(io), decoder_() {
        serial_.open(port_name);
        serial_.set_option(serial_port::baud_rate(baud_rate));
        serial_.set_option(serial_port::character_size(8));
        serial_.set_option(serial_port::parity(serial_port::parity::none));
        serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        serial_.set_option(
            serial_port::flow_control(serial_port::flow_control::none));

        std::ostringstream oss;
        oss << "Opened serial port " << port_name << " at " << baud_rate
            << " baud";
        log_info(oss.str());
    }

    void start() {
        log_info("Starting async serial read");
        do_read();
    }

   private:
    void do_read() {
        serial_.async_read_some(
            asio::buffer(read_buf_),
            [this](const std::error_code& ec, std::size_t bytes_transferred) {
                if (!ec) {
                    decoder_.append(read_buf_.data(), bytes_transferred);
                    do_read();
                } else {
                    log_error("Serial read error: " + ec.message());
                }
            });
    }

    serial_port serial_;
    SerialFrameDecoder decoder_;
    std::array<uint8_t, READ_CHUNK_SIZE> read_buf_{};
};

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <port> <baud>\n";
        std::cerr << "Example Linux:   " << argv[0] << " /dev/ttyUSB0 115200\n";
        std::cerr << "Example Windows: " << argv[0] << " COM3 115200\n";
        return 1;
    }

    const std::string port_name = argv[1];
    const unsigned int baud_rate =
        static_cast<unsigned int>(std::stoul(argv[2]));

    try {
        io_context io;
        SerialReceiver receiver(io, port_name, baud_rate);
        receiver.start();
        io.run();
    } catch (const std::exception& e) {
        log_error(std::string("Exception: ") + e.what());
        return 1;
    }

    return 0;
}
