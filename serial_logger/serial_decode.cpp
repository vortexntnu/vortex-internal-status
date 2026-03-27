#include <asio.hpp>
#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using asio::io_context;
using asio::serial_port;

static constexpr uint8_t UART_START_BYTE = 0xAA;
static constexpr std::size_t READ_CHUNK_SIZE = 256;
static constexpr std::size_t MAX_PAYLOAD_SIZE = 255;

// Set this to whatever MSG_CURRENT_MEASUREMENTS is in your embedded code.
static constexpr uint8_t MSG_CURRENT_MEASUREMENTS = 0x13;

struct Frame {
    uint8_t msg_id = 0;
    uint8_t length = 0;
    std::vector<uint8_t> payload;
    uint8_t checksum = 0;
};

static uint8_t compute_checksum(uint8_t msg_id, uint8_t length, const uint8_t* payload) {
    uint8_t csum = msg_id ^ length;
    for (uint8_t i = 0; i < length; ++i) {
        csum ^= payload[i];
    }
    return csum;
}

static bool decode_current_measurements(const Frame& frame, std::array<float, 8>& currents) {
    if (frame.msg_id != MSG_CURRENT_MEASUREMENTS) {
        return false;
    }

    if (frame.length != 8 * sizeof(float)) {
        std::cerr << "Unexpected payload length for current measurements: "
                  << static_cast<int>(frame.length) << "\n";
        return false;
    }

    for (std::size_t i = 0; i < 8; ++i) {
        std::memcpy(&currents[i],
                    &frame.payload[i * sizeof(float)],
                    sizeof(float));
    }

    return true;
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
            auto start_it = std::find(buffer_.begin(), buffer_.end(), UART_START_BYTE);
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
                std::cerr << "Invalid length: " << static_cast<int>(length) << "\n";
                buffer_.erase(buffer_.begin());
                continue;
            }

            const std::size_t full_frame_size = 4u + static_cast<std::size_t>(length);
            if (buffer_.size() < full_frame_size) {
                // Wait for more data
                return;
            }

            Frame frame;
            frame.msg_id = msg_id;
            frame.length = length;
            frame.payload.assign(buffer_.begin() + 3, buffer_.begin() + 3 + length);
            frame.checksum = buffer_[3 + length];

            const uint8_t expected =
                compute_checksum(frame.msg_id, frame.length, frame.payload.data());

            if (frame.checksum != expected) {
                std::cerr << "Checksum error. Received: 0x"
                          << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(frame.checksum)
                          << ", expected: 0x"
                          << std::setw(2)
                          << static_cast<int>(expected)
                          << std::dec << "\n";

                // Resync by discarding just the start byte and trying again
                buffer_.erase(buffer_.begin());
                continue;
            }

            handle_frame(frame);

            // Remove processed frame
            buffer_.erase(buffer_.begin(), buffer_.begin() + full_frame_size);
        }
    }

    void handle_frame(const Frame& frame) {
        std::cout << "Valid frame received: msg_id=0x"
                  << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(frame.msg_id)
                  << std::dec
                  << ", length=" << static_cast<int>(frame.length) << "\n";

        if (frame.msg_id == MSG_CURRENT_MEASUREMENTS) {
            std::array<float, 8> currents{};
            if (decode_current_measurements(frame, currents)) {
                std::cout << "Current measurements:\n";
                for (std::size_t i = 0; i < currents.size(); ++i) {
                    std::cout << "  I[" << i << "] = " << currents[i] << "\n";
                }
            }
        } else {
            std::cout << "Payload bytes:";
            for (uint8_t b : frame.payload) {
                std::cout << " 0x"
                          << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(b);
            }
            std::cout << std::dec << "\n";
        }
    }

    std::vector<uint8_t> buffer_;
};

class SerialReceiver {
public:
    SerialReceiver(io_context& io, const std::string& port_name, unsigned int baud_rate)
        : serial_(io), decoder_() {
        serial_.open(port_name);
        serial_.set_option(serial_port::baud_rate(baud_rate));
        serial_.set_option(serial_port::character_size(8));
        serial_.set_option(serial_port::parity(serial_port::parity::none));
        serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));
    }

    void start() {
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
                    std::cerr << "Serial read error: " << ec.message() << "\n";
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
    const unsigned int baud_rate = static_cast<unsigned int>(std::stoul(argv[2]));

    try {
        io_context io;
        SerialReceiver receiver(io, port_name, baud_rate);
        receiver.start();
        io.run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
