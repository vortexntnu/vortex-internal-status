#include <algorithm>
#include <array>
#include <asio.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using asio::io_context;
using asio::serial_port;

static constexpr uint8_t UART_START_BYTE = 0xAA;
static constexpr std::size_t READ_CHUNK_SIZE = 256;
static constexpr uint8_t MAX_PAYLOAD_SIZE = 255;

static constexpr uint8_t MSG_FLT_EVENT = 0x10;
static constexpr uint8_t MSG_PGOOD_EVENT = 0x11;
static constexpr uint8_t MSG_KILLSWITCH_EVENT = 0x12;
static constexpr uint8_t MSG_CURRENT_MEASUREMENTS = 0x13;

struct ChannelEvent {
    uint8_t channel = 0;
    uint8_t code = 0;
};

struct Frame {
    uint8_t msg_id = 0;
    uint8_t length = 0;
    std::vector<uint8_t> payload;
    uint8_t checksum = 0;
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

static std::string message_name(uint8_t msg_id) {
    switch (msg_id) {
        case MSG_FLT_EVENT:
            return "FLT_EVENT";
        case MSG_PGOOD_EVENT:
            return "PGOOD_EVENT";
        case MSG_KILLSWITCH_EVENT:
            return "KILLSWITCH_EVENT";
        case MSG_CURRENT_MEASUREMENTS:
            return "CURRENT_MEASUREMENTS";
        default:
            return "UNKNOWN";
    }
}

static std::string payload_to_hex(const std::vector<uint8_t>& payload) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < payload.size(); ++i) {
        if (i != 0) {
            oss << ' ';
        }
        oss << "0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(payload[i]);
    }
    return oss.str();
}

class CsvLogger {
   public:
    explicit CsvLogger(
        const std::string& log_dir = "/home/pi/can_logger") {
        std::filesystem::create_directories(log_dir);
        const std::string filename = make_log_filename(log_dir);
        out_.open(filename, std::ios::out | std::ios::app);
        if (!out_) {
            throw std::runtime_error("Failed to open CSV log file: " +
                                     filename);
        }

        out_ << "timestamp,msg_id_hex,msg_name,status,channel,code_hex,"
                "i0,i1,i2,i3,i4,i5,i6,i7,raw_payload\n";
        out_.flush();

        log_info("CSV logging to " + filename);
    }

    void log_event(uint8_t msg_id,
                   const std::string& status,
                   std::optional<uint8_t> channel,
                   std::optional<uint8_t> code,
                   const std::vector<uint8_t>& payload) {
        std::lock_guard<std::mutex> lock(mutex_);

        out_ << csv_escape(make_timestamp()) << ','
             << csv_escape(to_hex_byte(msg_id)) << ','
             << csv_escape(message_name(msg_id)) << ',' << csv_escape(status)
             << ',' << csv_escape(channel ? std::to_string(*channel) : "")
             << ',' << csv_escape(code ? to_hex_byte(*code) : "") << ','
             << ",,,,,,,,"  // i0..i7 empty
             << csv_escape(payload_to_hex(payload)) << '\n';

        out_.flush();
    }

    void log_currents(uint8_t msg_id,
                      const std::string& status,
                      const std::array<float, 8>& currents,
                      const std::vector<uint8_t>& payload) {
        std::lock_guard<std::mutex> lock(mutex_);

        out_ << csv_escape(make_timestamp()) << ','
             << csv_escape(to_hex_byte(msg_id)) << ','
             << csv_escape(message_name(msg_id)) << ',' << csv_escape(status)
             << ',' << ",,";

        for (std::size_t i = 0; i < currents.size(); ++i) {
            out_ << currents[i];
            out_ << ',';
        }

        out_ << csv_escape(payload_to_hex(payload)) << '\n';
        out_.flush();
    }

    void log_raw(uint8_t msg_id,
                 const std::string& status,
                 const std::vector<uint8_t>& payload) {
        std::lock_guard<std::mutex> lock(mutex_);

        out_ << csv_escape(make_timestamp()) << ','
             << csv_escape(to_hex_byte(msg_id)) << ','
             << csv_escape(message_name(msg_id)) << ',' << csv_escape(status)
             << ',' << ",,"
             << ",,,,,,,," << csv_escape(payload_to_hex(payload)) << '\n';

        out_.flush();
    }

   private:
    static std::string make_log_filename(const std::string& log_dir) {
        const auto now = std::chrono::system_clock::now();
        const auto t = std::chrono::system_clock::to_time_t(now);

        std::tm tm{};
        localtime_r(&t, &tm);

        std::ostringstream oss;
        oss << log_dir << "/serial_log_"
            << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
        return oss.str();
    }

    static std::string csv_escape(const std::string& value) {
        std::string out = "\"";
        for (char c : value) {
            if (c == '"') {
                out += "\"\"";
            } else {
                out += c;
            }
        }
        out += "\"";
        return out;
    }

    static std::string to_hex_byte(uint8_t value) {
        std::ostringstream oss;
        oss << "0x" << std::hex << std::uppercase << std::setw(2)
            << std::setfill('0') << static_cast<int>(value);
        return oss.str();
    }

    std::ofstream out_;
    std::mutex mutex_;
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
    explicit SerialFrameDecoder(CsvLogger& csv_logger)
        : csv_logger_(csv_logger) {}

    void append(const uint8_t* data, std::size_t length) {
        buffer_.insert(buffer_.end(), data, data + length);
        process_buffer();
    }

   private:
    static std::string hex_dump(const uint8_t* data, std::size_t length) {
        std::ostringstream oss;
        oss << std::hex << std::setfill('0');

        for (std::size_t i = 0; i < length; ++i) {
            if (i != 0) {
                oss << ' ';
            }
            oss << "0x" << std::setw(2) << static_cast<int>(data[i]);
        }

        return oss.str();
    }

    void process_buffer() {
        while (true) {
            if (buffer_.size() < 4) {
                return;
            }

            auto start_it =
                std::find(buffer_.begin(), buffer_.end(), UART_START_BYTE);
            if (start_it == buffer_.end()) {
                buffer_.clear();
                return;
            }

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
                oss << "Checksum error. "
                    << "msg_id=0x" << std::hex << std::setw(2)
                    << std::setfill('0') << static_cast<int>(frame.msg_id)
                    << ", length=" << std::dec << static_cast<int>(frame.length)
                    << ", received=0x" << std::hex << std::setw(2)
                    << static_cast<int>(frame.checksum) << ", expected=0x"
                    << std::setw(2) << static_cast<int>(expected)
                    << ", raw_frame=["
                    << hex_dump(buffer_.data(), full_frame_size) << "]";

                log_error(oss.str());

                csv_logger_.log_raw(frame.msg_id, "CHECKSUM_ERROR",
                                    frame.payload);

                // Resync by discarding just the start byte and trying again
                buffer_.erase(buffer_.begin());
                continue;
            }

            handle_frame(frame);
            buffer_.erase(buffer_.begin(), buffer_.begin() + full_frame_size);
        }
    }
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

                    csv_logger_.log_event(frame.msg_id, "OK", event.channel,
                                          event.code, frame.payload);
                } else {
                    log_info("Invalid FLT event frame");
                    csv_logger_.log_raw(frame.msg_id, "INVALID_FLT_EVENT",
                                        frame.payload);
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

                    csv_logger_.log_event(frame.msg_id, "OK", event.channel,
                                          event.code, frame.payload);
                } else {
                    log_info("Invalid PGOOD event frame");
                    csv_logger_.log_raw(frame.msg_id, "INVALID_PGOOD_EVENT",
                                        frame.payload);
                }
                break;
            }

            case MSG_KILLSWITCH_EVENT: {
                if (decode_killswitch_event(frame)) {
                    log_info("Killswitch event received");
                    csv_logger_.log_event(frame.msg_id, "OK", std::nullopt,
                                          std::nullopt, frame.payload);
                } else {
                    log_info("Invalid killswitch event frame");
                    csv_logger_.log_raw(frame.msg_id,
                                        "INVALID_KILLSWITCH_EVENT",
                                        frame.payload);
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

                    csv_logger_.log_currents(frame.msg_id, "OK", currents,
                                             frame.payload);
                } else {
                    log_info("Invalid current measurements frame");
                    csv_logger_.log_raw(frame.msg_id,
                                        "INVALID_CURRENT_MEASUREMENTS",
                                        frame.payload);
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

                csv_logger_.log_raw(frame.msg_id, "UNKNOWN_MESSAGE_ID",
                                    frame.payload);
                break;
            }
        }
    }

    CsvLogger& csv_logger_;
    std::vector<uint8_t> buffer_;
};

class SerialReceiver {
   public:
    SerialReceiver(io_context& io,
                   const std::string& port_name,
                   unsigned int baud_rate,
                   CsvLogger& csv_logger)
        : serial_(io), decoder_(csv_logger) {
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
        CsvLogger csv_logger("/home/pi/can_logger/logs");
        SerialReceiver receiver(io, port_name, baud_rate, csv_logger);
        receiver.start();
        io.run();
    } catch (const std::exception& e) {
        log_error(std::string("Exception: ") + e.what());
        return 1;
    }

    return 0;
}
