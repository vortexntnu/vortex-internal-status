#include "am_driver.hpp"
#include <utility>

AcousticModemDriver::AcousticModemDriver(const std::string& device,
                                         int baudrate,
                                         int channel,
                                         int level,
                                         bool diagnostic,
                                         float timeout)
    : io_(1),         // I/O context, required by the library
      drv_(io_),      // SerialDriver object
      cfg_(baudrate,  // configuration how UART should operate
           drivers::serial_driver::FlowControl::NONE,
           drivers::serial_driver::Parity::NONE,
           drivers::serial_driver::StopBits::ONE),
      // saves the device path to a member variable
      device_(device),
      channel_(channel),
      level_(level),
      diagnostic_(diagnostic) {
    // initialization of the port in device_
    msg_id=0;
    drv_.init_port(device_, cfg_);

    // creation of the actual SerialPort object (create to use open(), close()
    // ...)
    port_ = drv_.port();

    // actually open physical serial port
    port_->open();

    if (!(port_->is_open())) {
        std::cerr
            << "[Error] Serial port not open. Communication will not start."
            << std::endl;
        return;
    }

    this->set_channel(channel);
    this->set_level(level);

    if (diagnostic) {
        this->set_diagnostic_mode();
    } else {
        this->reset_diagnostic_mode();
    }

    // start waiting for data
    this->start_async_read();

    std::cout << "[INFO] Modem initialized on device " << this->device_
              << ", channel " << channel << ", level " << level
              << ", diagnostic mode = " << std::boolalpha << diagnostic
              << std::endl;
}

AcousticModemDriver::~AcousticModemDriver() {
    port_->close();
}

// AcousticModemDriver::AcousticModemDriver()
//     : io_(1),
//       drv_(io_),
//       cfg_(9600,
//            drivers::serial_driver::FlowControl::NONE,
//            drivers::serial_driver::Parity::NONE,
//            drivers::serial_driver::StopBits::ONE),
//       channel_(0),
//       level_(0),
//       diagnostic_(false) {
    
// }

uint16_t AcousticModemDriver::make_handshake(MsgType t){
    const uint16_t id  = (msg_id++ & 0x03FF);          // 10-bit rolling counter
    const uint16_t type = (uint16_t(t) & 0x0003);       // keep only 2 bits
    // [SYNC(4) | TYPE(2) | MSG_ID(10)]
    return uint16_t((uint16_t(HANDSHAKE_SYNC) << 12) |
                    (type << 10) |
                    id);
}

bool AcousticModemDriver::is_handshake(uint16_t packet){
    return ((packet>>12 & 0xF)==HANDSHAKE_SYNC);
}
MsgType AcousticModemDriver::type(uint16_t packet){
    return MsgType((packet>>10 & 0x3));
}
uint16_t AcousticModemDriver::id(uint16_t packet){
    return ((packet & 0x3FF));
}

size_t AcousticModemDriver::floats_for_type(MsgType t) {
  switch (t) {
    case MsgType::Type_1:  return 2;
    case MsgType::Type_2:  return 4;
    case MsgType::Type_3:  return 5;
    default:               return 0;
  }
}

void AcousticModemDriver::float_to_word(float v, uint16_t &w0, uint16_t &w1){
    uint8_t b[4];
    std::memcpy(b,&v,4);

    w0 = uint16_t(b[0]) | (uint16_t(b[1]) << 8); // low part of w1 = b[0], high part=b[1]
    w1 = uint16_t(b[2]) | (uint16_t(b[3]) << 8);
}

void AcousticModemDriver::send_word(uint16_t w){
    std::string s(2, '\0'); // string of 2 bytes
    s[0]=static_cast<char>(w & 0xFF);
    s[1]=static_cast<char>((w>>8) & 0xFF);
    send_two_bytes(s);
}

void AcousticModemDriver::send_message(MsgType type, const float* data){
    const size_t n= floats_for_type(type);
    // TODO: implement for fourth type of data or default case (return;)
    if(n==0 || data==nullptr){
        return;
    }
    send_word(make_handshake(type)); // send the first packet of 16 bit containing [SYNC(4) | TYPE(2) | MSG_ID(10)]
    for(int i=0;i<n;++i){
        uint16_t w0;
        uint16_t w1;
        float_to_word(data[i], w0, w1);
        send_word(w0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // we could implement it similar to send_msg
        send_word(w1);
    }
}

void AcousticModemDriver::rx_reset() {
    rx_receiving= false;
    rx_expected_words= 0;
    rx_received_words = 0;
}

void AcousticModemDriver::rx_start_handshake(uint16_t hs_word){
    
}

bool AcousticModemDriver::rx_rebuild_word(uint16_t w, MsgType &out_type, uint16_t &out_msg_id, float *out_floats, uint8_t &inout_capacity, std::chrono::milliseconds timeout){
    auto now = std::chrono::steady_clock::now();

    // timeout for incomplete message (the timeout is to be defined if it is needed)
    if (rx_receiving && (now - rx_last_rx > timeout)) {
        rx_reset();
    }
    rx_last_rx= now;



}


size_t AcousticModemDriver::send_data(std::string data) {
    // send data using serial driver's send(msg)
    std::vector<uint8_t> msg(data.begin(), data.end());
    size_t bytes = port_->send(msg);
    return bytes;
}

// overload send_data(char)
size_t AcousticModemDriver::send_data(char data) {
    // send data using serial driver's send(msg)
    std::vector<uint8_t> msg = {static_cast<uint8_t>(data)};
    size_t bytes = port_->send(msg);
    return bytes;
}

size_t AcousticModemDriver::send_two_bytes(std::string data) {
    // only send 2 bytes waiting 1 second after sending them
    if (data.length() != 2) {
        return 0;
    } else {
        size_t bytes = this->send_data(data);

        // 10bps
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return bytes;
    }
}

size_t AcousticModemDriver::send_msg(std::string data, float timeout) {
    size_t sum_sent_char = 0;

    if (data.length() % 2 != 0) {
        data += ' ';
    }
    for (int i = 0; i < data.length(); i += 2) {
        std::string chunk = data.substr(i, 2);
        size_t sent_chunk = this->send_two_bytes(chunk);
        if (sent_chunk != 0) {
            sum_sent_char += sent_chunk;
        }

        if (!(this->diagnostic_)) {
            // wait for transmission
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        auto start_time = std::chrono::steady_clock::now();

        while ((std::chrono::steady_clock::now() - start_time) <
               std::chrono::duration<float>(timeout)) {
            std::optional<std::vector<uint8_t>> packet = this->read_packet();
            if (!packet.has_value()) {
                break;
            }
            // std::vector<uint8_t>
            // packet_cast=static_cast<std::vector<uint8_t>>(*packet);
            std::optional<DiagnosticData> report = this->decode_packet(*packet);
            if (report.has_value() && report->TX_COMPLETE == 1) {
                std::cout << "Transmission complete for chunk: " << chunk
                          << std::endl;
                break;
            }
        }
    }
    return sum_sent_char;
}

// to set channel of communication
bool AcousticModemDriver::set_channel(int channel) {
    // check channel is correct number
    if (channel < 1 || channel > 12) {
        std::cout << "Warning: Channel " << channel
                  << " is not a valid channel, needs to be between 1 and 12."
                  << std::endl;
        return false;
    }
    // wait 1 sec between c and c to go in command mode
    this->send_data('c');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('c');

    switch (channel) {
        case 10:
            this->send_data('a');
            break;
        case 11:
            this->send_data('b');
            break;
        case 12:
            this->send_data('c');
            break;
        default:
            this->send_data(std::to_string(channel));
            break;
    }
    channel_ = channel;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
}

// to set power level
bool AcousticModemDriver::set_level(int level) {
    if (level < 1 || level > 4) {
        std::cout << "Warning: Level " << level
                  << " is not a valid Level, needs to be between 1 and 4."
                  << std::endl;
        return false;
    }
    this->send_data('l');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('l');

    this->send_data(std::to_string(level));
    level_ = level;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
}

// to set diagnostic mode
bool AcousticModemDriver::set_diagnostic_mode() {
    this->send_data('d');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('d');
    diagnostic_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
}

bool AcousticModemDriver::reset_diagnostic_mode() {
    this->send_data('t');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('t');
    diagnostic_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return false;
}

std::optional<DiagnosticData> AcousticModemDriver::request_report(
    float overall_timeout,
    std::optional<std::string> filename) {
    this->get_report();

    std::optional<std::vector<uint8_t>> packet = this->read_packet();
    if (!(packet.has_value())) {
        std::cout << "Packet is empty" << std::endl;
        return std::nullopt;
    }
    // std::vector<uint8_t>
    // packet_cast=static_cast<std::vector<uint8_t>>(*packet);
    std::cout << "Returning packet of length: "
              << std::string((*packet).begin(), (*packet).end()).length()
              << std::endl;

    std::optional<DiagnosticData> report = this->decode_packet(*packet);
    if (!(report.has_value())) {
        std::cout << "Failed to decode the packet." << std::endl;
        return std::nullopt;
    }
    // DiagnosticData report_cast=static_cast<DiagnosticData>(*report);
    this->update_state_from_report(*report);

    // TODO? implement saving report in json file
    if (filename.has_value()) {
        // TODO
    }

    return *report;
}

void AcousticModemDriver::update_state_from_report(DiagnosticData report) {
    this->channel_ = static_cast<int>(report.CHANNEL);
    this->level_ = static_cast<int>(report.POWER_LEVEL);
    this->diagnostic_ = report.DIAGNOSTIC_MODE;
    std::cout << "Updated channel: " << this->channel_
              << "Updated level: " << this->level_
              << "Updated diagnostic: " << this->diagnostic_ << std::endl;
}

void AcousticModemDriver::get_report() {
    this->send_data('r');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('r');
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void AcousticModemDriver::start_async_read() {
    port_->async_receive(
        [this](std::vector<uint8_t>& buffer, const size_t& bytes_transferred) {
            std::vector<uint8_t> data(buffer.begin(),
                                      buffer.begin() + bytes_transferred);
            this->read_callback(data);
            this->start_async_read();
        });
}

/**
 * mutex probably needed because of async_receive
 * asio lib creates a new thread, if we access that both when some data arrive
 * (push) and when we need to extract (pop)
 */
void AcousticModemDriver::read_callback(std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    queue.push(data);
}

/**
 * re-build packet based on header bits
 * checking for code, order and last
 */
std::optional<std::vector<uint8_t>> AcousticModemDriver::process_packet() {
    std::vector<uint8_t> fragment;
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (queue.empty()) {
            return std::nullopt;
        }
        fragment = std::move(queue.front());
        queue.pop();
        // mutex end
    }

    // to fix: using header?
    uint8_t byte0 = fragment[0];
    uint8_t byte1 = fragment[1];
    uint8_t type = (byte0 & 0xC0) >> 6;
    uint8_t order = (byte0 & 0x38) >> 3;
    bool last = (byte0 & 0x04) >> 2;
    uint16_t data = ((byte0 & 0x03) << 8) | byte1;
    map[type][order] = data;

    if (!last) {
        return std::nullopt;
    }

    bool complete = true;
    for (int i = 0; i <= order; ++i) {
        if (map[type].count(i) == 0) {
            complete = false;
            // last arrived but not every fragment is inside the map
            break;
        }
    }

    if (!complete) {
        // TODO: how to behave if not every fragment is present
        // for now it just return null and the message is lost, 
        // goal is to implement a way to retrieve the piece we lose
        full_message.clear();
        map.erase(type);
        bit_pos_ = 0;
        return std::nullopt;
    }

    // adding two bit at the start of the full message to understand which type
    // of message is
    append_bits(full_message, type, 2, bit_pos_);

    // need to concatenate 10 data bit for each fragment
    // we use order as number of package because we are working with the last
    // package order
    for (int i = 0; i <= order; i++) {
        append_bits(full_message, map[type][i], 10, bit_pos_);
    }

    auto message = full_message;

    // deleting data for next message
    full_message.clear();
    map.erase(type);
    bit_pos_ = 0;

    return message;
}

void AcousticModemDriver::append_bits(std::vector<uint8_t>& buffer,
                                      uint16_t bit_to_append,
                                      int count,
                                      int& bit_position) {
    for (int i = count - 1; i >= 0; --i) {
        // extract the bit
        bool bit = (bit_to_append >> i) & 1;

        // if we completed the previous element of the buffer, we create a new
        // one
        if (bit == 0) {
            buffer.push_back(0);
        }

        if (bit) {
            buffer.back() |= (1 << (7 - bit_position));
        }
        // increase the position we are adding the bit
        bit_position = (bit_position + 1) % 8;
    }
}

std::optional<std::vector<uint8_t>> AcousticModemDriver::read_packet() {
    // time duration to wait for a valid packet
    const auto time_duration = std::chrono::seconds(2);
    auto start_time = std::chrono::steady_clock::now();

    std::vector<uint8_t> buffer(64);  // 64 can be modified, didn't put 18 to
                                      // avoid cutting diagnostic packet in half

    while ((std::chrono::steady_clock::now() - start_time) < time_duration) {
      std::vector<uint8_t> temp_buffer(
          64);  // use this to store temporanealy buffer data, to add them to
                // buffer
      size_t bytes_read = port_->receive(temp_buffer);
      temp_buffer.resize(
          bytes_read);  // resize the temp_buffer to the actual byte read

      if (bytes_read == 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
      }

      buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());

      std::cout << "Buffer size: " << bytes_read << " bytes, "
                << "Buffer: " << std::string(buffer.begin(), buffer.end())
                << std::endl;

      if (bytes_read <= 17) {  // look for diagnostic packet
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
      }
      auto begin =
          find(buffer.begin(), buffer.end(), '$');  // returns iterator
      auto end = find(buffer.begin(), buffer.end(), '\n');

      if (begin != buffer.end() &&
          end != buffer.end()) {  // create and return diagnostic packet
          std::vector<uint8_t> packet(begin, end + 1);
          std::cout << "Returning packet: "
                    << std::string(packet.begin(), packet.end()) << std::endl;
          return packet;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
    if (buffer.size() == 0) {
        std::cout << "Returning no data";
        return std::nullopt;
    } else {
        std::cout << "Returning buffer: "
                  << std::string(buffer.begin(), buffer.end()) << std::endl;
        return buffer;
    }
}

std::optional<DiagnosticData> AcousticModemDriver::decode_packet(
    std::vector<uint8_t>& packet) {
    std::string packet_str(packet.begin(), packet.end());

    if (packet_str.size() != 18 || packet_str.front() != '$' ||
        packet_str.back() != '\n') {  // check if the packet is 18 bytes with
                                      // the start and end character
        return std::nullopt;
    }

    std::vector<uint8_t> data_bytes(
        packet.begin() + 1,
        packet.begin() + 17);  // remove start and end character

    DiagnosticPacket raw{};
    std::memcpy(
        &raw, data_bytes.data(),
        sizeof(data_bytes));  // copy raw bytes into struct DiagnosticPacket

    DiagnosticData data{};
    // Decode packet in DiagnosticData Struct
    data.TR_BLOCK[0] = static_cast<uint8_t>(raw.TR_BLOCK & 0xFF);
    data.TR_BLOCK[1] = static_cast<uint8_t>((raw.TR_BLOCK) >> 8 & 0xFF);
    data.BER = raw.BER;
    data.SIGNAL_POWER = raw.SIGNAL_POWER;
    data.NOISE_POWER = raw.SIGNAL_POWER;
    data.PACKET_VALID[0] = static_cast<uint8_t>(raw.PACKET_VALID & 0xFF);
    data.PACKET_VALID[1] = static_cast<uint8_t>((raw.PACKET_VALID) >> 8 & 0xFF);
    data.PACKET_INVALID = raw.PACKET_INVALID;
    data.GIT_REV = raw.GIT_REV;
    data.TIME[0] = raw.TIME_L;
    data.TIME[1] = raw.TIME_M;
    data.TIME[2] = raw.TIME_H;
    data.CHIP_ID[0] = static_cast<uint8_t>(raw.CHIP_ID & 0xFF);
    data.CHIP_ID[0] = static_cast<uint8_t>((raw.CHIP_ID) >> 8 & 0xFF);

    uint8_t hw = raw.HW_CH_FLAGS;
    data.HW_REV = hw & 0x03;
    data.CHANNEL = hw & 0x3C;
    data.TB_VALID = hw & 0x40;
    data.TX_COMPLETE = hw & 0x80;

    uint8_t ml = raw.MODE_LEVEL_FLAGS;
    data.DIAGNOSTIC_MODE = ml & 0x01;
    data.POWER_LEVEL = ml & 0x0C;

    return data;
}

void AcousticModemDriver::close() {
    port_->close();
}
