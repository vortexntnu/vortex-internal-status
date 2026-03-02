#include "am_driver.hpp"
#include <utility>

AcousticModemDriver::AcousticModemDriver(const std::string& device,
                                         int baudrate,
                                         int channel,
                                         int level,
                                         bool diagnostic,
                                         float timeout): 
                    io_(),
                    m_serial_port(io_),
                    device_(device),
                    channel_(channel),
                    level_(level),
                    diagnostic_(diagnostic) {
    // initialization of the port in device_
    msg_id=0;
    // actually open physical serial port
    this->open(device,baudrate);

    // not necessary now
    // if (!(port_->is_open())) {
    //     std::cerr
    //         << "[Error] Serial port not open. Communication will not start."
    //         << std::endl;
    //     return;
    // }

    this->set_channel(channel);
    this->set_level(level);

    // if (diagnostic) {
    //     this->set_diagnostic_mode();
    // } else {
    //     this->reset_diagnostic_mode();
    // }

    // start waiting for data
    //this->start_async_read();

    std::cout << "[INFO] Modem initialized on device " << this->device_
              << ", channel " << channel << ", level " << level
              << ", diagnostic mode = " << std::boolalpha << diagnostic
              << std::endl;
}

void AcousticModemDriver::open(const std::string& device, int& baudrate){
    m_recv_buffer.resize(m_recv_buffer_size);
    m_serial_port.open(device);
    m_serial_port.set_option(asio::serial_port_base::baud_rate(baudrate));
    m_serial_port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    m_serial_port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    m_serial_port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    start_async_read();
    io_thread_ = std::thread([this] { io_.run(); });
}

AcousticModemDriver::~AcousticModemDriver() {
    asio::error_code error;
    m_serial_port.close(error);
}


std::string AcousticModemDriver::make_handshake(MsgType t) {
    const uint16_t id   = (msg_id++ & 0x03FF);    // 10 bits
    const uint16_t type = (uint16_t(t) & 0x0003); // 2 bits

    // build the 16-bit word
    uint16_t w =(uint16_t(HANDSHAKE_SYNC) << 12) |
        (type << 10) |
        id;
    // convert to 2-byte string (little-endian)
    std::string s(2, '\0');
    s[0] = static_cast<char>(w & 0xFF);        // low byte
    s[1] = static_cast<char>((w >> 8) & 0xFF); // high byte

    return s;
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

// void AcousticModemDriver::float_to_word(float v, uint16_t &w0, uint16_t &w1){
//     uint8_t b[4];
//     std::memcpy(b,&v,4);

//     w0 = uint16_t(b[0]) | (uint16_t(b[1]) << 8); // low part of w1 = b[0], high part=b[1]
//     w1 = uint16_t(b[2]) | (uint16_t(b[3]) << 8);
// }
void AcousticModemDriver::float_to_word(float v, std::string &s0,std::string &s1){
    uint8_t b[4];
    std::memcpy(b, &v, 4);
    // Each string contains exactly 2 bytes (16 bits)
    // TODO: change type
    s0.assign(reinterpret_cast<const char*>(&b[0]), 2);
    s1.assign(reinterpret_cast<const char*>(&b[2]), 2);
}

float AcousticModemDriver::word_to_float(uint16_t w0, uint16_t w1){
    uint8_t b[4];
    b[0]=uint8_t(w0 & 0xFF);
    b[1]=uint8_t((w0 >> 8) & 0xFF);
    b[2]=uint8_t(w1 & 0xFF);
    b[3]=uint8_t((w1 >> 8) & 0xFF);
    float v;
    std::memcpy(&v, b, 4);
    return v;
}

// don't use it anymore
// void AcousticModemDriver::send_word(uint16_t w){
//     std::string s(2, '\0'); // string of 2 bytes
//     s[0]=static_cast<char>(w & 0xFF);
//     s[1]=static_cast<char>((w>>8) & 0xFF);
//     send_two_bytes(s);
// }

void AcousticModemDriver::send_message(MsgType type, const float* data){
    const size_t n= floats_for_type(type);
    // TODO: implement for fourth type of data or default case (return;)
    if(n==0 || data==nullptr){
        return;
    }
    send_two_bytes(make_handshake(type)); // send the first packet of 16 bit containing [SYNC(4) | TYPE(2) | MSG_ID(10)]
    for(int i=0;i<n;++i){
        std::string w0;
        std::string w1;
        float_to_word(data[i], w0, w1);
        send_two_bytes(w0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // we could implement it similar to send_msg
        send_two_bytes(w1);
    }
}

void AcousticModemDriver::rx_reset() {
    rx.rx_receiving= false;
    rx.rx_expected_words= 0;
    rx.rx_received_words = 0;
}

void AcousticModemDriver::rx_start_handshake(uint16_t hs_word){
    rx.rx_type=type(hs_word);
    rx.rx_msg_id=id(hs_word);
    rx.rx_expected_words = 2*(floats_for_type(rx.rx_type));
    rx.rx_received_words = 0;
    if( rx.rx_expected_words==0 ||  rx.rx_expected_words>RX_MAX_WORDS){
        rx.rx_receiving=false;
        return;
    }
    rx.rx_receiving=true;
}

bool AcousticModemDriver::rx_rebuild_word(uint16_t w, MsgType &out_type, uint16_t &out_msg_id,float *out_floats, uint8_t &inout_capacity, std::chrono::milliseconds timeout){
    auto now = std::chrono::steady_clock::now();
    
    // timeout for incomplete message (the timeout is to be defined and if it is needed)
    if (rx.rx_receiving && (now - rx.rx_last_rx > timeout)) {
        rx_reset();
    }
    rx.rx_last_rx= now;

    if(is_handshake(w)){
        // if the word is a handshake we start receiveing from the start
        rx_start_handshake(w);
        return false;
    }
    if(!rx.rx_receiving){
        return false;
    }
    if(rx.rx_received_words>rx.rx_expected_words){
        rx_reset();
        return false;
    }
    rx.rx_words[rx.rx_received_words++]=w;
    const size_t n_floats=floats_for_type(rx.rx_type);
    if(rx.rx_received_words==rx.rx_expected_words){
        // maybe add check for floats capacity but i don't think it's necessary

        for(size_t i=0; i<n_floats;i++){
            out_floats[i]=word_to_float(rx.rx_words[2*i],rx.rx_words[2*i+1]);
        }
        out_type = rx.rx_type;
        out_msg_id = rx.rx_msg_id;
        inout_capacity = n_floats; // output number of floats

        rx_reset();
        return true;
    }
    return false;
}
// it's a vector of uint8_t but it will be converted when used in ros2
bool AcousticModemDriver::try_pop_decoded(DecodedMessage& msg){
    std::lock_guard<std::mutex> lock(queue_mutex);
    if(decoded_queue.empty()){
        return false;
    }
    msg = decoded_queue.front();
    decoded_queue.pop(); 
    return true;
}
// send_data(char)
size_t AcousticModemDriver::send_data(char data) {
    // send data using serial driver's send(msg)
    std::vector<uint8_t> msg = {static_cast<uint8_t>(data)};
    size_t bytes = m_serial_port.write_some(asio::buffer(msg.data(), msg.size()));
    return bytes;
}

size_t AcousticModemDriver::send_two_bytes(std::string data) {
    // only send 2 bytes waiting 1 second after sending them
    if (data.length() != 2) {
        return 0;
    } else {
        std::vector<uint8_t> buff(data.begin(), data.end());
        // might use write() instead write_some()
        size_t bytes =m_serial_port.write_some(asio::buffer(buff.data(), 2));
        // 10bps
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return bytes;
    }
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
            this->send_data((char)channel);
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

    this->send_data((char)(level));
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

// to set diagnostic mode
bool AcousticModemDriver::set_parrot_mode() {
    this->send_data('p');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('p');
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

// std::optional<DiagnosticData> AcousticModemDriver::request_report(
//     float overall_timeout,
//     std::optional<std::string> filename) {
//     this->get_report();

//     std::optional<std::vector<uint8_t>> packet = this->read_packet(); // è read packet non decode
//     if (!(packet.has_value())) {
//         std::cout << "Packet is empty" << std::endl;
//         return std::nullopt;
//     }
//     // std::vector<uint8_t>
//     // packet_cast=static_cast<std::vector<uint8_t>>(*packet);
//     std::cout << "Returning packet of length: "
//               << std::string((*packet).begin(), (*packet).end()).length()
//               << std::endl;

//     std::optional<DiagnosticData> report = this->decode_packet(*packet);
//     if (!(report.has_value())) {
//         std::cout << "Failed to decode the packet." << std::endl;
//         return std::nullopt;
//     }
//     // DiagnosticData report_cast=static_cast<DiagnosticData>(*report);
//     this->update_state_from_report(*report);

//     // TODO? implement saving report in json file
//     if (filename.has_value()) {
//         // TODO
//     }

//     return *report;
// }

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
    m_func=[this](std::vector<uint8_t>& buffer, const size_t& bytes_transferred) {
            std::vector<uint8_t> data(buffer.begin(),
                                      buffer.begin() + bytes_transferred);
            this->read_callback(data);
        };
    m_serial_port.async_read_some(
        asio::buffer(m_recv_buffer),
        [this](std::error_code error, size_t bytes_transferred)
        {
        async_receive_handler(error, bytes_transferred);
        });
    
}

void AcousticModemDriver::async_receive_handler(const asio::error_code & error,size_t bytes_transferred) {
    if (error) {
        this->close();
        return;
    }
    if (bytes_transferred > 0 && m_func) {
        m_func(m_recv_buffer, bytes_transferred);
        m_serial_port.async_read_some(
            asio::buffer(m_recv_buffer),
            [this](std::error_code error, size_t bytes_transferred)
            {
                async_receive_handler(error, bytes_transferred);
            });
    }
}


/**
 * mutex probably needed because of async_receive
 * asio lib creates a new thread, if we access that both when some data arrive
 * (push) and when we need to extract (pop)
 */
void AcousticModemDriver::read_callback(std::vector<uint8_t>& data) {
    std::cout << "[DEBUG] read_callback got " << data.size() << " bytes\n";
    //std::lock_guard<std::mutex> lock(queue_mutex);

    if (data.size() < 2) return; // in this case we lose one byte data TODO: fix
    uint16_t word = (static_cast<uint16_t>(data[1]) << 8) | data[0];
    DecodedMessage msg_decoded{};
    bool complete=rx_rebuild_word(word,msg_decoded.type,msg_decoded.msg_id,msg_decoded.floats,msg_decoded.n_floats,std::chrono::milliseconds(200));
    if (complete) {
        // we use the mutex because the decoded queue will be used by ros2 layer to publish in correct topic
        std::lock_guard<std::mutex> lock(queue_mutex);
        decoded_queue.push(msg_decoded);
    }
}

/**
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
*/
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
    asio::error_code error;
    m_serial_port.close(error);
}
