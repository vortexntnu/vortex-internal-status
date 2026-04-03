#include "am_driver_split.hpp"

AcousticModemDriverSplit::AcousticModemDriverSplit(const std::string& tx_device,
                                                   const std::string& rx_device,
                                                   int baudrate,
                                                   int channel,
                                                   int level,
                                                   bool diagnostic,
                                                   float timeout)
    : io_(),
      tx_serial_port_(io_),
      rx_serial_port_(io_),
      tx_device_(tx_device),
      rx_device_(rx_device),
      channel_(channel),
      level_(level),
      diagnostic_(diagnostic)
{
    msg_id = 0;

    open(tx_device, rx_device, baudrate);
    set_channel(channel);
    set_level(level);

    std::cout << "[INFO] Split modem initialized TX=" << tx_device_
              << " RX=" << rx_device_
              << ", channel " << channel_
              << ", level " << level_
              << ", diagnostic mode = " << std::boolalpha << diagnostic_
              << std::endl;
}

void AcousticModemDriverSplit::open(const std::string& tx_device,
                                    const std::string& rx_device,
                                    int& baudrate)
{
    m_recv_buffer.resize(m_recv_buffer_size);

    tx_serial_port_.open(tx_device);
    rx_serial_port_.open(rx_device);

    tx_serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
    tx_serial_port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    tx_serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    tx_serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    rx_serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
    rx_serial_port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    rx_serial_port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    rx_serial_port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    start_async_read();
    io_thread_ = std::thread([this] { io_.run(); });
}

AcousticModemDriverSplit::~AcousticModemDriverSplit() {
    asio::error_code error;
    tx_serial_port_.close(error);
    rx_serial_port_.close(error);
}
void AcousticModemDriverSplit::close()
{
    asio::error_code error;
    tx_serial_port_.close(error);
    rx_serial_port_.close(error);
}

// send_data(char)
size_t AcousticModemDriverSplit::send_data(char data) {
    // send data using serial driver's send(msg)
    std::vector<uint8_t> msg = {static_cast<uint8_t>(data)};
    size_t bytes = tx_serial_port_.write_some(asio::buffer(msg.data(), msg.size()));
    return bytes;
}

size_t AcousticModemDriverSplit::send_two_bytes(std::string data) {
    // only send 2 bytes waiting 1 second after sending them
    if (data.length() != 2) {
        return 0;
    } else {
        std::vector<uint8_t> buff(data.begin(), data.end());
        // might use write() instead write_some()
        size_t bytes =tx_serial_port_.write_some(asio::buffer(buff.data(), 2));
        // 10bps
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return bytes;
    }
}

void AcousticModemDriverSplit::start_async_read() {
    m_func=[this](std::vector<uint8_t>& buffer, const size_t& bytes_transferred) {
            std::vector<uint8_t> data(buffer.begin(),
                                      buffer.begin() + bytes_transferred);
            this->read_callback(data);
        };
    rx_serial_port_.async_read_some(
        asio::buffer(m_recv_buffer),
        [this](std::error_code error, size_t bytes_transferred)
        {
        async_receive_handler(error, bytes_transferred);
        });
    
}

void AcousticModemDriverSplit::async_receive_handler(const asio::error_code & error,size_t bytes_transferred) {
    if (error) {
        this->close();
        return;
    }
    if (bytes_transferred > 0 && m_func) {
        m_func(m_recv_buffer, bytes_transferred);
        rx_serial_port_.async_read_some(
            asio::buffer(m_recv_buffer),
            [this](std::error_code error, size_t bytes_transferred)
            {
                async_receive_handler(error, bytes_transferred);
            });
    }
}

// methods copied by AcousticModemDriverSplit TODO: use helper file probalby better

bool AcousticModemDriverSplit::is_persistent(uint16_t w) const {
    return (((w >> 12) & 0xF) == PERSISTENT_SYNC);
}

std::string AcousticModemDriverSplit::make_persistent_cmd(PersistentCmd cmd){

    uint16_t w = (uint16_t(PERSISTENT_SYNC) << 12) | (uint16_t(cmd) & 0X0FFF);

    std::string s(2, '\0');
    s[0] = static_cast<char>(w & 0xFF);
    s[1] = static_cast<char>((w >> 8) & 0xFF);
    return s;
}

bool AcousticModemDriverSplit::consume_persistent(PersistentCmd& cmd){
    std::lock_guard<std::mutex> lock(persistent_mutex_);
    if (!new_persistent_available_ || !last_persistent_cmd_.has_value()) {
        return false;
    }
    cmd = *last_persistent_cmd_;
    new_persistent_available_ = false;
    return true;
}

uint16_t AcousticModemDriverSplit::reserve_msg_id() {
    uint16_t id = msg_id & 0x03FF;
    msg_id = (msg_id + 1) & 0x03FF;
    return id;
}

std::string AcousticModemDriverSplit::make_ack(MsgType t, uint16_t ack_id) {
    const uint16_t id   = (ack_id & 0x03FF);
    const uint16_t type = (uint16_t(t) & 0x0003);

    uint16_t w = (uint16_t(ACK_SYNC) << 12) | (type << 10) | id;

    std::string s(2, '\0');
    s[0] = static_cast<char>(w & 0xFF);
    s[1] = static_cast<char>((w >> 8) & 0xFF);
    return s;
}
bool AcousticModemDriverSplit::is_ack(uint16_t w) const {
    return (((w >> 12) & 0xF) == ACK_SYNC);
}


std::string AcousticModemDriverSplit::make_handshake(MsgType t, uint16_t id) {
    id &= 0x03FF; // 10 bits
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

bool AcousticModemDriverSplit::is_handshake(uint16_t packet){
    return ((packet>>12 & 0xF)==HANDSHAKE_SYNC);
}
MsgType AcousticModemDriverSplit::type(uint16_t packet){
    return MsgType((packet>>10 & 0x3));
}
uint16_t AcousticModemDriverSplit::id(uint16_t packet){
    return ((packet & 0x3FF));
}

size_t AcousticModemDriverSplit::floats_for_type(MsgType t) {
  switch (t) {
    case MsgType::Type_1:  return 2;
    case MsgType::Type_2:  return 4;
    case MsgType::Type_3:  return 5;
    default:               return 0;
  }
}
void AcousticModemDriverSplit::float_to_word(float v, std::string &s0,std::string &s1){
    uint8_t b[4];
    std::memcpy(b, &v, 4);
    // Each string contains exactly 2 bytes (16 bits)
    // TODO: change type
    s0.assign(reinterpret_cast<const char*>(&b[0]), 2);
    s1.assign(reinterpret_cast<const char*>(&b[2]), 2);
}

float AcousticModemDriverSplit::word_to_float(uint16_t w0, uint16_t w1){
    uint8_t b[4];
    b[0]=uint8_t(w0 & 0xFF);
    b[1]=uint8_t((w0 >> 8) & 0xFF);
    b[2]=uint8_t(w1 & 0xFF);
    b[3]=uint8_t((w1 >> 8) & 0xFF);
    float v;
    std::memcpy(&v, b, 4);
    return v;
}

size_t AcousticModemDriverSplit::send_message(MsgType type,uint16_t id, const float* data){
    const size_t n= floats_for_type(type);
    size_t a=0;
    // TODO: implement for fourth type of data or default case (return;)
    if(n==0 || data==nullptr){
        return a;
    }
    send_two_bytes(make_handshake(type, id)); // send the first packet of 16 bit containing [SYNC(4) | TYPE(2) | MSG_ID(10)]
    for(int i=0;i<n;++i){
        std::string w0;
        std::string w1;
        float_to_word(data[i], w0, w1);
        a+=send_two_bytes(w0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // we could implement it similar to send_msg
        a+=send_two_bytes(w1);
    }
    return a;
}

void AcousticModemDriverSplit::rx_reset() {
    rx.rx_receiving= false;
    rx.rx_expected_words= 0;
    rx.rx_received_words = 0;
}

void AcousticModemDriverSplit::rx_start_handshake(uint16_t hs_word){
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

bool AcousticModemDriverSplit::rx_rebuild_word(uint16_t w, MsgType &out_type, uint16_t &out_msg_id,float *out_floats, uint8_t &inout_capacity, std::chrono::milliseconds timeout){
    auto now = std::chrono::steady_clock::now();
    // timeout for incomplete message (the timeout is to be defined and if it is needed)
    if (rx.rx_receiving && (now - rx.rx_last_rx > timeout)) {
        rx_reset();

    }
    rx.rx_last_rx= now;


    if(!rx.rx_receiving){
        if(is_handshake(w)){
        // if the word is a handshake we start receiveing from the start
            rx_start_handshake(w);
        }
        return false;
    }
    if(rx.rx_received_words>=rx.rx_expected_words){
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
bool AcousticModemDriverSplit::try_pop_decoded(DecodedMessage& msg){
    std::lock_guard<std::mutex> lock(decoded_mutex);
    if(decoded_queue.empty()){
        return false;
    }
    msg = decoded_queue.front();
    decoded_queue.pop(); 
    return true;
}
bool AcousticModemDriverSplit::try_pop_ack(Ack &ack){
    std::lock_guard<std::mutex> lock(ack_mutex);
    if(ack_queue.empty()){
        return false;
    }
    ack=ack_queue.front();
    ack_queue.pop();
    return true;
}
// to set channel of communication
bool AcousticModemDriverSplit::set_channel(int channel) {
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
bool AcousticModemDriverSplit::set_level(int level) {
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
bool AcousticModemDriverSplit::set_diagnostic_mode() {
    this->send_data('d');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('d');
    diagnostic_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
}

// to set diagnostic mode
bool AcousticModemDriverSplit::set_parrot_mode() {
    this->send_data('p');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('p');
    diagnostic_ = true;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
}

bool AcousticModemDriverSplit::reset_diagnostic_mode() {
    this->send_data('t');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->send_data('t');
    diagnostic_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return false;
}

void AcousticModemDriverSplit::read_callback(std::vector<uint8_t>& data) {
    // if (data.size() < 2) return; // TODO: the serial channel is byte stream, POSSIBLE BUG      
    // uint16_t word = (static_cast<uint16_t>(data[1]) << 8) | data[0];

    // need to use the tdmalink functions to make it work

    // Important:
    // ACK words are only detected when the receiver is idle.
    // If we are currently reconstructing a message, all incoming words are treated
    // as payload to avoid corrupting the message reconstruction.
    // same process for Persistent mode, 
    rx_byte_buffer.insert(rx_byte_buffer.end(),data.begin(),data.end());
    while(rx_byte_buffer.size()>=2){
        uint16_t word = (static_cast<uint16_t>(rx_byte_buffer[1]) << 8) | rx_byte_buffer[0];
        rx_byte_buffer.erase(rx_byte_buffer.begin(),rx_byte_buffer.begin()+2);
        if(!rx.rx_receiving){
            if(is_ack(word)){
                Ack a;
                a.msg_id=(word & 0x3FF);
                a.type=MsgType((word >> 10) & 0x3);
                {
                    std::lock_guard<std::mutex> lock(ack_mutex);
                    ack_queue.push(a);
                }
                continue;
            }
            if(is_persistent(word)){
                PersistentCmd cmd=static_cast<PersistentCmd>(word & 0x0FFF);
                {
                    std::lock_guard<std::mutex> lock(persistent_mutex_);
                    if (new_persistent_available_ && last_persistent_cmd_ == cmd) {
                        return;
                    }
                    last_persistent_cmd_ = cmd;
                    new_persistent_available_ = true;
                }
                continue;
            }
        }
        DecodedMessage msg_decoded{};
        bool complete=rx_rebuild_word(word,msg_decoded.type,msg_decoded.msg_id,msg_decoded.floats,msg_decoded.n_floats,std::chrono::milliseconds(1000));
        if (complete) {
            // we use the mutex because the decoded queue will be used by ros2 layer to publish in correct topic
            std::lock_guard<std::mutex> lock(decoded_mutex);
            decoded_queue.push(msg_decoded);
        }
    }
}
