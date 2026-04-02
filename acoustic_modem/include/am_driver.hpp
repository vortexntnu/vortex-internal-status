#ifndef AM_DRIVER_HPP
#define AM_DRIVER_HPP

#include "am_driver_iface.hpp"
#include "am_types.hpp"
#include <chrono>
#include <fstream>  // per scrivere su file
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>
#include <asio.hpp>
#include <functional>
#include <thread>

// enum class PersistentCmd : uint16_t {
//     Surface = 1,
//     Abort   = 2,
//     Stop    = 3
// };

// enum class MsgType : uint8_t{
//     Type_def=0,
//     Type_1 = 1,
//     Type_2 = 2,
//     Type_3 = 3,
//     Type_4 = 4,
// };

struct DiagnosticData {
    uint8_t TR_BLOCK[2];  // 6 bits will be used by the packet header
    uint8_t BER;
    uint8_t SIGNAL_POWER;
    uint8_t NOISE_POWER;
    uint8_t PACKET_VALID[2];
    uint8_t PACKET_INVALID;
    uint8_t GIT_REV;
    uint8_t TIME[3];
    uint8_t CHIP_ID[2];
    uint8_t HW_REV : 2;
    uint8_t CHANNEL : 4;
    uint8_t TB_VALID : 1;
    uint8_t TX_COMPLETE : 1;
    uint8_t DIAGNOSTIC_MODE : 1;
    uint8_t POWER_LEVEL : 2;
};

class AcousticModemDriver : public IAcousticModemDriver {
    public:
    AcousticModemDriver();

    ~AcousticModemDriver();

    AcousticModemDriver(const std::string& device, int baudrate, int channel, int level, bool diagnostic, float timeout);

    // PERSISTENT_MODE
    bool is_persistent(uint16_t w) const;
    std::string make_persistent_cmd(PersistentCmd cmd);
    bool consume_persistent(PersistentCmd& cmd);


    std::string make_ack(MsgType t, uint16_t msg_id);
    bool is_ack(uint16_t w) const;
    uint16_t reserve_msg_id();
    bool try_pop_ack(Ack &ack);

    // NORMAL_MESSAGES
    std::string make_handshake(MsgType type, uint16_t id);
    bool is_handshake(uint16_t packet);
    MsgType type(uint16_t packet);
    uint16_t id(uint16_t packet);
    static size_t floats_for_type(MsgType t);
    static void float_to_word(float v, std::string &s0,std::string &s1);
    static float word_to_float(uint16_t w0, uint16_t w1);
    size_t send_message(MsgType type, uint16_t id, const float* data);
    size_t send_data(char data);
    size_t send_two_bytes(std::string data);

    // RX_LOGIC
    void rx_start_handshake(uint16_t hs_word);
    void rx_reset();
    bool rx_rebuild_word(uint16_t w, MsgType &out_type, uint16_t &out_msg_id, float *out_floats, uint8_t &inout_capacity, std::chrono::milliseconds timeout);
    bool try_pop_decoded(DecodedMessage& msg);
    void read_callback(std::vector<uint8_t>& data);

    // SET_MODES
    bool set_channel(int channel);
    bool set_level(int level);
    bool set_diagnostic_mode();
    bool set_parrot_mode();
    bool reset_diagnostic_mode();
    void update_state_from_report(DiagnosticData report);
    void get_report();
    std::optional<DiagnosticData> decode_packet(std::vector<uint8_t>& packet);

   private:
    // RX
    void start_async_read();
    void async_receive_handler(const asio::error_code & error,size_t bytes_transferred);
    std::function<void (std::vector<uint8_t> &, const size_t &)> m_func;
    std::thread io_thread_;
    static constexpr uint16_t RX_MAX_WORDS = 30;
    struct RxState{
        bool rx_receiving= false;
        MsgType rx_type=MsgType::Type_def;
        uint16_t rx_msg_id= 0;
        uint16_t rx_expected_words = 0;
        uint16_t rx_received_words = 0;
        uint16_t rx_words[RX_MAX_WORDS]{};
        uint8_t rx_n_floats= 0;
        std::chrono::steady_clock::time_point rx_last_rx{};
    };
    RxState rx;
    std::queue<DecodedMessage> decoded_queue;
    std::mutex decoded_mutex;

    void open(const std::string& device, int& baudrate);
    void close();
    static constexpr size_t m_recv_buffer_size{2048};
    std::vector<uint8_t> m_recv_buffer;

    struct DiagnosticPacket {
        uint16_t TR_BLOCK;
        uint8_t BER;
        uint8_t SIGNAL_POWER;
        uint8_t NOISE_POWER;
        uint16_t PACKET_VALID;
        uint8_t PACKET_INVALID;
        uint8_t GIT_REV;
        uint8_t TIME_L;
        uint8_t TIME_M;
        uint8_t TIME_H;
        uint16_t CHIP_ID;
        uint8_t HW_CH_FLAGS;       // 14  (contains HW_REV, CHANNEL, TB_VALID,
                                   // TX_COMPLETE)
        uint8_t MODE_LEVEL_FLAGS;  // 15 (contains DIAGNOSTIC_MODE, LEVEL)
    };

    // SYNC_PREFIXES
    static constexpr uint8_t HANDSHAKE_SYNC = 0xA; //1010
    static constexpr uint8_t ACK_SYNC= 0xB;  //1011
    static constexpr uint8_t PERSISTENT_SYNC= 0xC;  //1100

    // PERSISTENT
    std::optional<PersistentCmd> last_persistent_cmd_;
    std::mutex persistent_mutex_;
    bool new_persistent_available_ = false;

    // ACK
    std::queue<Ack> ack_queue;
    std::mutex ack_mutex;


    uint16_t msg_id;
    asio::io_context io_;
    asio::serial_port m_serial_port;
    std::string device_;
    int channel_;
    int level_;
    bool diagnostic_;
};

#endif
