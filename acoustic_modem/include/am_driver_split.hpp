#ifndef AM_DRIVER_SPLIT_HPP
#define AM_DRIVER_SPLIT_HPP

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


class AcousticModemDriverSplit : public IAcousticModemDriver {
public:
    AcousticModemDriverSplit(const std::string& tx_device,
                             const std::string& rx_device,
                             int baudrate,
                             int channel,
                             int level,
                             bool diagnostic,
                             float timeout);

    ~AcousticModemDriverSplit();

    void open(const std::string& tx_device,
              const std::string& rx_device,
              int& baudrate);

    void close();

    uint16_t reserve_msg_id() override;

    std::string make_ack(MsgType t, uint16_t ack_id) override;
    std::string make_persistent_cmd(PersistentCmd cmd) override;
    std::string make_handshake(MsgType type, uint16_t id);

    size_t send_data(char data);
    size_t send_two_bytes(std::string data) override;
    size_t send_message(MsgType type, uint16_t id, const float* data) override;

    bool try_pop_decoded(DecodedMessage& msg) override;
    bool try_pop_ack(Ack& ack) override;
    bool consume_persistent(PersistentCmd& cmd) override;

    bool set_channel(int channel);
    bool set_level(int level);
    bool set_diagnostic_mode();
    bool set_parrot_mode();
    bool reset_diagnostic_mode();

    void start_async_read();
    void async_receive_handler(const asio::error_code& error,
                               size_t bytes_transferred);
    void read_callback(std::vector<uint8_t>& data);

    // copied/reused protocol helpers
    bool is_persistent(uint16_t w) const;
    bool is_ack(uint16_t w) const;
    bool is_handshake(uint16_t packet);

    MsgType type(uint16_t packet);
    uint16_t id(uint16_t packet);

    size_t floats_for_type(MsgType t);
    void float_to_word(float v, std::string& s0, std::string& s1);
    float word_to_float(uint16_t w0, uint16_t w1);

    void rx_reset();
    void rx_start_handshake(uint16_t hs_word);
    bool rx_rebuild_word(uint16_t w, MsgType& out_type, uint16_t& out_msg_id,
                         float* out_floats, uint8_t& inout_capacity,
                         std::chrono::milliseconds timeout);

private:
    asio::io_context io_;
    asio::serial_port tx_serial_port_;
    asio::serial_port rx_serial_port_;
    std::thread io_thread_;

    std::vector<uint8_t> m_recv_buffer;
    const size_t m_recv_buffer_size = 256;

    std::function<void(std::vector<uint8_t>&, const size_t&)> m_func;

    std::string tx_device_;
    std::string rx_device_;

    int channel_;
    int level_;
    bool diagnostic_;
    uint16_t msg_id;

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
    std::vector<uint8_t> rx_byte_buffer;
    RxState rx;
    std::queue<DecodedMessage> decoded_queue;
    std::mutex decoded_mutex;

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
};

#endif