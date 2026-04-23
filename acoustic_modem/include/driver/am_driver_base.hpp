#ifndef AM_DRIVER_BASE_HPP
#define AM_DRIVER_BASE_HPP

#include "driver/am_driver_iface.hpp"
#include "types/am_types.hpp"
#include <chrono>
#include <fstream>  
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>
#include<utility>
#include <asio.hpp>
#include <functional>
#include <thread>


class AcousticModemDriverBase : public IAcousticModemDriver {
public:
    explicit AcousticModemDriverBase(int channel, int level, bool diagnostic,float timeout);

    virtual ~AcousticModemDriverBase() = default;

    uint16_t reserve_msg_id() override;

    std::string make_tdma_sync() override;
    std::string make_ack(MsgType t, uint16_t ack_id) override;
    std::string make_persistent_cmd(PersistentCmd cmd) override;
    
    size_t send_two_bytes(std::string data) override;
    size_t send_message(MsgType type, uint16_t id, const float* data) override;

    bool try_tdma_sync_event(std::chrono::steady_clock::time_point& rx_time) override;
    bool try_pop_decoded(DecodedMessage& msg) override;
    bool try_pop_ack(Ack& ack) override;
    bool consume_persistent(PersistentCmd& cmd) override;
    bool is_tdma_sync(uint16_t bytes) const override;

    bool set_channel(int channel);
    bool set_level(int level);
    bool set_diagnostic_mode();
    bool set_parrot_mode();
    bool reset_diagnostic_mode();
    
protected:
    virtual size_t write_raw(const uint8_t* data,size_t) =0;      
    
    size_t send_data(char data);
    void read_callback(std::vector<uint8_t>& data);

    bool is_persistent(uint16_t w) const;
    bool is_ack(uint16_t w) const;
    bool is_handshake(uint16_t packet) const;

    std::string make_handshake(MsgType type, uint16_t id);
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
    // SYNC_PREFIXES
    static constexpr uint8_t HANDSHAKE_SYNC = 0xA; //1010
    static constexpr uint8_t ACK_SYNC= 0xB;  //1011
    static constexpr uint8_t PERSISTENT_SYNC= 0xC;  //1100
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
    
    // ACK
    std::queue<Ack> ack_queue;
    std::mutex ack_mutex;
    
    // TDMA_SYNC
    std::mutex tdma_sync_;
    bool tdma_sync_received_=false;
    std::chrono::steady_clock::time_point last_tdma_sync_rx_;

    // PERSISTENT
    std::optional<PersistentCmd> last_persistent_cmd_;
    std::mutex persistent_mutex_;
    bool new_persistent_available_ = false;    
    
    uint16_t msg_id=0;
    int channel_;
    int level_;
    bool diagnostic_;
};

#endif