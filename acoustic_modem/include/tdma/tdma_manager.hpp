#include "driver/am_driver_single.hpp"
#include <cstdint>


struct TDMAConfig{
    std::uint8_t num_slots;
    std::uint8_t my_slot;
    std::chrono::seconds slot_duration=std::chrono::seconds(25);
    std::chrono::milliseconds guard=std::chrono::milliseconds(1000);
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::chrono::milliseconds sync_delay{std::chrono::seconds{5}};
    std::chrono::milliseconds sync_tx_duration{std::chrono::milliseconds(1700)};
    std::chrono::milliseconds estimated_prop_delay{std::chrono::milliseconds(0)};
};

class TDMAManager{
    public:
    TDMAManager(TDMAConfig config) : cfg(config){}
    
    std::uint8_t current_slot(std::chrono::steady_clock::time_point now) const;

    bool tx_allowed(std::chrono::steady_clock::time_point now) const;

    std::chrono::milliseconds time_since_slot_start(std::chrono::steady_clock::time_point now) const;

    bool is_my_slot(std::chrono::steady_clock::time_point now) const;

    bool in_guard_time(std::chrono::steady_clock::time_point now) const;

    void sync_rx(std::chrono::steady_clock::time_point rx_time);

    void sync_tx(std::chrono::steady_clock::time_point tx_time);

    void set_estimated_prop_delay(std::chrono::milliseconds d);

    bool is_synced() const;
    void clear_sync();

    private:
    TDMAConfig cfg;
    bool synced_{false};
};