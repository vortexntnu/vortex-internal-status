#include "am_driver.hpp"
#include <cstdint>


struct TDMAConfig{
    std::uint8_t num_slots;
    std::uint8_t my_slot;
    std::chrono::seconds slot_duration=std::chrono::seconds(25);
    std::chrono::milliseconds guard=std::chrono::milliseconds(1000);
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
};

class TDMAManager{
    public:
    TDMAManager(TDMAConfig config) : cfg(config){}
    
    std::uint8_t current_slot(std::chrono::steady_clock::time_point now) const;

    bool tx_allowed(std::chrono::steady_clock::time_point now) const;

    std::chrono::milliseconds time_since_slot_start(std::chrono::steady_clock::time_point now) const;

    bool is_my_slot(std::chrono::steady_clock::time_point now) const;

    bool in_guard_time(std::chrono::steady_clock::time_point now) const;

    private:
    TDMAConfig cfg;

};