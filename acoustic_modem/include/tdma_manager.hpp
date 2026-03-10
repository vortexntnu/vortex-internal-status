#include "am_driver.hpp"
#include <cstdint>

/**
 * Idea: use hybrid TDMA, to try to avoid collisions
 * 
 */

struct TDMAConfig{
    // should be two with just two nodes
    std::uint8_t num_slots;
    std::uint8_t my_slot;
    std::chrono::seconds slot_duration=std::chrono::seconds(25);
    std::chrono::milliseconds guard=std::chrono::milliseconds(1000);
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
};

class TDMAManager{
    public:
    TDMAManager(TDMAConfig config) : cfg(config){}
    
    // who is transmitting?
    std::uint8_t current_slot(std::chrono::steady_clock::time_point now) const;

    // am i allowed to transmit?
    bool tx_allowed(std::chrono::steady_clock::time_point now) const;

    private:
    TDMAConfig cfg;

};