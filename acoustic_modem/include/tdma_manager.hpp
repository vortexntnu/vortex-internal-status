#include "am_driver.hpp"

/**
 * Idea: use hybrid TDMA, to try to avoid collisions
 * 
 */

struct TDMAConfig{
    // should be two with just two nodes
    uint8_t num_slots;
    uint8_t my_slot;
    std::chrono::seconds slot=std::chrono::seconds(25);
    std::chrono::milliseconds guard=std::chrono::milliseconds(1000);
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
};

class TDMAManager{
    public:
    TDMAManager(TDMAConfig config) : cfg(config){}
    
    // who is transmitting?
    uint8_t current_slot(std::chrono::steady_clock::time_point now);

    // am i allowed to transmit?
    bool tx_allowed(std::chrono::steady_clock::time_point now);

    private:
    TDMAConfig cfg;

}