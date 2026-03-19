#include "am_driver.hpp"
#include <cstdint>

/**
 * Idea: use hybrid TDMA, to try to avoid collisions
 * 
 */

/*
==================== HYBRID TDMA (2 NODES) ====================

Setup:
- 2 nodes, TDMA
- Slot = 25 s
- Guard = 1 s
- Bitrate ~10 bps (very low)

Normal behavior:
- Each slot has a PRIMARY owner
- Primary should transmit immediately if it has data

Hybrid extension:
- If primary is silent, the other node (BACKUP) can reuse the slot

Rules:
1. First 1 second (guard time):
   - Only primary can transmit
   - Backup must stay silent

2. If no activity during guard:
   → slot is considered IDLE

3. Backup can transmit ONLY:
   - 1 packet per slot
   - Priority: ACK > DATA
   - DATA allowed: Type_1 with max 2 floats

Idle detection:
- No RX activity / no packet detected during guard
- No carrier sensing (protocol-level only)

Goal:
- Reuse empty slots safely
- Avoid collisions
- Keep behavior simple and predictable

===============================================================
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

    // Functions usable for hybrid tdma:
    std::chrono::milliseconds time_since_slot_start(std::chrono::steady_clock::time_point now) const;

    bool is_my_slot(std::chrono::steady_clock::time_point now) const;

    bool in_guard_time(std::chrono::steady_clock::time_point now) const;

    private:
    TDMAConfig cfg;

};