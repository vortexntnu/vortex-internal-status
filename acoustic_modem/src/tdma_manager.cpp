#include "tdma_manager.hpp"

// return type can be changed
std::uint8_t TDMAManager::current_slot(std::chrono::steady_clock::time_point now) const{
    auto cycle=cfg.num_slots*cfg.slot_duration;
    auto elapsed=now-cfg.t0;
    // how far are we in the cycle
    auto time_cycle=elapsed%cycle;
    uint8_t slot_i= time_cycle/cfg.slot_duration;
    return slot_i;
}

bool TDMAManager::tx_allowed(std::chrono::steady_clock::time_point now) const{
    if(current_slot(now)!=cfg.my_slot){
        return false;
    }
    auto cycle=cfg.num_slots*cfg.slot_duration;
    auto elapsed=now-cfg.t0;
    // how far are we in the cycle
    auto time_cycle=elapsed%cycle;
    uint8_t slot_i= time_cycle/cfg.slot_duration;
    auto offset_in_slot=time_cycle%cfg.slot_duration;

    if(offset_in_slot<cfg.guard){
        return false;
    }
    if(offset_in_slot>(cfg.slot_duration-cfg.guard)){
        return false;
    }
    return true;
}
