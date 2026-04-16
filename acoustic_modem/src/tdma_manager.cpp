#include "tdma_manager.hpp"

std::uint8_t TDMAManager::current_slot(std::chrono::steady_clock::time_point now) const{
    auto cycle=cfg.num_slots*cfg.slot_duration;
    auto elapsed=now-cfg.t0;
    auto time_cycle=elapsed%cycle;
    uint8_t slot_i= time_cycle/cfg.slot_duration;
    return slot_i;
}

bool TDMAManager::tx_allowed(std::chrono::steady_clock::time_point now) const{
    if (!synced_) {
        return false;
    }

    if (now < cfg.t0) {
        return false;
    }

    if(current_slot(now)!=cfg.my_slot){
        return false;
    }
    auto cycle=cfg.num_slots*cfg.slot_duration;
    auto elapsed=now-cfg.t0;
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

std::chrono::milliseconds TDMAManager::time_since_slot_start(std::chrono::steady_clock::time_point now) const{
    auto elapsed=now-cfg.t0;
    auto time_in_slot = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed % cfg.slot_duration);
    return time_in_slot;
}

bool TDMAManager::is_my_slot(std::chrono::steady_clock::time_point now) const{
    if (!synced_ || now < cfg.t0) {
        return false;
    }
    return (cfg.my_slot==current_slot(now));
}

bool TDMAManager::in_guard_time(std::chrono::steady_clock::time_point now) const{
    if (!synced_ || now < cfg.t0) {
        return false;
    }
    return time_since_slot_start(now)<cfg.guard;
}

void TDMAManager::sync_rx(std::chrono::steady_clock::time_point rx_time){
    cfg.t0=rx_time+cfg.sync_delay;
    synced_=true;
}

void TDMAManager::sync_tx(std::chrono::steady_clock::time_point tx_time){
    cfg.t0=tx_time+cfg.sync_delay+cfg.estimated_prop_delay+cfg.sync_tx_duration;
    synced_=true;
}

bool TDMAManager::is_synced() const{
    return synced_;
}

void TDMAManager::clear_sync(){
    synced_=false;
}

void TDMAManager::set_estimated_prop_delay(std::chrono::milliseconds d) {
    cfg.estimated_prop_delay = d;
}