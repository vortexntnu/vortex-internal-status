#include "can_registry.hpp"


void CanRegistry::add(const CanMessageDef& def) {
    map_[def.id] = def;
}

const CanMessageDef* CanRegistry::find(uint32_t id) const {
    auto it = map_.find(id);
    if (it != map_.end()) {
        return &it->second;
    }
    return nullptr;
}
