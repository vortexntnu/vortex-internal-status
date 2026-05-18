#ifndef CAN_REGISTRY_HPP_
#define CAN_REGISTRY_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <linux/can.h>

struct CanMessageDef {
    uint32_t id;
    const char* name;
    std::function<void(const canfd_frame& frame)> handle;
};

class CanRegistry {
public:
    void add(const CanMessageDef& def);
    const CanMessageDef* find(uint32_t id) const;

private:
    std::unordered_map<uint32_t, CanMessageDef> map_;
};

#endif  // !CAN_REGISTRY_HPP_
