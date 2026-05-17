#ifndef CAN_REGISTRY_HPP_
#define CAN_REGISTRY_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>

struct CanMessageDef {
    uint32_t id;
    const char* name;
    std::function<std::string(const uint8_t* data, size_t len)> decode;
};

class CanRegistry {
   public:
    void add(const CanMessageDef& def);
    const CanMessageDef* find(uint32_t id) const;

   private:
    std::unordered_map<uint32_t, CanMessageDef> map_;
};

#endif  // !CAN_REGISTRY_HPP_
