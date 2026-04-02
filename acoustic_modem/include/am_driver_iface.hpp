#ifndef AM_DRIVER_IFACE_HPP
#define AM_DRIVER_IFACE_HPP

#include "am_types.hpp"
#include <cstdint>
#include <optional>
#include <string>

enum class MsgType : uint8_t;
enum class PersistentCmd : uint16_t;

struct DecodedMessage;
struct Ack;

class IAcousticModemDriver {
public:
    virtual ~IAcousticModemDriver() = default;

    virtual uint16_t reserve_msg_id() = 0;

    virtual std::string make_ack(MsgType t, uint16_t ack_id) = 0;
    virtual std::string make_persistent_cmd(PersistentCmd cmd) = 0;

    virtual size_t send_two_bytes(std::string data) = 0;
    virtual size_t send_message(MsgType type, uint16_t id, const float* data) = 0;

    virtual bool try_pop_decoded(DecodedMessage& msg) = 0;
    virtual bool try_pop_ack(Ack& ack) = 0;
    virtual bool consume_persistent(PersistentCmd& cmd) = 0;
};

#endif