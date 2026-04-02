#ifndef AM_TYPES_HPP
#define AM_TYPES_HPP

#include<cstdint>

enum class PersistentCmd : uint16_t {
    Surface = 1,
    Abort   = 2,
    Stop    = 3
};

enum class MsgType : uint8_t{
    Type_def=0,
    Type_1 = 1,
    Type_2 = 2,
    Type_3 = 3,
    Type_4 = 4,
};

struct DecodedMessage {
    MsgType type;
    uint16_t msg_id;
    float floats[10];
    uint8_t n_floats;
};

// ACK
struct Ack {
    MsgType type;
    uint16_t msg_id;
};

#endif