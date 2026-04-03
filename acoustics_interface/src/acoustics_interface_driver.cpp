#include "acoustics_interface_driver.hpp"

AcousticsInterfaceDriver::AcousticsInterfaceDriver() {}

can_status AcousticsInterfaceDriver::init_can() {
    if (can_.init("can0") != can_status::OK) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    return can_.set_filter(ACOUSTICS_CAN_ID);
}

bool AcousticsInterfaceDriver::decode_frame(const struct canfd_frame& frame, AcousticsData& data) {
    if ((frame.can_id & CAN_SFF_MASK) != ACOUSTICS_CAN_ID) {
        return false;
    }

    if (frame.len != 16) {
        return false;
    }

    std::memcpy(&data.x,      &frame.data[0],  4);
    std::memcpy(&data.y,      &frame.data[4],  4);
    std::memcpy(&data.z,      &frame.data[8],  4);
    std::memcpy(&data.weight, &frame.data[12], 4);

    return true;
}

can_status AcousticsInterfaceDriver::read_acoustics(AcousticsData& data) {
    struct canfd_frame frame{};

    can_status status = can_.receive(frame);
    if (status != can_status::OK) {
        return status;
    }

    if (!decode_frame(frame, data)) {
        return can_status::ERR_RECEIVE;
    }

    return can_status::OK;
}
