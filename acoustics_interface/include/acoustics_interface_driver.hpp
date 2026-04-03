#ifndef ACOUSTCS_INTERFACE_DRIVER_HPP_
#define ACOUSTCS_INTERFACE_DRIVER_HPP_

#include <cstring>
#include "can_interface.hpp"

struct AcousticsData {
    float x;
    float y;
    float z;
    float weight;
};

class AcousticsInterfaceDriver {
public:
    AcousticsInterfaceDriver();

    can_status init_can();
    can_status read_acoustics(AcousticsData& data);
    can_status start_async_read(std::function<void(const AcousticsData&, can_status)> callback);
    void stop_async_read();

private:
    bool decode_frame(const struct canfd_frame& frame, AcousticsData& data);

    can_interface can_;
    static constexpr uint32_t ACOUSTICS_CAN_ID = 0x200;
};

#endif
