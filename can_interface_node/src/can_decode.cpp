#include <linux/can.h>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "can_decode.hpp"
#include "can_interface.hpp"
#include "can_registry.hpp"

std::string decode_alert_ssa(const uint8_t* data, size_t len) {
    if (len < 8) {
        return "invalid length";
    }

    uint16_t alarm =
        (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[0]);

    uint16_t ssa =
        (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[2]);

    uint16_t ssb =
        (static_cast<uint16_t>(data[5]) << 8) | static_cast<uint16_t>(data[4]);

    uint16_t ssc =
        (static_cast<uint16_t>(data[7]) << 8) | static_cast<uint16_t>(data[6]);

    char buffer[128];
    std::snprintf(
        buffer, sizeof(buffer), "alert=0x%04X,ssa=0x%04X,ssb=0x%04X,ssc=0x%04X",
        static_cast<unsigned int>(alarm), static_cast<unsigned int>(ssa),
        static_cast<unsigned int>(ssb), static_cast<unsigned int>(ssc));

    return std::string(buffer);
}

std::string decode_alert_pfa_1(const uint8_t* data, size_t len) {
    if (len < 8) {
        return "invalid length";
    }

    uint16_t pfa =
        (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[0]);

    uint16_t pfb =
        (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[2]);

    uint16_t pfc =
        (static_cast<uint16_t>(data[5]) << 8) | static_cast<uint16_t>(data[4]);

    uint16_t pfd =
        (static_cast<uint16_t>(data[7]) << 8) | static_cast<uint16_t>(data[6]);

    char buffer[128];
    std::snprintf(
        buffer, sizeof(buffer), "pfa=0x%04X,pfb=0x%04X,pfc=0x%04X,pfd=0x%04X",
        static_cast<unsigned int>(pfa), static_cast<unsigned int>(pfb),
        static_cast<unsigned int>(pfc), static_cast<unsigned int>(pfd));

    return std::string(buffer);
}

std::string decode_alert_pfa_2(const uint8_t* data, size_t len) {
    if (len < 2) {
        return "invalid length";
    }

    uint16_t fet =
        (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[0]);

    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "fet=0x%04X",
                  static_cast<unsigned int>(fet));

    return std::string(buffer);
}

std::string decode_current(const uint8_t* data, size_t len) {
    if (len < 6) {
        return "invalid length";
    }

    // --- Decode 16-bit current ---
    uint16_t raw_current =
        (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[0]);

    int16_t current = static_cast<int16_t>(raw_current);

    // --- Decode 32-bit raw CC2 counts ---
    uint32_t raw_counts = (static_cast<uint32_t>(data[5]) << 24) |
                          (static_cast<uint32_t>(data[4]) << 16) |
                          (static_cast<uint32_t>(data[3]) << 8) |
                          static_cast<uint32_t>(data[2]);

    int32_t current_counts = static_cast<int32_t>(raw_counts);

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "current=%d mA, counts=%ld",
                  static_cast<int>(current), static_cast<long>(current_counts));

    return std::string(buffer);
}

std::string decode_temp(const uint8_t* data, size_t len) {
    if (len < 6) {
        return "invalid length";
    }

    int16_t t1 = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) |
                                      static_cast<uint16_t>(data[0]));

    int16_t t2 = static_cast<int16_t>((static_cast<uint16_t>(data[3]) << 8) |
                                      static_cast<uint16_t>(data[2]));

    int16_t t3 = static_cast<int16_t>((static_cast<uint16_t>(data[5]) << 8) |
                                      static_cast<uint16_t>(data[4]));

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "t1=%d dC,t2=%d dC,t3=%d dC",
                  static_cast<int>(t1), static_cast<int>(t2),
                  static_cast<int>(t3));

    return std::string(buffer);
}

std::string decode_voltage(const uint8_t* data, size_t len) {
    if (len < 2 * CELLS_COUNT) {
        return "invalid length";
    }

    char buffer[256];
    int offset = 0;

    for (size_t i = 0; i < CELLS_COUNT; ++i) {
        uint16_t cell_mV = (static_cast<uint16_t>(data[2 * i + 1]) << 8) |
                           static_cast<uint16_t>(data[2 * i]);

        offset += std::snprintf(buffer + offset, sizeof(buffer) - offset,
                                "cell%zu=%u mV%s", i + 1,
                                static_cast<unsigned int>(cell_mV),
                                (i < CELLS_COUNT - 1) ? ", " : "");
    }

    return std::string(buffer);
}

std::string decode_pressure_sample(const uint8_t* data, size_t len) {
    if (len < sizeof(double)) {
        return "invalid length";
    }

    double pressure_hpa = 0.0;
    std::memcpy(&pressure_hpa, data, sizeof(double));

    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "P=%.6f hPa", pressure_hpa);
    return std::string(buffer);
}

std::string decode_leakage_alarm(const uint8_t* data, size_t len) {
    (void)data;
    (void)len;
    return "LEAKAGE ALARM";
}
