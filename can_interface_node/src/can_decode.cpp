#include "can_decode.hpp"

#include <cstring>
#include <cstdio>

namespace {

uint16_t read_u16_le(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1]) << 8;
}

int16_t read_i16_le(const uint8_t* data)
{
    return static_cast<int16_t>(read_u16_le(data));
}

uint32_t read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           static_cast<uint32_t>(data[1]) << 8 |
           static_cast<uint32_t>(data[2]) << 16 |
           static_cast<uint32_t>(data[3]) << 24;
}

int32_t read_i32_le(const uint8_t* data)
{
    return static_cast<int32_t>(read_u32_le(data));
}

}  // namespace

std::optional<BmsAlertSsa> parse_alert_ssa(const uint8_t* data, size_t len)
{
    if (len < 8) {
        return std::nullopt;
    }

    BmsAlertSsa result{};
    result.alarm = read_u16_le(&data[0]);
    result.ssa = read_u16_le(&data[2]);
    result.ssb = read_u16_le(&data[4]);
    result.ssc = read_u16_le(&data[6]);

    return result;
}

std::optional<BmsAlertPfa1> parse_alert_pfa_1(const uint8_t* data, size_t len)
{
    if (len < 8) {
        return std::nullopt;
    }

    BmsAlertPfa1 result{};
    result.pfa = read_u16_le(&data[0]);
    result.pfb = read_u16_le(&data[2]);
    result.pfc = read_u16_le(&data[4]);
    result.pfd = read_u16_le(&data[6]);

    return result;
}

std::optional<BmsAlertPfa2> parse_alert_pfa_2(const uint8_t* data, size_t len)
{
    if (len < 2) {
        return std::nullopt;
    }

    BmsAlertPfa2 result{};
    result.fet = read_u16_le(&data[0]);

    return result;
}


std::optional<BmsCurrent> parse_current(const uint8_t* data, size_t len)
{
    if (len < 6) {
        return std::nullopt;
    }

    BmsCurrent result{};
    result.current_mA = read_i16_le(&data[0]);
    result.current_counts = read_i32_le(&data[2]);

    return result;
}

std::optional<BmsTemperatures> parse_temp(const uint8_t* data, size_t len)
{
    if (len < 6) {
        return std::nullopt;
    }

    BmsTemperatures result{};
    result.temperatures_dC[0] = read_i16_le(&data[0]);
    result.temperatures_dC[1] = read_i16_le(&data[2]);
    result.temperatures_dC[2] = read_i16_le(&data[4]);

    return result;
}

std::optional<BmsCellVoltages> parse_voltage(const uint8_t* data, size_t len)
{
    if (len < 2 * CELLS_COUNT) {
        return std::nullopt;
    }

    BmsCellVoltages result{};

    for (size_t i = 0; i < CELLS_COUNT; ++i) {
        result.cell_voltages_mV[i] = read_u16_le(&data[2 * i]);
    }

    return result;
}


std::optional<PressureSample> parse_pressure_sample(const uint8_t* data, size_t len)
{
    if (len < sizeof(double)) {
        return std::nullopt;
    }

    PressureSample result{};
    std::memcpy(&result.pressure_hPa, data, sizeof(double));

    return result;
}

std::optional<LeakageAlarm> parse_leakage_alarm(const uint8_t* data, size_t len)
{
    (void)data;
    (void)len;

    LeakageAlarm result{};
    result.active = true;

    return result;
}
