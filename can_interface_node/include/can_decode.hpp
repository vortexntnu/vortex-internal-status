#ifndef CAN_DECODE_HPP_
#define CAN_DECODE_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>


#define NUM_ANGLES 2
#define CELLS_COUNT 6
#define CELLS_PAYLOAD_LEN 12
#define CAN_ID_BOTHOFF_CMD 0x200  // EXAMPLE VALUE
#define BOTHOFF_CMD_BYTE 0xA5
#define CAN_TEMP_ID 0x100  // EXAMPLE VALUE


#define CAN_PRESSURE_ID 0x480     // EXAMPLE VALUE
#define CAN_STANDBYMODE_ID 0x103  // EXAMPLE VALUE
#define CAN_RST_MCU 0x104         // EXAMPLE VALUE
// #define CAN_CURRENT_ID 0x105     //EXAMPLE VALUE

#define CAN_OPERATION_MODE_ID 0x50 
#define CAN_PI_STATUS 0x69

#define CAN_ALERT_SSA_ID 0x200u
#define CAN_ALERT_PFA_1_ID 0x201u
#define CAN_ALERT_PFA_2_ID 0x202u
#define CAN_CURRENT_ID 0x203u
#define CAN_VOLTAGE_ID 0x204  // EXAMPLE VALUE



struct BmsAlertSsa {
    uint16_t alarm;
    uint16_t ssa;
    uint16_t ssb;
    uint16_t ssc;
};

struct BmsAlertPfa1 {
    uint16_t pfa;
    uint16_t pfb;
    uint16_t pfc;
    uint16_t pfd;
};

struct BmsAlertPfa2 {
    uint16_t fet;
};

struct BmsCurrent {
    int16_t current_mA;
    int32_t current_counts;
};

struct BmsTemperatures {
    std::array<int16_t, 3> temperatures_dC;
};

struct BmsCellVoltages {
    std::array<uint16_t, CELLS_COUNT> cell_voltages_mV;
};

struct PressureSample {
    float temperature_c;
    float pressure_pa;
};

struct LeakageAlarm {
    bool active;
};

// Typed parsers.
// These are the functions you should use before publishing.
std::optional<BmsAlertSsa> parse_alert_ssa(const uint8_t* data, size_t len);
std::optional<BmsAlertPfa1> parse_alert_pfa_1(const uint8_t* data, size_t len);
std::optional<BmsAlertPfa2> parse_alert_pfa_2(const uint8_t* data, size_t len);
std::optional<BmsCurrent> parse_current(const uint8_t* data, size_t len);
std::optional<BmsTemperatures> parse_temp(const uint8_t* data, size_t len);
std::optional<BmsCellVoltages> parse_voltage(const uint8_t* data, size_t len);
std::optional<PressureSample> parse_pressure_sample(const uint8_t* data, size_t len);
std::optional<LeakageAlarm> parse_leakage_alarm(const uint8_t* data, size_t len);

#endif  // !CAN_DECODE_HPP_
