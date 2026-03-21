#include "can_interface.h"
#include <iostream>
#include <cstdint>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cstring>

// ─── Config ──────────────────────────────────────────────────────────────────

static constexpr uint32_t THRUSTER_CAN_ID   = 0x36C;
static constexpr int      NUM_THRUSTERS     = 8;
static constexpr uint16_t PWM_NEUTRAL       = 1500;   // µs
static constexpr uint16_t PWM_MIN           = 1000;   // µs – max reverse
static constexpr uint16_t PWM_MAX           = 2000;   // µs – max forward

// Which thruster index (0-based) to ramp. All others stay at neutral.
static constexpr int      THRUSTER_INDEX    = 0;

// Ramp parameters
static constexpr uint16_t RAMP_START        = 1500;   // µs – start at neutral
static constexpr uint16_t RAMP_END          = 2000;   // µs – ramp to full forward
static constexpr uint16_t RAMP_STEP         =   50;   // µs per step
static constexpr int      STEP_DELAY_MS     =  500;   // ms between steps

// ─── Helpers ─────────────────────────────────────────────────────────────────

std::string timestamp() {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now().time_since_epoch()).count();
    std::ostringstream oss;
    oss << "[" << std::setw(8) << std::setfill(' ') << ms % 100000 << "ms]";
    return oss.str();
}

// Build the 16-byte payload: 8 × uint16_t in little-endian order.
// thruster_index is set to pulse_width_us; all others are set to PWM_NEUTRAL.
void build_payload(uint8_t* out, int thruster_index, uint16_t pulse_width_us) {
    uint16_t values[NUM_THRUSTERS];
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        values[i] = (i == thruster_index) ? pulse_width_us : PWM_NEUTRAL;
    }
    // Copy as little-endian
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        out[i * 2]     = static_cast<uint8_t>(values[i] & 0xFF);        // low byte
        out[i * 2 + 1] = static_cast<uint8_t>((values[i] >> 8) & 0xFF); // high byte
    }
}

void print_payload(const uint8_t* data, int len) {
    std::cout << "  Payload:";
    for (int i = 0; i < len; i++) {
        if (i % 2 == 0) std::cout << "  [T" << (i / 2) << "] ";
        std::cout << std::hex << std::uppercase
                  << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    }
    std::cout << std::dec << std::endl;
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    can_interface can;

    if (can.init("can0") != can_status::OK) {
        std::cerr << "Failed to initialize CAN interface on can0" << std::endl;
        return 1;
    }
    std::cout << timestamp() << " CAN initialized on " << can.get_interface_name() << "\n\n";

    std::cout << "Thruster ramp test\n"
              << "  Thruster index : " << THRUSTER_INDEX << " (0-based)\n"
              << "  Ramp           : " << RAMP_START << " µs → " << RAMP_END
              << " µs  (step " << RAMP_STEP << " µs every " << STEP_DELAY_MS << " ms)\n"
              << "  Neutral        : " << PWM_NEUTRAL << " µs\n"
              << "  CAN ID         : 0x" << std::hex << THRUSTER_CAN_ID << std::dec << "\n\n";

    // ── Ramp up ──────────────────────────────────────────────────────────────
    std::cout << "── Ramp up ─────────────────────────────────────────────\n";
    for (uint16_t pw = RAMP_START; pw <= RAMP_END; pw += RAMP_STEP) {
        uint8_t payload[16];
        build_payload(payload, THRUSTER_INDEX, pw);

        std::cout << timestamp() << " TX  ID=0x" << std::hex << std::setw(3)
                  << std::setfill('0') << THRUSTER_CAN_ID << std::dec
                  << "  T" << THRUSTER_INDEX << "=" << pw << " µs\n";
        print_payload(payload, sizeof(payload));

        if (can.send(THRUSTER_CAN_ID, payload, sizeof(payload), false) != can_status::OK) {
            std::cerr << timestamp() << " ERROR: send failed at " << pw << " µs\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(STEP_DELAY_MS));
    }

    // ── Return to neutral ────────────────────────────────────────────────────
    std::cout << "\n── Return to neutral ───────────────────────────────────\n";
    {
        uint8_t payload[16];
        build_payload(payload, THRUSTER_INDEX, PWM_NEUTRAL);

        std::cout << timestamp() << " TX  ID=0x" << std::hex << std::setw(3)
                  << std::setfill('0') << THRUSTER_CAN_ID << std::dec
                  << "  T" << THRUSTER_INDEX << "=" << PWM_NEUTRAL << " µs (neutral)\n";
        print_payload(payload, sizeof(payload));

        if (can.send(THRUSTER_CAN_ID, payload, sizeof(payload), false) != can_status::OK) {
            std::cerr << timestamp() << " ERROR: send failed at neutral\n";
        }
    }

    std::cout << "\nRamp test complete.\n";
    return 0;
}