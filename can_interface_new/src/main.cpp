#include "can_interface.h"
#include <iostream>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>

// ─── CAN IDs ─────────────────────────────────────────────────────────────────

static constexpr uint32_t THRUSTER_CAN_ID   = 0x36C;
static constexpr uint32_t STATUS_CAN_ID     = 0x45A;

// ─── Thruster config ─────────────────────────────────────────────────────────

static constexpr int      NUM_THRUSTERS     = 8;
static constexpr uint16_t PWM_NEUTRAL       = 1500;   // µs
static constexpr uint16_t PWM_MIN           = 1000;   // µs – max reverse
static constexpr uint16_t PWM_MAX           = 2000;   // µs – max forward

static constexpr int      THRUSTER_INDEX    = 0;      // 0-based, which thruster to ramp

static constexpr uint16_t RAMP_START        = 1500;   // µs
static constexpr uint16_t RAMP_END          = 2000;   // µs
static constexpr uint16_t RAMP_STEP         =   50;   // µs per step
static constexpr int      STEP_DELAY_MS     =  500;   // ms between steps

// ─── 0x45A message type discriminator (data[0]) ──────────────────────────────

enum class StatusMsgType : uint8_t {
    CURRENT_MEASUREMENTS = 0x00,
    FLT_EVENT            = 0x01,
    PGOOD_EVENT          = 0x02,
    KILLSWITCH_EVENT     = 0x03,
};

// ─── Parsed message structs ───────────────────────────────────────────────────

struct FltEvent {
    uint8_t context;
};

struct PgoodEvent {
    uint8_t context;
};

struct KillswitchEvent {
    // no extra fields – presence of the frame is the event
};

struct CurrentMeasurements {
    float current_A[8];   // amps, one per thruster
};

// ─── Helpers ─────────────────────────────────────────────────────────────────

std::string timestamp() {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now().time_since_epoch()).count();
    std::ostringstream oss;
    oss << "[" << std::setw(8) << std::setfill(' ') << ms % 100000 << "ms]";
    return oss.str();
}

// Build the 16-byte thruster payload: 8 × uint16_t little-endian.
void build_thruster_payload(uint8_t* out, int thruster_index, uint16_t pulse_width_us) {
    uint16_t values[NUM_THRUSTERS];
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        values[i] = (i == thruster_index) ? pulse_width_us : PWM_NEUTRAL;
    }
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        out[i * 2]     = static_cast<uint8_t>(values[i] & 0xFF);
        out[i * 2 + 1] = static_cast<uint8_t>((values[i] >> 8) & 0xFF);
    }
}

void print_thruster_payload(const uint8_t* data) {
    std::cout << "  Payload:";
    for (int i = 0; i < NUM_THRUSTERS; i++) {
        uint16_t pw = static_cast<uint16_t>(data[i * 2]) |
                      (static_cast<uint16_t>(data[i * 2 + 1]) << 8);
        std::cout << "  [T" << i << "]=" << std::dec << pw << "µs";
    }
    std::cout << "\n";
}

// ─── 0x45A frame parsing & dispatch ─────────────────────────────────────────

void handle_flt_event(const FltEvent& e) {
    std::cout << timestamp() << " [RX 0x45A] FLT_EVENT"
              << "  context=0x" << std::hex << std::setw(2) << std::setfill('0')
              << (int)e.context << std::dec << "\n";
}

void handle_pgood_event(const PgoodEvent& e) {
    std::cout << timestamp() << " [RX 0x45A] PGOOD_EVENT"
              << "  context=0x" << std::hex << std::setw(2) << std::setfill('0')
              << (int)e.context << std::dec << "\n";
}

void handle_killswitch_event(const KillswitchEvent&) {
    std::cout << timestamp() << " [RX 0x45A] KILLSWITCH_EVENT\n";
}

void handle_current_measurements(const CurrentMeasurements& m) {
    std::cout << timestamp() << " [RX 0x45A] CURRENT_MEASUREMENTS\n";
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < 8; i++) {
        std::cout << "  T" << i << " = " << m.current_A[i] << " A\n";
    }
}

// Decode a raw CAN FD payload from 0x45A and dispatch to the right handler.
void dispatch_status_frame(const struct canfd_frame& frame) {
    if (frame.len < 1) {
        std::cerr << timestamp() << " [RX 0x45A] ERROR: empty frame\n";
        return;
    }

    const uint8_t* d = frame.data;
    auto type = static_cast<StatusMsgType>(d[0]);

    switch (type) {
        case StatusMsgType::FLT_EVENT: {
            FltEvent e;
            e.context = (frame.len >= 2) ? d[1] : 0x00;
            handle_flt_event(e);
            break;
        }
        case StatusMsgType::PGOOD_EVENT: {
            PgoodEvent e;
            e.context = (frame.len >= 2) ? d[1] : 0x00;
            handle_pgood_event(e);
            break;
        }
        case StatusMsgType::KILLSWITCH_EVENT: {
            handle_killswitch_event(KillswitchEvent{});
            break;
        }
        case StatusMsgType::CURRENT_MEASUREMENTS: {
            if (frame.len < 1 + 8 * static_cast<int>(sizeof(float))) {
                std::cerr << timestamp() << " [RX 0x45A] ERROR: current frame too short ("
                          << (int)frame.len << " bytes)\n";
                break;
            }
            CurrentMeasurements m;
            for (int i = 0; i < 8; i++) {
                memcpy(&m.current_A[i], &d[1 + i * sizeof(float)], sizeof(float));
            }
            handle_current_measurements(m);
            break;
        }
        default:
            std::cerr << timestamp() << " [RX 0x45A] UNKNOWN type=0x"
                      << std::hex << (int)d[0] << std::dec << "\n";
            break;
    }
}

// ─── Async receive callback ───────────────────────────────────────────────────

void can_rx_callback(const struct canfd_frame& frame, can_status status) {
    if (status != can_status::OK) {
        // Timeout on the receive thread – not fatal, just the 1 s poll window expiring
        return;
    }

    uint32_t id = frame.can_id & CAN_EFF_MASK;   // strip EFF/RTR/ERR flags

    if (id == STATUS_CAN_ID) {
        dispatch_status_frame(frame);
    }
    // Additional IDs can be dispatched here as the library grows
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main() {
    can_interface can;

    if (can.init("can0") != can_status::OK) {
        std::cerr << "Failed to initialize CAN interface on can0\n";
        return 1;
    }
    std::cout << timestamp() << " CAN initialized on " << can.get_interface_name() << "\n\n";

    // ── Start async receive ───────────────────────────────────────────────────
    if (can.start_async_receive(can_rx_callback) != can_status::OK) {
        std::cerr << "Failed to start async receive\n";
        return 1;
    }
    std::cout << timestamp() << " Async receive running (listening on 0x"
              << std::hex << STATUS_CAN_ID << std::dec << ")\n\n";

    // ── Thruster ramp ─────────────────────────────────────────────────────────
    std::cout << "Thruster ramp test\n"
              << "  Thruster index : " << THRUSTER_INDEX << " (0-based)\n"
              << "  Ramp           : " << RAMP_START << " µs → " << RAMP_END
              << " µs  (step " << RAMP_STEP << " µs every " << STEP_DELAY_MS << " ms)\n"
              << "  Neutral        : " << PWM_NEUTRAL << " µs\n"
              << "  CAN ID         : 0x" << std::hex << THRUSTER_CAN_ID << std::dec << "\n\n";

    std::cout << "── Ramp up ──────────────────────────────────────────────\n";
    int frames_sent = 0;
    for (uint16_t pw = RAMP_START; pw <= RAMP_END; pw = static_cast<uint16_t>(pw + RAMP_STEP)) {
        uint8_t payload[16];
        build_thruster_payload(payload, THRUSTER_INDEX, pw);

        std::cout << timestamp() << " [TX #" << ++frames_sent << "]"
                  << " ID=0x" << std::hex << std::setw(3) << std::setfill('0')
                  << THRUSTER_CAN_ID << std::dec
                  << "  T" << THRUSTER_INDEX << "=" << pw << " µs\n";
        print_thruster_payload(payload);

        if (can.send(THRUSTER_CAN_ID, payload, sizeof(payload), true) != can_status::OK) {
            std::cerr << timestamp() << " ERROR: send failed at " << pw << " µs\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(STEP_DELAY_MS));
    }

    // ── Return to neutral ─────────────────────────────────────────────────────
    std::cout << "\n── Return to neutral ────────────────────────────────────\n";
    {
        uint8_t payload[16];
        build_thruster_payload(payload, THRUSTER_INDEX, PWM_NEUTRAL);

        std::cout << timestamp() << " [TX #" << ++frames_sent << "]"
                  << " ID=0x" << std::hex << std::setw(3) << std::setfill('0')
                  << THRUSTER_CAN_ID << std::dec
                  << "  T" << THRUSTER_INDEX << "=" << PWM_NEUTRAL << " µs (neutral)\n";
        print_thruster_payload(payload);

        if (can.send(THRUSTER_CAN_ID, payload, sizeof(payload), true) != can_status::OK) {
            std::cerr << timestamp() << " ERROR: send failed at neutral\n";
        }
    }

    // Linger to catch any trailing status frames before stopping the receive thread
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    can.stop_async_receive();

    std::cout << "\nTest complete. Sent " << std::dec << frames_sent << " thruster frame(s).\n";
    return 0;
}