#include "can_interface.h"
#include <iostream>
#include <cstdint>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>

std::atomic<int> frames_received(0);

std::string timestamp() {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::ostringstream oss;
    oss << "[" << std::setw(8) << ms % 100000 << "ms]";
    return oss.str();
}

void can_callback(const struct canfd_frame& frame, can_status status) {
    if (status != can_status::OK) {
        std::cerr << timestamp() << " Receive error" << std::endl;
        return;
    }

    int count = ++frames_received;
    std::cout << timestamp() << " [RX #" << std::dec << count << "]"
              << " ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id
              << " Len: " << std::dec << (int)frame.len
              << " Data:";
    for (int i = 0; i < frame.len; i++) {
        std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0') << (int)frame.data[i];
    }
    std::cout << std::endl;
}

int main() {
    can_interface can;

    if (can.init("can0") != can_status::OK) {
        std::cerr << "Failed to initialize CAN interface" << std::endl;
        return 1;
    }

    std::cout << timestamp() << " CAN interface initialized on " << can.get_interface_name() << std::endl;

    if (can.start_async_receive(can_callback) != can_status::OK) {
        std::cerr << "Failed to start async receive" << std::endl;
        return 1;
    }

    std::cout << timestamp() << " Async receive started, sending frames at different intervals...\n" << std::endl;

    // Simulate realistic async behavior: send frames at irregular intervals
    // while the main thread is busy doing other work
    struct {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
        int delay_ms;   // delay BEFORE sending this frame
    } test_frames[] = {
        { 0x001, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 0   },
        { 0x002, {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 50  },
        { 0x003, {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 300 },  // longer gap
        { 0x004, {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 50  },
        { 0x005, {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 500 },  // longer gap
        { 0x006, {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 50  },
        { 0x007, {0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 50  },
    };

    int frames_sent = 0;
    for (auto& f : test_frames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(f.delay_ms));

        // Simulate main thread being busy while async receive runs in background
        std::cout << timestamp() << " [MAIN] Doing some work..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cout << timestamp() << " [MAIN] Work done, sending frame now" << std::endl;

        if (can.send(f.id, f.data, f.len, true) != can_status::OK) {
            std::cerr << timestamp() << " Failed to send frame" << std::endl;
        } else {
            frames_sent++;
            std::cout << timestamp() << " [TX #" << std::dec << frames_sent
                      << "] ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << f.id << "\n" << std::endl;
        }
    }

    // Wait for last frames to be received
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    can.stop_async_receive();

    std::cout << "\n" << timestamp() << " Summary: sent " << std::dec << frames_sent
              << " frames, received " << frames_received.load() << " frames" << std::endl;

    return 0;
}