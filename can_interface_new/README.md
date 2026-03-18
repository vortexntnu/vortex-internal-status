# Can Interface

A C++ library for sending and receiving CAN and CAN FD frames over Linux SocketCAN interfaces. Supports synchronous and asynchronous receive, sending, and ID filtering.

## Requirements

- Linux with SocketCAN support
- C++17 or higher

## Setting Up a Virtual CAN Interface

For development and testing without real CAN hardware, use a virtual CAN interface:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

> Note: This does not persist after reboot.

---

## Basic Asynchronous Receive Example - Counting Frames

This showcases a basic example that receives frames asynchronously and counts the number of frames received. Pay attention to the use of `std::atomic` for the `frames_received` variable.

```cpp
#include "../include/can_interface.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

int main() {
    can_interface can;
    can.init("vcan0"); // or your real interface e.g. "can0"

    std::atomic<int> frames_received(0);

    std::cout << "[main] Starting async receive..." << std::endl;

    // Asynchronous Recieve
    can.start_async_receive([&frames_received](const struct canfd_frame& frame, can_status status) {
        if (status == can_status::OK) {
            frames_received++;
            std::cout << "[async thread] Received frame ID: 0x" << std::hex << frame.can_id
                      << " | frames so far: " << std::dec << frames_received.load() << std::endl;
        }
    });

    // Sending some data
    for (int i = 0; i < 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        uint8_t data[] = {(uint8_t)i, 0xAB, 0xCD, 0xEF};
        std::cout << "[main] Sending frame " << i << "..." << std::endl;
        can.send(0x123, data, 4);
    }

    // Stop asynchrnously receiving
    can.stop_async_receive();

    std::cout << "[main] Done. Total frames received: " << frames_received.load() << std::endl;

    return 0;
}
```

## Collecting Frames for Later Processing

A useful way to use the `start_async_receive` function is to to collect frames inside the callback and process them later on in the thread. Since the callback runs on a background thread, access to shared data must be protected with a mutex.

```cpp
#include "../include/can_interface.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>

int main() {
    can_interface can;
    can.init("can0");
    can.clear_filters();

    std::vector<struct canfd_frame> collected_frames;
    std::mutex frames_mutex;

    // Collect frames in the background
    can.start_async_receive([&collected_frames, &frames_mutex](const struct canfd_frame& frame, can_status status) {
        if (status == can_status::OK) {
            std::lock_guard lock(frames_mutex);
            collected_frames.push_back(frame);
        }
    });

    // Do other work on the main thread while frames are being collected
    for (int i = 0; i < 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        uint8_t data[] = {(uint8_t)i, 0xAB, 0xCD, 0xEF};
        std::cout << "[main] Sending frame " << i << std::endl;
        can.send(0x123, data, 4);
    }

    can.stop_async_receive();

    // Process all collected frames
    std::lock_guard lock(frames_mutex);
    std::cout << "Processing " << collected_frames.size() << " collected frames:" << std::endl;
    for (const auto& frame : collected_frames) {
        std::cout << "  Frame ID: 0x" << std::hex << frame.can_id << std::endl;
        // your processing logic here
    }

    return 0;
}
```
