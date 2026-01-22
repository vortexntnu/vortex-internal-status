#include "am_driver.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

static void print_hex(const std::vector<uint8_t>& v) {
  std::cout << "  data: ";
  for (uint8_t b : v) {
    std::cout << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(b) << " ";
  }
  std::cout << std::dec << "\n";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: am_min_test <device>\n"
              << "Example: am_min_test /dev/pts/4\n";
    return 1;
  }

  const std::string dev = argv[1];

  int baud = 115200;
  int channel = 1;
  int level = 4;
  bool diagnostic = false;
  float timeout = 0.5f;

  std::cout << "Opening driver on: " << dev << "\n";

  AcousticModemDriver drv(dev, baud, channel, level, diagnostic, timeout);

  std::cout << "Listening... (Ctrl+C to stop)\n";

  while (true) {
    std::vector<uint8_t> out;

    // This must pop one received chunk from the internal queue.
    if (drv.try_pop_rx(out)) {
      std::cout << "RX chunk: " << out.size() << " bytes\n";
      print_hex(out);
    }

    // Avoid busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
