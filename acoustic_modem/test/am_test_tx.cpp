#include "am_driver.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

static std::string bytes2(uint8_t b0, uint8_t b1) {
  std::string s;
  s.resize(2);
  s[0] = static_cast<char>(b0);
  s[1] = static_cast<char>(b1);
  return s;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: am_test_tx <device>\n"
              << "Example: am_test_tx /dev/pts/5\n";
    return 1;
  }

  const std::string dev = argv[1];

  int baud = 9600;
  int channel = 1;
  int level = 4;
  bool diagnostic = false;
  float timeout = 0.5f;

  std::cout << "Opening driver on: " << dev << "\n";
  AcousticModemDriver drv(dev, baud, channel, level, diagnostic, timeout);
  std::this_thread::sleep_for(std::chrono::seconds(5));
  std::cout << "Sending tests...\n";

  // 1) ASCII test
  char a='A';
  drv.send_data(a);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 2) Raw 2-byte test (00 01)
  drv.send_two_bytes(bytes2(0x00, 0x01));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 3) Raw 2-byte test (AA 55)
  drv.send_two_bytes(bytes2(0xAA, 0x55));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 4) 16-bit word test
  // IMPORTANT: This will show you the byte order used by send_word()
//   drv.send_word(0xABCD);
//   std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 5) Repeat a few times so it’s easy to see on hexdump
//   for (int i = 0; i < 5; ++i) {
//     drv.send_word(0x1234);
//     std::this_thread::sleep_for(std::chrono::milliseconds(200));
//   }

  std::cout << "Done. Keep hexdump terminal open to verify output.\n";
  return 0;
}
