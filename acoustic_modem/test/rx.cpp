#include "../include/am_driver.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>
#include <csignal>

std::atomic<bool> running(true);

void signal_handler(int)
{
    running = false;
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " /dev/ttyUSBx\n";
        return 1;
    }

    std::string port = argv[1];

    int baud = 9600;
    int channel = 1;
    int level = 4;
    bool diagnostic = false;
    float timeout = 0.5f;

    AcousticModemDriver modem(port, baud, channel, level, diagnostic, timeout);

    AcousticModemDriver::DecodedMessage msg;
    std::cout << "Listening on " << port << "...\n";

    while (running)
    {
        //std::cout<<"nada\n";
        if (modem.try_pop_decoded(msg))
        {
            std::cout << "RX: ";

            for (int i=0;i<msg.n_floats;i++){
                std::cout << " " << msg.floats[i] << " ";
            }
            std::cout << std::dec << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!running)
            break;   // interrupted by Ctrl+C
        std::cin.clear();
        //std::cout<<"boh\n";
    }
    running = false;
    // rx_thread.join();

    std::cout << "Stopped.\n";
    return 0;
}