#include "power_sense_driver.hpp"
#include <cstdint>

PowerSenseDriver::PowerSenseDriver(std::uint8_t i2c_bus,
                                   std::uint8_t i2c_address,
                                   std::uint8_t resolution)
    : i2c_bus_(i2c_bus), i2c_address_(i2c_address), resolution_(resolution) {
    std::string i2c_filename = "/dev/i2c-" + std::to_string(i2c_bus_);
    bus_fd_ =
        open(i2c_filename.c_str(),
             O_RDWR);  // Open the i2c bus for reading and writing (0_RDWR)
    if (bus_fd_ < 0) {
        std::runtime_error("ERROR: Failed to open I2C bus " +
                           std::to_string(i2c_bus_) + " : " +
                           std::string(strerror(errno)));
    }
}

bool PowerSenseDriver::ping_powersense() {
    std::uint8_t dummy_data;

    // Set the I2C slave address
    if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
        throw std::runtime_error("Failed to set I2C address " +
                                 std::to_string(i2c_bus_) + " : " +
                                 std::string(strerror(errno)));
        return false;
    }

    // Try to read a single byte (ping test)
    if (read(bus_fd_, &dummy_data, 1) != 1) {
        return false;  // No response from MCP342x
    }

    return true;  // Device responded
}

bool PowerSenseDriver::start_conversion() {
    try {
        constexpr std::size_t i2c_data_size = 1;
        
        // Configure MCP342X for Channel 0 (Voltage)
        std::uint8_t config_channel_0 = ((resolution_ & ~MCP342X_CHANNEL_MASK) |
                                         (0 & MCP342X_CHANNEL_MASK)); // Channel 0

        // Configure MCP342X for Channel 1 (Current)
        std::uint8_t config_channel_1 = ((resolution_ & ~MCP342X_CHANNEL_MASK) |
                                         (1 & MCP342X_CHANNEL_MASK)); // Channel 1

        // Set I2C device
        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error("Failed to open I2C bus " +
                                     std::to_string(i2c_bus_) + " : " +
                                     std::string(strerror(errno)));
            return false;
        }

        // Start Conversion on Channel 0 (Voltage)
        if (write(bus_fd_, &config_channel_0, i2c_data_size) != i2c_data_size) {
            throw std::runtime_error("Error: Failed to write to I2C device for Channel 0 : " +
                                     std::string(strerror(errno)));
            return false;
        }

        // Delay between channel switches (MCP342X requires some time for settling)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // Start Conversion on Channel 1 (Current)
        if (write(bus_fd_, &config_channel_1, i2c_data_size) != i2c_data_size) {
            throw std::runtime_error("Error: Failed to write to I2C device for Channel 1 : " +
                                     std::string(strerror(errno)));
            return false;
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send PWM values - " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "ERROR: Failed to send PWM values - unknown error" << std::endl;
        return false;
    }

    return true;
}

void PowerSenseDriver::read_current_powersense(
    std::vector<std::uint16_t>& current_array) {
    try {
        constexpr std::size_t i2c_data_size =
            1 + 3 * 2;  // 3 thrusters * (1xMSB + 1xLSB)
        std::vector<std::uint8_t> i2c_data_array;
        i2c_data_array.reserve(i2c_data_size);

        if (ioctl(bus_fd_, I2C_SLAVE, i2c_address_) < 0) {
            throw std::runtime_error("Failed to open I2C bus " +
                                     std::to_string(i2c_bus_) + " : " +
                                     std::string(strerror(errno)));
            return;
        }

        if (write(bus_fd_, i2c_data_array.data(), i2c_data_size) !=
            i2c_data_size) {
            throw std::runtime_error("Error: Failed to write to I2C device : " +
                                     std::string(strerror(errno)));
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send PWM values - " << e.what()
                  << std::endl;
    } catch (...) {
        std::cerr << "ERROR: Failed to send PWM values - unknown error"
                  << std::endl;
    }
}
