#include "ms5837.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

MS5837::MS5837(const std::string& i2c_device, uint8_t address)
    : i2c_device_(i2c_device), address_(address), fd_(-1) {}

MS5837::~MS5837() {
    closeBus();
}

bool MS5837::openBus() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(i2c_device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::perror("open i2c device");
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "Failed to set I2C slave address 0x" << std::hex
                  << static_cast<int>(address_) << " on " << i2c_device_ << ": "
                  << std::strerror(errno) << std::dec << std::endl;

        ::close(fd_);
        fd_ = -1;
        return false;
    }
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::perror("I2C_SLAVE");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void MS5837::closeBus() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool MS5837::writeByte(uint8_t value) {
    if (fd_ < 0) {
        return false;
    }

    if (::write(fd_, &value, 1) != 1) {
        std::perror("I2C writeByte");
        return false;
    }

    return true;
}

bool MS5837::writeRead(uint8_t reg, uint8_t* rx, uint16_t len) {
    if (fd_ < 0 || rx == nullptr || len == 0) {
        return false;
    }

    if (::write(fd_, &reg, 1) != 1) {
        std::perror("I2C writeRead write");
        return false;
    }

    if (::read(fd_, rx, len) != static_cast<ssize_t>(len)) {
        std::perror("I2C writeRead read");
        return false;
    }

    return true;
}

bool MS5837::reset() {
    return writeByte(RESET);
}


bool MS5837::readPROM() {
    uint8_t rx[2] = {0, 0};

    for (uint8_t i = 0; i < 7; ++i) {
        const uint8_t addr = static_cast<uint8_t>(PROM_READ + (i * 2U));

        bool ok = false;

        for (int attempt = 1; attempt <= 5; ++attempt) {
            rx[0] = 0;
            rx[1] = 0;

            if (writeRead(addr, rx, 2)) {
                ok = true;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (!ok) {
            return false;
        }

        C_[i] = static_cast<uint16_t>(
            (static_cast<uint16_t>(rx[0]) << 8) |
            static_cast<uint16_t>(rx[1])
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return true;
}

bool MS5837::init() {
    if (!openBus()) {
        return false;
    }

    std::memset(C_, 0, sizeof(C_));
    D1_pres_ = 0;
    D2_temp_ = 0;
    TEMP_ = 0;
    P_ = 0;
    fluid_density_ = 1029.0f;
    model_ = MODEL_UNRECOGNISED;
    initialized_ = false;

    if (!reset()) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!readPROM()) {
        return false;
    }

    uint16_t prom_copy[8];
    std::memcpy(prom_copy, C_, sizeof(prom_copy));

    uint8_t crc_read = static_cast<uint8_t>(C_[0] >> 12);
    uint8_t crc_calculated = crc4(prom_copy);

    if (crc_calculated != crc_read) {
        std::cerr << "MS5837 CRC mismatch. Read=" << static_cast<int>(crc_read)
                  << " Calculated=" << static_cast<int>(crc_calculated)
                  << std::endl;
        return false;
    }

    if ((C_[1] < MODEL_30BA_MIN_SENSITIVITY) ||
        (C_[1] > MODEL_02BA_MAX_SENSITIVITY)) {
        model_ = MODEL_UNRECOGNISED;
    } else if (C_[1] > MODEL_02BA_30BA_SEPARATION) {
        model_ = MODEL_02BA;
    } else {
        model_ = MODEL_30BA;
    }

    initialized_ = true;
    return true;
}

void MS5837::setModel(Model model) {
    model_ = model;
}

MS5837::Model MS5837::getModel() const {
    return model_;
}

void MS5837::setFluidDensity(float density) {
    fluid_density_ = density;
}

bool MS5837::read() {
    if (!initialized_) {
        return false;
    }

    uint8_t cmd;
    uint8_t rx[3];

    cmd = CONVERT_D1_8192;
    if (!writeByte(cmd)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    cmd = ADC_READ;
    if (!writeRead(cmd, rx, 3)) {
        return false;
    }

    D1_pres_ = (static_cast<uint32_t>(rx[0]) << 16) |
               (static_cast<uint32_t>(rx[1]) << 8) |
               (static_cast<uint32_t>(rx[2]));

    cmd = CONVERT_D2_8192;
    if (!writeByte(cmd)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    cmd = ADC_READ;
    if (!writeRead(cmd, rx, 3)) {
        return false;
    }

    D2_temp_ = (static_cast<uint32_t>(rx[0]) << 16) |
               (static_cast<uint32_t>(rx[1]) << 8) |
               (static_cast<uint32_t>(rx[2]));

    calculate();
    return true;
}

void MS5837::calculate() {
    int32_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    dT =
        static_cast<int32_t>(D2_temp_ - (static_cast<uint32_t>(C_[5]) * 256UL));

    if (model_ == MODEL_02BA) {
        SENS = (static_cast<int64_t>(C_[1]) * 65536LL) +
               ((static_cast<int64_t>(C_[3]) * dT) / 128LL);
        OFF = (static_cast<int64_t>(C_[2]) * 131072LL) +
              ((static_cast<int64_t>(C_[4]) * dT) / 64LL);
        P_ = static_cast<int32_t>(
            (((static_cast<int64_t>(D1_pres_) * SENS) / 2097152LL) - OFF) /
            32768LL);
    } else {
        SENS = (static_cast<int64_t>(C_[1]) * 32768LL) +
               ((static_cast<int64_t>(C_[3]) * dT) / 256LL);
        OFF = (static_cast<int64_t>(C_[2]) * 65536LL) +
              ((static_cast<int64_t>(C_[4]) * dT) / 128LL);
        P_ = static_cast<int32_t>(
            (((static_cast<int64_t>(D1_pres_) * SENS) / 2097152LL) - OFF) /
            8192LL);
    }

    TEMP_ = static_cast<int32_t>(
        2000LL + ((static_cast<int64_t>(dT) * C_[6]) / 8388608LL));

    if (model_ == MODEL_02BA) {
        if ((TEMP_ / 100) < 20) {
            Ti = static_cast<int32_t>(
                (11LL * static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) /
                34359738368LL);
            OFFi = static_cast<int32_t>(
                (31LL * (TEMP_ - 2000) * (TEMP_ - 2000)) / 8LL);
            SENSi = static_cast<int32_t>(
                (63LL * (TEMP_ - 2000) * (TEMP_ - 2000)) / 32LL);
        }
    } else {
        if ((TEMP_ / 100) < 20) {
            Ti = static_cast<int32_t>(
                (3LL * static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) /
                8589934592LL);
            OFFi = static_cast<int32_t>(
                (3LL * (TEMP_ - 2000) * (TEMP_ - 2000)) / 2LL);
            SENSi = static_cast<int32_t>(
                (5LL * (TEMP_ - 2000) * (TEMP_ - 2000)) / 8LL);

            if ((TEMP_ / 100) < -15) {
                OFFi += static_cast<int32_t>(7LL * (TEMP_ + 1500L) *
                                             (TEMP_ + 1500L));
                SENSi += static_cast<int32_t>(4LL * (TEMP_ + 1500L) *
                                              (TEMP_ + 1500L));
            }
        } else {
            Ti = static_cast<int32_t>(
                (2LL * static_cast<int64_t>(dT) * static_cast<int64_t>(dT)) /
                137438953472LL);
            OFFi = static_cast<int32_t>(
                (static_cast<int64_t>(TEMP_ - 2000) * (TEMP_ - 2000)) / 16LL);
            SENSi = 0;
        }
    }

    OFF2 = OFF - OFFi;
    SENS2 = SENS - SENSi;

    TEMP_ -= Ti;

    if (model_ == MODEL_02BA) {
        P_ = static_cast<int32_t>(
            (((static_cast<int64_t>(D1_pres_) * SENS2) / 2097152LL) - OFF2) /
            32768LL);
    } else {
        P_ = static_cast<int32_t>(
            (((static_cast<int64_t>(D1_pres_) * SENS2) / 2097152LL) - OFF2) /
            8192LL);
    }
}

float MS5837::pressure(float conversion) const {
    if (model_ == MODEL_02BA) {
        return (static_cast<float>(P_) * conversion) / 100.0f;
    } else {
        return (static_cast<float>(P_) * conversion) / 10.0f;
    }
}

float MS5837::temperature() const {
    return static_cast<float>(TEMP_) / 100.0f;
}

float MS5837::depth() const {
    float pressure_pa = pressure(PressureUnit::Pa);
    return (pressure_pa - 101300.0f) / (fluid_density_ * 9.80665f);
}

float MS5837::altitude() const {
    float pressure_mbar = pressure(PressureUnit::mbar);
    return (1.0f - std::pow((pressure_mbar / 1013.25f), 0.190284f)) *
           145366.45f * 0.3048f;
}

uint8_t MS5837::crc4(uint16_t prom[8]) {
    uint16_t n_rem = 0;

    prom[0] &= 0x0FFFU;
    prom[7] = 0U;

    for (uint8_t i = 0; i < 16; i++) {
        if (i & 1U) {
            n_rem ^= static_cast<uint16_t>(prom[i >> 1] & 0x00FFU);
        } else {
            n_rem ^= static_cast<uint16_t>(prom[i >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000U) {
                n_rem = static_cast<uint16_t>((n_rem << 1) ^ 0x3000U);
            } else {
                n_rem = static_cast<uint16_t>(n_rem << 1);
            }
        }
    }

    n_rem = static_cast<uint16_t>((n_rem >> 12) & 0x000FU);
    return static_cast<uint8_t>(n_rem ^ 0x00U);
}
