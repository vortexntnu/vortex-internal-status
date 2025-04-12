#include "../include/psm_orin_driver.h"
#include <stdint.h>
#include <unistd.h>

static int i2c_fd = -1;

static int i2c_write(uint8_t* data, uint8_t length, uint8_t address) {
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to select I2C device");
        return -1;
    }

    if (write(i2c_fd, data, length) != length) {
        perror("Failed to write to I2C device");
        return -1;
    }
    return 0;
}


static int i2c_read(uint8_t* data, uint8_t length, uint8_t address) {
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to select I2C device");
        return -1;
    }

    if (read(i2c_fd, data, length) != length) {
        perror("Failed to read from I2C device");
        return -1;
    }

    return 0;
}

static int i2c_write_read(uint8_t* writeData,
                          uint8_t writeLength,
                          uint8_t* readData,
                          uint8_t readLength,
                          uint8_t address) {
    if (i2c_write(writeData, writeLength, address)) {
        return -1;
    }

    if (i2c_read(readData, readLength, address)) {
        return -1;
    }

    return 0;
}

static inline int start_conversion(uint16_t config) {
    config |= CFG_OS_SINGLE;
    uint8_t i2c_data[3];
    i2c_data[0] = REG_CFG;
    i2c_data[1] = (config >> 8) & 0xFF;
    i2c_data[2] = config & 0xFF;

    return i2c_write(i2c_data, 3, PSM_ADDRESS);
}

int i2c_init() {
    const char* i2c_device = I2C_DEVICE_PATH;
    if ((i2c_fd = open(i2c_device, O_RDWR)) < 0) {
        perror("Failed to open I2C device!");
        return -1;
    }
    return 0;
}

int read_psm_measurements(double* voltage, double* current) {
    static uint16_t default_config =
        CFG_OS_SINGLE | CFG_MUX_DIFF_0_1 | CFG_PGA_6_144V | CFG_MODE_SINGLE |
        CFG_DR_128SPS | CFG_COMP_MODE | CFG_COMP_POL | CFG_COMP_LAT |
        CFG_COMP_QUE_DIS;

    uint16_t config = default_config;
    config &= ~0x7000;
    config |= CFG_MUX_DIFF_0_1;
    if (start_conversion(config))
        return -1;

    usleep(10000);

    uint8_t i2c_data[2];
    if (i2c_write_read(REG_CONV, 1, i2c_data, 2, PSM_ADDRESS))
        return -1;
    int16_t raw_voltage = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);
    *voltage =
        ((raw_voltage * VOLTAGE_RANGE) / 32768.0) * VOLTAGE_SCALE + DIODE_LOSS;

    config = default_config;
    config &= ~0x7000;
    config |= CFG_MUX_DIFF_2_3;
    if (start_conversion(config))
        return -1;
    usleep(10000);

    if (i2c_write_read(REG_CONV, 1, i2c_data, 2, PSM_ADDRESS))
        return -1;
    int16_t raw_current = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);
    *current = (CURRENT_OFFSET - ((raw_current * VOLTAGE_RANGE) / 32768.0)) /
               CURRENT_SENSITIVITY;
    return 0;
}

int read_pressure(double* pressure) {
    uint8_t write_data[3] = {MPRLS_REG, 0, 0};
    uint8_t i2c_data[4];

    if (i2c_write_read(write_data, 3, i2c_data, 4, MPRLS_ADDRESS)) {
        return -1;
    }
    uint8_t status = i2c_data[0];
    int32_t pressure_counts =
        (int32_t) ((i2c_data[1] << 16) | (i2c_data[2] << 8) | i2c_data[3]);
    double scale = (PRESSURE_MAX - PRESSURE_MIN) / (double)(COUNTS_MAX - COUNTS_MIN);
    *pressure = (pressure_counts - COUNTS_MIN) * scale + PRESSURE_MIN;

    return 0;
}
