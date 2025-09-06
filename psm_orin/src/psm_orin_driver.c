#include "psm_orin_driver.h"

static int i2c_write(int bus_fd,
                     uint8_t* data,
                     uint8_t length,
                     uint8_t address) {
    if (write(bus_fd, data, length) != length) {
        perror("Failed to write to I2C device");
        return -1;
    }
    return 0;
}

static int i2c_read(int bus_fd,
                    uint8_t* data,
                    uint8_t length,
                    uint8_t address) {
    if (read(bus_fd, data, length) != length) {
        perror("Failed to read from I2C device");
        return -1;
    }
    return 0;
}

static int i2c_write_read(int bus_fd,
                          uint8_t* writeData,
                          uint8_t writeLength,
                          uint8_t* readData,
                          uint8_t readLength,
                          uint8_t address) {
    if (i2c_write(bus_fd, writeData, writeLength, address)) {
        return -1;
    }
    usleep(5000);
    if (i2c_read(bus_fd, readData, readLength, address)) {
        return -1;
    }
    return 0;
}

static int start_conversion(int bus_fd, uint16_t config) {
    config |= CFG_OS_SINGLE;
    uint8_t i2c_data[3];
    i2c_data[0] = REG_CFG;
    i2c_data[1] = (config >> 8) & 0xFF;
    i2c_data[2] = config & 0xFF;

    return i2c_write(bus_fd, i2c_data, 3, PSM_ADDRESS);
}

int i2c_init(int* bus_fd, uint8_t address) {
    const char* i2c_device = I2C_DEVICE_PATH;
    if ((*bus_fd = open(i2c_device, O_RDWR)) < 0) {
        perror("Failed to open I2C device!");
        return -1;
    }
    if (ioctl(*bus_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to select I2C device");
        return -1;
    }
    return 0;
}

void i2c_close(int* bus_fd) {
    close(*bus_fd);
    *bus_fd = -1;
}

int read_psm_measurements(int bus_fd, double* voltage, double* current) {
    static const uint16_t default_config =
        CFG_OS_SINGLE | CFG_MUX_DIFF_0_1 | CFG_PGA_6_144V | CFG_MODE_SINGLE |
        CFG_DR_128SPS | CFG_COMP_MODE | CFG_COMP_POL | CFG_COMP_LAT |
        CFG_COMP_QUE_DIS;
    uint8_t reg_conv = REG_CONV;

    uint16_t config = default_config;
    config &= ~0x7000;
    config |= CFG_MUX_DIFF_0_1;

    if (start_conversion(bus_fd, config))
        return -1;

    usleep(10000);

    uint8_t i2c_data[2];
    if (i2c_write_read(bus_fd, &reg_conv, 1, i2c_data, 2, PSM_ADDRESS))
        return -1;
    int16_t raw_voltage = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);
    calculate_voltage(voltage, raw_voltage);

    config = default_config;
    config &= ~0x7000;
    config |= CFG_MUX_DIFF_2_3;

    if (start_conversion(bus_fd, config))
        return -1;
    usleep(10000);

    if (i2c_write_read(bus_fd, &reg_conv, 1, i2c_data, 2, PSM_ADDRESS))
        return -1;

    int16_t raw_current = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);
    calculate_current(current, raw_current);
    return 0;
}

int read_telemetry(int bus_fd, double* voltage, double* current) {
    uint8_t i2c_data[4];

    if (i2c_read(bus_fd, i2c_data, 4, TELEMETRY_ADDRESS)) {
        return -1;
    }

    int16_t raw_voltage = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);
    calculate_voltage(voltage, raw_voltage);

    int16_t raw_current = (int16_t)((i2c_data[2] << 8) | i2c_data[3]);
    calculate_current(current, raw_current);
    return 0;
}

int read_pressure(int bus_fd, double* pressure) {
    uint8_t write_data[3] = {MPRLS_REG, 0, 0};
    uint8_t i2c_data[4];

    if (i2c_write_read(bus_fd, write_data, 3, i2c_data, 4, MPRLS_ADDRESS)) {
        return -1;
    }
    // uint8_t status = i2c_data[0];
    int32_t pressure_counts =
        (int32_t)((i2c_data[1] << 16) | (i2c_data[2] << 8) | i2c_data[3]);
    calculate_pressure(pressure, pressure_counts);
    return 0;
}
