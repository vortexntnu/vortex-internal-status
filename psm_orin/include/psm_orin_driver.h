
#ifndef PSM_ORIN_DRIVER_H
#define PSM_ORIN_DRIVER_H

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define MPRLS_ADDRESS 0x18
#define MPRLS_REG 0xAA

#define TELEMETRY_ADDRESS 0x69
#define MPRLS_REG 0xAA

#define PSM_ADDRESS 0x48
#define REG_CONV 0x00
#define REG_CFG 0x01

#define CFG_OS_SINGLE 0x8000
#define CFG_MUX_DIFF_0_1 0x0000
#define CFG_MUX_DIFF_2_3 0x3000
#define CFG_PGA_6_144V 0x0000
#define CFG_MODE_SINGLE 0x0100
#define CFG_DR_128SPS 0x0080
#define CFG_COMP_MODE 0x0010
#define CFG_COMP_POL 0x0008
#define CFG_COMP_LAT 0x0004
#define CFG_COMP_QUE_DIS 0x0003

#define VOLTAGE_SCALE 11.236
#define VOLTAGE_RANGE 6.144
#define DIODE_LOSS 0.8
// #define CURRENT_OFFSET      0.595
#define CURRENT_OFFSET 0.6004
#define CURRENT_SENSITIVITY 0.0255

#define COUNTS_MIN 0x066666
#define COUNTS_MAX 0x399999
#define PRESSURE_MIN 0
#define PRESSURE_MAX 250

#define I2C_BUS 7
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define I2C_DEVICE_PATH "/dev/i2c-" TOSTRING(I2C_BUS)

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 *@brief i2c initialize
 *@param bus_fd pointer to socket fd
 *@param address PSM address
 *@return -1 on fail
           0 on success
 */
int i2c_init(int* bus_fd, uint8_t address);

/**
 *@brief Reads current and voltage for the PSM
 *@param voltage pointer to double holding voltage
 *@param current pointer to double holding current
 *@return -1 on failure and 0 on success
 */
int read_psm_measurements(int bus_fd, double* voltage, double* current);

/**
 *@brief Reads current and voltage for the MPRLS
 *@param pressure pointer to double holding pressure
 *@return -1 on failure and 0 on success
 */
int read_pressure(int bus_fd, double* pressure);

/**
 *@brief Reads current and voltage for the PSM
 *@param voltage pointer to double holding voltage
 *@param current pointer to double holding current
 *@return -1 on failure and 0 on success
 */
int read_telemetry(int bus_fd, double* voltage, double* current);
/**
 *@brief closes i2c socket
 *@param bus_fd pointer to socket fd
 */
void i2c_close(int* bus_fd);

static inline void calculate_voltage(double* voltage, int16_t raw_voltage) {
    *voltage =
        ((raw_voltage * VOLTAGE_RANGE) / 32768.0) * VOLTAGE_SCALE + DIODE_LOSS;
}

static inline void calculate_current(double* current, int16_t raw_current) {
    *current = (CURRENT_OFFSET - ((raw_current * VOLTAGE_RANGE) / 32768.0)) /
               CURRENT_SENSITIVITY;
}

static inline void calculate_pressure(double* pressure,
                                      int32_t pressure_counts) {
    double scale =
        (PRESSURE_MAX - PRESSURE_MIN) / (double)(COUNTS_MAX - COUNTS_MIN);
    *pressure = (pressure_counts - COUNTS_MIN) * scale + PRESSURE_MIN;
}

#ifdef __cplusplus
}
#endif  // __cplusplus
#endif  //  PSM_ORIN_DRIVER_H
