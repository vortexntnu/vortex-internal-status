
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
#define MPRLS_REG     0xAA


#define PSM_ADDRESS 0x48
#define REG_CONV    0x00
#define REG_CFG     0x01

#define CFG_OS_SINGLE    0x8000
#define CFG_MUX_DIFF_0_1 0x0000
#define CFG_MUX_DIFF_2_3 0x3000
#define CFG_PGA_6_144V   0x0000
#define CFG_MODE_SINGLE  0x0100
#define CFG_DR_128SPS    0x0080
#define CFG_COMP_MODE    0x0010
#define CFG_COMP_POL     0x0008
#define CFG_COMP_LAT     0x0004
#define CFG_COMP_QUE_DIS 0x0003

#define VOLTAGE_SCALE       11.236
#define VOLTAGE_RANGE       6.144
#define DIODE_LOSS          0.8
// #define CURRENT_OFFSET      0.595
#define CURRENT_OFFSET      0.6004
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

int i2c_init();
int read_psm_measurements(double* voltage, double* current);
int read_pressure(double* pressure);

#ifdef __cplusplus
}
#endif  // __cplusplus
#endif  //  PSM_ORIN_DRIVER_H
