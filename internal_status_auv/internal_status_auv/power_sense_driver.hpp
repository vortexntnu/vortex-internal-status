
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define MCP342X_ADDRESS 0x69

#define MCP342X_MODE_ONESHOT	0x00
#define MCP342X_MODE_CONTINUOUS	0x10


// Channel definitions
// MCP3421 & MCP3425 have only the one channel and ignore this param
// MCP3422, MCP3423, MCP3426 & MCP3427 have two channels and treat 3 & 4 as repeats of 1 & 2 respectively
// MCP3424 & MCP3428 have all four channels
#define	MCP342X_CHANNEL_1	0x00
#define	MCP342X_CHANNEL_2	0x20
#define	MCP342X_CHANNEL_3	0x40
#define	MCP342X_CHANNEL_4	0x60
#define	MCP342X_CHANNEL_MASK	0x60


// Sample size definitions - these also affect the sampling rate
// 12-bit has a max sample rate of 240sps
// 14-bit has a max sample rate of  60sps
// 16-bit has a max sample rate of  15sps
// 18-bit has a max sample rate of   3.75sps (MCP3421, MCP3422, MCP3423, MCP3424 only)
#define MCP342X_SIZE_12BIT	0x00
#define MCP342X_SIZE_14BIT	0x04
#define MCP342X_SIZE_16BIT	0x08
#define MCP342X_SIZE_18BIT	0x0C
#define MCP342X_SIZE_MASK	0x0C


// Programmable Gain definitions
#define MCP342X_GAIN_1X	0x00
#define MCP342X_GAIN_2X	0x01
#define MCP342X_GAIN_4X	0x02
#define MCP342X_GAIN_8X	0x03
#define MCP342X_GAIN_MASK 0x03


// /RDY bit definition
#define MCP342X_RDY	0x80



class PowerSenseDriver {
   public:
    PowerSenseDriver(std::uint8_t i2c_bus, std::uint8_t i2c_address, std::uint8_t resolution);

    bool ping_powersense();
    bool start_conversion();
    void read_voltage_powersense(std::vector<std::uint16_t> &voltage_array);
    void read_current_powersense(std::vector<std::uint16_t> &current_array);
  

   private:
    int bus_fd_;  ///< file descriptor for the I2C bus (integer >0 that uniquely
                  ///< identifies the device. -1 if it fails)

    int i2c_bus_;
    int i2c_address_;
    int resolution_;
    
};
