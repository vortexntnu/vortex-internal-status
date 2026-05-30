#ifndef EXTERNAL_PRESSURE_HPP_
#define EXTERNAL_PRESSURE_HPP_

#include <cstdint>
#include <string>

namespace PressureUnit {
constexpr float Pa = 100.0f;
constexpr float hPa = 1.0f;
constexpr float kPa = 0.1f;
constexpr float mbar = 1.0f;
constexpr float bar = 0.001f;
constexpr float atm = 0.000986923f;
constexpr float Torr = 0.750062f;
constexpr float psi = 0.0145037738f;
}  // namespace PressureUnit

class MS5837 {
   public:
    enum Model : uint8_t {
        MODEL_UNRECOGNISED = 0,
        MODEL_02BA = 1,
        MODEL_30BA = 2
    };
    // namespace PressureUnit

    explicit MS5837(const std::string& i2c_device = "/dev/i2c-1",
                    uint8_t address = 0x76);
    ~MS5837();

    bool init();
    bool read();

    void setModel(Model model);
    Model getModel() const;

    void setFluidDensity(float density);
    void setAtmosphericPressure(float pressure_pa);
    void setGravity(float gravity);

    float pressure(float conversion = PressureUnit::Pa) const;
    float temperature() const;
    float depth() const;
    float altitude() const;

    bool isInitialized() const { return initialized_; }

   private:
    static constexpr uint8_t RESET = 0x1E;
    static constexpr uint8_t ADC_READ = 0x00;
    static constexpr uint8_t PROM_READ = 0xA0;
    static constexpr uint8_t CONVERT_D1_8192 = 0x4A;
    static constexpr uint8_t CONVERT_D2_8192 = 0x5A;

    static constexpr uint16_t MODEL_02BA_MAX_SENSITIVITY = 49000U;
    static constexpr uint16_t MODEL_02BA_30BA_SEPARATION = 37000U;
    static constexpr uint16_t MODEL_30BA_MIN_SENSITIVITY = 26000U;

    bool openBus();
    void closeBus();

    bool reset();
    bool readPROM();
    void calculate();

    bool writeByte(uint8_t value);
    bool writeRead(uint8_t reg, uint8_t* rx, uint16_t len);

    static uint8_t crc4(uint16_t prom[8]);

   private:
    std::string i2c_device_;
    uint8_t address_;
    int fd_;

    uint16_t C_[8]{};
    uint32_t D1_pres_{0};
    uint32_t D2_temp_{0};

    int32_t TEMP_{0};
    int32_t P_{0};

    float fluid_density_{1029.0f};
    float atmospheric_pressure_{101300.0f};
    float gravity_{9.80665f};
    Model model_{MODEL_UNRECOGNISED};
    bool initialized_{false};
};

#endif  // !EXTERNAL_PRESSURE_HPP_
