#pragma once

#include "BarometerBase.h"
#include "BUS_I2C.h"
#include "BUS_SPI.h"


class BarometerQMP6988 : public BarometerBase {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x56;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x70;
    static constexpr uint8_t CHIP_ID = 0x58;
    enum { MAX_SPI_FREQUENCY_HZ = 10000000 };
public:
    enum { CALIBRATION_DATA_SIZE = 25 };
    union calibration_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        int32_t a0;
        int16_t a1;
        int16_t a2;
        int32_t b00;
        int16_t bt1;
        int16_t bt2;
        int16_t bp1;
        int16_t b11;
        int16_t bp2;
        int16_t b12;
        int16_t b21;
        int16_t bp3;
    };
    union pressure_temperature_data_u { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 6 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t pressure_msb;
            uint8_t pressure_lsb;
            uint8_t pressure_xlsb;
            uint8_t temperature_msb;
            uint8_t temperature_lsb;
            uint8_t temperature_xlsb;
        } value;
    };
    struct ik_data_t {
        int32_t a0;
        int32_t b00;
        int32_t a1;
        int32_t a2;
        int64_t bt1;
        int64_t bt2;
        int64_t bp1;
        int64_t b11;
        int64_t bp2;
        int64_t b12;
        int64_t b21;
        int64_t bp3;
    };
public:
#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
    // SPI constructors
    BarometerQMP6988(uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::stm32_spi_pins_t& pins);
    BarometerQMP6988(uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    BarometerQMP6988(BusBase::bus_index_e I2C_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    BarometerQMP6988(const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) : BarometerQMP6988(BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    explicit BarometerQMP6988(const BusI2c::stm32_i2c_pins_t& pins) : BarometerQMP6988(pins, I2C_ADDRESS) {}

    BarometerQMP6988(BusBase::bus_index_e I2C_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address);
    BarometerQMP6988(const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) : BarometerQMP6988(BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    explicit BarometerQMP6988(const BusI2c::i2c_pins_t& pins) : BarometerQMP6988(pins, I2C_ADDRESS) {}
#endif
    virtual int init() override;
    virtual void readTemperatureAndPressure() override;
    virtual float calculateAltitudeMeters(float pressurePascals, float temperatureCelsius) override;

    void readCalibrationData();
    int16_t convertTemperature(int32_t dt) const;
    int32_t convertPressure(int32_t dp, int16_t tx) const;
private:
#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    ik_data_t _ikData {};
};
