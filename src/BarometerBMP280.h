#pragma once

#include "BarometerBase.h"
#include "BUS_I2C.h"
#include "BUS_SPI.h"


class BarometerBMP280 : public BarometerBase {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x76;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x77;
    static constexpr uint8_t CHIP_ID = 0x58;
    enum { MAX_SPI_FREQUENCY_HZ = 10000000 };
public:
    union calibration_data_u { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 24 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            // temperature calibration
            uint16_t dig_T1;
            int16_t dig_T2;
            int16_t dig_T3;
            // pressure calibration
            uint16_t dig_P1;
            int16_t dig_P2;
            int16_t dig_P3;
            int16_t dig_P4;
            int16_t dig_P5;
            int16_t dig_P6;
            int16_t dig_P7;
            int16_t dig_P8;
            int16_t dig_P9;
        } value;
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
public:
#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
    // SPI constructors
    BarometerBMP280(uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::stm32_spi_pins_t& pins);
    BarometerBMP280(uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    BarometerBMP280(BusBase::bus_index_e I2C_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    BarometerBMP280(const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) : BarometerBMP280(BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    explicit BarometerBMP280(const BusI2c::stm32_i2c_pins_t& pins) : BarometerBMP280(pins, I2C_ADDRESS) {}

    BarometerBMP280(BusBase::bus_index_e I2C_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address);
    BarometerBMP280(const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) : BarometerBMP280(BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    explicit BarometerBMP280(const BusI2c::i2c_pins_t& pins) : BarometerBMP280(pins, I2C_ADDRESS) {}
#endif
    virtual int init() override;
    virtual void readTemperatureAndPressure() override;
    virtual float calculateAltitudeMeters(float pressurePascals, float temperatureCelsius) override;
private:
#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    int32_t _temperatureFine {};
    calibration_data_u _calibrationData {};
};
