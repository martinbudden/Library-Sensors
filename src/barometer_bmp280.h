#pragma once

#include "barometer_base.h"
#include "bus_i2c.h"
#include "bus_spi.h"


class BarometerBmp280 : public BarometerBase {
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
    BarometerBmp280(uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::stm32_spi_pins_t& pins);
    BarometerBmp280(uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    BarometerBmp280(BusBase::bus_index_e i2c_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    BarometerBmp280(const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) : BarometerBmp280(BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    explicit BarometerBmp280(const BusI2c::stm32_i2c_pins_t& pins) : BarometerBmp280(pins, I2C_ADDRESS) {}

    BarometerBmp280(BusBase::bus_index_e i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address);
    BarometerBmp280(const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) : BarometerBmp280(BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    explicit BarometerBmp280(const BusI2c::i2c_pins_t& pins) : BarometerBmp280(pins, I2C_ADDRESS) {}
#endif
    virtual int init() override;
    virtual void read_temperature_and_pressure() override;
    virtual float calculate_altitude_meters(float pressure_pascals, float temperature_celsius) override;
private:
#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    int32_t _temperatureFine {};
    calibration_data_u _calibrationData {};
};
