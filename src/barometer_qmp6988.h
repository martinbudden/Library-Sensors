#pragma once

#include "barometer_base.h"
#include "bus_i2c.h"
#include "bus_spi.h"


class BarometerQmp6988 : public BarometerBase {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x56;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x70;
    static constexpr uint8_t CHIP_ID = 0x58;
    static constexpr uint32_t MAX_SPI_FREQUENCY_HZ = 10000000;
public:
    static constexpr size_t CALIBRATION_DATA_SIZE = 25;
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
        static constexpr size_t DATA_SIZE = 6;
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
    BarometerQmp6988(uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins);
    BarometerQmp6988(uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    BarometerQmp6988(uint8_t i2c_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t i2c_address);
    BarometerQmp6988(const BusI2c::stm32_i2c_pins_t& pins, uint8_t i2c_address) : BarometerQmp6988(BusI2c::BUS_INDEX_0, pins, i2c_address) {}
    explicit BarometerQmp6988(const BusI2c::stm32_i2c_pins_t& pins) : BarometerQmp6988(pins, I2C_ADDRESS) {}

    BarometerQmp6988(uint8_t i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t i2c_address);
    BarometerQmp6988(const BusI2c::i2c_pins_t& pins, uint8_t i2c_address) : BarometerQmp6988(BusI2c::BUS_INDEX_0, pins, i2c_address) {}
    explicit BarometerQmp6988(const BusI2c::i2c_pins_t& pins) : BarometerQmp6988(pins, I2C_ADDRESS) {}
#endif
    virtual int init() override;
    virtual void read_temperature_and_pressure() override;
    virtual float calculate_altitude_meters(float pressure_pascals, float temperature_celsius) override;

    void read_calibration_data();
    int16_t convert_temperature(int32_t dt) const;
    int32_t convert_pressure(int32_t dp, int16_t tx) const;
private:
#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    ik_data_t _ik_data {};
};
