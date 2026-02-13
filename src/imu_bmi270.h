#pragma once

#include "bus_i2c.h"
#include "bus_spi.h"
#include "imu_base.h"


class IMU_BMI270 : public IMU_Base {
public:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_BMI270_USE_SPI_BUS)
    // SPI constructors
    IMU_BMI270(uint8_t axis_order, uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::stm32_spi_pins_t& pins);
    IMU_BMI270(uint8_t axis_order, uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    IMU_BMI270(uint8_t axis_order, BusBase::bus_index_e i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address);
    IMU_BMI270(uint8_t axis_order, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) : IMU_BMI270(axis_order, BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    IMU_BMI270(uint8_t axis_order, const BusI2c::i2c_pins_t& pins) : IMU_BMI270(axis_order, pins, I2C_ADDRESS) {}

    IMU_BMI270(uint8_t axis_order, BusBase::bus_index_e i2c_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    IMU_BMI270(uint8_t axis_order, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) : IMU_BMI270(axis_order, BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    IMU_BMI270(uint8_t axis_order, const BusI2c::stm32_i2c_pins_t& pins) : IMU_BMI270(axis_order, pins, I2C_ADDRESS) {}
#endif
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
public:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x69;
#pragma pack(push, 1)
    union mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            int16_t x;
            int16_t y;
            int16_t z;
        } value;
    };
    union acc_gyro_data_t {
        enum { DATA_SIZE = 12 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
            int16_t gyro_x;
            int16_t gyro_y;
            int16_t gyro_z;
        } value;
    };
#pragma pack(pop)
    struct spi_acc_gyro_data_t {
        std::array<uint8_t, BusBase::SPI_PRE_READ_BUFFER_SIZE> preReadBuffer; // buffer for the transmit byte sent as part of a read
        acc_gyro_data_t accGyro;
    };
    static_assert(sizeof(spi_acc_gyro_data_t) == sizeof(acc_gyro_data_t) + BusBase::SPI_PRE_READ_BUFFER_SIZE);
public:
    void loadConfigurationData();
    virtual void set_interrupt_driven() override;

    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;

    virtual xyz_t read_gyro_rps() override;
    virtual xyz_t read_gyro_dps() override;
    virtual xyz_t read_acc() override;
    virtual acc_gyro_rps_t read_acc_gyro_rps() override;
    virtual acc_gyro_rps_t get_acc_gyro_rps() const override;
private:
    acc_gyro_rps_t acc_gyro_rpsFromRaw(const acc_gyro_data_t::value_t& data) const;
private:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_BMI270_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    spi_acc_gyro_data_t _spiAccGyroData {};
};
