#pragma once

#include "BUS_I2C.h"
#include "BUS_SPI.h"
#include "IMU_Base.h"

class IMU_MPU6000 : public IMU_Base {
public:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_MPU6000_USE_SPI_BUS)
    // SPI constructor
    IMU_MPU6000(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins);
    IMU_MPU6000(axis_order_e axisOrder, uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::spi_pins_t& pins);
#else
    // I2C constructors
    IMU_MPU6000(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    IMU_MPU6000(axis_order_e axisOrder, const BUS_I2C::stm32_i2c_pins_t& pins, uint8_t I2C_address) : IMU_MPU6000(axisOrder, BUS_I2C::BUS_INDEX_0, pins, I2C_address) {}
    IMU_MPU6000(axis_order_e axisOrder, const BUS_I2C::stm32_i2c_pins_t& pins) : IMU_MPU6000(axisOrder, pins, I2C_ADDRESS) {}

    IMU_MPU6000(axis_order_e axisOrder, BUS_BASE::bus_index_e I2C_index, const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address);
    IMU_MPU6000(axis_order_e axisOrder, const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address) : IMU_MPU6000(axisOrder, BUS_I2C::BUS_INDEX_0, pins, I2C_address) {}
    IMU_MPU6000(axis_order_e axisOrder, const BUS_I2C::i2c_pins_t& pins) : IMU_MPU6000(axisOrder, pins, I2C_ADDRESS) {}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
    IMU_MPU6000(axis_order_e axisOrder, TwoWire& wire, const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address);
#endif
#endif
    virtual int init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* busMutex) override;
public:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
    static constexpr uint8_t I2C_ADDRESS_ALTERNATIVE = 0x69;
#pragma pack(push, 1)
    union mems_sensor_data_t {
        enum { DATA_SIZE = 6 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t x_h;
            uint8_t x_l;
            uint8_t y_h;
            uint8_t y_l;
            uint8_t z_h;
            uint8_t z_l;
        } value;
    };
    union acc_temperature_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        enum { DATA_SIZE = 14 };
        std::array<uint8_t, DATA_SIZE> data;
        struct value_t {
            uint8_t acc_x_h;
            uint8_t acc_x_l;
            uint8_t acc_y_h;
            uint8_t acc_y_l;
            uint8_t acc_z_h;
            uint8_t acc_z_l;
            uint8_t temperature_h;
            uint8_t temperature_l;
            uint8_t gyro_x_h;
            uint8_t gyro_x_l;
            uint8_t gyro_y_h;
            uint8_t gyro_y_l;
            uint8_t gyro_z_h;
            uint8_t gyro_z_l;
        } value;
    };
#pragma pack(pop)
    struct spi_acc_gyro_data_t {
        std::array<uint8_t, BUS_BASE::SPI_PRE_READ_BUFFER_SIZE> preReadBuffer; // buffer for the transmit byte sent as part of a read
        acc_temperature_gyro_data_t accGyro;
    };
    static_assert(sizeof(spi_acc_gyro_data_t) == sizeof(acc_temperature_gyro_data_t) + BUS_BASE::SPI_PRE_READ_BUFFER_SIZE);
public:
    virtual void setInterruptDriven() override;

    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual acc_gyro_rps_t readAccGyroRPS() override;
    virtual acc_gyro_rps_t getAccGyroRPS() const override;
private:
    xyz_t gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const;
    xyz_t accFromRaw(const mems_sensor_data_t::value_t& data) const;
    acc_gyro_rps_t accGyroRPSFromRaw(const acc_temperature_gyro_data_t::value_t& data) const;
private:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_MPU6000_USE_SPI_BUS)
    BUS_SPI _bus; //!< SPI bus interface
#else
    BUS_I2C _bus; //!< I2C bus interface
#endif
    spi_acc_gyro_data_t _spiAccGyroData {};
};
