#pragma once

#include "BUS_I2C.h"
#include "BUS_SPI.h"
#include "IMU_Base.h"


class IMU_BMI270 : public IMU_Base {
public:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_BMI270_USE_SPI_BUS)
    // SPI constructors
    IMU_BMI270(axis_order_e axisOrder, uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::stm32_spi_pins_t& pins);
    IMU_BMI270(axis_order_e axisOrder, uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    IMU_BMI270(axis_order_e axisOrder, BusBase::bus_index_e I2C_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address);
    IMU_BMI270(axis_order_e axisOrder, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) : IMU_BMI270(axisOrder, BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    IMU_BMI270(axis_order_e axisOrder, const BusI2c::i2c_pins_t& pins) : IMU_BMI270(axisOrder, pins, I2C_ADDRESS) {}

    IMU_BMI270(axis_order_e axisOrder, BusBase::bus_index_e I2C_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address);
    IMU_BMI270(axis_order_e axisOrder, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) : IMU_BMI270(axisOrder, BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    IMU_BMI270(axis_order_e axisOrder, const BusI2c::stm32_i2c_pins_t& pins) : IMU_BMI270(axisOrder, pins, I2C_ADDRESS) {}
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
    virtual void setInterruptDriven() override;

    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;

    virtual xyz_t readGyroRPS() override;
    virtual xyz_t readGyroDPS() override;
    virtual xyz_t readAcc() override;
    virtual acc_gyro_rps_t readAccGyroRPS() override;
    virtual acc_gyro_rps_t getAccGyroRPS() const override;
private:
    acc_gyro_rps_t accGyroRPSFromRaw(const acc_gyro_data_t::value_t& data) const;
private:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_BMI270_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    spi_acc_gyro_data_t _spiAccGyroData {};
};
