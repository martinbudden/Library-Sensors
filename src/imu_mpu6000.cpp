#include "imu_mpu6000.h"

//#define LIBRARY_SENSORS_SERIAL_DEBUG
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
#include <HardwareSerial.h>
#else
#include <Arduino.h>
#endif
#endif
#include <cassert>

namespace { // use anonymous namespace to make items local to this translation unit

constexpr uint8_t REG_SAMPLE_RATE_DIVIDER = 0x19;
constexpr uint8_t REG_CONFIG                = 0x1A;
    constexpr uint8_t DLPF_CFG_260_HZ   = 0b00000000; // FS = 8kHz only
    constexpr uint8_t DLPF_CFG_184_HZ   = 0b00000001;
    constexpr uint8_t DLPF_CFG_94_HZ    = 0b00000010;
    constexpr uint8_t DLPF_CFG_44_HZ    = 0b00000011;
    constexpr uint8_t DLPF_CFG_21_HZ    = 0b00000100;
    constexpr uint8_t DLPF_CFG_10_HZ    = 0b00000101;
    constexpr uint8_t DLPF_CFG_5_HZ     = 0b00000110;
constexpr uint8_t REG_GYRO_CONFIG           = 0x1B;
    constexpr uint8_t GYRO_RANGE_250_DPS    = 0b00000000;
    constexpr uint8_t GYRO_RANGE_500_DPS    = 0b00001000;
    constexpr uint8_t GYRO_RANGE_1000_DPS   = 0b00010000;
    constexpr uint8_t GYRO_RANGE_2000_DPS   = 0b00011000;
constexpr uint8_t REG_ACCEL_CONFIG          = 0x1C;
    constexpr uint8_t ACCEL_RANGE_2G        = 0b00000000;
    constexpr uint8_t ACCEL_RANGE_4G        = 0b00001000;
    constexpr uint8_t ACCEL_RANGE_8G        = 0b00010000;
    constexpr uint8_t ACCEL_RANGE_16G       = 0b00011000;

constexpr uint8_t REG_INT_PIN_CONFIG        = 0x37;
    constexpr uint8_t INT_LEVEL_ACTIVE_LOW  = 0b10000000;
    constexpr uint8_t INT_LEVEL_ACTIVE_HIGH = 0;
    constexpr uint8_t INT_OPEN_DRAIN        = 0b01000000;
    constexpr uint8_t INT_PUSH_PULL         = 0;
    constexpr uint8_t INT_ENABLE_LATCHED    = 0b00100000;
    constexpr uint8_t INT_ENABLE_PULSE      = 0;
    constexpr uint8_t INT_CLEAR_READ_ANY    = 0b00010000; // cleared on any read
    constexpr uint8_t INT_CLEAR_READ_STATUS = 0; // cleared only by reading REG_INT_STATUS
    constexpr uint8_t FSYNCH_ACTIVE_LOW     = 0b00001000; // interrupt on FSYNCH pin active high
    constexpr uint8_t FSYNCH_ACTIVE_HIGH    = 0;
    constexpr uint8_t FSYNCH_INT_ENABLE     = 0b00000100; // enable interrup on FSYNCH pin
    constexpr uint8_t FSYNCH_INT_DISABLE    = 0;
constexpr uint8_t REG_INT_ENABLE            = 0x38;
    constexpr uint8_t DATA_READY_ENABLE     = 0b00000001;
constexpr uint8_t REG_INT_STATUS            = 0x3A;

constexpr uint8_t REG_ACCEL_XOUT_H          = 0x3B;
constexpr uint8_t REG_ACCEL_XOUT_L          = 0x3C;
constexpr uint8_t REG_ACCEL_YOUT_H          = 0x3D;
constexpr uint8_t REG_ACCEL_YOUT_L          = 0x3E;
constexpr uint8_t REG_ACCEL_ZOUT_H          = 0x3F;
constexpr uint8_t REG_ACCEL_ZOUT_L          = 0x40;
constexpr uint8_t REG_TEMP_OUT_H            = 0x41;
constexpr uint8_t REG_TEMP_OUT_L            = 0x42;
constexpr uint8_t REG_GYRO_XOUT_H           = 0x43;
constexpr uint8_t REG_GYRO_XOUT_L           = 0x44;
constexpr uint8_t REG_GYRO_YOUT_H           = 0x45;
constexpr uint8_t REG_GYRO_YOUT_L           = 0x46;
constexpr uint8_t REG_GYRO_ZOUT_H           = 0x47;
constexpr uint8_t REG_GYRO_ZOUT_L           = 0x48;

constexpr uint8_t REG_USER_CTRL             = 0x6A;
    constexpr uint8_t I2C_INTERFACE_DISABLED = 0b00010000;
constexpr uint8_t REG_PWR_MGMT_1            = 0x6B;
    constexpr uint8_t CLKSEL_INTERNAL_8_MHZ  = 0x00;
    constexpr uint8_t CLKSEL_PLL_X_AXIS_GYRO = 0x01;
    constexpr uint8_t CLKSEL_PLL_Y_AXIS_GYRO = 0x02;
    constexpr uint8_t CLKSEL_PLL_Z_AXIS_GYRO = 0x03;
    constexpr uint8_t CLKSEL_EXTERNAL_32768_HZ = 0x04;
    constexpr uint8_t CLKSEL_EXTERNAL_19p2_MHZ = 0x05;
constexpr uint8_t REG_PWR_MGMT_2            = 0x6C;

constexpr uint8_t REG_WHO_AM_I              = 0x75;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)

#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_MPU6000_USE_SPI_BUS)
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::stm32_spi_pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, uint32_t frequency, BusBase::bus_index_e SPI_index, const BusSpi::spi_pins_t& pins) :
    IMU_Base(axisOrder, _bus),
    _bus(frequency, SPI_index, pins)
{
}
#else
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, BusBase::bus_index_e I2C_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, BusBase::bus_index_e I2C_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
IMU_MPU6000::IMU_MPU6000(axis_order_e axisOrder, TwoWire& wire, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    IMU_Base(axisOrder, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif

int IMU_MPU6000::init(uint32_t targetOutputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* busMutex) // NOLINT(readability-function-cognitive-complexity)
{
#if defined(LIBRARY_SENSORS_IMU_BUS_MUTEX_REQUIRED)
    _busMutex = static_cast<SemaphoreHandle_t>(busMutex);
#else
    (void)busMutex;
#endif
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temperature_gyro_data_t) == acc_temperature_gyro_data_t::DATA_SIZE);

    // MSP compatible gyro and acc identifiers
    enum { MSP_GYRO_ID_MPU6050 = 2, MSP_GYRO_ID_MPU6000 = 4 };
    enum { MSP_ACC_ID_MPU6050 = 2, MSP_ACC_ID_MPU6000 = 3 };

    _gyroIdMSP = MSP_GYRO_ID_MPU6000;
    _accIdMSP = MSP_ACC_ID_MPU6000;

    _bus.setDeviceDataRegister(REG_ACCEL_XOUT_H, reinterpret_cast<uint8_t*>(&_spiAccGyroData), sizeof(_spiAccGyroData));

    // Disable Primary I2C Interface
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_MPU6000_USE_SPI_BUS)
    _bus.writeRegister(REG_USER_CTRL, I2C_INTERFACE_DISABLED);
    delayMs(15);
#endif

    // clock source: PLL with Z axis gyro reference
    _bus.writeRegister(REG_PWR_MGMT_1, CLKSEL_PLL_Z_AXIS_GYRO);
    delayMs(15);
    _bus.writeRegister(REG_PWR_MGMT_2, 0x00);
    delayMs(15);

    // Configure interrupts
    _bus.writeRegister(REG_INT_PIN_CONFIG, INT_LEVEL_ACTIVE_HIGH | INT_PUSH_PULL | INT_ENABLE_PULSE | INT_CLEAR_READ_ANY | FSYNCH_INT_DISABLE); // cppcheck-suppress [badBitmaskCheck, knownConditionTrueFalse]
    delayMs(15);
    _bus.writeRegister(REG_INT_ENABLE, DATA_READY_ENABLE);
    delayMs(15);

    // calculate the GYRO_SAMPLE_RATE_DIVIDER values to write to the REG_SAMPLE_RATE_DIVIDER register
    // sample rate = gyroscopeOutputRate / (1 + sampleRateDivider)
    const uint8_t GYRO_SAMPLE_RATE_DIVIDER =
        targetOutputDataRateHz == 0 ? 0 : // default to 8kHz
        targetOutputDataRateHz > 4000 ? 0 : // div by 1
        targetOutputDataRateHz > 2000 ? 1 : // div by 2
        targetOutputDataRateHz > 1000 ? 3 : // div by 4
        targetOutputDataRateHz > 500  ? 7 : // div by 8
        targetOutputDataRateHz > 250  ? 15 : 31;
    // report the value that was actually set
    _gyroSampleRateHz =
        GYRO_SAMPLE_RATE_DIVIDER  == 0 ? 8000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 1 ? 4000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 3 ? 2000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 7 ? 1000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 15 ? 500 : 125;

    _bus.writeRegister(REG_SAMPLE_RATE_DIVIDER, GYRO_SAMPLE_RATE_DIVIDER);
    delayMs(15);
    _bus.writeRegister(REG_CONFIG, DLPF_CFG_260_HZ); // DLPF_CFG_260_HZ sets base sample rate to 8kHz
    delayMs(15);

    // calculate the GYRO_RANGE bit values to write to the REG_GYRO_CONFIG0 register
    uint8_t GYRO_RANGE = 0;
    switch (gyroSensitivity) {
    case GYRO_FULL_SCALE_125_DPS:
        [[fallthrough]];
    case GYRO_FULL_SCALE_250_DPS:
        GYRO_RANGE = GYRO_RANGE_250_DPS;
        _gyroResolutionDPS = 250.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_500_DPS:
        GYRO_RANGE = GYRO_RANGE_500_DPS;
        _gyroResolutionDPS = 500.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_1000_DPS:
        GYRO_RANGE = GYRO_RANGE_1000_DPS;
        _gyroResolutionDPS = 1000.0F / 32768.0F;
        break;
    default:
        GYRO_RANGE = GYRO_RANGE_2000_DPS;
        _gyroResolutionDPS = 2000.0F / 32768.0F;
        break;
    }
    _gyroResolutionRPS = _gyroResolutionDPS * DEGREES_TO_RADIANS;
    _bus.writeRegister(REG_GYRO_CONFIG, GYRO_RANGE);
    delayMs(15);

    _accSampleRateHz = 1000;
    uint8_t ACCEL_RANGE = 0;
    switch (accSensitivity) {
    case ACC_FULL_SCALE_2G:
        ACCEL_RANGE = ACCEL_RANGE_2G;
        _accResolution = 2.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_4G:
        ACCEL_RANGE = ACCEL_RANGE_4G;
        _accResolution = 4.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_8G:
        ACCEL_RANGE = ACCEL_RANGE_8G;
        _accResolution = 8.0F / 32768.0F;
        break;
    default:
        ACCEL_RANGE = ACCEL_RANGE_16G;
        _accResolution = 16.0F / 32768.0F;
        break;
    }
    _bus.writeRegister(REG_ACCEL_CONFIG, ACCEL_RANGE);
    delayMs(15);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyroSampleRateHz);
}

void IMU_MPU6000::setInterruptDriven()
{
    // set interrupt level as configured in init()
    _bus.setInterruptDriven(BusBase::IRQ_EDGE_RISE);
}

IMU_Base::xyz_int32_t IMU_MPU6000::readGyroRaw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    busSemaphoreTake();
    _bus.readRegister(REG_ACCEL_XOUT_H, &gyro.data[0], sizeof(gyro));
    busSemaphoreGive();

    return xyz_int32_t {
        .x = static_cast<int16_t>((gyro.value.x_h << 8U) | gyro.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((gyro.value.y_h << 8U) | gyro.value.y_l),
        .z = static_cast<int16_t>((gyro.value.z_h << 8U) | gyro.value.z_l)
    };
}

IMU_Base::xyz_int32_t IMU_MPU6000::readAccRaw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    busSemaphoreTake();
    _bus.readRegister(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    busSemaphoreGive();

    return xyz_int32_t {
        .x = static_cast<int16_t>((acc.value.x_h << 8U) | acc.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((acc.value.y_h << 8U) | acc.value.y_l),
        .z = static_cast<int16_t>((acc.value.z_h << 8U) | acc.value.z_l)
    };
}

xyz_t IMU_MPU6000::readGyroRPS()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    busSemaphoreTake(_busMutex);
    _bus.readRegister(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    busSemaphoreGive(_busMutex);

    return gyroRPS_FromRaw(gyro.value);
}

xyz_t IMU_MPU6000::readGyroDPS()
{
    return readGyroRPS() * RADIANS_TO_DEGREES;
}

xyz_t IMU_MPU6000::readAcc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    busSemaphoreTake(_busMutex);
    _bus.readRegister(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    busSemaphoreGive(_busMutex);

    return accFromRaw(acc.value);
}

FAST_CODE acc_gyro_rps_t IMU_MPU6000::readAccGyroRPS()
{
    busSemaphoreTake(_busMutex);
    _bus.readRegister(REG_ACCEL_XOUT_H, &_spiAccGyroData.accGyro.data[0], sizeof(_spiAccGyroData.accGyro));
    //_bus.readDeviceData();
    busSemaphoreGive(_busMutex);

    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE acc_gyro_rps_t IMU_MPU6000::getAccGyroRPS() const
{
    return accGyroRPSFromRaw(_spiAccGyroData.accGyro.value);
}

xyz_t IMU_MPU6000::gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const
{
    // static cast to int16_t to sign extend the 8 bit values
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyroResolutionRPS - _gyroOffset.y,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyroResolutionRPS - _gyroOffset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyroResolutionRPS - _gyroOffset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyroResolutionRPS - _gyroOffset.y,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyroResolutionRPS - _gyroOffset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyroResolutionRPS - _gyroOffset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyroResolutionRPS - _gyroOffset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyroResolutionRPS - _gyroOffset.z,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyroResolutionRPS - _gyroOffset.y
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyroResolutionRPS - _gyroOffset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyroResolutionRPS - _gyroOffset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyroResolutionRPS - _gyroOffset.z
    };
#else
    const xyz_t gyro {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyroResolutionRPS - _gyroOffset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyroResolutionRPS - _gyroOffset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyroResolutionRPS - _gyroOffset.z
    };
    return mapAxes(gyro);
#endif
}

xyz_t IMU_MPU6000::accFromRaw(const mems_sensor_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _accResolution - _accOffset.y,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _accResolution - _accOffset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _accResolution - _accOffset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _accResolution - _accOffset.y,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _accResolution - _accOffset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _accResolution - _accOffset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _accResolution - _accOffset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _accResolution - _accOffset.z,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _accResolution - _accOffset.y
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _accResolution - _accOffset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _accResolution - _accOffset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _accResolution - _accOffset.z
    };
#else
    const xyz_t acc = {
        .x = static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _accResolution - _accOffset.x,
        .y = static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _accResolution - _accOffset.y,
        .z = static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _accResolution - _accOffset.z
    };
    return mapAxes(acc);
#endif
}

acc_gyro_rps_t IMU_MPU6000::accGyroRPSFromRaw(const acc_temperature_gyro_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z,
            .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z,
            .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y
        }
    };
#else
    // Axis order mapping done at run-time
    // return values are fully calculated for each `case` to allow eliding copy on return (return value optimization, RVO)
    // speed optimization is more important than size optimization here
    switch (_axisOrder) {
    case XPOS_YPOS_ZPOS: // NOLINT(bugprone-branch-clone)
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
            },
            .acc = {
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
            }
        };
    case YNEG_XPOS_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
            },
            .acc = {
                .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
            }
        };
    case XNEG_YNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
            },
            .acc = {
                .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
            }
        };
    case YPOS_XNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
            },
            .acc = {
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
            }
        };
    case XPOS_ZPOS_YNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.y,
                .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.z
            },
            .acc = {
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.y,
                .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.z
            }
        };
    default:
        return acc_gyro_rps_t {
            .gyroRPS = mapAxes({
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
            }),
            .acc = mapAxes({
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _accResolution - _accOffset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _accResolution - _accOffset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _accResolution - _accOffset.z
            })
        };
        break;
    } // end switch
#endif
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)
