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

#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuMpu6000_USE_SPI_BUS)
ImuMpu6000::ImuMpu6000(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
ImuMpu6000::ImuMpu6000(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
#else
ImuMpu6000::ImuMpu6000(uint8_t axis_order, uint8_t i2c_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, i2c_index, pins)
{
}
ImuMpu6000::ImuMpu6000(uint8_t axis_order, uint8_t i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, i2c_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
ImuMpu6000::ImuMpu6000(uint8_t axis_order, TwoWire& wire, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif

int ImuMpu6000::init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) // NOLINT(readability-function-cognitive-complexity)
{
#if defined(LIBRARY_SENSORS_IMU_BUS_MUTEX_REQUIRED)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    (void)bus_mutex;
#endif
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_temperature_gyro_data_t) == acc_temperature_gyro_data_t::DATA_SIZE);

    // MSP compatible gyro and acc identifiers
    enum { MSP_GYRO_ID_MPU6050 = 2, MSP_GYRO_ID_MPU6000 = 4 };
    enum { MSP_ACC_ID_MPU6050 = 2, MSP_ACC_ID_MPU6000 = 3 };

    _gyro_id_msp = MSP_GYRO_ID_MPU6000;
    _acc_id_msp = MSP_ACC_ID_MPU6000;

    _bus.set_device_data_register(REG_ACCEL_XOUT_H, reinterpret_cast<uint8_t*>(&_spiAccGyroData), sizeof(_spiAccGyroData));

    // Disable Primary I2C Interface
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuMpu6000_USE_SPI_BUS)
    _bus.write_register(REG_USER_CTRL, I2C_INTERFACE_DISABLED);
    delay_ms(15);
#endif

    // clock source: PLL with Z axis gyro reference
    _bus.write_register(REG_PWR_MGMT_1, CLKSEL_PLL_Z_AXIS_GYRO);
    delay_ms(15);
    _bus.write_register(REG_PWR_MGMT_2, 0x00);
    delay_ms(15);

    // Configure interrupts
    _bus.write_register(REG_INT_PIN_CONFIG, INT_LEVEL_ACTIVE_HIGH | INT_PUSH_PULL | INT_ENABLE_PULSE | INT_CLEAR_READ_ANY | FSYNCH_INT_DISABLE); // cppcheck-suppress [badBitmaskCheck, knownConditionTrueFalse]
    delay_ms(15);
    _bus.write_register(REG_INT_ENABLE, DATA_READY_ENABLE);
    delay_ms(15);

    // calculate the GYRO_SAMPLE_RATE_DIVIDER values to write to the REG_SAMPLE_RATE_DIVIDER register
    // sample rate = gyroscopeOutputRate / (1 + sampleRateDivider)
    const uint8_t GYRO_SAMPLE_RATE_DIVIDER =
        target_output_data_rate_hz == 0 ? 0 : // default to 8kHz
        target_output_data_rate_hz > 4000 ? 0 : // div by 1
        target_output_data_rate_hz > 2000 ? 1 : // div by 2
        target_output_data_rate_hz > 1000 ? 3 : // div by 4
        target_output_data_rate_hz > 500  ? 7 : // div by 8
        target_output_data_rate_hz > 250  ? 15 : 31;
    // report the value that was actually set
    _gyro_sample_rate_hz =
        GYRO_SAMPLE_RATE_DIVIDER  == 0 ? 8000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 1 ? 4000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 3 ? 2000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 7 ? 1000 :
        GYRO_SAMPLE_RATE_DIVIDER  == 15 ? 500 : 125;

    _bus.write_register(REG_SAMPLE_RATE_DIVIDER, GYRO_SAMPLE_RATE_DIVIDER);
    delay_ms(15);
    _bus.write_register(REG_CONFIG, DLPF_CFG_260_HZ); // DLPF_CFG_260_HZ sets base sample rate to 8kHz
    delay_ms(15);

    // calculate the GYRO_RANGE bit values to write to the REG_GYRO_CONFIG0 register
    uint8_t GYRO_RANGE = 0;
    switch (gyro_sensitivity) {
    case GYRO_FULL_SCALE_125_DPS:
        [[fallthrough]];
    case GYRO_FULL_SCALE_250_DPS:
        GYRO_RANGE = GYRO_RANGE_250_DPS;
        _gyro_resolution_dps = 250.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_500_DPS:
        GYRO_RANGE = GYRO_RANGE_500_DPS;
        _gyro_resolution_dps = 500.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_1000_DPS:
        GYRO_RANGE = GYRO_RANGE_1000_DPS;
        _gyro_resolution_dps = 1000.0F / 32768.0F;
        break;
    default:
        GYRO_RANGE = GYRO_RANGE_2000_DPS;
        _gyro_resolution_dps = 2000.0F / 32768.0F;
        break;
    }
    _gyro_resolution_rps = _gyro_resolution_dps * DEGREES_TO_RADIANS;
    _bus.write_register(REG_GYRO_CONFIG, GYRO_RANGE);
    delay_ms(15);

    _acc_sample_rate_hz = 1000;
    uint8_t ACCEL_RANGE = 0;
    switch (acc_sensitivity) {
    case ACC_FULL_SCALE_2G:
        ACCEL_RANGE = ACCEL_RANGE_2G;
        _acc_resolution = 2.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_4G:
        ACCEL_RANGE = ACCEL_RANGE_4G;
        _acc_resolution = 4.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_8G:
        ACCEL_RANGE = ACCEL_RANGE_8G;
        _acc_resolution = 8.0F / 32768.0F;
        break;
    default:
        ACCEL_RANGE = ACCEL_RANGE_16G;
        _acc_resolution = 16.0F / 32768.0F;
        break;
    }
    _bus.write_register(REG_ACCEL_CONFIG, ACCEL_RANGE);
    delay_ms(15);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyro_sample_rate_hz);
}

void ImuMpu6000::set_interrupt_driven()
{
    // set interrupt level as configured in init()
    _bus.set_interrupt_driven(BusBase::IRQ_EDGE_RISE);
}

ImuBase::xyz_int32_t ImuMpu6000::read_gyro_raw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take();
    _bus.read_register(REG_ACCEL_XOUT_H, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give();

    return xyz_int32_t {
        .x = static_cast<int16_t>((gyro.value.x_h << 8U) | gyro.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((gyro.value.y_h << 8U) | gyro.value.y_l),
        .z = static_cast<int16_t>((gyro.value.z_h << 8U) | gyro.value.z_l)
    };
}

ImuBase::xyz_int32_t ImuMpu6000::read_acc_raw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take();
    _bus.read_register(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    bus_semaphore_give();

    return xyz_int32_t {
        .x = static_cast<int16_t>((acc.value.x_h << 8U) | acc.value.x_l), // static cast to int16_t to sign extend the 8 bit values
        .y = static_cast<int16_t>((acc.value.y_h << 8U) | acc.value.y_l),
        .z = static_cast<int16_t>((acc.value.z_h << 8U) | acc.value.z_l)
    };
}

xyz_t ImuMpu6000::read_gyro_rps()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_GYRO_XOUT_H, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give(_bus_mutex);

    return gyroRPS_FromRaw(gyro.value);
}

xyz_t ImuMpu6000::read_gyro_dps()
{
    return read_gyro_rps() * RADIANS_TO_DEGREES;
}

xyz_t ImuMpu6000::read_acc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_XOUT_H, &acc.data[0], sizeof(acc));
    bus_semaphore_give(_bus_mutex);

    return accFromRaw(acc.value);
}

FAST_CODE acc_gyro_rps_t ImuMpu6000::read_acc_gyro_rps()
{
    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_XOUT_H, &_spiAccGyroData.accGyro.data[0], sizeof(_spiAccGyroData.accGyro));
    //_bus.read_device_data();
    bus_semaphore_give(_bus_mutex);

    return acc_gyro_rpsFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE acc_gyro_rps_t ImuMpu6000::get_acc_gyro_rps() const
{
    return acc_gyro_rpsFromRaw(_spiAccGyroData.accGyro.value);
}

xyz_t ImuMpu6000::gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const
{
    // static cast to int16_t to sign extend the 8 bit values
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
#else
    const xyz_t gyro {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _gyro_resolution_rps - _gyro_offset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _gyro_resolution_rps - _gyro_offset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _gyro_resolution_rps - _gyro_offset.z
    };
    return map_axes(gyro);
#endif
}

xyz_t ImuMpu6000::accFromRaw(const mems_sensor_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .y =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .y = -static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z,
        .z = -static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =  static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .y =  static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .z =  static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
#else
    const xyz_t acc = {
        .x = static_cast<float>(static_cast<int16_t>((data.x_h << 8U) | data.x_l)) * _acc_resolution - _acc_offset.x,
        .y = static_cast<float>(static_cast<int16_t>((data.y_h << 8U) | data.y_l)) * _acc_resolution - _acc_offset.y,
        .z = static_cast<float>(static_cast<int16_t>((data.z_h << 8U) | data.z_l)) * _acc_resolution - _acc_offset.z
    };
    return map_axes(acc);
#endif
}

acc_gyro_rps_t ImuMpu6000::acc_gyro_rpsFromRaw(const acc_temperature_gyro_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z,
            .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y
        },
        .acc = {
            .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z,
            .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y
        }
    };
#else
    // Axis order mapping done at run-time
    // return values are fully calculated for each `case` to allow eliding copy on return (return value optimization, RVO)
    // speed optimization is more important than size optimization here
    switch (_axis_order) {
    case XPOS_YPOS_ZPOS: // NOLINT(bugprone-branch-clone)
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case YNEG_XPOS_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case XNEG_YNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case YPOS_XNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.x,
                .y = -static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            }
        };
    case XPOS_ZPOS_YNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z = -static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.y,
                .z = -static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.z
            }
        };
    default:
        return acc_gyro_rps_t {
            .gyroRPS = map_axes({
                .x =  static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8U) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8U) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8U) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
            }),
            .acc = map_axes({
                .x =  static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8U) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
                .y =  static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8U) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
                .z =  static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8U) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
            })
        };
        break;
    } // end switch
#endif
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,hicpp-signed-bitwise)
