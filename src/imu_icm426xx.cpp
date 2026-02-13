#include "imu_icm426xx.h"
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

constexpr uint8_t REG_SENSOR_CONFIG0        = 0x03;

constexpr uint8_t REG_DEVICE_CONFIG         = 0x11;
    constexpr uint8_t DEVICE_CONFIG_DEFAULT         = 0b00000000;
constexpr uint8_t REG_INT_CONFIG            = 0x14;
    constexpr uint8_t INT1_MODE_LATCHED             = 0b00000100;
    constexpr uint8_t INT1_MODE_PULSED              = 0b00000000;
    constexpr uint8_t INT1_DRIVE_CIRCUIT_PUSH_PULL  = 0b00000010;
    constexpr uint8_t INT1_POLARITY_ACTIVE_HIGH     = 0b00000001;
constexpr uint8_t REG_FIFO_CONFIG           = 0x16;

constexpr uint8_t REG_TEMP_DATA1            = 0x1D;
constexpr uint8_t REG_TEMP_DATA0            = 0x1E;
constexpr uint8_t REG_ACCEL_DATA_X1         = 0x1F;
constexpr uint8_t REG_ACCEL_DATA_X0         = 0x20;
constexpr uint8_t REG_ACCEL_DATA_Y1         = 0x21;
constexpr uint8_t REG_ACCEL_DATA_Y0         = 0x22;
constexpr uint8_t REG_ACCEL_DATA_Z1         = 0x23;
constexpr uint8_t REG_ACCEL_DATA_Z0         = 0x24;
constexpr uint8_t REG_GYRO_DATA_X1          = 0x25;
constexpr uint8_t REG_GYRO_DATA_X0          = 0x26;
constexpr uint8_t REG_GYRO_DATA_Y1          = 0x27;
constexpr uint8_t REG_GYRO_DATA_Y0          = 0x28;
constexpr uint8_t REG_GYRO_DATA_Z1          = 0x29;
constexpr uint8_t REG_GYRO_DATA_Z0          = 0x2A;

constexpr uint8_t REG_TMST_FSYNCH           = 0x2B;
constexpr uint8_t REG_TMST_FSYNCHL          = 0x2C;
constexpr uint8_t REG_INT_STATUS            = 0x2D;

constexpr uint8_t REG_FIFO_COUNTH           = 0x2E;
constexpr uint8_t REG_FIFO_COUNTL           = 0x2F;
constexpr uint8_t REG_FIFO_DATA             = 0x30;

constexpr uint8_t REG_APEX_DATA0            = 0x31;
constexpr uint8_t REG_APEX_DATA1            = 0x33;
constexpr uint8_t REG_APEX_DATA2            = 0x34;
constexpr uint8_t REG_APEX_DATA3            = 0x34;
constexpr uint8_t REG_APEX_DATA4            = 0x35;
constexpr uint8_t REG_APEX_DATA5            = 0x36;

constexpr uint8_t REG_INT_STATUS2           = 0x37;
constexpr uint8_t REG_INT_STATUS3           = 0x38;

constexpr uint8_t REG_SIGNAL_PATH_RESET     = 0x4B;
constexpr uint8_t REG_INTF_CONFIG0          = 0x4C;
    constexpr uint8_t SENSOR_DATA_BIG_ENDIAN    = 0b00010000; // default
    constexpr uint8_t SENSOR_DATA_LITTLE_ENDIAN = 0b00000000; // datasheet gives very little information on how this works
    constexpr uint8_t UI_SIFS_CFG_DISABLE_I2C   = 0b00000011;
constexpr uint8_t REG_INTF_CONFIG1          = 0x4D;
    constexpr uint8_t AFSR_DISABLE          = 0x40;

constexpr uint8_t REG_PWR_MGMT0             = 0x4E;
    constexpr uint8_t PWR_OFF               = 0x00;
    constexpr uint8_t PWR_TEMP_ENABLED      = 0b00000000;
    constexpr uint8_t PWR_GYRO_LOW_NOISE    = 0b00001100;
    constexpr uint8_t PWR_ACCEL_LOW_NOISE   = 0b00000011;

constexpr uint8_t REG_GYRO_CONFIG0          = 0x4F;
    constexpr uint8_t GYRO_RANGE_2000_DPS   = 0b00000000;
    constexpr uint8_t GYRO_RANGE_1000_DPS   = 0b00100000;
    constexpr uint8_t GYRO_RANGE_500_DPS    = 0b01000000;
    constexpr uint8_t GYRO_RANGE_250_DPS    = 0b01100000;
    constexpr uint8_t GYRO_RANGE_125_DPS    = 0b10000000;
    constexpr uint8_t GYRO_RANGE_62p5_DPS   = 0b10100000;
    constexpr uint8_t GYRO_RANGE_31p25_DPS  = 0b11000000;
    constexpr uint8_t GYRO_RANGE_15p625_DPS = 0b11100000;

    constexpr uint8_t GYRO_ODR_32000_HZ   = 0b00000001;
    constexpr uint8_t GYRO_ODR_16000_HZ   = 0b00000010;
    constexpr uint8_t GYRO_ODR_8000_HZ    = 0b00000011;
    constexpr uint8_t GYRO_ODR_4000_HZ    = 0b00000100;
    constexpr uint8_t GYRO_ODR_2000_HZ    = 0b00000101;
    constexpr uint8_t GYRO_ODR_1000_HZ    = 0b00000110;
    constexpr uint8_t GYRO_ODR_200_HZ     = 0b00000111;
    constexpr uint8_t GYRO_ODR_100_HZ     = 0b00001000;
    constexpr uint8_t GYRO_ODR_50_HZ      = 0b00001001;
    constexpr uint8_t GYRO_ODR_25_HZ      = 0b00001010;
    constexpr uint8_t GYRO_ODR_12p5_HZ    = 0b00001011;
    constexpr uint8_t GYRO_ODR_500_HZ     = 0b00001111;

constexpr uint8_t REG_ACCEL_CONFIG0         = 0x50;
    constexpr uint8_t ACCEL_RANGE_16G = 0b00000000;
    constexpr uint8_t ACCEL_RANGE_8G  = 0b00100000;
    constexpr uint8_t ACCEL_RANGE_4G  = 0b01000000;
    constexpr uint8_t ACCEL_RANGE_2G  = 0b01100000;

    constexpr uint8_t ACCEL_ODR_32000_HZ   = 0b00000001;
    constexpr uint8_t ACCEL_ODR_16000_HZ   = 0b00000010;
    constexpr uint8_t ACCEL_ODR_8000_HZ    = 0b00000011;
    constexpr uint8_t ACCEL_ODR_4000_HZ    = 0b00000100;
    constexpr uint8_t ACCEL_ODR_2000_HZ    = 0b00000101;
    constexpr uint8_t ACCEL_ODR_1000_HZ    = 0b00000110;
    constexpr uint8_t ACCEL_ODR_200_HZ     = 0b00000111;
    constexpr uint8_t ACCEL_ODR_100_HZ     = 0b00001000;
    constexpr uint8_t ACCEL_ODR_50_HZ      = 0b00001001;
    constexpr uint8_t ACCEL_ODR_25_HZ      = 0b00001010;
    constexpr uint8_t ACCEL_ODR_12p5_HZ    = 0b00001011;
    constexpr uint8_t ACCEL_ODR_500_HZ     = 0b00001111;

constexpr uint8_t REG_GYRO_CONFIG1          = 0x51;
constexpr uint8_t REG_GYRO_ACCEL_CONFIG0    = 0x52;
    constexpr uint8_t ACCEL_FILTER_LOW_LATENCY  = 0b11110000;
    constexpr uint8_t GYRO_FILTER_LOW_LATENCY   = 0b00001111;
constexpr uint8_t REG_ACCEL_CONFIG1         = 0x53;

constexpr uint8_t REG_TMST_CONFIG           = 0x54;
constexpr uint8_t REG_APEX_CONFIG           = 0x56;
constexpr uint8_t REG_SMD_CONFIG            = 0x57;
constexpr uint8_t REG_FIFO_CONFIG1          = 0x5F;
constexpr uint8_t REG_FIFO_CONFIG2          = 0x60;
constexpr uint8_t REG_FIFO_CONFIG3          = 0x61;
constexpr uint8_t REG_FSYNC_CONFIG          = 0x62;
constexpr uint8_t REG_INT_CONFIG0           = 0x63;
    constexpr uint8_t INT_CLEAR_ON_STATUS_BIT_READ = 0b00000000;
constexpr uint8_t REG_INT_CONFIG1           = 0x64;
    constexpr uint8_t INT_ASYNC_RESET           = 0b00001000; // this bit should be set to 0 for proper INT1 and INT2 pin operation
    constexpr uint8_t INT_TPULSE_DURATION_8US   = 0b01000000; // interrupt puls duration 8us, required for ODR >=4kHz
    constexpr uint8_t INT_TDEASSERT_DISABLE     = 0b00100000; // required for ODR >= 4kHz

constexpr uint8_t REG_INT_SOURCE0           = 0x65;
    constexpr uint8_t INT1_UI_DATA_READY_ENABLED     = 0b00001000;
constexpr uint8_t REG_INT_SOURCE1           = 0x66;
constexpr uint8_t REG_INT_SOURCE3           = 0x68;
constexpr uint8_t REG_INT_SOURCE4           = 0x69;

constexpr uint8_t REG_FIFO_LOST_PKT0        = 0x6C;
constexpr uint8_t REG_FIFO_LOST_PKT1        = 0x6D;
constexpr uint8_t REG_SELF_TEST_CONFIG3     = 0x70;

constexpr uint8_t REG_WHO_AM_I              = 0x75;
    constexpr uint8_t WHO_AM_I_RESPONSE_ICM42605  = 0x43;
    constexpr uint8_t WHO_AM_I_RESPONSE_ICM42688P = 0x47;

constexpr uint8_t REG_BANK_SEL              = 0x76;
constexpr uint8_t REG_INTF_CONFIG4          = 0x7A;
constexpr uint8_t REG_INTF_CONFIG5          = 0x7B;
constexpr uint8_t REG_INTF_CONFIG6          = 0x7C;

// User Bank 1 Register Map
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC2     = 0x0B;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC3     = 0x0C;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC4     = 0x0D;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC5     = 0x0E;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC6     = 0x0F;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC7     = 0x10;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC8     = 0x11;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC9     = 0x12;
constexpr uint8_t REG_BANK1_GYRO_CONFIG_STATIC10    = 0x13;

// User Bank 2 Register Map
constexpr uint8_t REG_BANK2_ACCEL_CONFIG_STATIC2    = 0x03;
constexpr uint8_t REG_BANK2_ACCEL_CONFIG_STATIC3    = 0x04;
constexpr uint8_t REG_BANK2_ACCEL_CONFIG_STATIC4    = 0x05;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_IMU_ICM426XX_USE_SPI_BUS)
ImuIcm426xx::ImuIcm426xx(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);
}
ImuIcm426xx::ImuIcm426xx(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
#else
ImuIcm426xx::ImuIcm426xx(uint8_t axis_order, uint8_t i2c_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, i2c_index, pins)
{
}
ImuIcm426xx::ImuIcm426xx(uint8_t axis_order, uint8_t i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, i2c_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
ImuIcm426xx::ImuIcm426xx(uint8_t axis_order, TwoWire& wire, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif

int ImuIcm426xx::init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) // NOLINT(readability-function-cognitive-complexity)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

#if defined(LIBRARY_SENSORS_IMU_BUS_MUTEX_REQUIRED)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    (void)bus_mutex;
#endif

    // MSP compatible gyro and acc identifiers
    enum { MSP_GYRO_ID_ICM42605 = 12, MSP_GYRO_ID_ICM42688P = 13 };
    enum { MSP_ACC_ID_ICM42605 = 11, MSP_ACC_ID_ICM42688P = 12 };

    _gyro_id_msp = MSP_GYRO_ID_ICM42605;
    _acc_id_msp = MSP_ACC_ID_ICM42605;

    _bus.set_device_data_register(REG_ACCEL_DATA_X1, reinterpret_cast<uint8_t*>(&_spiAccGyroData), sizeof(_spiAccGyroData));

    /*
    Turn off ACC and GYRO so they can be configured.

    From section 12.9, REGISTER VALUES MODIFICATION, in ICM-42688-P datasheet:

    The only register settings that user can modify during sensor operation are for ODR selection, FSR selection, and sensor mode changes
    (register parameters GYRO_ODR, ACCEL_ODR, GYRO_FS_SEL, ACCEL_FS_SEL, GYRO_MODE, ACCEL_MODE).
    User MUST NOT modify any other register values during sensor operation.
    The following procedure must be used for other register values modification:
        1. Turn Accel and Gyro Off
        2. Modify register values
        3. Turn Accel and/or Gyro On
    */
    _bus.write_register(REG_BANK_SEL, 0);
    _bus.write_register(REG_PWR_MGMT0, PWR_OFF);

    _bus.write_register(REG_DEVICE_CONFIG, DEVICE_CONFIG_DEFAULT); // default reset configuration
    delay_ms(1);

#if !defined(FRAMEWORK_TEST)
    const uint8_t chip_id = _bus.read_register_with_timeout(REG_WHO_AM_I, 100);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.print("IMU init, chip_id:0x");
    Serial.println(chip_id, HEX);
#endif
    if (chip_id != WHO_AM_I_RESPONSE_ICM42605 && chip_id != WHO_AM_I_RESPONSE_ICM42688P) {
        return NOT_DETECTED;
    }
#endif
// NOLINTBEGIN(hicpp-signed-bitwise)
    delay_ms(1);

    // set AntiAlias filter, see pages 28ff of TDK ICM-42688-P Datasheet
    static constexpr uint8_t GYRO_AAF_DELT = 38;
    static constexpr uint16_t GYRO_AAF_DELTSQR = 1440;
    static constexpr uint8_t GYRO_AAF_BITSH = 4; // gives 3dB Bandwidth of 2029 Hz
    _bus.write_register(REG_BANK_SEL, 1);
    _bus.write_register(REG_BANK1_GYRO_CONFIG_STATIC3, GYRO_AAF_DELT);
    _bus.write_register(REG_BANK1_GYRO_CONFIG_STATIC4, static_cast<uint8_t>(GYRO_AAF_DELTSQR & 0xFFU));
    _bus.write_register(REG_BANK1_GYRO_CONFIG_STATIC5, (GYRO_AAF_BITSH << 4U) | (GYRO_AAF_DELTSQR >> 8U));
    _bus.write_register(REG_BANK_SEL, 0);

    // REG_GYRO_CONFIG1 defaults to first order gyro ui filter, 3rd order GYRO_DEC2_M2_ORD filter, so is left unchanged
    _bus.write_register(REG_GYRO_ACCEL_CONFIG0, ACCEL_FILTER_LOW_LATENCY | GYRO_FILTER_LOW_LATENCY);

    // Configure interrupts
    _bus.write_register(REG_INT_CONFIG, INT1_MODE_PULSED | INT1_DRIVE_CIRCUIT_PUSH_PULL | INT1_POLARITY_ACTIVE_HIGH); // cppcheck-suppress badBitmaskCheck
    _bus.write_register(REG_INT_CONFIG0, INT_CLEAR_ON_STATUS_BIT_READ);
    // set interrupt pulse duration to 8us and disable de-assert duration, both required for ODR >= 4kHz
    _bus.write_register(REG_INT_CONFIG1, INT_TPULSE_DURATION_8US | INT_TDEASSERT_DISABLE);
    _bus.write_register(REG_INT_SOURCE0, INT1_UI_DATA_READY_ENABLED);

    // Configure INTF
    _bus.write_register(REG_INTF_CONFIG0, SENSOR_DATA_LITTLE_ENDIAN | UI_SIFS_CFG_DISABLE_I2C); // cppcheck-suppress badBitmaskCheck
    // Disable AFSR to prevent stalls in gyro output (undocumented in datasheet)
    constexpr uint8_t CONFIG1_DEFAULT_VALUE = 0b10010001;
    constexpr uint8_t CONFIG1_AFSR_MASK     = 0b00111111;
    constexpr uint8_t CONFIG1_AFSR_DISABLE  = 0b01000000;
#if false
    uint8_t intFConfig1 = _bus.read_register(REG_INTF_CONFIG1);
    intFConfig1 &= CONFIG1_AFSR_MASK;
    intFConfig1 |= CONFIG1_AFSR_DISABLE;
    _bus.write_register(REG_INTF_CONFIG1, intFConfig1);
#else
    _bus.write_register(REG_INTF_CONFIG1, (CONFIG1_DEFAULT_VALUE & CONFIG1_AFSR_MASK) | CONFIG1_AFSR_DISABLE);
#endif

    // Turn on gyro and acc on before configuring Output Data Rate(ODR) and Full Scale Rate (FSRP)
    _bus.write_register(REG_PWR_MGMT0, static_cast<uint8_t>(PWR_TEMP_ENABLED | PWR_GYRO_LOW_NOISE | PWR_ACCEL_LOW_NOISE)); // cppcheck-suppress badBitmaskCheck
    delay_ms(1);

// NOLINTEND(hicpp-signed-bitwise)

    // calculate the GYRO_ODR bit values to write to the REG_GYRO_CONFIG0 register
    const uint8_t GYRO_ODR =
        target_output_data_rate_hz == 0 ? GYRO_ODR_8000_HZ : // default to 8kHz
        target_output_data_rate_hz > 16000 ? GYRO_ODR_32000_HZ :
        target_output_data_rate_hz > 8000 ? GYRO_ODR_16000_HZ :
        target_output_data_rate_hz > 4000 ? GYRO_ODR_8000_HZ :
        target_output_data_rate_hz > 2000 ? GYRO_ODR_4000_HZ :
        target_output_data_rate_hz > 1000 ? GYRO_ODR_2000_HZ :
        target_output_data_rate_hz > 500 ? GYRO_ODR_1000_HZ :
        target_output_data_rate_hz > 200 ? GYRO_ODR_500_HZ :
        target_output_data_rate_hz > 100 ? GYRO_ODR_200_HZ :
        target_output_data_rate_hz > 50 ? GYRO_ODR_100_HZ :
        target_output_data_rate_hz > 25 ? GYRO_ODR_50_HZ :
        target_output_data_rate_hz > 12 ? GYRO_ODR_25_HZ : GYRO_ODR_12p5_HZ;
    // report the value that was actually set
    _gyro_sample_rate_hz =
        GYRO_ODR == GYRO_ODR_32000_HZ  ? 32000 :
        GYRO_ODR == GYRO_ODR_16000_HZ ? 16000 :
        GYRO_ODR == GYRO_ODR_8000_HZ ? 8000 :
        GYRO_ODR == GYRO_ODR_4000_HZ ? 4000 :
        GYRO_ODR == GYRO_ODR_2000_HZ ? 2000 :
        GYRO_ODR == GYRO_ODR_1000_HZ ? 1000 :
        GYRO_ODR == GYRO_ODR_500_HZ ? 500 :
        GYRO_ODR == GYRO_ODR_200_HZ ? 200 :
        GYRO_ODR == GYRO_ODR_100_HZ ? 100 :
        GYRO_ODR == GYRO_ODR_50_HZ ? 50 :
        GYRO_ODR == GYRO_ODR_25_HZ ? 25 : 12;

    // calculate the GYRO_RANGE bit values to write to the REG_GYRO_CONFIG0 register
    uint8_t GYRO_RANGE = 0;
    switch (gyro_sensitivity) {
    case GYRO_FULL_SCALE_125_DPS:
        GYRO_RANGE = GYRO_RANGE_125_DPS;
        _gyro_resolution_dps = 125.0F / 32768.0F;
        break;
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

    _bus.write_register(REG_GYRO_CONFIG0, GYRO_RANGE | GYRO_ODR);
    delay_ms(1);


    // calculate the ACCEL_ODR bit values to write to the REG_ACCEL_CONFIG0 register
    const uint8_t ACCEL_ODR =
        target_output_data_rate_hz == 0 ? ACCEL_ODR_8000_HZ : // default to 8kHz
        target_output_data_rate_hz > 16000 ? ACCEL_ODR_32000_HZ :
        target_output_data_rate_hz > 8000 ? ACCEL_ODR_16000_HZ :
        target_output_data_rate_hz > 4000 ? ACCEL_ODR_8000_HZ :
        target_output_data_rate_hz > 2000 ? ACCEL_ODR_4000_HZ :
        target_output_data_rate_hz > 1000 ? ACCEL_ODR_2000_HZ :
        target_output_data_rate_hz > 500 ? ACCEL_ODR_1000_HZ :
        target_output_data_rate_hz > 200 ? ACCEL_ODR_500_HZ :
        target_output_data_rate_hz > 100 ? ACCEL_ODR_200_HZ :
        target_output_data_rate_hz > 50 ? ACCEL_ODR_100_HZ :
        target_output_data_rate_hz > 25 ? ACCEL_ODR_50_HZ :
        target_output_data_rate_hz > 12 ? ACCEL_ODR_25_HZ : ACCEL_ODR_12p5_HZ;
    // report the value that was actually set
    _acc_sample_rate_hz =
        ACCEL_ODR == ACCEL_ODR_32000_HZ  ? 32000 :
        ACCEL_ODR == ACCEL_ODR_16000_HZ ? 16000 :
        ACCEL_ODR == ACCEL_ODR_8000_HZ ? 8000 :
        ACCEL_ODR == ACCEL_ODR_4000_HZ ? 4000 :
        ACCEL_ODR == ACCEL_ODR_2000_HZ ? 2000 :
        ACCEL_ODR == ACCEL_ODR_1000_HZ ? 1000 :
        ACCEL_ODR == ACCEL_ODR_500_HZ ? 500 :
        ACCEL_ODR == ACCEL_ODR_200_HZ ? 200 :
        ACCEL_ODR == ACCEL_ODR_100_HZ ? 100 :
        ACCEL_ODR == ACCEL_ODR_50_HZ ? 50 :
        ACCEL_ODR == ACCEL_ODR_25_HZ ? 25 : 12;

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

    _bus.write_register(REG_ACCEL_CONFIG0, ACCEL_RANGE | ACCEL_ODR);
    delay_ms(1);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyro_sample_rate_hz);
}

void ImuIcm426xx::set_interrupt_driven()
{
    // set interrupt level as configured in init()
    _bus.set_interrupt_driven(BusBase::IRQ_EDGE_RISE);
}

ImuBase::xyz_int32_t ImuIcm426xx::read_gyro_raw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take();
    _bus.read_register(REG_GYRO_DATA_X1, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give();

    return xyz_int32_t {
        .x = gyro.value.x,
        .y = gyro.value.y,
        .z = gyro.value.z
    };
}

ImuBase::xyz_int32_t ImuIcm426xx::read_acc_raw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take();
    _bus.read_register(REG_ACCEL_DATA_X1, &acc.data[0], sizeof(acc));
    bus_semaphore_give();

    return xyz_int32_t {
        .x = acc.value.x,
        .y = acc.value.y,
        .z = acc.value.z
    };
}

xyz_t ImuIcm426xx::read_gyro_rps()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_GYRO_DATA_X1, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give(_bus_mutex);

    return gyroRPS_FromRaw(gyro.value);
}

xyz_t ImuIcm426xx::read_gyro_dps()
{
    return read_gyro_rps() * RADIANS_TO_DEGREES;
}

xyz_t ImuIcm426xx::read_acc()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_DATA_X1, &acc.data[0], sizeof(acc));
    bus_semaphore_give(_bus_mutex);

    return accFromRaw(acc.value);
}

FAST_CODE acc_gyro_rps_t ImuIcm426xx::read_acc_gyro_rps()
{
    bus_semaphore_take(_bus_mutex);
    _bus.read_register(REG_ACCEL_DATA_X1, &_spiAccGyroData.accGyro.data[0], sizeof(_spiAccGyroData.accGyro));
    //_bus.read_device_data();
    bus_semaphore_give(_bus_mutex);

    return acc_gyro_rpsFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE acc_gyro_rps_t ImuIcm426xx::get_acc_gyro_rps() const
{
    return acc_gyro_rpsFromRaw(_spiAccGyroData.accGyro.value);
}

xyz_t ImuIcm426xx::gyroRPS_FromRaw(const mems_sensor_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(data.x) * _gyro_resolution_rps - _gyro_offset.x,
        .y =   static_cast<float>(data.y) * _gyro_resolution_rps - _gyro_offset.y,
        .z =   static_cast<float>(data.z) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -(static_cast<float>(data.y) * _gyro_resolution_rps - _gyro_offset.y),
        .y =   static_cast<float>(data.x) * _gyro_resolution_rps - _gyro_offset.x,
        .z =   static_cast<float>(data.z) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(data.y) * _gyro_resolution_rps - _gyro_offset.y,
        .y = -(static_cast<float>(data.x) * _gyro_resolution_rps - _gyro_offset.x),
        .z =   static_cast<float>(data.z) * _gyro_resolution_rps - _gyro_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =   static_cast<float>(data.x) * _gyro_resolution_rps - _gyro_offset.x,
        .y =   static_cast<float>(data.z) * _gyro_resolution_rps - _gyro_offset.z,
        .z = -(static_cast<float>(data.y) * _gyro_resolution_rps - _gyro_offset.y)
    };
#else
    const xyz_t gyro {
        .x =  static_cast<float>(data.x) * _gyro_resolution_rps - _gyro_offset.x,
        .y =  static_cast<float>(data.y) * _gyro_resolution_rps - _gyro_offset.y,
        .z =  static_cast<float>(data.z) * _gyro_resolution_rps - _gyro_offset.z
    };
    return map_axes(gyro);
#endif
}

xyz_t ImuIcm426xx::accFromRaw(const mems_sensor_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(data.x) * _acc_resolution - _acc_offset.x,
        .y =   static_cast<float>(data.y) * _acc_resolution - _acc_offset.y,
        .z =   static_cast<float>(data.z) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return xyz_t {
        .x = -(static_cast<float>(data.y) * _acc_resolution - _acc_offset.y),
        .y =   static_cast<float>(data.x) * _acc_resolution - _acc_offset.x,
        .z =   static_cast<float>(data.z) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return xyz_t {
        .x =   static_cast<float>(data.y) * _acc_resolution - _acc_offset.y,
        .y = -(static_cast<float>(data.x) * _acc_resolution - _acc_offset.x),
        .z =   static_cast<float>(data.z) * _acc_resolution - _acc_offset.z
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return xyz_t {
        .x =   static_cast<float>(data.x) * _acc_resolution - _acc_offset.x,
        .y =   static_cast<float>(data.z) * _acc_resolution - _acc_offset.z,
        .z = -(static_cast<float>(data.y) * _acc_resolution - _acc_offset.y)
    };
#else
    const xyz_t acc = {
        .x = static_cast<float>(data.x) * _acc_resolution - _acc_offset.x,
        .y = static_cast<float>(data.y) * _acc_resolution - _acc_offset.y,
        .z = static_cast<float>(data.z) * _acc_resolution - _acc_offset.z
    };
    return map_axes(acc);
#endif
}

acc_gyro_rps_t ImuIcm426xx::acc_gyro_rpsFromRaw(const acc_gyro_data_t::value_t& data) const
{
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
            .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
            .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
            .y = -(static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x),
            .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
            .y = -(static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x),
            .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -(static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x),
            .y = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y),
            .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -(static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x),
            .y = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y),
            .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y),
            .y =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
            .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y),
            .y =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
            .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z,
            .z = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y)
        },
        .acc = {
            .x =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z,
            .z = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y)
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
                .x =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
                .y =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
                .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
                .y =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
                .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
            }
        };
    case YPOS_XNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
                .y = -(static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x),
                .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
                .y = -(static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x),
                .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
            }
        };
    case XNEG_YNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -(static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x),
                .y = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y),
                .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x = -(static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x),
                .y = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y),
                .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
            }
        };
    case YNEG_XPOS_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y),
                .y =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
                .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
            },
            .acc = {
                .x = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y),
                .y =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
                .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
            }
        };
    case XPOS_YNEG_ZNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
                .y = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y),
                .z = -(static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
                .y = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y),
                .z = -(static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z)
            }
        };
    case YPOS_XPOS_ZNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
                .y =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
                .z = -(static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
                .y =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.x,
                .z = -(static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z)
            }
        };
    case XNEG_YPOS_ZNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -(static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x),
                .y =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
                .z = -(static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x = -(static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x),
                .y =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
                .z = -(static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z)
            }
        };
    case YNEG_XNEG_ZNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -(static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y),
                .y = -(static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x),
                .z = -(static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z)
            },
            .acc = {
                .x = -(static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y),
                .y = -(static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x),
                .z = -(static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z)
            }
        };
    default:
        return acc_gyro_rps_t {
            .gyroRPS = map_axes({
                .x =   static_cast<float>(data.gyro_x) * _gyro_resolution_rps - _gyro_offset.x,
                .y =   static_cast<float>(data.gyro_y) * _gyro_resolution_rps - _gyro_offset.y,
                .z =   static_cast<float>(data.gyro_z) * _gyro_resolution_rps - _gyro_offset.z
            }),
            .acc = map_axes({
                .x =   static_cast<float>(data.acc_x) * _acc_resolution - _acc_offset.x,
                .y =   static_cast<float>(data.acc_y) * _acc_resolution - _acc_offset.y,
                .z =   static_cast<float>(data.acc_z) * _acc_resolution - _acc_offset.z
            })
        };
    } // end switch
#endif
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
