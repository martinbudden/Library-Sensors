#include "imu_lsm6ds3tr_c.h"
//#define LIBRARY_SENSORS_SERIAL_DEBUG
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
#include <HardwareSerial.h>
#else
#include <Arduino.h>
#endif
#endif
#include <cassert>

/*
https://github.com/STMicroelectronics/lsm6ds3tr-c-pid
https://github.com/STMicroelectronics/ism330dhcx-pid
https://github.com/STMicroelectronics/lsm6dsox-pid
*/

namespace { // use anonymous namespace to make items local to this translation unit

constexpr uint8_t REG_RESERVED_00           = 0x00;
constexpr uint8_t REG_FUNC_CFG_ACCESS       = 0x01;
constexpr uint8_t REG_RESERVED_03           = 0x03;

#if defined(USE_ImuLsmds63trC)

constexpr uint8_t REG_RESERVED_02           = 0x02;
constexpr uint8_t REG_SENSOR_SYNC_TIME_FRAME= 0x04;
constexpr uint8_t REG_SENSOR_SYNC_RES_RATIO = 0x05;
constexpr uint8_t REG_FIFO_CTRL1            = 0x06;
constexpr uint8_t REG_FIFO_CTRL2            = 0x07;
constexpr uint8_t REG_FIFO_CTRL3            = 0x08;
constexpr uint8_t REG_FIFO_CTRL4            = 0x09;
constexpr uint8_t REG_FIFO_CTRL5            = 0x0A;
constexpr uint8_t REG_DRDY_PULSE_CFG_G      = 0x0B;
constexpr uint8_t REG_RESERVED_0C           = 0x0C;
constexpr uint8_t REG_MASTER_CONFIG         = 0x1A;

#elif defined(USE_IMU_ISM330DHCX)

constexpr uint8_t REG_PIN_CTRL              = 0x02;
constexpr uint8_t REG_RESERVED_04           = 0x04;
constexpr uint8_t REG_RESERVED_05           = 0x05;
constexpr uint8_t REG_RESERVED_06           = 0x06;
constexpr uint8_t REG_FIFO_CTRL1            = 0x07;
constexpr uint8_t REG_FIFO_CTRL2            = 0x08;
constexpr uint8_t REG_FIFO_CTRL3            = 0x09;
constexpr uint8_t REG_FIFO_CTRL4            = 0x0A;
constexpr uint8_t REG_COUNTER_BDR_REG1      = 0x0B;
constexpr uint8_t REG_COUNTER_BDR_REG2      = 0x0C;
constexpr uint8_t REG_ALL_INT_SRC           = 0x1A;

#elif defined(USE_IMU_LSM6DSOX)

constexpr uint8_t REG_PIN_CTRL              = 0x02;
constexpr uint8_t REG_S4S_TPH_L             = 0x04;
constexpr uint8_t REG_S4S_TPH_H             = 0x05;
constexpr uint8_t REG_S4S_RR                = 0x06;
constexpr uint8_t REG_FIFO_CTRL1            = 0x07;
constexpr uint8_t REG_FIFO_CTRL2            = 0x08;
constexpr uint8_t REG_FIFO_CTRL3            = 0x09;
constexpr uint8_t REG_FIFO_CTRL4            = 0x0A;
constexpr uint8_t REG_COUNTER_BDR_REG1      = 0x0B;
constexpr uint8_t REG_COUNTER_BDR_REG2      = 0x0C;
constexpr uint8_t REG_ALL_INT_SRC           = 0x1A;

#endif

constexpr uint8_t REG_DATA_READY_PULSE_CONFIG = 0x0B;
    const uint8_t DATA_READY_PULSED         = 0b10000000;
constexpr uint8_t REG_INT1_CTRL             = 0x0D;
    constexpr uint8_t INT1_DRDY_G           =0b00000010;
constexpr uint8_t REG_INT2_CTRL             = 0x0E;
    constexpr uint8_t INT2_DRDY_G           =0b00000010;
constexpr uint8_t REG_WHO_AM_I              = 0x0F;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C = 0x6A;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_ISM330DHCX = 0x6B;
    constexpr uint8_t REG_WHO_AM_I_RESPONSE_LSM6DSOX = 0x6C;
constexpr uint8_t REG_CTRL1_XL              = 0x10;
    constexpr uint8_t ACC_RANGE_2G    =     0b0000;
    constexpr uint8_t ACC_RANGE_4G    =     0b1000;
    constexpr uint8_t ACC_RANGE_8G    =     0b1100;
    constexpr uint8_t ACC_RANGE_16G   =     0b0100;
    constexpr uint8_t ACC_ODR_12p5_HZ = 0b00010000;
    constexpr uint8_t ACC_ODR_26_HZ   = 0b00100000;
    constexpr uint8_t ACC_ODR_52_HZ   = 0b00110000;
    constexpr uint8_t ACC_ODR_104_HZ  = 0b01000000;
    constexpr uint8_t ACC_ODR_208_HZ  = 0b010100000;
    constexpr uint8_t ACC_ODR_416_HZ  = 0b01100000;
    constexpr uint8_t ACC_ODR_833_HZ  = 0b01110000;
    constexpr uint8_t ACC_ODR_1666_HZ = 0b10000000;
    constexpr uint8_t ACC_ODR_3332_HZ = 0b10010000;
    constexpr uint8_t ACC_ODR_6664_HZ = 0b10100000;
constexpr uint8_t REG_CTRL2_G               = 0x11;
    constexpr uint8_t GYRO_RANGE_125_DPS   = 0b0010;
    constexpr uint8_t GYRO_RANGE_245_DPS   = 0b0000; // LSM6DS3TR_C
    constexpr uint8_t GYRO_RANGE_250_DPS   = 0b0000; // ISM330DHCX, LSM6DSOX
    constexpr uint8_t GYRO_RANGE_500_DPS   = 0b0100;
    constexpr uint8_t GYRO_RANGE_1000_DPS  = 0b1000;
    constexpr uint8_t GYRO_RANGE_2000_DPS  = 0b1100;
    constexpr uint8_t GYRO_ODR_12p5_HZ = 0b00010000;
    constexpr uint8_t GYRO_ODR_26_HZ   = 0b00100000;
    constexpr uint8_t GYRO_ODR_52_HZ   = 0b00110000;
    constexpr uint8_t GYRO_ODR_104_HZ  = 0b01000000;
    constexpr uint8_t GYRO_ODR_208_HZ  = 0b010100000;
    constexpr uint8_t GYRO_ODR_416_HZ  = 0b01100000;
    constexpr uint8_t GYRO_ODR_833_HZ  = 0b01110000;
    constexpr uint8_t GYRO_ODR_1666_HZ = 0b10000000;
    constexpr uint8_t GYRO_ODR_3332_HZ = 0b10010000;
    constexpr uint8_t GYRO_ODR_6664_HZ = 0b10100000;
constexpr uint8_t REG_CTRL3_C               = 0x12;
    constexpr uint8_t BDU                   = 0b01000000;
    constexpr uint8_t IF_INC                = 0b00000100;
    constexpr uint8_t SW_RESET              = 0b00000001;
constexpr uint8_t REG_CTRL4_C               = 0x13;
    constexpr uint8_t I2C_DISABLE           = 0b00000100;
    constexpr uint8_t LPF1_SEL_G            = 0b00000010;
constexpr uint8_t REG_CTRL5_C               = 0x14;
constexpr uint8_t REG_CTRL6_C               = 0x15;
    constexpr uint8_t XL_HM_MODE_DISABLE    = 0b00010000;
    constexpr uint8_t LPF1_MEDIUM_HI        = 0x00;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 351Hz, ISM330DHCX: 297Hz, LSM6DSOX: 335.5Hz
    constexpr uint8_t LPF1_MEDIUM_LO        = 0x01;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 237Hz, ISM330DHCX: 223Hz, LSM6DSOX: 232.0Hz
    constexpr uint8_t LPF1_LO               = 0x02;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz - LSM6DS3TR_C: 172Hz, ISM330DHCX: 154Hz, LSM6DSOX: 171.1Hz
    constexpr uint8_t LPF1_HI               = 0x03;   // (bits 2:0) gyro LPF1 cutoff at 6667 Hz   LSM6DS3TR_C: 937Hz, ISM330DHCX: 470Hz, LSM6DSOX: 609.0Hz
constexpr uint8_t REG_CTRL7_G               = 0x16; // this includes high pass filter (HPF)
constexpr uint8_t REG_CTRL8_XL              = 0x17;
constexpr uint8_t REG_CTRL9_XL              = 0x18;
constexpr uint8_t REG_CTRL10_C              = 0x19;
constexpr uint8_t REG_WAKE_UP_SRC           = 0x1B;
constexpr uint8_t REG_TAP_SRC               = 0x1C;
constexpr uint8_t REG_D6D_SRC               = 0x1D;
constexpr uint8_t REG_STATUS_REG            = 0x1E;
constexpr uint8_t REG_RESERVED_1F           = 0x1F;

constexpr uint8_t REG_OUT_TEMP_L            = 0x20;
constexpr uint8_t REG_OUT_TEMP_H            = 0x22;
constexpr uint8_t REG_OUTX_L_G              = 0x22;
constexpr uint8_t REG_OUTX_H_G              = 0x23;
constexpr uint8_t REG_OUTY_L_G              = 0x24;
constexpr uint8_t REG_OUTY_H_G              = 0x25;
constexpr uint8_t REG_OUTZ_L_G              = 0x26;
constexpr uint8_t REG_OUTZ_H_G              = 0x27;
constexpr uint8_t REG_OUTX_L_ACC            = 0x28;
constexpr uint8_t REG_OUTX_H_ACC            = 0x29;
constexpr uint8_t REG_OUTY_L_ACC            = 0x2A;
constexpr uint8_t REG_OUTY_H_ACC            = 0x2B;
constexpr uint8_t REG_OUTZ_L_ACC            = 0x2C;
constexpr uint8_t REG_OUTZ_H_ACC            = 0x2D;

} // end namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

/*!
Gyroscope data rates up to 6.4 kHz, accelerometer up to 1.6 kHz
*/
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuLsmds63trC_USE_SPI_BUS)
ImuLsmds63trC::ImuLsmds63trC(uint8_t axis_order, uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::stm32_spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
ImuLsmds63trC::ImuLsmds63trC(uint8_t axis_order, uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::spi_pins_t& pins) :
    ImuBase(axis_order, _bus),
    _bus(frequency, spi_index, pins)
{
}
#else
ImuLsmds63trC::ImuLsmds63trC(uint8_t axis_order, BusBase::bus_index_e i2c_index, const BusI2c::stm32_i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, i2c_index, pins)
{
}
ImuLsmds63trC::ImuLsmds63trC(uint8_t axis_order, BusBase::bus_index_e i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, i2c_index, pins)
{
}
#if !defined(FRAMEWORK_RPI_PICO) && !defined(FRAMEWORK_ESPIDF) &&!defined(FRAMEWORK_STM32_CUBE) && !defined(FRAMEWORK_TEST)
ImuLsmds63trC::ImuLsmds63trC(uint8_t axis_order, TwoWire& wire, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) :
    ImuBase(axis_order, _bus),
    _bus(I2C_address, wire, pins)
{
}
#endif
#endif

int ImuLsmds63trC::init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) // NOLINT(readability-function-cognitive-complexity)
{
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

#if defined(LIBRARY_SENSORS_IMU_BUS_MUTEX_REQUIRED)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    (void)bus_mutex;
#endif
    static_assert(sizeof(mems_sensor_data_t) == mems_sensor_data_t::DATA_SIZE);
    static_assert(sizeof(acc_gyro_data_t) == acc_gyro_data_t::DATA_SIZE);

    // MSP compatible gyro and acc identifiers
    _gyro_id_msp = 18;
    _acc_id_msp = 19;

    _bus.set_device_data_register(REG_OUTX_L_G, &_spiAccGyroData.preReadBuffer[0], sizeof(_spiAccGyroData));

    _bus.write_register(REG_CTRL3_C, SW_RESET); // software reset
    delay_ms(100);

    const uint8_t chip_id = _bus.read_register_with_timeout(REG_WHO_AM_I, 100);
    delay_ms(1);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.print("IMU init, chip_id:0x");
    Serial.println(chip_id, HEX);
#endif
    //assert(chip_id == REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C || chip_id == REG_WHO_AM_I_RESPONSE_ISM330DHCX || chip_id == REG_WHO_AM_I_RESPONSE_LSM6DSOX);
    if (chip_id != REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C && chip_id != REG_WHO_AM_I_RESPONSE_ISM330DHCX && chip_id != REG_WHO_AM_I_RESPONSE_LSM6DSOX) {
        return NOT_DETECTED;
    }

    // set data ready pulsed
    _bus.write_register(REG_DATA_READY_PULSE_CONFIG, DATA_READY_PULSED);
    delay_ms(1);
    // INT pins are by default forced to ground, so active high
    _bus.write_register(REG_INT1_CTRL, INT1_DRDY_G); // Enable gyro data ready on INT1 pin
    delay_ms(1);
    _bus.write_register(REG_INT2_CTRL, INT2_DRDY_G); // Enable gyro data ready on INT2 pin
    delay_ms(1);
    _bus.write_register(REG_CTRL3_C, BDU | IF_INC); // Block Data Update and automatically increment registers when read via serial interface (I2C or SPI)
    delay_ms(1);
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuLsmds63trC_USE_SPI_BUS)
    _bus.write_register(REG_CTRL4_C, LPF1_SEL_G | I2C_DISABLE);  // enable gyro LPF, disable I2C
#else
    _bus.write_register(REG_CTRL4_C, LPF1_SEL_G); // enable gyro LPF
#endif
    delay_ms(1);

    // calculate the GYRO_ODR bit values to write to the REG_CTRL2_G register
    const uint8_t GYRO_ODR =
        target_output_data_rate_hz == 0 ? GYRO_ODR_6664_HZ :
        target_output_data_rate_hz > 3332 ? GYRO_ODR_6664_HZ :
        target_output_data_rate_hz > 1666 ? GYRO_ODR_3332_HZ :
        target_output_data_rate_hz > 833 ? GYRO_ODR_1666_HZ :
        target_output_data_rate_hz > 416 ? GYRO_ODR_833_HZ :
        target_output_data_rate_hz > 208 ? GYRO_ODR_416_HZ :
        target_output_data_rate_hz > 104 ? GYRO_ODR_208_HZ :
        target_output_data_rate_hz > 52 ? GYRO_ODR_104_HZ :
        target_output_data_rate_hz > 26 ? GYRO_ODR_52_HZ :
        target_output_data_rate_hz > 13 ? GYRO_ODR_26_HZ : GYRO_ODR_12p5_HZ;
    // report the value that was actually set
    _gyro_sample_rate_hz =
        GYRO_ODR == GYRO_ODR_6664_HZ ? 6664 :
        GYRO_ODR == GYRO_ODR_3332_HZ ? 3332 :
        GYRO_ODR == GYRO_ODR_1666_HZ ? 1666 :
        GYRO_ODR == GYRO_ODR_833_HZ ? 833 :
        GYRO_ODR == GYRO_ODR_416_HZ ? 416 :
        GYRO_ODR == GYRO_ODR_208_HZ ? 208 :
        GYRO_ODR == GYRO_ODR_104_HZ ? 104:
        GYRO_ODR == GYRO_ODR_52_HZ ? 52 :
        GYRO_ODR == GYRO_ODR_26_HZ ? 26 : 12;

    // set the anti-alias LPF filter according to the gyro sampling rate
    _bus.write_register(REG_CTRL6_C, _gyro_sample_rate_hz > 3000 ? LPF1_HI : LPF1_MEDIUM_HI);
    delay_ms(1);

    switch (gyro_sensitivity) {
    case GYRO_FULL_SCALE_125_DPS: // NOLINT(bugprone-branch-clone) false positive
        _bus.write_register(REG_CTRL2_G, GYRO_RANGE_125_DPS | GYRO_ODR);
        _gyro_resolution_dps = 245.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_250_DPS:
        _bus.write_register(REG_CTRL2_G, GYRO_RANGE_245_DPS | GYRO_ODR); // cppcheck-suppress badBitmaskCheck
        _gyro_resolution_dps = 245.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_500_DPS:
        _bus.write_register(REG_CTRL2_G, GYRO_RANGE_500_DPS | GYRO_ODR);
        _gyro_resolution_dps = 500.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_1000_DPS:
        _bus.write_register(REG_CTRL2_G, GYRO_RANGE_1000_DPS | GYRO_ODR);
        _gyro_resolution_dps = 1000.0F / 32768.0F;
        break;
    case GYRO_FULL_SCALE_2000_DPS:
        [[fallthrough]];
    default:
        _bus.write_register(REG_CTRL2_G, GYRO_RANGE_2000_DPS | GYRO_ODR);
        _gyro_resolution_dps = 2000.0F / 32768.0F;
        break;
    }
    _gyro_resolution_rps = _gyro_resolution_dps * DEGREES_TO_RADIANS;
    delay_ms(1);

    // calculate the ACC_ODR bit values to write to the REG_CTRL1_XL register
    const uint8_t ACC_ODR =
        target_output_data_rate_hz == 0 ? ACC_ODR_6664_HZ :
        target_output_data_rate_hz > 3332 ? ACC_ODR_6664_HZ :
        target_output_data_rate_hz > 1666 ? ACC_ODR_3332_HZ :
        target_output_data_rate_hz > 833 ? ACC_ODR_1666_HZ :
        target_output_data_rate_hz > 416 ? ACC_ODR_833_HZ :
        target_output_data_rate_hz > 208 ? ACC_ODR_416_HZ :
        target_output_data_rate_hz > 104 ? ACC_ODR_208_HZ :
        target_output_data_rate_hz > 52 ? ACC_ODR_104_HZ :
        target_output_data_rate_hz > 26 ? ACC_ODR_52_HZ :
        target_output_data_rate_hz > 13 ? ACC_ODR_26_HZ : ACC_ODR_12p5_HZ;
    // report the value that was actually set
    _acc_sample_rate_hz =
        ACC_ODR == ACC_ODR_6664_HZ ? 6664 :
        ACC_ODR == ACC_ODR_3332_HZ ? 3332 :
        ACC_ODR == ACC_ODR_1666_HZ ? 1666 :
        ACC_ODR == ACC_ODR_833_HZ ? 833 :
        ACC_ODR == ACC_ODR_416_HZ ? 416 :
        ACC_ODR == ACC_ODR_208_HZ ? 208 :
        ACC_ODR == ACC_ODR_104_HZ ? 104:
        ACC_ODR == ACC_ODR_52_HZ ? 52 :
        ACC_ODR == ACC_ODR_26_HZ ? 26 : 12;

    switch (acc_sensitivity) {
    case ACC_FULL_SCALE_2G:
        _bus.write_register(REG_CTRL1_XL, ACC_RANGE_2G | ACC_ODR); // cppcheck-suppress badBitmaskCheck
        _acc_resolution = 2.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_4G:
        _bus.write_register(REG_CTRL1_XL, ACC_RANGE_4G | ACC_ODR);
        _acc_resolution = 4.0F / 32768.0F;
        break;
    case ACC_FULL_SCALE_8G:
        _bus.write_register(REG_CTRL1_XL, ACC_RANGE_8G | ACC_ODR);
        _acc_resolution = 8.0F / 32768.0F;
        break;
    default:
        _bus.write_register(REG_CTRL1_XL, ACC_RANGE_16G | ACC_ODR);
        _acc_resolution = 16.0F / 32768.0F;
        break;
    }
    delay_ms(1);

    // return the gyro sample rate actually set
    return static_cast<int>(_gyro_sample_rate_hz);
}

void ImuLsmds63trC::set_interrupt_driven()
{
    // set interrupt level as configured in init()
    _bus.set_interrupt_driven(BusBase::IRQ_EDGE_RISE);
}

ImuBase::xyz_int32_t ImuLsmds63trC::read_gyro_raw()
{
    mems_sensor_data_t gyro; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take();
    _bus.read_register(REG_OUTX_L_G, &gyro.data[0], sizeof(gyro));
    bus_semaphore_give();

    return xyz_int32_t {
        .x = gyro.value.x,
        .y = gyro.value.y,
        .z = gyro.value.z
    };
}

ImuBase::xyz_int32_t ImuLsmds63trC::read_acc_raw()
{
    mems_sensor_data_t acc; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)

    bus_semaphore_take();
    _bus.read_register(REG_OUTX_L_ACC, &acc.data[0], sizeof(acc));
    bus_semaphore_give();

    return xyz_int32_t {
        .x = acc.value.x,
        .y = acc.value.y,
        .z = acc.value.z
    };
}

FAST_CODE acc_gyro_rps_t ImuLsmds63trC::read_acc_gyro_rps()
{
    bus_semaphore_take();
    _bus.read_device_data();
    //_bus.read_device_data_dma(); // for testing
    //_bus.read_register(REG_OUTX_L_G, &_spiAccGyroData.accGyro.data[0], sizeof(_spiAccGyroData.accGyro));
    bus_semaphore_give();

    return acc_gyro_rpsFromRaw(_spiAccGyroData.accGyro.value);
}

/*!
Return the gyroAcc data that was read in the ISR
*/
FAST_CODE acc_gyro_rps_t ImuLsmds63trC::get_acc_gyro_rps() const
{
    return acc_gyro_rpsFromRaw(_spiAccGyroData.accGyro.value);
}

acc_gyro_rps_t ImuLsmds63trC::acc_gyro_rpsFromRaw(const acc_gyro_data_t::value_t& data) const
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
