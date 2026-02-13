#include "barometer_bmp280.h"
//#define LIBRARY_SENSORS_SERIAL_DEBUG
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
#include <HardwareSerial.h>
#endif

#include <cmath>

namespace { // use anonymous namespace to make items local to this translation unit
    constexpr uint8_t REG_DIG_T1        = 0x88;
    constexpr uint8_t REG_DIG_T2        = 0x8A;
    constexpr uint8_t REG_DIG_T3        = 0x8C;
    constexpr uint8_t REG_DIG_P1        = 0x8E;
    constexpr uint8_t REG_DIG_P2        = 0x90;
    constexpr uint8_t REG_DIG_P3        = 0x92;
    constexpr uint8_t REG_DIG_P4        = 0x94;
    constexpr uint8_t REG_DIG_P5        = 0x96;
    constexpr uint8_t REG_DIG_P6        = 0x98;
    constexpr uint8_t REG_DIG_P7        = 0x9A;
    constexpr uint8_t REG_DIG_P8        = 0x9C;
    constexpr uint8_t REG_DIG_P9        = 0x9E;
    constexpr uint8_t REG_CHIPID        = 0xD0;
        constexpr uint8_t CHIPID_RESPONSE  = 0x58;
    constexpr uint8_t REG_VERSION       = 0xD1;
    constexpr uint8_t REG_SOFTRESET     = 0xE0;
    constexpr uint8_t REG_CAL26         = 0xE1; // R calibration = 0xE1-0xF0
    constexpr uint8_t REG_STATUS        = 0xF3;
    constexpr uint8_t REG_CONTROL       = 0xF4;
        constexpr uint8_t SAMPLING_NONE     = 0x00;
        constexpr uint8_t SAMPLING_X1       = 0x01;
        constexpr uint8_t SAMPLING_X2       = 0x02; // 2x oversampling
        constexpr uint8_t SAMPLING_X4       = 0x03; // 4x oversampling
        constexpr uint8_t SAMPLING_X8       = 0x04; // 8x oversampling
        constexpr uint8_t SAMPLING_X16      = 0x05; // 16x oversampling
        constexpr uint8_t MODE_SLEEP        = 0x00;
        constexpr uint8_t MODE_FORCED       = 0x01;
        constexpr uint8_t MODE_NORMAL       = 0x03;
        constexpr uint8_t MODE_SOFT_RESET_CODE = 0xB6;
        constexpr uint8_t TEMPERATURE_OSR = SAMPLING_X1;
        constexpr uint8_t PRESSURE_OSR = SAMPLING_X8;
        // note that useing MODE_FORCED means that REG_CONTROL must be set before each reading
        constexpr uint8_t MEASUREMENT_MODE = (TEMPERATURE_OSR << 5) | (PRESSURE_OSR << 2) | MODE_FORCED; // NOLINT(hicpp-signed-bitwise)
    constexpr uint8_t REG_CONFIG        = 0xF5;
        // Filtering level
        constexpr uint8_t FILTER_OFF        = 0x00;
        constexpr uint8_t FILTER_X2         = 0x01;
        constexpr uint8_t FILTER_X4         = 0x02;
        constexpr uint8_t FILTER_X8         = 0x03;
        constexpr uint8_t FILTER_X16        = 0x04;
        // Standby duration in milliseconds
        constexpr uint8_t STANDBY_MS_0p5    = 0x00; // 0.5ms
        constexpr uint8_t STANDBY_MS_62p5   = 0x01; // 62.5ms
        constexpr uint8_t STANDBY_MS_125    = 0x02;
        constexpr uint8_t STANDBY_MS_250    = 0x03;
        constexpr uint8_t STANDBY_MS_500    = 0x04;
        constexpr uint8_t STANDBY_MS_1000   = 0x05;
        constexpr uint8_t STANDBY_MS_2000   = 0x06;
        constexpr uint8_t STANDBY_MS_4000   = 0x07;
        constexpr uint8_t SPI_3WIRE_ENABLE  = 0x01;
    constexpr uint8_t REG_PRESSURE_MSB  = 0xF7;
    constexpr uint8_t REG_PRESSURE_LSB  = 0xF8;
    constexpr uint8_t REG_PRESSURE_XLSB = 0xF9;
    constexpr uint8_t REG_TEMPERATURE_MSB   = 0xFA;
    constexpr uint8_t REG_TEMPERATURE_LSB   = 0xFB;
    constexpr uint8_t REG_TEMPERATURE_XLSB  = 0xFC;
} // end namespace

#if defined(LIBRARY_SENSORS_BAROMETER_USE_SPI_BUS)
BarometerBmp280::BarometerBmp280(uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins) :
    BarometerBase(_bus),
    _bus(frequency, spi_index, pins)
{
}
BarometerBmp280::BarometerBmp280(uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins) :
    BarometerBase(_bus),
    _bus(frequency, spi_index, pins)
{
}
#else
BarometerBmp280::BarometerBmp280(uint8_t i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t i2c_address) :
    BarometerBase(_bus),
    _bus(i2c_address, i2c_index, pins)
{
}
#endif

int BarometerBmp280::init()
{
#if !defined(FRAMEWORK_TEST)
    const uint8_t chip_id = _bus.read_register_with_timeout(REG_CHIPID, 100);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.print("IMU init, chip_id:0x");
    Serial.println(chip_id, HEX);
#endif
    if (chip_id != CHIPID_RESPONSE) {
        return NOT_DETECTED;
    }
#endif
    /*_calibration_data.value.dig_T1 = read_register16_LE(REG_DIG_T1);
    _calibration_data.value.dig_T2 = read_register16S_LE(REG_DIG_T2);
    _calibration_data.value.dig_T3 = read_register16S_LE(REG_DIG_T3);

    _calibration_data.value.dig_P1 = read_register16_LE(REG_DIG_P1);
    _calibration_data.value.dig_P2 = read_register16S_LE(REG_DIG_P2);
    _calibration_data.value.dig_P3 = read_register16S_LE(REG_DIG_P3);
    _calibration_data.value.dig_P4 = read_register16S_LE(REG_DIG_P4);
    _calibration_data.value.dig_P5 = read_register16S_LE(REG_DIG_P5);
    _calibration_data.value.dig_P6 = read_register16S_LE(REG_DIG_P6);
    _calibration_data.value.dig_P7 = read_register16S_LE(REG_DIG_P7);
    _calibration_data.value.dig_P8 = read_register16S_LE(REG_DIG_P8);
    _calibration_data.value.dig_P9 = read_register16S_LE(REG_DIG_P9);*/

    _bus.read_register(REG_DIG_T1, &_calibration_data.data[0], sizeof(_calibration_data));
    delay_ms(1);

    /*
    don't need to set REG_CONFIG, since default of 0 gives us what we need, namely:
    const uint8_t standbyMs = STANDBY_MS_0p5;
    const uint8_t filter = FILTER_OFF;
    const uint8_t spi3wire = 0;
    _bus.write_register(REG_CONFIG, (standbyMs << 5) | (filter << 2) | spi3wire); // cppcheck-suppress badBitmaskCheck
    delay_ms(1);
    */

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,hicpp-signed-bitwise,readability-magic-numbers)
    _bus.write_register(REG_CONTROL, MEASUREMENT_MODE);
    delay_ms(1);

    read_temperature_and_pressure();
    _pressure_at_reference_altitude = _pressure_pascals;
    _reference_altitude = 0.0F;

    enum { TIME_INIT_MAX = 20 }; // 20/16 = 1.25 ms
    enum { TIME_MEASURE_PER_OSRS_MAX = 37 }; // 37/16 = 2.3125 ms
    enum { TIME_SETUP_PRESSURE_MAX = 10 }; // 10/16 = 0.625 ms

    constexpr uint32_t delay_ms = ((TIME_INIT_MAX + TIME_MEASURE_PER_OSRS_MAX * (((1 << TEMPERATURE_OSR) >> 1) + ((1 << PRESSURE_OSR) >> 1)) + TIME_SETUP_PRESSURE_MAX + 15) / 16);
    _sample_rate_hz = 1000 / delay_ms; // delay ~ 23ms, sample rate ~ 43.5Hz
    return static_cast<int>(_sample_rate_hz);
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,hicpp-signed-bitwise,readability-magic-numbers)
}

void BarometerBmp280::read_temperature_and_pressure()
{
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-type-union-access,hicpp-signed-bitwise,readability-magic-numbers)
    const auto T1 = static_cast<int32_t>(_calibration_data.value.dig_T1);
    const auto T2 = static_cast<int32_t>(_calibration_data.value.dig_T2);
    const auto T3 = static_cast<int32_t>(_calibration_data.value.dig_T3);
    const auto P1 = static_cast<int64_t>(_calibration_data.value.dig_P1);
    const auto P2 = static_cast<int64_t>(_calibration_data.value.dig_P2);
    const auto P3 = static_cast<int64_t>(_calibration_data.value.dig_P3);
    const auto P4 = static_cast<int64_t>(_calibration_data.value.dig_P4);
    const auto P5 = static_cast<int64_t>(_calibration_data.value.dig_P5);
    const auto P6 = static_cast<int64_t>(_calibration_data.value.dig_P6);
    const auto P7 = static_cast<int64_t>(_calibration_data.value.dig_P7);
    const auto P8 = static_cast<int64_t>(_calibration_data.value.dig_P8);
    const auto P9 = static_cast<int64_t>(_calibration_data.value.dig_P9);

    // burst read of temperature and pressure data
    _bus.write_register(REG_CONTROL, MEASUREMENT_MODE);
    // read together in burst so data is consistent, as specified in datasheet
    pressure_temperature_data_u pt; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)
    _bus.read_register(REG_PRESSURE_MSB, &pt.data[0], sizeof(pt));
    const auto adcT = static_cast<int32_t>(
            ((static_cast<uint32_t>(pt.value.temperature_msb) << 16) |
            (static_cast<uint32_t>(pt.value.temperature_lsb) << 8) |
            static_cast<uint32_t>(pt.value.temperature_xlsb)) >> 4
        );
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.printf("temp:%x:%x:%x\r\n", pt.value.temperature_msb, pt.value.temperature_lsb, pt.value.temperature_xlsb);
    Serial.printf("pres:%x:%x:%x\r\n", pt.value.pressure_msb, pt.value.pressure_lsb, pt.value.pressure_xlsb);
#endif
    // set _temperatureFine.
    const int32_t vt1 = (((adcT >> 3) - (T1 << 1))) * T2;
    const int32_t vt2 = ((((adcT >> 4) - T1) * ((adcT >> 4) - T1)) >> 12) * T3;
    _temperatureFine = (vt1 >> 11) + (vt2 >> 14);
    _temperature_celsius = static_cast<float>((_temperatureFine * 5 + 128) >> 8) / 100.0F;

    auto vp1 = static_cast<int64_t>(_temperatureFine) - 128000;
    int64_t vp2 = vp1 * vp1 * P6;
    vp2 = vp2 + ((vp1 * P5) << 17);
    vp2 = vp2 + (P4 << 35);
    vp1 = ((vp1 * vp1 * P3) >> 8) + ((vp1 * P2) << 12);
    vp1 = ((((static_cast<int64_t>(1)) << 47) + vp1)) * P1 >> 33;

    if (vp1 == 0) {
        return; // avoid division by zero
    }
    const auto adcP = static_cast<int32_t>(
            ((static_cast<uint32_t>(pt.value.pressure_msb) << 16) |
            (static_cast<uint32_t>(pt.value.pressure_lsb) << 8) |
            static_cast<uint32_t>(pt.value.pressure_xlsb)) >> 4
        );
    int64_t p = 1048576 - adcP;
    p = (((p << 31) - vp2) * 3125) / vp1;
    vp1 = (P9 * (p >> 13) * (p >> 13)) >> 25;
    vp2 = (P8 * p) >> 19;

    p = ((p + vp1 + vp2) >> 8) + (P7 << 4);
    _pressure_pascals =  static_cast<float>(p) / 256.0F;
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-type-union-access,hicpp-signed-bitwise,readability-magic-numbers)
}

float BarometerBmp280::calculate_altitude_meters(float pressure, float temperature)
{
    (void)temperature;
    return 44330.0F * (1.0F - std::pow(pressure/_pressure_at_reference_altitude, 0.1903F)); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}
