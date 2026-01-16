#include "BarometerQMP6988.h"
//#define LIBRARY_SENSORS_SERIAL_DEBUG
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
#include <HardwareSerial.h>
#endif

#include <cmath>

namespace { // use anonymous namespace to make items local to this translation unit
    constexpr uint8_t REG_CALIBRATION_DATA = 0xA0;
    constexpr uint8_t REG_CHIPID        = 0xD1;
        constexpr uint8_t CHIPID_RESPONSE  = 0x5C;
    constexpr uint8_t REG_VERSION       = 0xD1;
    constexpr uint8_t REG_RESET         = 0xE0;
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
        constexpr uint8_t MEASUREMENT_MODE = (TEMPERATURE_OSR << 5) | (PRESSURE_OSR << 2) | MODE_NORMAL; // NOLINT(hicpp-signed-bitwise)
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
BarometerQMP6988::BarometerQMP6988(uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins) :
    BarometerBase(_bus),
    _bus(frequency, SPI_index, pins)
{
}
BarometerQMP6988::BarometerQMP6988(uint32_t frequency, BUS_BASE::bus_index_e SPI_index, const BUS_SPI::spi_pins_t& pins) :
    BarometerBase(_bus),
    _bus(frequency, SPI_index, pins)
{
}
#else
BarometerQMP6988::BarometerQMP6988(BUS_BASE::bus_index_e I2C_index, const BUS_I2C::i2c_pins_t& pins, uint8_t I2C_address) :
    BarometerBase(_bus),
    _bus(I2C_address, I2C_index, pins)
{
}
#endif

int BarometerQMP6988::init()
{
#if !defined(FRAMEWORK_TEST)
    const uint8_t chipID = _bus.readRegisterWithTimeout(REG_CHIPID, 100);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.print("IMU init, chipID:0x");
    Serial.println(chipID, HEX);
#endif
    if (chipID != CHIPID_RESPONSE) {
        return NOT_DETECTED;
    }
#endif

    readCalibrationData();
    delayMs(1);


// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,hicpp-signed-bitwise,readability-magic-numbers)
    _bus.writeRegister(REG_CONTROL, MEASUREMENT_MODE);
    delayMs(1);

    _referenceAltitude = 0.0F;

    enum { TIME_INIT_MAX = 20 }; // 20/16 = 1.25 ms
    enum { TIME_MEASURE_PER_OSRS_MAX = 37 }; // 37/16 = 2.3125 ms
    enum { TIME_SETUP_PRESSURE_MAX = 10 }; // 10/16 = 0.625 ms

    constexpr uint32_t delayMs = ((TIME_INIT_MAX + TIME_MEASURE_PER_OSRS_MAX * (((1 << TEMPERATURE_OSR) >> 1) + ((1 << PRESSURE_OSR) >> 1)) + TIME_SETUP_PRESSURE_MAX + 15) / 16);
    _sampleRateHz = 1000 / delayMs; // delay ~ 23ms, sample rate ~ 43.5Hz
    return static_cast<int>(_sampleRateHz);
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,hicpp-signed-bitwise,readability-magic-numbers)
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-type-union-access,hicpp-signed-bitwise,readability-magic-numbers)
void BarometerQMP6988::readCalibrationData()
{
    std::array<uint8_t, CALIBRATION_DATA_SIZE> data {};
    _bus.readRegister(REG_CALIBRATION_DATA, &data[0], CALIBRATION_DATA_SIZE);

    calibration_data_t calibration {};

    calibration.a0 = static_cast<int32_t>(((data[18] << 12) | (data[19] << 4) | (data[24] & 0x0F)) << 12);
    calibration.a0 >>= 12;

    calibration.a1 = static_cast<int16_t>(((data[20]) << 8) | data[21]);
    calibration.a2 = static_cast<int16_t>(((data[22]) << 8) | data[23]);

    calibration.b00 = static_cast<int32_t>(((data[0] << 12) | (data[1] << 4) | ((data[24] & 0xf0) >> 4)) << 12);
    calibration.b00 >>= 12;

    calibration.bt1 = static_cast<int16_t>(((data[2])  << 8) | data[3]);
    calibration.bt2 = static_cast<int16_t>(((data[4])  << 8) | data[5]);
    calibration.bp1 = static_cast<int16_t>(((data[6])  << 8) | data[7]);
    calibration.b11 = static_cast<int16_t>(((data[8])  << 8) | data[9]);
    calibration.bp2 = static_cast<int16_t>(((data[10]) << 8) | data[11]);
    calibration.b12 = static_cast<int16_t>(((data[12]) << 8) | data[13]);
    calibration.b21 = static_cast<int16_t>(((data[14]) << 8) | data[15]);
    calibration.bp3 = static_cast<int16_t>(((data[16]) << 8) | data[17]);

    _ikData.a0  = calibration.a0;   // 20Q4
    _ikData.b00 = calibration.b00;  // 20Q4

    _ikData.a1 = 3608L    * static_cast<int32_t>(calibration.a1)  - 1731677965L; // 31Q23
    _ikData.a2 = 16889L   * static_cast<int32_t>(calibration.a2)  - 87619360L;   // 30Q47

    _ikData.bt1 = 2982L   * static_cast<int64_t>(calibration.bt1) + 107370906L;  // 28Q15
    _ikData.bt2 = 329854L * static_cast<int64_t>(calibration.bt2) + 108083093L;  // 34Q38
    _ikData.bp1 = 19923L  * static_cast<int64_t>(calibration.bp1) + 1133836764L; // 31Q20
    _ikData.b11 = 2406L   * static_cast<int64_t>(calibration.b11) + 118215883L;  // 28Q34
    _ikData.bp2 = 3079L   * static_cast<int64_t>(calibration.bp2) - 181579595L;  // 29Q43
    _ikData.b12 = 6846L   * static_cast<int64_t>(calibration.b12) + 85590281L;   // 29Q53
    _ikData.b21 = 13836L  * static_cast<int64_t>(calibration.b21) + 79333336L;   // 29Q60
    _ikData.bp3 = 2915L   * static_cast<int64_t>(calibration.bp3) + 157155561L;  // 28Q65
}

void BarometerQMP6988::readTemperatureAndPressure()
{
    pressure_temperature_data_u pt; // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init,misc-const-correctness)
    _bus.readRegister(REG_PRESSURE_MSB, &pt.data[0], sizeof(pt));

    constexpr int32_t OFFSET = 8388608;
    const auto pressureRead =
        static_cast<uint32_t>((static_cast<uint32_t>(pt.data[0]) << 16) | (static_cast<uint16_t>(pt.data[1]) << 8) | pt.data[2]);
    const auto pressureRaw  = static_cast<int32_t>(pressureRead - OFFSET);

    const auto temperatureRead =
        static_cast<uint32_t>((static_cast<uint32_t>(pt.data[3]) << 16) | (static_cast<uint16_t>(pt.data[4]) << 8) | pt.data[5]);
    const auto temperatureRaw  = static_cast<int32_t>(temperatureRead - OFFSET);

    const int16_t temperature = convertTemperature(temperatureRaw);
    const int32_t pressure = convertPressure(pressureRaw, temperature);
    _temperatureCelsius = static_cast<float>(temperature) / 256.0F;
    _pressurePascals    = static_cast<float>(pressure) / 16.0F;
}

int16_t BarometerQMP6988::convertTemperature(int32_t dt) const
{
    const int64_t dt64 = dt;
    // wk1: 60Q4 // bit size
    const int64_t wk1 = (static_cast<int64_t>(_ikData.a1) * dt64);      // 31Q23+24-1=54 (54Q23)
    int64_t wk2 = (static_cast<int64_t>(_ikData.a2) * dt64) >> 14;      // 30Q47+24-1=53 (39Q33)
    wk2 = (wk2 * dt64) >> 10;                                           // 39Q33+24-1=62 (52Q23)
    wk2 = ((wk1 + wk2) / 32767) >> 19;                                  // 54,52->55Q23 (20Q04)

    const auto ret = static_cast<int16_t>((_ikData.a0 + wk2) >> 4);  // 21Q4 -> 17Q0
    return ret;
}

int32_t BarometerQMP6988::convertPressure(int32_t dp, int16_t tx) const
{
    const int64_t dp64 = dp;
    const int64_t tx64 = tx;

    // wk1 = 48Q16 // bit size
    int64_t wk1 = (_ikData.bt1 * tx64);         // 28Q15+16-1=43 (43Q15)
    int64_t wk2 = (_ikData.bp1 * dp64) >> 5;    // 31Q20+24-1=54 (49Q15)
    wk1 += wk2;  // 43,49->50Q15
    wk2 = (_ikData.bt2 * tx64) >> 1;            // 34Q38+16-1=49 (48Q37)
    wk2 = (wk2 * tx64) >> 8;                    // 48Q37+16-1=63 (55Q29)
    int64_t wk3 = wk2;                          // 55Q29
    wk2 = (_ikData.b11 * tx64) >> 4;            // 28Q34+16-1=43 (39Q30)
    wk2 = (wk2 * dp64) >> 1;                    // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                                 // 55,61->62Q29
    wk2 = (_ikData.bp2 * dp64) >> 13;           // 29Q43+24-1=52 (39Q30)
    wk2 = (wk2 * dp64) >> 1;                    // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                                 // 62,61->63Q29
    wk1 += wk3 >> 14;                           // Q29 >> 14 -> Q15
    wk2 = (_ikData.b12 * tx64);                 // 29Q53+16-1=45 (45Q53)
    wk2 = (wk2 * tx64) >> 22;                   // 45Q53+16-1=61 (39Q31)
    wk2 = (wk2 * dp64) >> 1;                    // 39Q31+24-1=62 (61Q30)
    wk3 = wk2;                                  // 61Q30
    wk2 = (_ikData.b21 * tx64) >> 6;            // 29Q60+16-1=45 (39Q54)
    wk2 = (wk2 * dp64) >> 23;                   // 39Q54+24-1=62 (39Q31)
    wk2 = (wk2 * dp64) >> 1;                    // 39Q31+24-1=62 (61Q20)
    wk3 += wk2;                                 // 61,61->62Q30
    wk2 = (_ikData.bp3 * dp64) >> 12;           // 28Q65+24-1=51 (39Q53)
    wk2 = (wk2 * dp64) >> 23;                   // 39Q53+24-1=62 (39Q30)
    wk2 = (wk2 * dp64);                         // 39Q30+24-1=62 (62Q30)
    wk3 += wk2;                                 // 62,62->63Q30
    wk1 += wk3 >> 15;                           // Q30 >> 15 = Q15
    wk1 /= 32767L;
    wk1 >>= 11;                                 // Q15 >> 7 = Q4
    wk1 += _ikData.b00;                         // Q4 + 20Q4
    // wk1 >>= 4; // 28Q4 -> 24Q0

    const auto ret = static_cast<int32_t>(wk1);
    return ret;
}

float BarometerQMP6988::calculateAltitudeMeters(float pressure, float temperature)
{
    return (std::pow((101325.0F / pressure), 1.0F / 5.257F) - 1.0F) * (temperature + 273.15F) / 0.0065F;
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-type-union-access,hicpp-signed-bitwise,readability-magic-numbers)
