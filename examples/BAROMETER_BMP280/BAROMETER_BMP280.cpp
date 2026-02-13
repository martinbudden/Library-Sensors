#include <Arduino.h>
#include <BarometerBMP280.h>
#include <Filters.h>

//#if defined(TARGET_M5STACK_STAMPS3_FLY)
static constexpr BusBase::bus_index_e i2cBusIndex = BusBase::BUS_INDEX_0;
static constexpr BusI2c::i2c_pins_t i2cPins {.sda=3,.scl=4,.irq=BusI2c::IRQ_NOT_SET};
//#endif


BarometerBMP280* barometer {};

void setup()
{
    Serial.begin(115200);
    delay(1000); // delay to allow serial port to initialize before first print

    static BarometerBMP280 bmp280(i2cBusIndex , i2cPins, BarometerBMP280::I2C_ADDRESS);

    barometer = &bmp280;

    const int sampleRate = barometer->init();

    Serial.printf("\r\nSample rate:%d\r\n", sampleRate);
    Serial.printf("Reference Altitude:%f\r\n", barometer->get_reference_altitude());
    Serial.printf("Pressure at reference Altitude:%f\r\n", barometer->get_pressure_at_reference_altitude());
}

float calculate_altitude_meters(float pressure, float temperature)
{
    return (std::pow((101325.0F/pressure), 1.0F/5.257F) - 1.0F) * (temperature + 273.15F) / 0.0065F;
}

void loop()
{
    static uint32_t timePreviousMs =  0;
    static FilterMovingAverage<100> altitudeFilter {};

    barometer->read_temperature_and_pressure();

    const float pressure_pascals = barometer->get_pressure_pascals();
    const float temperature_celsius = barometer->get_temperature_celsius();
    const float altitudeMeters = barometer->calculate_altitude_meters(pressure_pascals, temperature_celsius);
    const float alt2 = calculate_altitude_meters(pressure_pascals, temperature_celsius);
    const float altitudeMA = altitudeFilter.filter(altitudeMeters);

    uint32_t timeMs = millis();
    if (timeMs > timePreviousMs + 500) {
        timePreviousMs = timeMs;
        Serial.println();
        Serial.printf("Pressure:%8.2f,  Temperature:%4.1f\r\n", static_cast<double>(pressure_pascals), static_cast<double>(temperature_celsius));
        Serial.printf("altitude:%6.2f,  alt2:%6.2f, altitudeMA:%6.2f\r\n", static_cast<double>(altitudeMeters), static_cast<double>(alt2), static_cast<double>(altitudeMA));
    }
    delay(25);
}
