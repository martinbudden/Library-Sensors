#include <Arduino.h>
#include <BarometerBMP280.h>

//#if defined(TARGET_M5STACK_STAMPS3_FLY)
static constexpr BUS_BASE::bus_index_e i2cBusIndex = BUS_BASE::BUS_INDEX_0;
static constexpr BUS_I2C::i2c_pins_t i2cPins {.sda=3,.scl=4,.irq=BUS_I2C::IRQ_NOT_SET};
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
    Serial.printf("Reference Altitude:%f\r\n", barometer->getReferenceAltitude());
    Serial.printf("Pressure at reference Altitude:%f\r\n", barometer->getPressureAtReferenceAltitude());
}

void loop()
{
    barometer->readPressurePascals();

    const float pressurePascals = barometer->getPressurePascals();
    const float temperatureCelsius = barometer->getTemperatureCelsius();
    const float altitudeMeters = barometer->calculateAltitudeMeters(pressurePascals);

    Serial.println();
    Serial.printf("Pressure:%8.2f, altitude:%6.2f, Temperature:%4.1f\r\n", static_cast<double>(pressurePascals), static_cast<double>(altitudeMeters), static_cast<double>(temperatureCelsius));

    delay(500);
}
