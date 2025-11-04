#include <BarometerBMP280.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_bmp280()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static const BarometerBMP280 barometer(spiFrequency, BUS_SPI::BUS_INDEX_0, BUS_SPI::spi_pins_t{});
#else
    static const BarometerBMP280 barometer(BUS_I2C::i2c_pins_t{});
#endif
    barometer.init();
    TEST_ASSERT_EQUAL(0.0F, BarometerBMP280::calculateAltitude(1.0F, 1.0F));
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_bps280);

    UNITY_END();
}
