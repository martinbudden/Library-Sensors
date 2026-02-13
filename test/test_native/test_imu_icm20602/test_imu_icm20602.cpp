#include <imu_icm20602.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_icm20602()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 1000000;
    static const ImuIcm20602 imu(ImuBase::XPOS_YPOS_ZPOS, spiFrequency, BusSpi::BUS_INDEX_0, BusSpi::spi_pins_t{});
#else
    static const ImuIcm20602 imu(ImuBase::XPOS_YPOS_ZPOS, BusI2c::i2c_pins_t{});
#endif
    TEST_ASSERT_EQUAL(0, imu.get_flags());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_icm20602);

    UNITY_END();
}
