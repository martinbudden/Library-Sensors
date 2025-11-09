#include <IMU_MPU6886.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_mpu6886()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 1000000;
    static const IMU_MPU6886 imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BUS_SPI::BUS_INDEX_0, BUS_SPI::spi_pins_t{});
#else
    static const IMU_MPU6886 imu(IMU_Base::XPOS_YPOS_ZPOS, BUS_I2C::i2c_pins_t{});
#endif
    TEST_ASSERT_EQUAL(0, imu.getFlags());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_mpu6886);

    UNITY_END();
}
