#include <imu_lsm6ds3tr_c.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_lsm6ds3tr_c()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    const ImuLsmds63trC imu(ImuBase::XPOS_YPOS_ZPOS, spiFrequency, BusSpi::BUS_INDEX_0,
        BusSpi::spi_pins_t{});
#else
    constexpr uint8_t SDA_pin = 0;
    constexpr uint8_t SCL_pin = 0;
    const ImuLsmds63trC imu(ImuBase::XPOS_YPOS_ZPOS, BusI2c::i2c_pins_t{.sda=IMU_I2C_SDA_PIN, .scl=IMU_I2C_SCL_PIN});
#endif
    TEST_ASSERT_EQUAL(0, imu.getFlags());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_lsm6ds3tr_c);

    UNITY_END();
}
