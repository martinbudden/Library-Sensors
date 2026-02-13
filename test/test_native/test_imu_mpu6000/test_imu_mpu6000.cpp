#include <imu_mpu6000.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_mpu6000()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static ImuMpu6000 imu(ImuBase::XPOS_YPOS_ZPOS, spiFrequency, BusSpi::BUS_INDEX_0, BusSpi::spi_pins_t{});
#else
    static ImuMpu6000 imu(ImuBase::XPOS_YPOS_ZPOS, BusI2c::i2c_pins_t{});
#endif
    TEST_ASSERT_EQUAL(0, imu.get_flags());

    imu.init(8000, ImuBase::GYRO_FULL_SCALE_MAX, ImuBase::ACC_FULL_SCALE_MAX, nullptr);
    TEST_ASSERT_EQUAL(4, imu.get_gyro_id_msp());
    TEST_ASSERT_EQUAL(3, imu.get_acc_id_msp());

    TEST_ASSERT_EQUAL_FLOAT(2000.0F / 32768.0F, imu.get_gyro_resolution_dps());
    TEST_ASSERT_EQUAL_FLOAT(16.0F / 32768.0F, imu.get_acc_resolution());
    TEST_ASSERT_EQUAL(8000, imu.get_gyro_sample_rate_hz());
    TEST_ASSERT_EQUAL(1000, imu.get_acc_sample_rate_hz());

    imu.init(4000, ImuBase::GYRO_FULL_SCALE_2000_DPS, ImuBase::ACC_FULL_SCALE_16G, nullptr);
    TEST_ASSERT_EQUAL_FLOAT(2000.0F / 32768.0F, imu.get_gyro_resolution_dps());
    TEST_ASSERT_EQUAL_FLOAT(16.0F / 32768.0F, imu.get_acc_resolution());
    TEST_ASSERT_EQUAL(4000, imu.get_gyro_sample_rate_hz());
    TEST_ASSERT_EQUAL(1000, imu.get_acc_sample_rate_hz());

    imu.init(2000, ImuBase::GYRO_FULL_SCALE_1000_DPS, ImuBase::ACC_FULL_SCALE_8G, nullptr);
    TEST_ASSERT_EQUAL_FLOAT(1000.0F / 32768.0F, imu.get_gyro_resolution_dps());
    TEST_ASSERT_EQUAL_FLOAT(8.0F / 32768.0F, imu.get_acc_resolution());
    TEST_ASSERT_EQUAL(2000, imu.get_gyro_sample_rate_hz());
    TEST_ASSERT_EQUAL(1000, imu.get_acc_sample_rate_hz());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_mpu6000);

    UNITY_END();
}
