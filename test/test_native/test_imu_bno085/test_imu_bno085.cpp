#include <imu_bno085.h>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

ImuBno085& newBNO085()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static ImuBno085 imu(ImuBase::XPOS_YPOS_ZPOS, spiFrequency, BusSpi::BUS_INDEX_0, BusSpi::spi_pins_t{});
#else
    static ImuBno085 imu(ImuBase::XPOS_YPOS_ZPOS, BusI2c::i2c_pins_t{});
#endif
    return imu;
}
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_bno085_channel_input_sensor_reports()
{
    ImuBno085 imu = newBNO085();
    ImuBno085::ShtpPacket packet;

    packet.header.channel = ImuBno085::CHANNEL_INPUT_SENSOR_REPORTS;
    packet.data[9] = 0x03;
    packet.data[10] = 0x05;
    packet.data[11] = 0x38;
    packet.data[12] = 0x75;
    packet.data[13] = 0x61;
    packet.data[14] = 0xD7;
    packet.data[15] = 0x11;
    packet.data[16] = 0xF9;

    packet.data[5] = ImuBno085::SENSOR_REPORTID_ACCELEROMETER;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_ACCELEROMETER, imu.parse_input_sensor_report(packet));
    const ImuBno085::sensor_output_t acc = imu.get_acc_data();
    TEST_ASSERT_EQUAL(0x0503, acc.x);
    TEST_ASSERT_EQUAL(0x7538, acc.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), acc.z);

    packet.data[5] = ImuBno085::SENSOR_REPORTID_GYROSCOPE_CALIBRATED;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_GYROSCOPE_CALIBRATED, imu.parse_input_sensor_report(packet));
    const ImuBno085::sensor_output_t gyro_rps = imu.get_gyro_rps_data();
    TEST_ASSERT_EQUAL(0x0503, gyro_rps.x);
    TEST_ASSERT_EQUAL(0x7538, gyro_rps.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), gyro_rps.z);

    packet.data[5] = ImuBno085::SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED, imu.parse_input_sensor_report(packet));
    const ImuBno085::sensor_output_t mag = imu.get_mag_data();
    TEST_ASSERT_EQUAL(0x0503, mag.x);
    TEST_ASSERT_EQUAL(0x7538, mag.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), mag.z);

    packet.data[5] = ImuBno085::SENSOR_REPORTID_LINEAR_ACCELERATION;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_LINEAR_ACCELERATION, imu.parse_input_sensor_report(packet));
    const ImuBno085::sensor_output_t accLinear = imu.get_acc_linear_data();
    TEST_ASSERT_EQUAL(0x0503, accLinear.x);
    TEST_ASSERT_EQUAL(0x7538, accLinear.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), accLinear.z);

    packet.data[5] = ImuBno085::SENSOR_REPORTID_GRAVITY;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_GRAVITY, imu.parse_input_sensor_report(packet));
    const ImuBno085::sensor_output_t gravity = imu.get_gravity_data();
    TEST_ASSERT_EQUAL(0x0503, gravity.x);
    TEST_ASSERT_EQUAL(0x7538, gravity.y);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), gravity.z);

    packet.data[5] = ImuBno085::SENSOR_REPORTID_GAME_ROTATION_VECTOR;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_GAME_ROTATION_VECTOR, imu.parse_input_sensor_report(packet));
    const ImuBno085::rotation_vector_t rotation_vector = imu.get_rotation_vector_data();
    TEST_ASSERT_EQUAL(0x0503, rotation_vector.i);
    TEST_ASSERT_EQUAL(0x7538, rotation_vector.j);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), rotation_vector.k);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xF911), rotation_vector.real);
}

void test_bno085_channel_gyro_integrated_rotation_vector_report()
{
    ImuBno085 imu = newBNO085();

    ImuBno085::ShtpPacket packet;

    packet.header.channel = ImuBno085::CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT;
    packet.data[0] = 0x03;
    packet.data[1] = 0x05;
    packet.data[2] = 0x38;
    packet.data[3] = 0x75;
    packet.data[4] = 0x61;
    packet.data[5] = 0xD7;
    packet.data[6] = 0x11;
    packet.data[7] = 0xF9;
    TEST_ASSERT_EQUAL(ImuBno085::SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, imu.parseGyro_integrated_rotation_vectorReport(packet));

    const ImuBno085::gyro_integrated_rotation_vector_t gyroRotation = imu.get_gyro_integrated_rotation_vector_data();
    TEST_ASSERT_EQUAL(0x0503, gyroRotation.i);
    TEST_ASSERT_EQUAL(0x7538, gyroRotation.j);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xD761), gyroRotation.k);
    TEST_ASSERT_EQUAL(static_cast<int16_t>(0xF911), gyroRotation.real);

    const Quaternion orientation = imu.read_orientation();
    constexpr int orientation_Q_point = 14;
    //constexpr int gyro_Q_point = 10;

    // BNO085 uses [real, i, j, k] for quaternion, VectorQuaternionMatrix uses [w, x, y, z]
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.real) * powf(2, -orientation_Q_point), orientation.getW());
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.i) * powf(2, -orientation_Q_point), orientation.getX());
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.j) * powf(2, -orientation_Q_point), orientation.getY());
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(gyroRotation.k) * powf(2, -orientation_Q_point), orientation.getZ());

    constexpr int Q_point = 14;
    const float multiplier = pow(2, Q_point * -1);
    constexpr float multiplier2 = 1.0F / (1U << Q_point); // NOLINT(hicpp-signed-bitwise)
    TEST_ASSERT_EQUAL_FLOAT(multiplier, multiplier2);
}

void test_bno085()
{
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    constexpr uint32_t spiFrequency = 2000000;
    static const ImuBno085 imu(ImuBase::XPOS_YPOS_ZPOS, spiFrequency, BusSpi::BUS_INDEX_0, BusSpi::spi_pins_t{});
#else
    static const ImuBno085 imu(ImuBase::XPOS_YPOS_ZPOS, BusI2c::i2c_pins_t{});
#endif
    TEST_ASSERT_EQUAL(ImuBase::IMU_AUTO_CALIBRATES | ImuBase::IMU_PERFORMS_SENSOR_FUSION, imu.get_flags());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_bno085_channel_input_sensor_reports);
    RUN_TEST(test_bno085_channel_gyro_integrated_rotation_vector_report);
    RUN_TEST(test_bno085);

    UNITY_END();
}
