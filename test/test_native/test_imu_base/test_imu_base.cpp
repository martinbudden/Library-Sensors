#include <imu_null.h>
#include <unity.h>



void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)
void test_map_axes()
{
    const xyz_t input { .x =3.0F, .y = 5.0F, .z = 7.0F };
    xyz_t output {};

    static const ImuNull XPOS_YPOS_ZPOS(ImuBase::XPOS_YPOS_ZPOS);
    output = XPOS_YPOS_ZPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    static const ImuNull YPOS_XNEG_ZPOS(ImuBase::YPOS_XNEG_ZPOS);
    output = YPOS_XNEG_ZPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    static const ImuNull XNEG_YNEG_ZPOS(ImuBase::XNEG_YNEG_ZPOS);
    output = XNEG_YNEG_ZPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    static const ImuNull YNEG_XPOS_ZPOS(ImuBase::YNEG_XPOS_ZPOS);
    output = YNEG_XPOS_ZPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);


    static const ImuNull XPOS_YNEG_ZNEG(ImuBase::XPOS_YNEG_ZNEG);
    output = XPOS_YNEG_ZNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);

    static const ImuNull YPOS_XPOS_ZNEG(ImuBase::YPOS_XPOS_ZNEG);
    output = YPOS_XPOS_ZNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);

    static const ImuNull XNEG_YPOS_ZNEG(ImuBase::XNEG_YPOS_ZNEG);
    output = XNEG_YPOS_ZNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);

    static const ImuNull YNEG_XNEG_ZNEG(ImuBase::YNEG_XNEG_ZNEG);
    output = YNEG_XNEG_ZNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.z);


    static const ImuNull ZPOS_YNEG_XPOS(ImuBase::ZPOS_YNEG_XPOS);
    output = ZPOS_YNEG_XPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);

    static const ImuNull YPOS_ZPOS_XPOS(ImuBase::YPOS_ZPOS_XPOS);
    output = YPOS_ZPOS_XPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);

    static const ImuNull ZNEG_YPOS_XPOS(ImuBase::ZNEG_YPOS_XPOS);
    output = ZNEG_YPOS_XPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);

    static const ImuNull YNEG_ZNEG_XPOS(ImuBase::YNEG_ZNEG_XPOS);
    output = YNEG_ZNEG_XPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.z);


    static const ImuNull ZPOS_YPOS_XNEG(ImuBase::ZPOS_YPOS_XNEG);
    output = ZPOS_YPOS_XNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);

    static const ImuNull YPOS_ZNEG_XNEG(ImuBase::YPOS_ZNEG_XNEG);
    output = YPOS_ZNEG_XNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);

    static const ImuNull ZNEG_YNEG_XNEG(ImuBase::ZNEG_YNEG_XNEG);
    output = ZNEG_YNEG_XNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);

    static const ImuNull YNEG_ZPOS_XNEG(ImuBase::YNEG_ZPOS_XNEG);
    output = YNEG_ZPOS_XNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.z);


    static const ImuNull ZPOS_XPOS_YPOS(ImuBase::ZPOS_XPOS_YPOS);
    output = ZPOS_XPOS_YPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);

    static const ImuNull XNEG_ZPOS_YPOS(ImuBase::XNEG_ZPOS_YPOS);
    output = XNEG_ZPOS_YPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);

    static const ImuNull ZNEG_XNEG_YPOS(ImuBase::ZNEG_XNEG_YPOS);
    output = ZNEG_XNEG_YPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);

    static const ImuNull XPOS_ZNEG_YPOS(ImuBase::XPOS_ZNEG_YPOS);
    output = XPOS_ZNEG_YPOS.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.z);


    static const ImuNull ZPOS_XNEG_YNEG(ImuBase::ZPOS_XNEG_YNEG);
    output = ZPOS_XNEG_YNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);

    static const ImuNull XNEG_ZNEG_YNEG(ImuBase::XNEG_ZNEG_YNEG);
    output = XNEG_ZNEG_YNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);

    static const ImuNull ZNEG_XPOS_YNEG(ImuBase::ZNEG_XPOS_YNEG);
    output = ZNEG_XPOS_YNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(-input.z, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);

    static const ImuNull XPOS_ZPOS_YNEG(ImuBase::XPOS_ZPOS_YNEG);
    output = XPOS_ZPOS_YNEG.map_axes(input);
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.y);
    TEST_ASSERT_EQUAL_FLOAT(-input.y, output.z);
}

void test_map_axes_inversion()
{
    const xyz_t input { .x =3.0F, .y = 5.0F, .z = 7.0F };

    for (uint8_t axis_order = ImuBase::XPOS_YPOS_ZPOS; axis_order <= ImuBase::YNEG_XPOS_ZPOS_45; ++axis_order) {
        const xyz_t intermediate = ImuBase::map_axes(input, axis_order);
        const xyz_t output =       ImuBase::map_axes(intermediate, ImuBase::axis_order_inverse(axis_order));
        TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
        TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
        TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);
    }

    auto axis_order = ImuBase::XPOS_YPOS_ZPOS_45;
    xyz_t intermediate = ImuBase::map_axes(input, axis_order);
    xyz_t output =       ImuBase::map_axes(intermediate, ImuBase::axis_order_inverse(axis_order));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    axis_order = ImuBase::YPOS_XNEG_ZPOS_45;
    intermediate = ImuBase::map_axes(input, axis_order);
    output =       ImuBase::map_axes(intermediate, ImuBase::axis_order_inverse(axis_order));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    axis_order = ImuBase::XNEG_YNEG_ZPOS_45;
    intermediate = ImuBase::map_axes(input, axis_order);
    output =       ImuBase::map_axes(intermediate, ImuBase::axis_order_inverse(axis_order));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);

    axis_order = ImuBase::YNEG_XPOS_ZPOS_45;
    intermediate = ImuBase::map_axes(input, axis_order);
    output =       ImuBase::map_axes(intermediate, ImuBase::axis_order_inverse(axis_order));
    TEST_ASSERT_EQUAL_FLOAT(input.x, output.x);
    TEST_ASSERT_EQUAL_FLOAT(input.y, output.y);
    TEST_ASSERT_EQUAL_FLOAT(input.z, output.z);
}

void test_raw()
{
    static ImuNull imu;

    ImuBase::xyz_int32_t gyroRaw = imu.read_gyro_raw();
    TEST_ASSERT_EQUAL(0, gyroRaw.x);
    TEST_ASSERT_EQUAL(0, gyroRaw.y);
    TEST_ASSERT_EQUAL(0, gyroRaw.z);

    imu.setGyroRaw({3, 5, 7});
    gyroRaw = imu.read_gyro_raw();
    TEST_ASSERT_EQUAL(3, gyroRaw.x);
    TEST_ASSERT_EQUAL(5, gyroRaw.y);
    TEST_ASSERT_EQUAL(7, gyroRaw.z);

    ImuBase::xyz_int32_t accRaw = imu.read_acc_raw();
    TEST_ASSERT_EQUAL(0, accRaw.x);
    TEST_ASSERT_EQUAL(0, accRaw.y);
    TEST_ASSERT_EQUAL(0, accRaw.z);

    imu.setAccRaw({11, 13, 17});
    accRaw = imu.read_acc_raw();
    TEST_ASSERT_EQUAL(11, accRaw.x);
    TEST_ASSERT_EQUAL(13, accRaw.y);
    TEST_ASSERT_EQUAL(17, accRaw.z);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-init-variables,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_map_axes);
    RUN_TEST(test_map_axes_inversion);
    RUN_TEST(test_raw);

    UNITY_END();
}
