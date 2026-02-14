#if defined(LIBRARY_SENSORS_IMU_USE_M5_UNIFIED)
#include <M5Unified.h>

#include <IMU_M5Unified.h>
#include <cassert>

ImuM5Unified::ImuM5Unified(uint8_t axis_order) :
    ImuBase(axis_order, IMU_AUTO_CALIBRATES)
{
}

int ImuM5Unified::init(uint32_t output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex)
{
    (void)output_data_rate_hz;
    (void)gyro_sensitivity;
    (void)acc_sensitivity;

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyro_id_msp = MSP_GYRO_ID_DEFAULT;
    _acc_id_msp = MSP_ACC_ID_DEFAULT;

    _gyro_sample_rate_hz = 500;
    _acc_sample_rate_hz = 500;

#if defined(FRAMEWORK_USE_FREERTOS)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    _bus_mutex = bus_mutex;
#endif

#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_ZPOS_XNEG_YNEG)
    M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg);
#else
    setAxisOrder(_axis_order);
#endif

    // return the gyro sample rate actually set
    return _gyro_sample_rate_hz;
}

void ImuM5Unified::setAxisOrder(uint8_t axis_order)
{
    _axis_order = axis_order;

    switch (axis_order) {
    case XPOS_YPOS_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos);
        break;
    case YPOS_XNEG_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos);
        break;
    case XNEG_YNEG_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_pos);
        break;
    case YNEG_XPOS_ZPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos);
        break;

    case XPOS_YNEG_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_neg);
        break;
    case YPOS_XPOS_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_neg);
        break;
    case XNEG_YPOS_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_neg);
        break;
    case YNEG_XNEG_ZNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_neg);
        break;

    case ZPOS_YNEG_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_pos);
        break;
    case YPOS_ZPOS_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_pos);
        break;
    case ZNEG_YPOS_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_pos);
        break;
    case YNEG_ZNEG_XPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_pos);
        break;

    case ZPOS_YPOS_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_x_neg);
        break;
    case YPOS_ZNEG_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_pos, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_neg);
        break;
    case ZNEG_YNEG_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_x_neg);
        break;
    case YNEG_ZPOS_XNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_y_neg, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg);
        break;

    case ZPOS_XPOS_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_pos);
        break;
    case XNEG_ZPOS_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_pos);
        break;
    case ZNEG_XNEG_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_pos);
        break;
    case XPOS_ZNEG_YPOS:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_pos);
        break;

    case ZPOS_XNEG_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_y_neg);
        break;
    case XNEG_ZNEG_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg, m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_y_neg);
        break;
    case ZNEG_XPOS_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_z_neg, m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_y_neg);
        break;
    case XPOS_ZPOS_YNEG:
        M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_pos, m5::IMU_Class::axis_z_pos, m5::IMU_Class::axis_y_neg);
        break;

    default:
        assert(false && "IMU orientation invalid.");
        break;
    }
}

ImuBase::xyz_int32_t ImuM5Unified::read_acc_raw()
{
    assert(false && ("M5Unified variants should not call read_acc_raw")); // NOLINT(readability-simplify-boolean-expr)
    return xyz_int32_t {};
}

xyz_t ImuM5Unified::read_acc()
{
    // This is very slow on the M5 Atom.
    bus_semaphore_take(_bus_mutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    bus_semaphore_give(_bus_mutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t acc = {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.accel.x,
        .y = data.accel.y,
        .z = data.accel.z
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };

    return acc;
}

ImuBase::xyz_int32_t ImuM5Unified::read_gyro_raw()
{
    assert(false && ("M5Unified variants should not call read_gyro_raw")); // NOLINT(readability-simplify-boolean-expr)
    return xyz_int32_t {};
}

xyz_t ImuM5Unified::read_gyro_rps()
{
    // This is very slow on the M5 Atom.
    bus_semaphore_take(_bus_mutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    bus_semaphore_give(_bus_mutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t gyro_rps {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.gyro.x * DEGREES_TO_RADIANS,
        .y = data.gyro.y * DEGREES_TO_RADIANS,
        .z = data.gyro.z * DEGREES_TO_RADIANS
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyro_rps;
}

xyz_t ImuM5Unified::read_gyro_dps()
{
    // This is very slow on the M5 Atom.
    bus_semaphore_take(_bus_mutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    bus_semaphore_give(_bus_mutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    const xyz_t gyro_dps {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .x = data.gyro.x,
        .y = data.gyro.y,
        .z = data.gyro.z
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
    return gyro_dps;
}

FAST_CODE acc_gyro_rps_t ImuM5Unified::read_acc_gyro_rps()
{
    // This is very slow on the M5 Atom.
    bus_semaphore_take(_bus_mutex);
    [[maybe_unused]] const auto imu_update = M5.Imu.update();
    bus_semaphore_give(_bus_mutex);

    const m5::IMU_Class::imu_data_t& data = M5.Imu.getImuData();
    return acc_gyro_rps_t {
        // NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
        .gyro_rps = {
            .x = data.gyro.x * DEGREES_TO_RADIANS,
            .y = data.gyro.y * DEGREES_TO_RADIANS,
            .z = data.gyro.z * DEGREES_TO_RADIANS
        },
        .acc = {
            .x = data.accel.x,
            .y = data.accel.y,
            .z = data.accel.z
        }
        // NOLINTEND(cppcoreguidelines-pro-type-union-access)
    };
}

#endif