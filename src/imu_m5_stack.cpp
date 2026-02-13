#if defined(LIBRARY_SENSORS_IMU_USE_M5_STACK)

#include "imu_m5_stack.h"
#include <M5Stack.h>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };
} // end namespace


ImuM5Stack::ImuM5Stack(uint8_t axis_order) :
    ImuBase(axis_order, IMU_AUTO_CALIBRATES)
{
    _gyro_resolution_dps = GYRO_2000DPS_RES;
    _gyro_resolution_rps = GYRO_2000DPS_RES * DEGREES_TO_RADIANS;
    _acc_resolution = ACC_8G_RES;
    _gyro_sample_rate_hz = 500;
    _acc_sample_rate_hz = 500;
}

int ImuM5Stack::init(uint32_t outputDataRateHz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex)
{
    (void)outputDataRateHz;
    (void)gyro_sensitivity;
    (void)acc_sensitivity;

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyro_id_msp = MSP_GYRO_ID_DEFAULT;
    _acc_id_msp = MSP_ACC_ID_DEFAULT;

#if defined(FRAMEWORK_USE_FREERTOS)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    _bus_mutex = bus_mutex;
#endif

    // Set up FIFO for IMU
    // IMU data frequency is 500Hz
    bus_semaphore_take(_bus_mutex);
    M5.IMU.setFIFOEnable(false);
    bus_semaphore_give(_bus_mutex);

    // return the gyro sample rate actually set
    return _gyro_sample_rate_hz;
}

ImuBase::xyz_int32_t ImuM5Stack::read_acc_raw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    bus_semaphore_take(_bus_mutex);
    M5.IMU.get_accelAdc(&x, &y, &z);
    bus_semaphore_give(_bus_mutex);

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t ImuM5Stack::read_acc()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    bus_semaphore_take(_bus_mutex);
    M5.IMU.get_accelAdc(&x, &y, &z);
    bus_semaphore_give(_bus_mutex);

    const xyz_t acc {
        .x = static_cast<float>(x) * _acc_resolution - _acc_offset.x,
        .y = static_cast<float>(y) * _acc_resolution - _acc_offset.y,
        .z = static_cast<float>(z) * _acc_resolution - _acc_offset.z
    };
    return map_axes(acc);
}

ImuBase::xyz_int32_t ImuM5Stack::read_gyro_raw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    bus_semaphore_take(_bus_mutex);
    M5.IMU.get_gyroAdc(&x, &y, &z);
    bus_semaphore_give(_bus_mutex);

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t ImuM5Stack::read_gyro_rps()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    bus_semaphore_take(_bus_mutex);
    M5.IMU.get_gyroAdc(&x, &y, &z);
    bus_semaphore_give(_bus_mutex);

    const xyz_t gyroRPS {
        .x = static_cast<float>(x) * _gyro_resolution_rps - _gyro_offset.x,
        .y = static_cast<float>(y) * _gyro_resolution_rps - _gyro_offset.y,
        .z = static_cast<float>(z) * _gyro_resolution_rps - _gyro_offset.z,
    };
    return map_axes(gyroRPS);
}

xyz_t ImuM5Stack::read_gyro_dps()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    bus_semaphore_take(_bus_mutex);
    M5.IMU.get_gyroAdc(&x, &y, &z);
    bus_semaphore_give(_bus_mutex);

    const xyz_t gyroDPS {
        .x = static_cast<float>(x - _gyro_offset.x) * _gyro_resolution_dps,
        .y = static_cast<float>(y - _gyro_offset.y) * _gyro_resolution_dps,
        .z = static_cast<float>(z - _gyro_offset.z) * _gyro_resolution_dps,
    };
    return map_axes(gyroDPS);
}

FAST_CODE acc_gyro_rps_t ImuM5Stack::read_acc_gyro_rps()
{
    const acc_gyro_rps_t gyroAcc {
        .gyroRPS = read_gyro_rps(),
        .acc = read_acc()
    };

    return gyroAcc;
}

acc_gyro_rps_t ImuM5Stack::acc_gyro_rpsFromRaw(const acc_temperature_gyro_data_t& data) const
{
// NOLINTBEGIN(hicpp-signed-bitwise)
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x),
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .y = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _acc_resolution - _acc_offset.x),
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x),
            .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _acc_resolution - _acc_offset.x),
            .y = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y),
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _acc_resolution - _acc_offset.y),
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return acc_gyro_rps_t {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z,
            .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y)
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _acc_resolution - _acc_offset.z,
            .z = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _acc_resolution - _acc_offset.y)
        }
    };
#else
    // Axis order mapping done at run-time
    const acc_gyro_rps_t acc_gyro_rps {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyro_resolution_rps - _gyro_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyro_resolution_rps - _gyro_offset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyro_resolution_rps - _gyro_offset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _acc_resolution - _acc_offset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _acc_resolution - _acc_offset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _acc_resolution - _acc_offset.z
        }
    };

    switch (_axis_order) {
    case XPOS_YPOS_ZPOS:
        return acc_gyro_rps;
        break;
    case YNEG_XPOS_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -acc_gyro_rps.gyroRPS.y,
                .y =  acc_gyro_rps.gyroRPS.x,
                .z =  acc_gyro_rps.gyroRPS.z
            },
            .acc = {
                .x = -acc_gyro_rps.acc.y,
                .y =  acc_gyro_rps.acc.x,
                .z =  acc_gyro_rps.acc.z
            }
        };
        break;
    case XNEG_YNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x = -acc_gyro_rps.gyroRPS.x,
                .y = -acc_gyro_rps.gyroRPS.y,
                .z =  acc_gyro_rps.gyroRPS.z
            },
            .acc = {
                .x = -acc_gyro_rps.acc.x,
                .y = -acc_gyro_rps.acc.y,
                .z =  acc_gyro_rps.acc.z
            }
        };
        break;
    case YPOS_XNEG_ZPOS:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  acc_gyro_rps.gyroRPS.y,
                .y = -acc_gyro_rps.gyroRPS.x,
                .z =  acc_gyro_rps.gyroRPS.z
            },
            .acc = {
                .x =  acc_gyro_rps.acc.y,
                .y = -acc_gyro_rps.acc.x,
                .z =  acc_gyro_rps.acc.z
            }
        };
        break;
    case XPOS_ZPOS_YNEG:
        return acc_gyro_rps_t {
            .gyroRPS = {
                .x =  acc_gyro_rps.gyroRPS.x,
                .y =  acc_gyro_rps.gyroRPS.z,
                .z = -acc_gyro_rps.gyroRPS.y
            },
            .acc = {
                .x = -acc_gyro_rps.acc.x,
                .y =  acc_gyro_rps.acc.z,
                .z = -acc_gyro_rps.acc.y
            }
        };
        break;
    default:
        return acc_gyro_rps_t {
            .gyroRPS = map_axes(acc_gyro_rps.gyroRPS),
            .acc = map_axes(acc_gyro_rps.acc)
        };
        break;
    } // end switch

    return acc_gyro_rps;
#endif
// NOLINTEND(hicpp-signed-bitwise)
}
#endif
