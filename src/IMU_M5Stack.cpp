#if defined(LIBRARY_SENSORS_IMU_USE_M5_STACK)

#include "IMU_M5Stack.h"
#include <M5Stack.h>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_8G_RES { 8.0 / 32768.0 };
} // end namespace


IMU_M5_STACK::IMU_M5_STACK(axis_order_e axisOrder) :
    IMU_Base(axisOrder, IMU_AUTO_CALIBRATES)
{
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * DEGREES_TO_RADIANS;
    _accResolution = ACC_8G_RES;
    _gyroSampleRateHz = 500;
    _accSampleRateHz = 500;
}

int IMU_M5_STACK::init(uint32_t outputDataRateHz, gyro_sensitivity_e gyroSensitivity, acc_sensitivity_e accSensitivity, void* busMutex)
{
    (void)outputDataRateHz;
    (void)gyroSensitivity;
    (void)accSensitivity;

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for MPU6886
    _gyroIdMSP = MSP_GYRO_ID_DEFAULT;
    _accIdMSP = MSP_ACC_ID_DEFAULT;

#if defined(FRAMEWORK_USE_FREERTOS)
    _busMutex = static_cast<SemaphoreHandle_t>(busMutex);
#else
    _busMutex = busMutex;
#endif

    // Set up FIFO for IMU
    // IMU data frequency is 500Hz
    busSemaphoreTake(_busMutex);
    M5.IMU.setFIFOEnable(false);
    busSemaphoreGive(_busMutex);

    // return the gyro sample rate actually set
    return _gyroSampleRateHz;
}

IMU_Base::xyz_int32_t IMU_M5_STACK::readAccRaw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    busSemaphoreTake(_busMutex);
    M5.IMU.getAccelAdc(&x, &y, &z);
    busSemaphoreGive(_busMutex);

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t IMU_M5_STACK::readAcc()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    busSemaphoreTake(_busMutex);
    M5.IMU.getAccelAdc(&x, &y, &z);
    busSemaphoreGive(_busMutex);

    const xyz_t acc {
        .x = static_cast<float>(x) * _accResolution - _accOffset.x,
        .y = static_cast<float>(y) * _accResolution - _accOffset.y,
        .z = static_cast<float>(z) * _accResolution - _accOffset.z
    };
    return mapAxes(acc);
}

IMU_Base::xyz_int32_t IMU_M5_STACK::readGyroRaw()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    busSemaphoreTake(_busMutex);
    M5.IMU.getGyroAdc(&x, &y, &z);
    busSemaphoreGive(_busMutex);

    return xyz_int32_t {.x = x, .y = y, .z = z };
}

xyz_t IMU_M5_STACK::readGyroRPS()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    busSemaphoreTake(_busMutex);
    M5.IMU.getGyroAdc(&x, &y, &z);
    busSemaphoreGive(_busMutex);

    const xyz_t gyroRPS {
        .x = static_cast<float>(x) * _gyroResolutionRPS - _gyroOffset.x,
        .y = static_cast<float>(y) * _gyroResolutionRPS - _gyroOffset.y,
        .z = static_cast<float>(z) * _gyroResolutionRPS - _gyroOffset.z,
    };
    return mapAxes(gyroRPS);
}

xyz_t IMU_M5_STACK::readGyroDPS()
{
    int16_t x {};
    int16_t y {};
    int16_t z {};

    busSemaphoreTake(_busMutex);
    M5.IMU.getGyroAdc(&x, &y, &z);
    busSemaphoreGive(_busMutex);

    const xyz_t gyroDPS {
        .x = static_cast<float>(x - _gyroOffset.x) * _gyroResolutionDPS,
        .y = static_cast<float>(y - _gyroOffset.y) * _gyroResolutionDPS,
        .z = static_cast<float>(z - _gyroOffset.z) * _gyroResolutionDPS,
    };
    return mapAxes(gyroDPS);
}

FAST_CODE IMU_Base::accGyroRPS_t IMU_M5_STACK::readAccGyroRPS()
{
    const accGyroRPS_t gyroAcc {
        .gyroRPS = readGyroRPS(),
        .acc = readAcc()
    };

    return gyroAcc;
}

IMU_Base::accGyroRPS_t IMU_M5_STACK::accGyroRPSFromRaw(const acc_temperature_gyro_data_t& data) const
{
// NOLINTBEGIN(hicpp-signed-bitwise)
#if defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YPOS_XNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x),
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .y = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _accResolution - _accOffset.x),
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XNEG_YNEG_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x),
            .y = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y),
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _accResolution - _accOffset.x),
            .y = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _accResolution - _accOffset.y),
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS)
    return accGyroRPS_t {
        .gyroRPS = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y),
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _accResolution - _accOffset.y),
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };
#elif defined(LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_ZPOS_YNEG)
    return accGyroRPS_t {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z,
            .z = -(static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y)
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _accResolution - _accOffset.z,
            .z = -(static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _accResolution - _accOffset.y)
        }
    };
#else
    // Axis order mapping done at run-time
    const accGyroRPS_t accGyroRPS {
        .gyroRPS = {
            .x =   static_cast<float>(static_cast<int16_t>((data.gyro_x_h << 8) | data.gyro_x_l)) * _gyroResolutionRPS - _gyroOffset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.gyro_y_h << 8) | data.gyro_y_l)) * _gyroResolutionRPS - _gyroOffset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.gyro_z_h << 8) | data.gyro_z_l)) * _gyroResolutionRPS - _gyroOffset.z
        },
        .acc = {
            .x =   static_cast<float>(static_cast<int16_t>((data.acc_x_h << 8) | data.acc_x_l)) * _accResolution - _accOffset.x,
            .y =   static_cast<float>(static_cast<int16_t>((data.acc_y_h << 8) | data.acc_y_l)) * _accResolution - _accOffset.y,
            .z =   static_cast<float>(static_cast<int16_t>((data.acc_z_h << 8) | data.acc_z_l)) * _accResolution - _accOffset.z
        }
    };

    switch (_axisOrder) {
    case XPOS_YPOS_ZPOS:
        return accGyroRPS;
        break;
    case YNEG_XPOS_ZPOS:
        return accGyroRPS_t {
            .gyroRPS = {
                .x = -accGyroRPS.gyroRPS.y,
                .y =  accGyroRPS.gyroRPS.x,
                .z =  accGyroRPS.gyroRPS.z
            },
            .acc = {
                .x = -accGyroRPS.acc.y,
                .y =  accGyroRPS.acc.x,
                .z =  accGyroRPS.acc.z
            }
        };
        break;
    case XNEG_YNEG_ZPOS:
        return accGyroRPS_t {
            .gyroRPS = {
                .x = -accGyroRPS.gyroRPS.x,
                .y = -accGyroRPS.gyroRPS.y,
                .z =  accGyroRPS.gyroRPS.z
            },
            .acc = {
                .x = -accGyroRPS.acc.x,
                .y = -accGyroRPS.acc.y,
                .z =  accGyroRPS.acc.z
            }
        };
        break;
    case YPOS_XNEG_ZPOS:
        return accGyroRPS_t {
            .gyroRPS = {
                .x =  accGyroRPS.gyroRPS.y,
                .y = -accGyroRPS.gyroRPS.x,
                .z =  accGyroRPS.gyroRPS.z
            },
            .acc = {
                .x =  accGyroRPS.acc.y,
                .y = -accGyroRPS.acc.x,
                .z =  accGyroRPS.acc.z
            }
        };
        break;
    case XPOS_ZPOS_YNEG:
        return accGyroRPS_t {
            .gyroRPS = {
                .x =  accGyroRPS.gyroRPS.x,
                .y =  accGyroRPS.gyroRPS.z,
                .z = -accGyroRPS.gyroRPS.y
            },
            .acc = {
                .x = -accGyroRPS.acc.x,
                .y =  accGyroRPS.acc.z,
                .z = -accGyroRPS.acc.y
            }
        };
        break;
    default:
        return accGyroRPS_t {
            .gyroRPS = mapAxes(accGyroRPS.gyroRPS),
            .acc = mapAxes(accGyroRPS.acc)
        };
        break;
    } // end switch

    return accGyroRPS;
#endif
// NOLINTEND(hicpp-signed-bitwise)
}
#endif
