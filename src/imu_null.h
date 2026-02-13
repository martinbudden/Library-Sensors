#pragma once

#include "imu_base.h"

/*!
NULL IMU, useful for test code.
*/
class IMU_Null : public IMU_Base {
public:
    virtual ~IMU_Null() override = default;
    IMU_Null(const IMU_Null&) = delete;
    IMU_Null& operator=(const IMU_Null&) = delete;
    IMU_Null(IMU_Null&&) = delete;
    IMU_Null& operator=(IMU_Null&&) = delete;

    explicit IMU_Null(uint8_t axis_order) : IMU_Base(axis_order) {}
    IMU_Null() : IMU_Null(IMU_Base::XPOS_YPOS_ZPOS) {}

// NOLINTBEGIN(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;
// NOLINTEND(cppcoreguidelines-explicit-virtual-functions,hicpp-use-override,modernize-use-override)
public: // for testing
    void setGyroRaw(const xyz_int32_t& gyroRaw) { _gyro_raw = gyroRaw; }
    void setAccRaw(const xyz_int32_t& accRaw) { _acc_raw = accRaw; }
private:
    xyz_int32_t _gyro_raw {};
    xyz_int32_t _acc_raw {};
};
