#pragma once

#include "imu_base.h"


class IMU_M5_UNIFIED : public IMU_Base {
public:
    explicit IMU_M5_UNIFIED(uint8_t axis_order);
public:
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;

    virtual xyz_t read_gyro_rps() override;
    virtual xyz_t read_gyro_dps() override;
    virtual xyz_t read_acc() override;
    virtual acc_gyro_rps_t read_acc_gyro_rps() override;

    void setAxisOrder(uint8_t axis_order);
};
