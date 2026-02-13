#pragma once

#include "imu_base.h"


class ImuM5Stack : public ImuBase {
public:
    explicit ImuM5Stack(uint8_t axis_order);
public:
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;

    virtual xyz_t read_gyro_rps() override;
    virtual xyz_t read_gyro_dps() override;
    virtual xyz_t read_acc() override;
    virtual acc_gyro_rps_t read_acc_gyro_rps() override;

private:
    struct acc_temperature_gyro_data_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        static constexpr size_t DATA_SIZE = 14;
        uint8_t acc_x_h;
        uint8_t acc_x_l;
        uint8_t acc_y_h;
        uint8_t acc_y_l;
        uint8_t acc_z_h;
        uint8_t acc_z_l;
        uint8_t temperature_h;
        uint8_t temperature_l;
        uint8_t gyro_x_h;
        uint8_t gyro_x_l;
        uint8_t gyro_y_h;
        uint8_t gyro_y_l;
        uint8_t gyro_z_h;
        uint8_t gyro_z_l;
    };
    acc_gyro_rps_t acc_gyro_rpsFromRaw(const acc_temperature_gyro_data_t& data) const;
};
