#pragma once

#include "bus_i2c.h"
#include "bus_spi.h"
#include "imu_base.h"

/*!
The BNO085 is a System in Package (SiP) that integrates a triaxial accelerometer, triaxial gyroscope,
magnetometer and a 32-bit ARM Cortex-M0+ microcontroller running Hillcrest’s SH-2 (Sensor Hub 2) firmware.

Communication with the BNO085 is via Hillcrest's Sensor Hub Transport Protocol (SHTP) over SPI or I2C.
*/
class ImuBno085 : public ImuBase {
public:
    static constexpr uint8_t I2C_ADDRESS = 0x4A;

    static constexpr uint16_t MAX_PACKET_SIZE = 320; // The SH-2 protocol allows packets can be up to 32K, but 320 bytes is sufficient for BNO085
    static constexpr uint8_t MAX_I2C_READ_LENGTH = 32; // Arduino I2C reads are limited to 32 bytes

    static constexpr uint8_t CHANNEL_COMMAND = 0;
    static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
    static constexpr uint8_t CHANNEL_SENSOR_HUB_CONTROL = 2;
    static constexpr uint8_t CHANNEL_INPUT_SENSOR_REPORTS = 3;
    static constexpr uint8_t CHANNEL_WAKE_UP_INPUT_SENSOR_REPORTS = 4;
    static constexpr uint8_t CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT = 5;
    static constexpr uint8_t CHANNEL_COUNT = 6;

    static constexpr uint8_t SENSOR_REPORTID_ACCELEROMETER = 0x01; // Q point = 8
    static constexpr uint8_t SENSOR_REPORTID_GYROSCOPE_CALIBRATED = 0x02; // Q point = 9; radians/second
    static constexpr uint8_t SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED = 0x03; // Q point = 4
    static constexpr uint8_t SENSOR_REPORTID_LINEAR_ACCELERATION = 0x04; // Q point = 8
    //quaternion referenced to magnetic north and gravity. It is produced by fusing the outputs of the accelerometer; gyroscope and magnetometer.
    static constexpr uint8_t SENSOR_REPORTID_ROTATION_VECTOR = 0x05; // Q point = 14
    static constexpr uint8_t SENSOR_REPORTID_GRAVITY = 0x06; // Q point = 8
    static constexpr uint8_t SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED = 0x07; // Q point = 9
    // produced by fusing the outputs of the accelerometer and the gyroscope (ie no magnetometer).
    static constexpr uint8_t SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08; // Q point = 14
    static constexpr uint8_t SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR = 0x09;
    static constexpr uint8_t SENSOR_REPORTID_TAP_DETECTOR = 0x10;
    static constexpr uint8_t SENSOR_REPORTID_STEP_COUNTER = 0x11;
    static constexpr uint8_t SENSOR_REPORTID_STABILITY_CLASSIFIER = 0x13;
    static constexpr uint8_t SENSOR_REPORTID_RAW_ACCELEROMETER = 0x14;
    static constexpr uint8_t SENSOR_REPORTID_RAW_GYROSCOPE = 0x15;
    static constexpr uint8_t SENSOR_REPORTID_RAW_MAGNETOMETER = 0x16;
    static constexpr uint8_t SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E;
    static constexpr uint8_t SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR = 0x28;
    static constexpr uint8_t SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR = 0x29;
    // supports higher data rates than the more accurate Rotation Vector
    static constexpr uint8_t SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A; // Q point =14 for orientation; Q point = 10 for gyro
    static constexpr uint8_t SENSOR_REPORTID_GET_FEATURE_REQUEST = 0xFE;
    static constexpr uint8_t SENSOR_REPORTID_SET_FEATURE_COMMAND = 0xFD;
    static constexpr uint8_t SENSOR_REPORTID_GET_FEATURE_RESPONSE = 0xFC;
    static constexpr uint8_t SENSOR_REPORTID_FORCE_SENSOR_FLUSH = 0xF0;
    static constexpr uint8_t SENSOR_REPORTID_FLUSH_COMPLETED = 0xEF;

#pragma pack(push, 1)
    struct ShtpHeader {
        uint8_t lengthLSB;
        uint8_t lengthMSB;
        uint8_t channel;
        uint8_t sequence_number;
    };
    struct ShtpPacket {
        ShtpHeader header;
        std::array<uint8_t, MAX_PACKET_SIZE> data;
    };
    struct command_message_t {
        uint8_t report_id; // 0xF2
        uint8_t sequence_number;
        uint8_t command;
        std::array<uint8_t, 9> P;
    };
    struct command_response_t {
        uint8_t report_id; // 0xF1
        uint8_t sequence_number;
        uint8_t command;
        uint8_t command_sequence_number;
        uint8_t response_sequence_number;
        std::array<uint8_t, 11> R;
    };
    struct sensor_output_t {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t accuracy;
        uint8_t sequence_number;
        uint16_t delay;
    };
    struct sensor_output_uncalibrated_gyro_t {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t accuracy;
        uint8_t sequence_number;
        uint16_t delay;
        int16_t biasX;
        int16_t biasY;
        int16_t biasZ;
    };
    // The BNO085 data sheet uses the term "rotation vector" rather than "orientation quaternion" used in the Stabilized Vehicle software
    struct rotation_vector_t {
        int16_t i;
        int16_t j;
        int16_t k;
        int16_t real;
        uint16_t accuracy;
        uint16_t radianAccuracy;
    };
    struct gyro_integrated_rotation_vector_t {
        // gyro value
        int16_t i;
        int16_t j;
        int16_t k;
        // quaternion
        int16_t real;
        int16_t x;
        int16_t y;
        int16_t z;
    };
#pragma pack(pop)
public:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuBno085_USE_SPI_BUS)
    // SPI constructors
    ImuBno085(uint8_t axis_order, uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::stm32_spi_pins_t& pins);
    ImuBno085(uint8_t axis_order, uint32_t frequency, BusBase::bus_index_e spi_index, const BusSpi::spi_pins_t& pins);
#else
    // I2C constructors
    ImuBno085(uint8_t axis_order, BusBase::bus_index_e i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address);
    ImuBno085(uint8_t axis_order, const BusI2c::i2c_pins_t& pins, uint8_t I2C_address) : ImuBno085(axis_order, BusI2c::BUS_INDEX_0, pins, I2C_address) {}
    ImuBno085(uint8_t axis_order, const BusI2c::i2c_pins_t& pins) : ImuBno085(axis_order, pins, I2C_ADDRESS) {}
#endif
public:
    virtual int init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex) override;
    void set_feature_command(uint8_t report_id, uint32_t time_between_reports_us, uint32_t specific_config);
    virtual xyz_int32_t read_gyro_raw() override;
    virtual xyz_int32_t read_acc_raw() override;
    virtual xyz_t read_gyro_rps() override;
    virtual Quaternion read_orientation() override;
public:
    xyz_t get_acc() const;
    xyz_t get_acc_linear() const;
    xyz_t get_gyro_rps() const;
    xyz_t get_mag() const;
    xyz_t get_gravity() const;
    uint16_t parse_input_sensor_report(const ShtpPacket& packet);
    uint16_t parseGyro_integrated_rotation_vectorReport(const ShtpPacket& packet);
    uint16_t parse_command_response(const ShtpPacket& packet);
    bool send_command_calibrate_motion_engine();
    bool send_command_save_dynamic_calibration_data();
    // for unit testing
    gyro_integrated_rotation_vector_t get_gyro_integrated_rotation_vectorData() const { return _gyro_integrated_rotation_vector; }
    rotation_vector_t get_rotation_vector_data() const { return _rotation_vector; }
    sensor_output_t get_acc_data() const { return _acc; }
    sensor_output_t get_acc_linear_data() const { return _acc_linear; }
    sensor_output_t get_gyro_rps_Data() const { return _gyro_rps; }
    sensor_output_t get_mag_data() const { return _mag; }
    sensor_output_t get_gravity_data() const { return _gravity; }
    sensor_output_t get_acc_raw_data() const { return _acc_raw; }
    sensor_output_t get_gyro_raw_data() const { return _gyro_raw; }
    sensor_output_t get_mag_raw_data() const { return _mag_raw; }
    sensor_output_uncalibrated_gyro_t get_gyro_uncalibrated_rps_Data() const { return _gyro_uncalibrated_rps; }
private:
    bool read_packet_and_parse();
    bool read_packet();
    bool read_data(size_t read_length);
    bool send_packet(uint8_t channelNumber, uint8_t data_length);
    bool send_command(uint8_t command);
protected:
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuBno085_USE_SPI_BUS)
    BusSpi _bus; //!< SPI bus interface
#else
    BusI2c _bus; //!< I2C bus interface
#endif
    uint32_t _timestamp {};
    uint32_t _orientation_available {false};
    uint32_t _gyro_available {false};
    Quaternion _axis_order_quaternion;
    // SHTP (Sensor Hub Transport Protocol)
    ShtpPacket _shtp_packet {};
    std::array<uint8_t, CHANNEL_COUNT> _sequence_number {}; //There are 6 com channels. Each channel has its own sequence number

    uint8_t _reset_complete_received = false; // set true when Reset Complete packet received.
    uint8_t _calibration_status {}; //R0 of COMMAND_CALIBRATE_MOTION_ENGINE Response

    // combined gyro and rotation for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR
    gyro_integrated_rotation_vector_t _gyro_integrated_rotation_vector {};
    rotation_vector_t _rotation_vector {};

    command_message_t _command_message {};

    sensor_output_t _acc {};
    sensor_output_t _acc_linear {}; // Acceleration of the device with gravity removed
    sensor_output_t _gyro_rps {};
    sensor_output_t _mag {};
    sensor_output_t _gravity {};
    sensor_output_t _acc_raw {};
    sensor_output_t _gyro_raw {};
    sensor_output_t _mag_raw {};
    sensor_output_uncalibrated_gyro_t _gyro_uncalibrated_rps {};
};
