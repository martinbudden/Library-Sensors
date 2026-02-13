//#define LIBRARY_SENSORS_SERIAL_DEBUG
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
#include <HardwareSerial.h>
#endif

#include "imu_bno085.h"
#include <cassert>
#include <cstring>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
} // end namespace

static constexpr uint8_t BUS_TIMEOUT_MS = 100;

static constexpr uint8_t REPORT_ID_COMMAND_RESPONSE = 0xF1;
static constexpr uint8_t REPORT_ID_COMMAND_REQUEST = 0xF2;
static constexpr uint8_t REPORT_ID_FRS_READ_RESPONSE = 0xF3;
static constexpr uint8_t REPORT_ID_FRS_READ_REQUEST = 0xF4;
static constexpr uint8_t REPORT_ID_FRS_WRITE_RESPONSE = 0xF5;
static constexpr uint8_t REPORT_ID_FRS_WRITE_DATA_REQUEST = 0xF6;
static constexpr uint8_t REPORT_ID_FRS_WRITE_REQUEST = 0xF7;
static constexpr uint8_t REPORT_ID_PRODUCT_ID_RESPONSE = 0xF8;
static constexpr uint8_t REPORT_ID_PRODUCT_ID_REQUEST = 0xF9;
static constexpr uint8_t REPORT_ID_TIMESTAMP_REBASE = 0xFA;
static constexpr uint8_t REPORT_ID_BASE_TIMESTAMP_REFERENCE= 0xFB;

static constexpr uint8_t EXECUTABLE_RESET_COMPLETE = 0x1;

static constexpr uint8_t COMMAND_REPORT_ERRORS = 0x01;
static constexpr uint8_t COMMAND_COUNTER_COMMANDS = 0x02; // sub-commands specified in P0
static constexpr uint8_t COMMAND_TARE = 0x03;
static constexpr uint8_t COMMAND_INITIALIZE = 0x04;
static constexpr uint8_t COMMAND_SAVE_DYNAMIC_CALIBRATION_DATA = 0x06; // DCD
static constexpr uint8_t COMMAND_CALIBRATE_MOTION_ENGINE = 0x07;
static constexpr uint8_t COMMAND_CONFIGURE_PERIODIC_DCD_SAVE = 0x09;
static constexpr uint8_t COMMAND_GET_OSCILLATOR_TYPE = 0x0A;
static constexpr uint8_t COMMAND_CLEAR_DCD_AND_RESET = 0x0B;

// System orientation rotation quaternions.
// The system orientation FRS record (0x2D3E) applies a rotation to the sensor outputs and all the derived outputs.
// The record is a unit quaternion, with each coordinate represented as a 32-bit fixed point number with a Q-point of 30.

// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS) || defined(LIBRARY_SENSORS_ImuBno085_USE_SPI_BUS)
ImuBno085::ImuBno085(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::stm32_spi_pins_t& pins) :
    ImuBase(axis_order, _bus, IMU_AUTO_CALIBRATES | IMU_PERFORMS_SENSOR_FUSION),
    _bus(frequency, spi_index, pins),
    _axis_order_quaternion(axisOrientations[axis_order])
{
}
ImuBno085::ImuBno085(uint8_t axis_order, uint32_t frequency, uint8_t spi_index, const BusSpi::spi_pins_t& pins) :
    ImuBase(axis_order, _bus, IMU_AUTO_CALIBRATES | IMU_PERFORMS_SENSOR_FUSION),
    _bus(frequency, spi_index, pins),
    _axis_order_quaternion(axisOrientations[axis_order])
{
}
#else
ImuBno085::ImuBno085(uint8_t axis_order, uint8_t i2c_index, const BusI2c::i2c_pins_t& pins, uint8_t i2c_address) :
    ImuBase(axis_order, _bus, IMU_AUTO_CALIBRATES | IMU_PERFORMS_SENSOR_FUSION),
    _bus(i2c_address, i2c_index, pins),
    _axis_order_quaternion(axisOrientations[axis_order])
{
}
#endif

int ImuBno085::init(uint32_t target_output_data_rate_hz, uint8_t gyro_sensitivity, uint8_t acc_sensitivity, void* bus_mutex)
{
    assert(target_output_data_rate_hz <= 400);
    (void)gyro_sensitivity;
    (void)acc_sensitivity;

    // MSP compatible gyro and acc identifiers, use defaults, since no MSP value for BNO085
    _gyro_id_msp = MSP_GYRO_ID_DEFAULT;
    _acc_id_msp = MSP_ACC_ID_DEFAULT;

    _gyro_sample_rate_hz = target_output_data_rate_hz;
    _acc_sample_rate_hz = target_output_data_rate_hz;
#if defined(LIBRARY_SENSORS_IMU_BUS_MUTEX_REQUIRED)
    _bus_mutex = static_cast<SemaphoreHandle_t>(bus_mutex);
#else
    (void)bus_mutex;
#endif

    // transmit reset byte on channel 1
    _shtp_packet.data[0] = 1; //Reset
    send_packet(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
    delay_ms(50);
    //Read all incoming data and flush it
    while (read_packet_and_parse()) { delay_ms(1); }

    _shtp_packet.data[0] = REPORT_ID_PRODUCT_ID_REQUEST;
    _shtp_packet.data[1] = 0;
    send_packet(CHANNEL_SENSOR_HUB_CONTROL, 2);
    delay_ms(50);
    while (read_packet_and_parse()) { delay_ms(1); }

    // default to update every 2500 microseconds, 400Hz, which is highest supported rate
    set_feature_command(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, target_output_data_rate_hz == 0 ? 2500 : 1000000 / target_output_data_rate_hz, 0);
    delay_ms(100);
    while (read_packet_and_parse()) { delay_ms(1); }

    return static_cast<int>(target_output_data_rate_hz);
}

void ImuBno085::set_feature_command(uint8_t report_id, uint32_t time_between_reports_us, uint32_t specific_config)
{
    _shtp_packet.data[0] = SENSOR_REPORTID_SET_FEATURE_COMMAND;                          // Set feature command. Reference page 55
    _shtp_packet.data[1] = report_id;                                                     // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    _shtp_packet.data[2] = 0;                                                            // Feature flags
    _shtp_packet.data[3] = 0;                                                            // Change sensitivity (LSB)
    _shtp_packet.data[4] = 0;                                                            // Change sensitivity (MSB)
    _shtp_packet.data[5] = static_cast<uint8_t>((time_between_reports_us >> 0) & 0xFFU);    // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    _shtp_packet.data[6] = static_cast<uint8_t>((time_between_reports_us >> 8) & 0xFFU);    // Report interval
    _shtp_packet.data[7] = static_cast<uint8_t>((time_between_reports_us >> 16) & 0xFFU);   // Report interval
    _shtp_packet.data[8] = static_cast<uint8_t>((time_between_reports_us >> 24) & 0xFFU);   // Report interval (MSB)
    _shtp_packet.data[9] = 0;                                                            // Batch Interval (LSB)
    _shtp_packet.data[10] = 0;                                                           // Batch Interval
    _shtp_packet.data[11] = 0;                                                           // Batch Interval
    _shtp_packet.data[12] = 0;                                                           // Batch Interval (MSB)
    _shtp_packet.data[13] = static_cast<uint8_t>((specific_config >> 0) & 0xFFU);         // Sensor-specific config (LSB)
    _shtp_packet.data[14] = static_cast<uint8_t>((specific_config >> 8) & 0xFFU);         // Sensor-specific config
    _shtp_packet.data[15] = static_cast<uint8_t>((specific_config >> 16) & 0xFFU);        // Sensor-specific config
    _shtp_packet.data[16] = static_cast<uint8_t>((specific_config >> 24) & 0xFFU);        // Sensor-specific config (MSB)

    send_packet(CHANNEL_SENSOR_HUB_CONTROL, 17);
}

ImuBase::xyz_int32_t ImuBno085::read_gyro_raw()
{
    ImuBase::xyz_int32_t ret {};
    return ret;
}

ImuBase::xyz_int32_t ImuBno085::read_acc_raw()
{
    ImuBase::xyz_int32_t ret {};
    return ret;
}

Quaternion ImuBno085::read_orientation()
{
    if (!_orientation_available) {
        read_packet_and_parse();
    }
    _orientation_available = false;
    // for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A // Q point = 14 for orientation, Q point = 10 for gyro
    constexpr unsigned int Q_point = 14;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    const Quaternion _integrated_rotation_vector(
        static_cast<float>(_gyro_integrated_rotation_vector.real) * multiplier,
        static_cast<float>(_gyro_integrated_rotation_vector.i) * multiplier,
        static_cast<float>(_gyro_integrated_rotation_vector.j) * multiplier,
        static_cast<float>(_gyro_integrated_rotation_vector.k) * multiplier
    );
    return _axis_order_quaternion * _integrated_rotation_vector;
}

xyz_t ImuBno085::read_gyro_rps()
{
    if (!_gyro_available) {
        read_packet_and_parse();
    }
    _gyro_available = false;
    // for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A // Q point = 14 for orientation, Q point = 10 for gyro
    constexpr unsigned int Q_point = 10;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_gyro_integrated_rotation_vector.x) * multiplier,
        .y = static_cast<float>(_gyro_integrated_rotation_vector.y) * multiplier,
        .z = static_cast<float>(_gyro_integrated_rotation_vector.z) * multiplier
    };
}

xyz_t ImuBno085::get_acc() const
{
    constexpr unsigned int Q_point = 8;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_acc.x) * multiplier,
        .y = static_cast<float>(_acc.y) * multiplier,
        .z = static_cast<float>(_acc.z) * multiplier
    };
}


xyz_t ImuBno085::get_acc_linear() const
{
    constexpr unsigned int Q_point = 8;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_acc_linear.x) * multiplier,
        .y = static_cast<float>(_acc_linear.y) * multiplier,
        .z = static_cast<float>(_acc_linear.z) * multiplier
    };
}

xyz_t ImuBno085::get_gyro_rps() const
{
    constexpr unsigned int Q_point = 9;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_gyro_rps.x) * multiplier,
        .y = static_cast<float>(_gyro_rps.y) * multiplier,
        .z = static_cast<float>(_gyro_rps.z) * multiplier
    };
}

xyz_t ImuBno085::get_mag() const
{
    constexpr unsigned int Q_point = 4;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_mag.x) * multiplier,
        .y = static_cast<float>(_mag.y) * multiplier,
        .z = static_cast<float>(_mag.z) * multiplier
    };
}

xyz_t ImuBno085::get_gravity() const
{
    constexpr unsigned int Q_point = 8;
    constexpr float multiplier = 1.0F / (1U << Q_point);
    return xyz_t {
        .x = static_cast<float>(_gravity.x) * multiplier,
        .y = static_cast<float>(_gravity.y) * multiplier,
        .z = static_cast<float>(_gravity.z) * multiplier
    };
}

uint16_t ImuBno085::parse_command_response(const ShtpPacket& packet)
{
    const uint8_t report_id = packet.data[0];
    if (report_id == REPORT_ID_COMMAND_RESPONSE) {
        //The BNO085 responds with this report to command requests.
        const uint8_t command = packet.data[2];
        if (command == COMMAND_CALIBRATE_MOTION_ENGINE) {
            _calibration_status = packet.data[0]; //R0 - Status (0 = success, non-zero = fail)
        }
        return report_id;
    }

    return 0;
}


/*!
The gyro integrated rotation vector input reports are sent via the special gyro channel.
They DO NOT INCLUDE the usual report_id, sequence_number, status, and delay fields
Rates of up to 400 Hz are supported.
*/
uint16_t ImuBno085::parseGyro_integrated_rotation_vectorReport(const ShtpPacket& packet)
{
    _orientation_available = true;
    _gyro_available = true;
    _gyro_integrated_rotation_vector.i    = static_cast<uint16_t>(packet.data[1])  << 8U | packet.data[0];
    _gyro_integrated_rotation_vector.j    = static_cast<uint16_t>(packet.data[3])  << 8U | packet.data[2];
    _gyro_integrated_rotation_vector.k    = static_cast<uint16_t>(packet.data[5])  << 8U | packet.data[4];
    _gyro_integrated_rotation_vector.real = static_cast<uint16_t>(packet.data[7])  << 8U | packet.data[6];
    _gyro_integrated_rotation_vector.x    = static_cast<uint16_t>(packet.data[9])  << 8U | packet.data[8];
    _gyro_integrated_rotation_vector.y    = static_cast<uint16_t>(packet.data[11]) << 8U | packet.data[10];
    _gyro_integrated_rotation_vector.z    = static_cast<uint16_t>(packet.data[13]) << 8U | packet.data[12];
    //Serial.printf("gyro:%d,%d,%d\r\n", _gyro_integrated_rotation_vector.x, _gyro_integrated_rotation_vector.y, _gyro_integrated_rotation_vector.z);

    return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
}

/*!
Parse the input sensor report packet.
Packet format is:
packet.header       4 byte header
packet.data[0:4]    5 byte timestamp
packet.data[5:8]    4 bytes of data containing report_id, sequence_number, status, and delay.
packet.date[9..]    sensor data starts
*/
uint16_t ImuBno085::parse_input_sensor_report(const ShtpPacket& packet)
{
    if (packet.data[0] == REPORT_ID_BASE_TIMESTAMP_REFERENCE) {
        _timestamp = (static_cast<uint32_t>(packet.data[4]) << 24U)
            | (static_cast<uint32_t>(packet.data[3]) << 16U)
            | (static_cast<uint32_t>(packet.data[2]) << 8U)
            | static_cast<uint32_t>(packet.data[1]);
    }

    /*
    Bits 1:0 - indicate the status of a sensor.
    0 - Unreliable
    1 - Accuracy low
    2 - Accuracy medium
    3 - Accuracy high
    Bits 7:2 - Delay upper bits: 6 most-significant bits of report delay
    */
    const uint8_t report_id = packet.data[5];
    const uint8_t sequence_number = packet.data[6];
    const uint8_t status = packet.data[7]; //Get status bits
    const uint8_t accuracy = status & 0x03U;
    const uint16_t delay = static_cast<uint16_t>(status & 0xFCU) << 6U | packet.data[8];

    const int16_t dataX = static_cast<int16_t>(packet.data[10]) << 8U | packet.data[9]; // NOLINT(hicpp-signed-bitwise)
    const int16_t dataY = static_cast<int16_t>(packet.data[12]) << 8U | packet.data[11]; // NOLINT(hicpp-signed-bitwise)
    const int16_t dataZ = static_cast<int16_t>(packet.data[14]) << 8U | packet.data[13]; // NOLINT(hicpp-signed-bitwise)

    switch (report_id) {
    case SENSOR_REPORTID_ACCELEROMETER:
        _acc.x = dataX;
        _acc.y = dataY;
        _acc.z = dataZ;
        _acc.accuracy = accuracy;
        _acc.sequence_number = sequence_number;
        _acc.delay = delay;
        break;
    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
        _gyro_rps.x = dataX;
        _gyro_rps.y = dataY;
        _gyro_rps.z = dataZ;
        _gyro_rps.accuracy = accuracy;
        _gyro_rps.sequence_number = sequence_number;
        _gyro_rps.delay = delay;
        break;
    case SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED:
        _mag.x = dataX;
        _mag.y = dataY;
        _mag.z = dataZ;
        _mag.accuracy = accuracy;
        _mag.sequence_number = sequence_number;
        _mag.delay = delay;
        break;
    case SENSOR_REPORTID_LINEAR_ACCELERATION:
        _acc_linear.x = dataX;
        _acc_linear.y = dataY;
        _acc_linear.z = dataZ;
        _acc_linear.accuracy = accuracy;
        _acc_linear.sequence_number = sequence_number;
        _acc_linear.delay = delay;
        break;
    case SENSOR_REPORTID_GRAVITY:
        _gravity.x = dataX;
        _gravity.y = dataY;
        _gravity.z = dataZ;
        _gravity.accuracy = accuracy;
        _gravity.sequence_number = sequence_number;
        _gravity.delay = delay;
        break;
    case SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED:
        _gyro_uncalibrated_rps.x = dataX;
        _gyro_uncalibrated_rps.y = dataY;
        _gyro_uncalibrated_rps.z = dataZ;
        _gyro_uncalibrated_rps.accuracy = accuracy;
        _gyro_uncalibrated_rps.sequence_number = sequence_number;
        _gyro_uncalibrated_rps.delay = delay;
        _gyro_uncalibrated_rps.biasX  = static_cast<uint16_t>(packet.data[16]) << 8U | packet.data[15];
        _gyro_uncalibrated_rps.biasY  = static_cast<uint16_t>(packet.data[18]) << 8U | packet.data[17];
        _gyro_uncalibrated_rps.biasZ  = static_cast<uint16_t>(packet.data[20]) << 8U | packet.data[19];
        break;
    case SENSOR_REPORTID_ROTATION_VECTOR: // NOLINT(bugprone-branch-clone) false positive
        [[fallthrough]];
    case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR:
        _rotation_vector.radianAccuracy = static_cast<uint16_t>(packet.data[18]) << 8U | packet.data[17];
        [[fallthrough]];
    // the GAME rotation vectors do not report radianAccuracy
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR:
        _rotation_vector.i = dataX;
        _rotation_vector.j = dataY;
        _rotation_vector.k = dataZ;
        _rotation_vector.real = static_cast<uint16_t>(packet.data[16]) << 8U | packet.data[15];
        _rotation_vector.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_ACCELEROMETER:
        _acc_raw.x = dataX;
        _acc_raw.y = dataY;
        _acc_raw.z = dataZ;
        _acc_raw.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_GYROSCOPE:
        _gyro_raw.x = dataX;
        _gyro_raw.y = dataY;
        _gyro_raw.z = dataZ;
        _gyro_raw.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_MAGNETOMETER:
        _mag_raw.x = dataX;
        _mag_raw.y = dataY;
        _mag_raw.z = dataZ;
        _mag_raw.accuracy = accuracy;
        break;
    case REPORT_ID_COMMAND_RESPONSE:
        //The BNO085 responds with this report to command requests. It's up to us to remember which command we issued.
        if (packet.data[2] == COMMAND_CALIBRATE_MOTION_ENGINE) {
            _calibration_status = packet.data[10]; //R0 - Status (0 = success, non-zero = fail)
        }
        break;
    default:
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
        return 0;
    }

    return report_id;
}

bool ImuBno085::read_packet_and_parse()
{
    static_assert(sizeof(ShtpHeader) == 4);
    if (read_packet()) {
        //Check to see if this packet is a sensor reporting its data to us
        switch (_shtp_packet.header.channel) {
        case CHANNEL_SENSOR_HUB_CONTROL: // NOLINT(bugprone-branch-clone) false positive
            parse_command_response(_shtp_packet);
            break;
        case CHANNEL_INPUT_SENSOR_REPORTS:
            parse_input_sensor_report(_shtp_packet);
            break;
        case CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT:
            parseGyro_integrated_rotation_vectorReport(_shtp_packet);
            break;
        }
        return true;
    }
    return false;
}

bool ImuBno085::read_packet()
{
    // No interrupt pin set then we rely on receivePacket() to timeout. Strictly speaking this does not follow the SH-2 transport protocol.
    if (_bus.read_bytes_with_timeout(reinterpret_cast<uint8_t*>(&_shtp_packet.header), sizeof(ShtpHeader), BUS_TIMEOUT_MS) == false) { // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        return false;
    }

    uint16_t data_length = ((static_cast<uint16_t>(_shtp_packet.header.lengthMSB)) << 8) | (static_cast<uint16_t>(_shtp_packet.header.lengthLSB));
    data_length &= 0x7FFFU; // Clear the most significant bit.
    //Serial.printf("data_lengthMSB:%0x\r\n", _shtp_packet.header.lengthMSB);
    //Serial.printf("data_lengthLSB:%0x\r\n", _shtp_packet.header.lengthLSB);
    //Serial.printf("data_lengthA:%0x\r\n", data_length);
    if (data_length <= sizeof(ShtpHeader)) {
        //Packet is empty
        return false;
    }
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.printf("\r\ndata_length:%3d(0x%3X) CH:%d, SN:%d\r\n", data_length, data_length, _shtp_packet.header.channel, _shtp_packet.header.sequence_number);
#endif
    read_data(data_length - sizeof(ShtpHeader));
    //Serial.printf("timeStamp:0x%02x:%02x:%02x:%02x:%02x\r\n", _shtp_packet.timestamp[0], _shtp_packet.timestamp[1], _shtp_packet.timestamp[2], _shtp_packet.timestamp[3], _shtp_packet.timestamp[4]);
    //Serial.printf("data: %02x,%02x,%02x,%02x,%02x\r\n", _shtp_packet.data[0], _shtp_packet.data[1], _shtp_packet.data[2], _shtp_packet.data[3], _shtp_packet.data[4]);

    // Check for a reset complete packet
    if (_shtp_packet.header.channel == CHANNEL_EXECUTABLE && _shtp_packet.data[0] == EXECUTABLE_RESET_COMPLETE) {
        //Serial.printf("Reset\r\n");
        _reset_complete_received = true;
    }
    return true;
}

/*!
Perform multiple reads until all bytes are read
The ShtpPacket data buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
*/
bool ImuBno085::read_data(size_t read_length)
{
    size_t index = 0;
    size_t bytes_to_read = read_length;

    //Setup a series of chunked 32 byte reads
    while (bytes_to_read > 0) {
        //Serial.printf("bytes_to_read:%d\r\n", bytes_to_read);
        size_t read_count = bytes_to_read;
        if (read_count + sizeof(ShtpHeader) > MAX_I2C_READ_LENGTH) {
            read_count = MAX_I2C_READ_LENGTH - sizeof(ShtpHeader);
        }

        std::array<uint8_t, MAX_I2C_READ_LENGTH> data;
        if (_bus.read_bytes_with_timeout(reinterpret_cast<uint8_t*>(&data[0]), read_count + sizeof(ShtpHeader), BUS_TIMEOUT_MS) == false) { // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,readability-simplify-boolean-expr)
            return false;
        }

#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
        const int data_length = (~0x8000U) & ((static_cast<uint16_t>(data[1]) << 8U) | static_cast<uint16_t>(data[0]));
        Serial.printf("data_length:%3d(0x%3X) CH:%d, SN:%d\r\n", data_length, data_length, data[2], data[3]);
#endif

        //Serial.printf("read_count:%d\r\n", read_count);
        // Read a chunk of data
        if (index + read_count <= MAX_PACKET_SIZE) {
            memcpy(&_shtp_packet.data[index], &data[4], read_count);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
            for (int ii = 0; ii < read_count; ++ii) {
                Serial.printf("%02x ", _shtp_packet.data[index + ii]);
            }
            if (read_length > 0) {
                Serial.printf("\r\n");
            }
#endif
            index += read_count;
        } else {
            // no room for the data, so just throw it away
        }
        bytes_to_read -= read_count;
    }
    return true;
}

// NOTE: Arduino has a maximum 32 byte send.
bool ImuBno085::send_packet(uint8_t channel_number, uint8_t data_length)
{
    const uint8_t packet_length = data_length + sizeof(_shtp_packet.header); // Add four bytes for the header
    _shtp_packet.header.lengthLSB = packet_length & 0xFF;
    _shtp_packet.header.lengthMSB = packet_length >> 8;
    _shtp_packet.header.channel = channel_number;
    _shtp_packet.header.sequence_number = _sequence_number[channel_number]++;
    _bus.write_bytes(reinterpret_cast<uint8_t*>(&_shtp_packet), packet_length); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    return true;
}

bool ImuBno085::send_command_calibrate_motion_engine()
{
    _command_message.P.fill(0);
    _command_message.P[0] = 1; // acc calibration enabled
    _command_message.P[1] = 1; // gyro calibration enabled
    // all other calibration disabled
    return send_command(COMMAND_CALIBRATE_MOTION_ENGINE);
}

bool ImuBno085::send_command_save_dynamic_calibration_data()
{
    _command_message.P.fill(0);
    return send_command(COMMAND_SAVE_DYNAMIC_CALIBRATION_DATA);
}

bool ImuBno085::send_command(uint8_t command)
{
    _command_message.report_id = REPORT_ID_COMMAND_REQUEST;
    _command_message.command = command;
    ++_command_message.sequence_number;
    _bus.write_bytes(reinterpret_cast<uint8_t*>(&_command_message), sizeof(_command_message)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

    return true;
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
