# Sensors Library ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library implements drivers various sensors, including:

1. IMUs (Inertial Management Units), that is combined gyroscopes and accelerometers.
2. Barometers

## IMUs

The following IMUs are currently implemented:

| IMU                                                                                        | ID          | SPI      | I2C      |
| -------------------------------------------------------------------------------------------| ----------- | ---------| -------- |
| Bosch [BMI270](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)       | BMI270      | untested | working  |
| CEVA [BNO085](https://www.ceva-ip.com/product/bno-9-axis-imu/)                             | BNO085      | untested | working  |
| TDK [ICM-20602](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-20602/)     | ICM420602   | untested | untested |
| TDK [ICM-42605](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42605/)     | ICM426xx    | untested | untested |
| TDK [ICM-42688-P](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) | ICM426xx    | working  | untested |
| TDK [MPU-6000](https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=MPU-6000) | MPU6000 | untested | untested |
| ST [ISM330DHCX](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)                    | ISM330DHCX  | working  | working  |
| ST [LSM6DS3TR-C](https://www.st.com/en/mems-and-sensors/lsm6ds3tr-c.html)                  | LSM6DS3TR_C | working  | working  |
| ST [LSM6DSOX](https://www.st.com/en/mems-and-sensors/lsm6dsox.html)                        | LSM6DSOX    | working  | working  |
| InvenSense MPU-6886                                                                        | MPU6886     | untested | working  |
| M5Stack Unified                                                                            | N/A         | N/A      | working  |

The ICM-42605 and ICM-42688 are broadly compatible and share the same driver.

The LSM6DS3TR-C, ISM330DHCX, and LSM6DSOX are broadly compatible and share the same driver.

The MPU-6886 is an IMU that is used by M5Stack devices, it does not seem to be used anywhere else.

## Barometers

The following barometers are currently implemented:

| Barometers                                                                                                     | ID          | SPI      | I2C      |
| ---------------------------------------------------------------------------------------------------------------| ----------- | ---------| -------- |
| Bosch [BMP280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf) | BMP280      | untested | untested |

## SPI Build Flags

By default the drivers are configured to us I2C. To use SPI, define the `LIBRARY_SENSORS_IMU_USE_SPI_BUS`. This can be done in the `build_flags` section of `platformio.ini`.

## Frameworks

The Arduino Framework and the Raspberry Pi Pico SDK Framework are supported.

The ESPIDF Framework is currently supported for SPI but not I2C.

Support for the STM32 Framework is under development.

## Dependencies

This library uses the [VectorQuaternionMatrix library](https://github.com/martinbudden/Library-VectorQuaternionMatrix)
for its `xyz_t`(3D vector) and `Quaternion` classes.

## Simplified class diagram

`BUS_SPI` and `BUS_I2C` are statically (build-time) polymorphic, not dynamically (run-time) polymorphic.<br>
They have functions that have the same names and signatures, but these functions are not virtual.<br>
This is deliberate.<br>
This means the SPI and I2C read and write functions can be called directly, rather than indirectly via a virtual function pointer.
Some of these functions are short enough that they may be inlined. So in some cases an indirect function call is replaced by an
inlined function. This is important because many of these functions are highly time critical.<br>
It does result in an unconventional class tree: each concrete IMU class has two pointers to its bus: a pointer to `BUS_BASE` in `IMU_BASE`
and a reference to either `BUS_I2C` or `BUS_SPI` in the class itself.

`IMU_BNO085` is shown using `I2C_BUS`. It can be configured by a build flag to use `SPI_BUS`.

`IMU_BMI270`, `IMU_LSM6DS3TR_C`, and `IMU_ICM426xx` are shown using `SPI_BUS`. They can be configured by build flags to use `I2C_BUS`.

```mermaid
classDiagram
    class BUS_BASE {
        _deviceRegister uint8_t
        _deviceReadBuf uint8_t*
        _deviceReadLength size_t
        WAIT_DATA_READY() int32_t
        WAIT_DATA_READY(uint32_t ticksToWait) int32_t
        SIGNAL_DATA_READY_FROM_ISR()
    }

    IMU_Base <|-- IMU_M5_UNIFIED
    class IMU_M5_UNIFIED {
        readGyroRaw() xyz_int32_t override
        readAccRaw() xyz_int32_t override

        readGyroRPS() xyz_t override
        readGyroDPS() xyz_t override
        readAcc() xyz_t override
        readAccGyroRPS() accGyroRPS_t override
    }
    IMU_M5_UNIFIED *-- BUS_I2C

    IMU_Base <|-- IMU_MPU6886
    class IMU_MPU6886 {
        setInterruptDriven() override
        setGyroOffset(const xyz_int32_t& gyroOffset) override

        readGyroRaw() xyz_int32_t override
        readAccRaw() xyz_int32_t override

        readGyroRPS() xyz_t override
        readGyroDPS() xyz_t override
        readAcc() xyz_t override
        readAccGyroRPS() accGyroRPS_t override
        getAccGyroRPS() accGyroRPS_t override
    }
    IMU_MPU6886 *-- BUS_I2C

    IMU_Base <|-- IMU_BNO085
    class IMU_BNO085 {
        readGyroRaw() xyz_int32_t override
        readAccRaw() xyz_int32_t override
        readGyroRPS() xyz_t override
        readOrientation() Quaternion override
    }
    IMU_BNO085 *-- BUS_I2C

    BUS_BASE <|-- BUS_I2C
    class BUS_I2C {
        _I2C_index i2c_index_e
        _pins pins_t
        _I2C_address uint8_t

        setInterruptDriven()
        setDeviceDataRegister(uint8_t deviceRegister, uint8_t* readBuf, size_t readLength)

        readDeviceData() bool
        readRegister(uint8_t reg) uint8_t
        readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) uint8_t
        readRegister(uint8_t reg, uint8_t* data, size_t length) bool
        readBytes(uint8_t* data, size_t length) bool
        readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) bool
        writeRegister(uint8_t reg, uint8_t data) uint8_t
        writeRegister(uint8_t reg, const uint8_t* data, size_t length) uint8_t
        writeBytes(const uint8_t* data, size_t length) uint8_t
    }

    BUS_BASE <|-- BUS_SPI
    class BUS_SPI {
        _SPI_index SPI_index_e
        _pins pins_t

        setInterruptDriven()
        setDeviceDataRegister(uint8_t deviceRegister, uint8_t* readBuf, size_t readLength)

        readDeviceData() bool
        readRegister(uint8_t reg) uint8_t
        readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) uint8_t
        readRegister(uint8_t reg, uint8_t* data, size_t length) bool
        readBytes(uint8_t* data, size_t length) bool
        readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) bool
        writeRegister(uint8_t reg, uint8_t data) uint8_t
        writeRegister(uint8_t reg, const uint8_t* data, size_t length) uint8_t
        writeBytes(const uint8_t* data, size_t length) uint8_t
    }

    class IMU_Base {
        <<abstract>>
        _axisOrder axis_order_e
        _gyroResolutionRPS float
        _gyroResolutionDPS float
        _accResolution float
        _gyroSampleRateHz uint32_t
        _accSampleRateHz uint32_t
        _gyroOffset xyz_int32_t
        _accOffset xyz_int32_t
        WAIT_IMU_DATA_READY() int32_t
        WAIT_IMU_DATA_READY(uint32_t ticksToWait) int32_t

        readGyroRaw() xyz_int32_t  *
        readAccRaw() xyz_int32_t *

        virtual init()
        virtual setInterruptDriven()
        virtual getGyroOffset() xyz_int32_t
        virtual setGyroOffset()
        virtual getAccOffset() xyz_int32_t
        virtual setAccOffset()

        virtual getAccOneG_Raw() int32_t

        virtual readGyroRPS() xyz_t
        virtual readGyroDPS() xyz_t
        virtual readAcc() xyz_t
        virtual readAccGyroRPS() accGyroRPS_t
        virtual getAccGyroRPS() accGyroRPS_t

        virtual readOrientation() Quaternion
    }
    IMU_Base *-- BUS_BASE

    IMU_Base <|-- IMU_BMI270
    class IMU_BMI270 {
        setInterruptDriven() override

        readGyroRaw() xyz_int32_t override
        readAccRaw() xyz_int32_t override

        readGyroRPS() xyz_t override
        readGyroDPS() xyz_t override
        readAcc() xyz_t override
        readAccGyroRPS() accGyroRPS_t override
        getAccGyroRPS() accGyroRPS_t override
    }
    IMU_BMI270 *-- BUS_SPI

    IMU_Base <|-- IMU_LSM6DS3TR_C
    class IMU_LSM6DS3TR_C {
        setInterruptDriven() override

        readGyroRaw() xyz_int32_t override
        readAccRaw() xyz_int32_t override
        readAccGyroRPS() accGyroRPS_t override
        getAccGyroRPS() accGyroRPS_t override
    }
    IMU_LSM6DS3TR_C *-- BUS_SPI

    IMU_Base <|-- IMU_ICM426xx
    class IMU_ICM426xx {
        setInterruptDriven() override

        readGyroRaw() xyz_int32_t override
        readAccRaw() xyz_int32_t override

        readGyroRPS() xyz_t override
        readGyroDPS() xyz_t override
        readAcc() xyz_t override
        readAccGyroRPS() accGyroRPS_t override
        getAccGyroRPS() accGyroRPS_t override
    }
    IMU_ICM426xx *-- BUS_SPI
```
