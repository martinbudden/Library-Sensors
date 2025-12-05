#include <Arduino.h>
#include <IMU_BMI270.h>

#define IMU_AXIS_ORDER IMU_Base::XPOS_YPOS_ZPOS
static IMU_Base* imu;

#if defined(TARGET_M5STACK_ATOMS3R)
#include <M5Unified.h>
#endif

void setup()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

#if defined(TARGET_M5STACK_ATOMS3R)
    auto cfg = M5.config(); // NOLINT(readability-static-accessed-through-instance)
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Power.begin();

    Serial.begin(115200);

    // statically allocate a BMI270 IMU object
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, BUS_I2C::IMU_I2C_PINS);
#else
    Serial.begin(115200);
    enum { SPI_8_MEGAHERTZ = 8000000, SPI_10_MEGAHERTZ = 10000000, SPI_20_MEGAHERTZ = 20000000 };
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, SPI_20_MEGAHERTZ,  BUS_SPI::IMU_SPI_INDEX, BUS_SPI::IMU_SPI_PINS);
#endif
    imu = &imuSensor;

    // initialize the IMU
    imu->init();
}

void loop()
{
    // take a gyro reading
    const xyz_t gyroDPS =  imu->readGyroDPS();

    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    // take an accelerometer reading
    const xyz_t acc =  imu->readAcc();

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
}
