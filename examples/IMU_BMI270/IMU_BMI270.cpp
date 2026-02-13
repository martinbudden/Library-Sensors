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
    delay(400); // delay to allow serial port to initialize before first print

    // statically allocate a BMI270 IMU object
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, BusI2c::IMU_I2C_PINS);
#else
    Serial.begin(115200);
    delay(1000); // delay to allow serial port to initialize before first print

    // we need to deselect the optical flow chip, which is on the same SPI bus as the IMU.
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    const BusSpi::spi_pins_t opticalFlowPins = BusSpi::OPTICAL_FLOW_PINS;
    const gpio_config_t opticalFlowConfig = {
        .pin_bit_mask = (1ULL << opticalFlowPins.cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    const esp_err_t gpioErr = gpio_config(&opticalFlowConfig);
    if (gpioErr != ESP_OK) {
        Serial.printf("gpioErr:%d\r\n", gpioErr);
    }
    const esp_err_t setErr = gpio_set_level(static_cast<gpio_num_t>(opticalFlowPins.cs), 1);
    if (setErr != ESP_OK) {
        Serial.printf("setErr:%d\r\n", setErr);
    }
#else
    //const BusSpi::spi_pins_t spiPins = BusSpi::IMU_SPI_PINS;
    //SPI.begin(spiPins.sck, spiPins.cipo, spiPins.copi, spiPins.cs);
    //const BusSpi::spi_pins_t opticalFlowPins = BusSpi::OPTICAL_FLOW_PINS;
    const BusSpi::spi_pins_t opticalFlowPins = BusSpi::spi_pins_t{.cs=12,.sck=44,.cipo=43,.copi=14,.irq=0xFF};
    pinMode(opticalFlowPins.cs, OUTPUT);
    digitalWrite(opticalFlowPins.cs, 1);
#endif

    enum { SPI_8_MEGAHERTZ = 8000000, SPI_10_MEGAHERTZ = 10000000, SPI_20_MEGAHERTZ = 20000000 };
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, SPI_8_MEGAHERTZ,  BusSpi::IMU_SPI_INDEX, BusSpi::IMU_SPI_PINS);
#endif

    imu = &imuSensor;

    // initialize the IMU
    const int gyroSampleRate = imu->init();

    Serial.printf("gyro sample rate:%d\r\n", gyroSampleRate);
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
