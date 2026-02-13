#include "Targets.h"
#include <Arduino.h>
#include <imu_lsm6ds3tr_c.h>
#include <imu_mpu6000.h>

static ImuBase* imu;
static bool ledOn = false;
//#define INTERRUPT_DRIVEN


enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};
struct port_pin_t {
    uint8_t port;
    uint8_t pin;
};
static inline GPIO_TypeDef* gpioPort(port_pin_t portPin) { return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + portPin.port*(GPIOB_BASE - GPIOA_BASE)); }
static inline uint16_t gpioPin(port_pin_t portPin) { return static_cast<uint16_t>(1U << portPin.pin); }


void setup()
{
    Serial.begin(115200);

    // Configure LED pin
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = gpioPin(GPIO_LED_0),
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
#if !defined(FRAMEWORK_STM32_CUBE_F1)
        .Alternate = 0
#endif
    };
    HAL_GPIO_Init(gpioPort(GPIO_LED_0), &GPIO_InitStruct);
    HAL_GPIO_WritePin(gpioPort(GPIO_LED_0), gpioPin(GPIO_LED_0), GPIO_PIN_RESET);

    // statically allocate the IMU object
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    static IMU_CLASS imuStatic(ImuBase::XPOS_YPOS_ZPOS, SPI_FREQUENCY_HZ, BusSpi::IMU_SPI_INDEX, BusSpi::IMU_SPI_PINS);
#else
    static IMU_CLASS imuStatic(ImuBase::XPOS_YPOS_ZPOS, BusI2c::IMU_I2C_PINS);
#endif

    imu = &imuStatic;
    // initialize the IMU
    imu->init();
#if defined(INTERRUPT_DRIVEN)
    imu->set_interrupt_driven();
#endif
}

void loop()
{
    // wait for the IMU data ready interrupt
#if defined(INTERRUPT_DRIVEN)
    imu->WAIT_IMU_DATA_READY();
#else
    imu->read_acc_gyro_rps();
#endif

    Serial.println();

    // get the gyro data read in the Interrupt Service Routine
    const acc_gyro_rps_t acc_gyro_rps =  imu->get_acc_gyro_rps();

    // convert the gyro data from radians per second to degrees per second
    const xyz_t gyroDPS =  acc_gyro_rps.gyroRPS * ImuBase::RADIANS_TO_DEGREES;
    Serial.println();
    Serial.print("gyroX:");
    Serial.print(gyroDPS.x, 1);
    Serial.print(" gyroY:");
    Serial.print(gyroDPS.y, 1);
    Serial.print(" gyroZ:");
    Serial.println(gyroDPS.z, 1);

    const xyz_t acc =  acc_gyro_rps.acc;

    Serial.print("accX:");
    Serial.print(acc.x);
    Serial.print(" accY:");
    Serial.print(acc.y);
    Serial.print(" accZ:");
    Serial.println(acc.z);

    delay(500);
    ledOn = !ledOn;
    HAL_GPIO_WritePin(gpioPort(GPIO_LED_0), gpioPin(GPIO_LED_0), ledOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
