#if defined (LIBRARY_SENSORS_USE_EMPTY_MAIN)

#include "imu_LSM6DS3TR_C.h"
static void setupIMU()
{
    enum {PA=0, PB=1, PC=2, PD=3, PE=4, PF=5, PG=6, PH=7};

    //const BusSpi::spi_pins_t pins = IMU_SPI_PINS;

#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    static constexpr uint32_t spiFrequency = 20000000;
    const IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, spiFrequency, BusSpi::IMU_SPI_INDEX, BusSpi::IMU_SPI_PINS);
    (void)imu;
#else
    enum { I2C_Address = 0x68 };
    //const IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, BusI2c::i2c_pins_t{.sda=11,.scl=14,.irq=0xFF});
    const IMU_LSM6DS3TR_C imu(IMU_Base::XPOS_YPOS_ZPOS, BusI2c::stm32_i2c_pins_t{.sda={PB,11},.scl={PB,14},.irq={0,0xFF}});
    (void)imu;
#endif
}

#if defined(FRAMEWORK_RPI_PICO)
int main()
{
    setupIMU();
    return 0;
}

#elif defined(FRAMEWORK_ESPIDF)

extern "C" void app_main()
{
    setupIMU();
}

#elif defined(FRAMEWORK_STM32_CUBE)

int main()
{
    setupIMU();
    return 0;
}

#elif defined(FRAMEWORK_TEST)

int main()
{
    setupIMU();
    return 0;
}

#else // defaults to FRAMEWORK_ARDUINO

#include <Arduino.h>


void setup()
{
    setupIMU();
}

void loop()
{
}
#endif // FRAMEWORK

#endif //     -D LIBRARY_SENSORS_USE_EMPTY_MAIN
