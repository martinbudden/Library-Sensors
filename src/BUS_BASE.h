#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#endif
#endif

#if defined(FRAMEWORK_RPI_PICO)

#elif defined(FRAMEWORK_ESPIDF)

#if !defined(FAST_CODE)
#define FAST_CODE IRAM_ATTR
#endif
#if !defined(IRAM_ATTR)
#define IRAM_ATTR
#endif

#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_gpio.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_gpio.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>
#endif

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO
#if defined(FRAMEWORK_ARDUINO_RPI_PICO)
#elif defined(FRAMEWORK_ARDUINO_ESP32)
#endif // FRAMEWORK_ARDUINO

#endif // FRAMEWORK

#if !defined(FAST_CODE)
#define FAST_CODE
#endif


/*!
Base class for BUS_I2C and BUS_SPI.

Note
bus_index is zero-based.

RPI Pico bus index is zero-based so, for SPI, BUS_INDEX_0 corresponds to spi0

ESPIDF bus index is one-based so, for SPI, BUS_INDEX_0 corresponds to SPI1_HOST

STM32 bus index is one-based so, for SPI, BUS_INDEX_0 corresponds to SPI1
*/
class BUS_BASE {
public:
    enum bus_index_e : uint8_t { BUS_INDEX_0, BUS_INDEX_1, BUS_INDEX_2, BUS_INDEX_3, BUS_INDEX_4, BUS_INDEX_5, BUS_INDEX_6, BUS_INDEX_7 };
    static constexpr uint8_t READ_BIT = 0x80U;
    enum { SPI_PRE_READ_BUFFER_SIZE = 4 }; // to reserve space for the transmit instruction that occurs before a read
#if defined(FRAMEWORK_ESPIDF)
    enum { SPI_PRE_READ_BUFFER_OFFSET = SPI_PRE_READ_BUFFER_SIZE };
#else
    enum { SPI_PRE_READ_BUFFER_OFFSET = SPI_PRE_READ_BUFFER_SIZE - 1 };
#endif
    enum { IRQ_NOT_SET = 0xFF };
    enum irq_level_e { IRQ_LEVEL_LOW = 0x01, IRQ_LEVEL_HIGH = 0x02, IRQ_EDGE_FALL = 0x04, IRQ_EDGE_RISE = 0x08, IRQ_EDGE_CHANGE = 0x04|0x08 };
    struct port_pin_t {
        uint8_t port;
        uint8_t pin;
    };
public:
    /*!
    The deviceDataRegister is the register that is read to provide the data of primary interest.
    This sets up a pre-read buffer for the register, so the register can be read using a single SPI transmit/receive instruction.
    This can be used, in particular, to simplify DMA on some architectures.
    */
    void setDeviceDataRegister(uint8_t deviceDataRegister, uint8_t* readBuf, size_t readLength) {
        _deviceDataRegister = deviceDataRegister | READ_BIT;
        _deviceReadBuf = readBuf + SPI_PRE_READ_BUFFER_OFFSET;
        *_deviceReadBuf = _deviceDataRegister;
        _deviceReadLength = readLength - SPI_PRE_READ_BUFFER_OFFSET;
    }
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    static inline GPIO_TypeDef* gpioPort(port_pin_t portPin) { return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + portPin.port*(GPIOB_BASE - GPIOA_BASE)); }
    static inline uint16_t gpioPin(port_pin_t portPin) { return static_cast<uint16_t>(1U << portPin.pin); }
#endif
protected:
    uint8_t _deviceDataRegister {}; // the device register that is read in the readDeviceData() function
    uint8_t* _deviceReadBuf {};
    size_t _deviceReadLength {};
#if defined(FRAMEWORK_USE_FREERTOS)
    mutable uint32_t _dataReadyQueueItem {}; // this is just a dummy item whose value is not used
    enum { IMU_DATA_READY_QUEUE_LENGTH = 1 };
    std::array<uint8_t, IMU_DATA_READY_QUEUE_LENGTH * sizeof(_dataReadyQueueItem)> _dataReadyQueueStorageArea {};
    StaticQueue_t _dataReadyQueueStatic {};
    QueueHandle_t _dataReadyQueue {};
public:
    inline int32_t WAIT_DATA_READY() const { return xQueueReceive(_dataReadyQueue, &_dataReadyQueueItem, portMAX_DELAY); }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { return xQueueReceive(_dataReadyQueue, &_dataReadyQueueItem, ticksToWait); } // returns pdPASS(1) if queue read, pdFAIL(0) if timeout
    inline void SIGNAL_DATA_READY_FROM_ISR() const { xQueueSendFromISR(_dataReadyQueue, &_dataReadyQueueItem, nullptr); }
#else
public:
    inline int32_t WAIT_DATA_READY() const { return 0; }
    inline int32_t WAIT_DATA_READY(uint32_t ticksToWait) const { (void)ticksToWait; return 0; }
    inline void SIGNAL_DATA_READY_FROM_ISR() const {}
#endif
};
