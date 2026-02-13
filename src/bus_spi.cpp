#include "bus_spi.h"

//#define LIBRARY_SENSORS_SERIAL_DEBUG
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
#if defined(FRAMEWORK_ARDUINO_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)// ESP32, ARDUINO_ARCH_ESP32 defined in platform.txt
#include <HardwareSerial.h>
#else
#include <Arduino.h>
#endif
#endif

#include <cassert>

#if defined(FRAMEWORK_RPI_PICO)
//#include <boards/pico.h> // for PICO_DEFAULT_LED_PIN
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/spi.h>
#include <pico/binary_info.h>
#include <pico/stdlib.h>
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <cstring>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#elif defined(FRAMEWORK_TEST)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#else // defaults to FRAMEWORK_ARDUINO
//#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <Arduino.h>
#include <SPI.h>
#endif

BusSpi* BusSpi::self {nullptr}; // copy of this for use in ISRs

/*!
NOTE: the RPI Pico cannot do hardware control of CS during multibyte transactions, since the embedded hardware
toggles CS for each byte output. To do this PIO is required, see
https://github.com/raspberrypi/pico-examples/blob/33854562cd08398c4b48bb1ed7fa022c2177076d/pio/spi/spi.pio#L75
for an example.
*/
void BusSpi::cs_select([[maybe_unused]]const BusSpi& bus)
{
#if !defined(LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT)
#if defined(FRAMEWORK_RPI_PICO)
    asm volatile("nop \n nop \n nop"); // NOLINT(hicpp-no-assembler)
    gpio_put(bus._pins.cs.pin, 0);  // Active low
    asm volatile("nop \n nop \n nop"); // NOLINT(hicpp-no-assembler)
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    gpio_set_level(static_cast<gpio_num_t>(bus._pins.cs.pin), 0);
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    HAL_GPIO_WritePin(gpioPort(bus._pins.cs), gpioPin(bus._pins.cs), GPIO_PIN_RESET);
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    //*bus._csOut &= ~bus._csBit; // set _csOut low
    digitalWrite(bus._pins.cs.pin, LOW);
#endif // FRAMEWORK
#endif // LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT
}

void BusSpi::cs_deselect([[maybe_unused]]const BusSpi& bus)
{
#if !defined(LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT)
#if defined(FRAMEWORK_RPI_PICO)
    asm volatile("nop \n nop \n nop"); // NOLINT(hicpp-no-assembler)
    gpio_put(bus._pins.cs.pin, 1);
    asm volatile("nop \n nop \n nop"); // NOLINT(hicpp-no-assembler)
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    gpio_set_level(static_cast<gpio_num_t>(bus._pins.cs.pin), 1);
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    HAL_GPIO_WritePin(gpioPort(bus._pins.cs), gpioPin(bus._pins.cs), GPIO_PIN_SET);
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    //*bus._csOut |= bus._csBit; // set _csOut high
    digitalWrite(bus._pins.cs.pin, HIGH);
#endif // FRAMEWORK
#endif // LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT
}

#if defined(FRAMEWORK_ESPIDF) && false
FAST_CODE static void spi_pre_transaction_callback(spi_transaction_t* trans)
{
    gpio_num_t* csPin = static_cast<gpio_num_t *>(trans->user);
    gpio_set_level(*csPin, 0); // select, cs is active low
}

FAST_CODE static void spi_post_transaction_callback(spi_transaction_t* trans)
{
    gpio_num_t* csPin = static_cast<gpio_num_t *>(trans->user);
    gpio_set_level(*csPin, 1); // deselect, cs is active low
}
#endif

/*!
Read device data using DMA.
This is essentially the same code as in dataReadyISR which allows it to be tested without the extra complication of using interrupts.s
*/
FAST_CODE bool BusSpi::readDeviceDataDMA() // NOLINT(readability-make-member-function-const)
{
//dma_channel_set_irq0_enabled(channel, enabled);
//dma_channel_acknowledge_irq0(channel)
//dma_channel_get_irq0_status (uint channel)
//dma_channel_set_read_addr
//dma_channel_set_write_addr
//dma_channel_set_trans_count
#if defined(FRAMEWORK_RPI_PICO)
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    *_deviceReadBuf = _deviceDataRegister;
    dma_channel_set_trans_count(_dmaTxChannel, _deviceReadLength, DONT_START_YET);
    dma_channel_set_trans_count(_dmaRxChannel, _deviceReadLength, DONT_START_YET);
    dma_channel_set_write_addr(_dmaRxChannel, _deviceReadBuf, DONT_START_YET);
#if 0
    *_deviceReadBuf = _deviceDataRegister;
    dma_channel_configure(_dmaTxChannel, &dmaTxChannelConfig,
                          &spi_get_hw(_spi)->dr, // destination, write data to SPI data register
                          &_deviceRegister, // source, send the value of the IMU register we want to read
                          _deviceReadLength, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                          DONT_START_YET);
    dma_channel_configure(_dmaRxChannel, &dmaRxChannelConfig,
                          _deviceReadBuf, // destination, write SPI data to data
                          &spi_get_hw(_spi)->dr, // source, read data from SPI
                          _deviceReadLength, // element count (each element is 8 bits)
                          DONT_START_YET);
#endif
    cs_select(*this);
    dma_start_channel_mask((1U << _dmaTxChannel) | (1U << _dmaRxChannel));
    // wait for rx to complete
    dma_channel_wait_for_finish_blocking(_dmaRxChannel);
    cs_deselect(*this);
    return true;

#elif defined(FRAMEWORK_ESPIDF)
    return true;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    cs_select(*this);
    *_deviceReadBuf = _deviceDataRegister;
    HAL_SPI_TransmitReceive_DMA(&_spi, _deviceReadBuf, _deviceReadBuf, _deviceReadLength);
    return true;
#elif defined(FRAMEWORK_TEST)
    return true;
#else // defaults to FRAMEWORK_ARDUINO

    return readDeviceData();

#endif // FRAMEWORK
    return false;
}


/*!
IMU Data Ready interrupt service routine (ISR)
Called when IMU interrupt pin signals an interrupt.
Currently support only one interrupt, but could index action off gpio pin
*/
#if defined(FRAMEWORK_RPI_PICO)
void __not_in_flash_func(BusSpi::dataReadyISR)(unsigned int gpio, uint32_t events) // NOLINT(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
{
    // The IMU has indicated it has new data, so initiate a read.
    (void)gpio;
    (void)events;
    // reading the register resets the interrupt
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
#if defined(LIBRARY_SENSORS_USE_SPI_DMA_IN_ISR)
    // data ready signalled in dmaRxCompleteISR
    *self->_deviceReadBuf = self->_deviceDataRegister;
    dma_channel_set_trans_count(self->_dmaTxChannel, self->_deviceReadLength, DONT_START_YET);
    dma_channel_set_trans_count(self->_dmaRxChannel, self->_deviceReadLength, DONT_START_YET);
    dma_channel_set_write_addr(self->_dmaRxChannel, self->_deviceReadBuf, DONT_START_YET);
    cs_select(*self);
    // start DMA
    dma_start_channel_mask((1U << self->_dmaTxChannel) | (1U << self->_dmaRxChannel));
    // wait for rx to complete
    //dma_channel_wait_for_finish_blocking(self->_dmaRxChannel);
    //cs_deselect(*bus);
#else
    self->readDeviceDataDMA();
    self->SIGNAL_DATA_READY_FROM_ISR();
#endif // LIBRARY_SENSORS_USE_SPI_DMA_IN_ISR
}

void BusSpi::dmaRxCompleteISR()
{
    self->cs_deselect(*self);
    //gpio_put(PICO_DEFAULT_LED_PIN, 1);
    dma_channel_acknowledge_irq0(self->_dmaRxChannel);
    self->SIGNAL_DATA_READY_FROM_ISR();
}

#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

/*!
IMU Data Ready Interrupt service routine (ISR)
Called when IMU interrupt pin signals an interrupt.
*/
extern "C" __weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // The IMU has indicated it has new data, so initiate a read.
    if (GPIO_Pin != BusSpi::self->getIrqPin()) {
        return;
    }
#if defined(LIBRARY_SENSORS_USE_SPI_DMA_IN_ISR)
    BusSpi::self->readDeviceDataDMA();
#else
    BusSpi::self->readDeviceData();
    BusSpi::self->SIGNAL_DATA_READY_FROM_ISR();
#endif
}
/*!
Receive Complete Callback ISR
*/
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi);

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    (void)hspi;
    BusSpi::self->SIGNAL_DATA_READY_FROM_ISR();
}

#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

FAST_CODE void BusSpi::dataReadyISR()
{
#if defined(LIBRARY_SENSORS_USE_SPI_DMA)
    static_assert(false); // assert false until this is implemented in FRAMEWORK_ARDUINO
#else
    // for the moment, just read the predefined register into the predefined read buffer
    self->readDeviceData();
    self->SIGNAL_DATA_READY_FROM_ISR();
#endif
}
#endif // FRAMEWORK

BusSpi::~BusSpi() // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
{
#if defined(LIBRARY_SENSORS_USE_SPI_DMA)
#if defined(FRAMEWORK_RPI_PICO)
    dma_channel_unclaim(_dmaRxChannel);
    dma_channel_cleanup(_dmaRxChannel);
    dma_channel_unclaim(_dmaTxChannel);
    dma_channel_cleanup(_dmaTxChannel);
#endif
#endif
}

BusSpi::BusSpi(uint32_t frequencyHz, bus_index_e SPI_index, const spi_pins_t& pins) :
    _frequencyHz(frequencyHz)
    ,_SPI_index(SPI_index)
    ,_pins {
        .cs =   {0,pins.cs},
        .sck =  {0,pins.sck},
        .cipo = {0,pins.cipo},
        .copi = {0,pins.copi},
        .irq =  {0,pins.irq}
    }
#if defined(FRAMEWORK_RPI_PICO)
    ,_spi(SPI_index == BUS_INDEX_1 ? spi1 : spi0)
    ,_dmaInterruptNumber(DMA_IRQ_0)
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F1)
#include <stm32f1xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F3)
#include <stm32f3xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F4)
#include <stm32f4xx_hal_spi.h>
#elif defined(FRAMEWORK_STM32_CUBE_F7)
#include <stm32f7xx_hal_spi.h>
#endif
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    ,_spi(SPI)
#endif // FRAMEWORK
{
    init();
}

BusSpi::BusSpi(uint32_t frequencyHz, bus_index_e SPI_index, const stm32_spi_pins_t& pins) :
    _frequencyHz(frequencyHz)
    ,_SPI_index(SPI_index)
    ,_pins(pins)
#if defined(FRAMEWORK_RPI_PICO)
    ,_spi(SPI_index == BUS_INDEX_1 ? spi1 : spi0)
    ,_dmaInterruptNumber(DMA_IRQ_0)
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
    ,_spi(SPI)
#endif // FRAMEWORK
{
    init();
}

void BusSpi::init()
{
#if !defined(LIBRARY_SENSORS_USE_NO_SPI_INIT)
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.print("BusSpi::init\r\n");
#endif

#if defined(FRAMEWORK_USE_FREERTOS)
    _dataReadyQueue = xQueueCreateStatic(IMU_DATA_READY_QUEUE_LENGTH, sizeof(_dataReadyQueueItem), &_dataReadyQueueStorageArea[0], &_dataReadyQueueStatic);
    configASSERT(_dataReadyQueue);
    const UBaseType_t messageCount = uxQueueMessagesWaiting(_dataReadyQueue);
    assert(messageCount == 0);
    (void)messageCount;
#endif

#if defined(FRAMEWORK_RPI_PICO)
    static_assert(static_cast<int>(IRQ_LEVEL_LOW) == GPIO_IRQ_LEVEL_LOW);
    static_assert(static_cast<int>(IRQ_LEVEL_HIGH) == GPIO_IRQ_LEVEL_HIGH);
    static_assert(static_cast<int>(IRQ_EDGE_FALL) == GPIO_IRQ_EDGE_FALL);
    static_assert(static_cast<int>(IRQ_EDGE_RISE) == GPIO_IRQ_EDGE_RISE);

    gpio_init(_pins.cs.pin);
    gpio_set_dir(_pins.cs.pin, GPIO_OUT);
    // chip select is active-low, so we initialise it to high
    gpio_put(_pins.cs.pin, 1);

    spi_init(_spi, _frequencyHz);
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST); // channel, bits per transfer, polarity, phase, order
    gpio_set_function(_pins.cipo.pin, GPIO_FUNC_SPI);
    gpio_set_function(_pins.sck.pin, GPIO_FUNC_SPI);
    gpio_set_function(_pins.copi.pin, GPIO_FUNC_SPI);
#if defined(LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT)
    gpio_set_function(_pins.cs.pin, GPIO_FUNC_SPI); // map the CS pin, so the RP2040/2340 will automatically toggle the CS pin for SPI
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(_pins.cipo.pin, _pins.copi.pin, _pins.sck.pin, _pins.cs.pin, GPIO_FUNC_SPI));
#else
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(_pins.cipo.pin, _pins.copi.pin, _pins.sck.pin, GPIO_FUNC_SPI));
    bi_decl(bi_1pin_with_name(_pins.cs.pin, "SPI CS"));
#endif

#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    const spi_bus_config_t buscfg = {
        .mosi_io_num = _pins.copi.pin,
        .miso_io_num = _pins.cipo.pin,
        .sclk_io_num = _pins.sck.pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = 0,  //< Output data IO default level when no transaction.
        .max_transfer_sz = 256,
        .flags = 0,       // Abilities of bus to be checked by the driver. Or-ed value of ``SPICOMMON_BUSFLAG_*`` flags.
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,    //< Select cpu core to register SPI ISR.
        .intr_flags = 0   // Interrupt flag for the bus to set the priority, and IRAM attribute
    };

    spi_host_device_t spiHostDevice {};
    if (_SPI_index == BUS_INDEX_0) { // NOLINT(bugprone-branch-clone)
        spiHostDevice = SPI1_HOST;
    } else if (_SPI_index == BUS_INDEX_1) {
        spiHostDevice = SPI2_HOST;
#if (SOC_SPI_PERIPH_NUM > 2)
    } else if (_SPI_index == BUS_INDEX_2) {
       spiHostDevice = SPI3_HOST;
#endif
    }
    static const char *TAG = "BusSpi::init";
    esp_err_t err = spi_bus_initialize(spiHostDevice, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(err));
        return;
    }

    const gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << _pins.cs.pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    const esp_err_t gpioErr = gpio_config(&cs_config);
    if (gpioErr != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config fail: %s", esp_err_to_name(gpioErr));
        //Serial.printf("gpioErr:%d\r\n", gpioErr);
    }
    // chip select is active-low, so we initialise it to high
    const esp_err_t setErr = gpio_set_level(static_cast<gpio_num_t>(_pins.cs.pin), 1);
    if (setErr != ESP_OK) {
        ESP_LOGE(TAG, "gpio_set_level fail: %s", esp_err_to_name(gpioErr));
        //Serial.printf("setErr:%d\r\n", setErr);
    }

    const spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = BITS_PER_BYTE,
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,         ///< Duty cycle of positive clock, in 1/256th increments (128 = 50%/50% duty). Setting this to 0 (=not setting it) is equivalent to setting this to 128.
        .cs_ena_pretrans = 0, // not used
        .cs_ena_posttrans = 0,  // not used
        .clock_speed_hz = static_cast<int>(_frequencyHz),
        .input_delay_ns = 0, // if this is needed, it is better to reduce the clock speed
        .sample_point = SPI_SAMPLING_POINT_PHASE_0, // default
        .spics_io_num = _pins.cs.pin,
        .flags = 0,  // 0 not used
        .queue_size = 4,
        .pre_cb = nullptr,
        .post_cb = nullptr,
    };
    err = spi_bus_add_device(spiHostDevice, &devcfg, &_spi); // sets device handle _spi
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(err));
        return;
    }

#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)

    // STM32 indices are 1-based
    if (_SPI_index == BUS_INDEX_0) {
        _spi.Instance = SPI1;
#if defined(SPI2)
    } else if (_SPI_index == BUS_INDEX_1) {
        _spi.Instance = SPI2;
#endif
#if defined(SPI3)
    } else if (_SPI_index == BUS_INDEX_2) {
        _spi.Instance = SPI3;
#endif
#if defined(SPI4)
    } else if (_SPI_index == BUS_INDEX_3) {
        _spi.Instance = SPI4;
#endif
#if defined(SPI5)
    } else if (_SPI_index == BUS_INDEX_4) {
        _spi.Instance = SPI5;
#endif
#if defined(SPI6)
    } else if (_SPI_index == BUS_INDEX_5) {
        _spi.Instance = SPI6;
#endif
    }
    if (gpioPort(_pins.sck) == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
#if defined(GPIOB)
    if (gpioPort(_pins.sck) == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
#endif
#if defined(GPIOC)
    if (gpioPort(_pins.sck) == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif
#if defined(GPIOD)
    if (gpioPort(_pins.sck) == GPIOD) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif
#if defined(GPIOE)
    if (gpioPort(_pins.sck) == GPIOE) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif

    // Configure GPIO pin for SPI IRQ
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = gpioPin(_pins.irq);
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpioPort(_pins.irq), &GPIO_InitStruct);

#if false
    // Configure GPIO pins for SPI
    GPIO_InitStruct.Pin = gpioPin(_pins.sck) | gpioPin(_pins.cipo) | gpioPin(_pins.copi);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = (_SPI_index == BUS_INDEX_0) ? GPIO_AF5_SPI1 : (_SPI_index == BUS_INDEX_1) ? GPIO_AF5_SPI2 : GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // Configure GPIO pin for SPI SCK
    GPIO_InitStruct.Pin = gpioPin(_pins.sck);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(gpioPort(_pins.irq), &GPIO_InitStruct);
#endif

    // EXTI interrupt init
    //HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    _spi.Instance = (_SPI_index == BUS_INDEX_0) ? SPI1 :
#if defined(FRAMEWORK_STM32_CUBE_F1)
                    SPI2;
#else
                    (_SPI_index == BUS_INDEX_1) ? SPI2 : SPI3;
#endif

    _spi.Init.Mode = SPI_MODE_MASTER;
    _spi.Init.Direction = SPI_DIRECTION_2LINES;
    _spi.Init.DataSize = SPI_DATASIZE_8BIT;
    _spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    _spi.Init.CLKPhase = SPI_PHASE_1EDGE;
#if defined(LIBRARY_SENSORS_USE_SPI_HARDWARE_CHIP_SELECT)
    //_spi.Init.NSS = SPI_NSS_HARD_OUTPUT | SPI_NSS_HARD_INPUT;
    _spi.Init.NSS = SPI_NSS_HARD_OUTPUT;
#else
    _spi.Init.NSS = SPI_NSS_SOFT;
#endif
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    _spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    _spi.Init.TIMode = SPI_TIMODE_DISABLE;
    _spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    _spi.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&_spi);
#elif defined(FRAMEWORK_TEST)

#else // defaults to FRAMEWORK_ARDUINO

    _csBit = digitalPinToBitMask(_pins.cs.pin); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    _csOut = portOutputRegister(digitalPinToPort(_pins.cs.pin)); // NOLINT(cppcoreguidelines-prefer-member-initializer)
    pinMode(_pins.cs.pin, OUTPUT);
    digitalWrite(_pins.cs.pin, 1);
    cs_deselect(*this);
    _spi.begin();

#endif // FRAMEWORK

    configureDMA();

#endif // LIBRARY_SENSORS_USE_NO_SPI_INIT
}

void BusSpi::configureDMA()
{
#if defined(LIBRARY_SENSORS_USE_SPI_DMA)

#if defined(FRAMEWORK_RPI_PICO)
    // DMA transmit channel, writes value of IMU register to SPI
    _dmaTxChannel = dma_claim_unused_channel(true);
    dma_channel_config dmaTxChannelConfig = dma_channel_get_default_config(_dmaTxChannel);
    channel_config_set_transfer_data_size(&dmaTxChannelConfig, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_dreq(&dmaTxChannelConfig, spi_get_dreq(_spi, true));
    channel_config_set_write_increment(&dmaTxChannelConfig, false); // always write to SPI data register
    channel_config_set_read_increment(&dmaTxChannelConfig, false); // don't increment, we value of IMU register each time
    channel_config_set_irq_quiet(&dmaTxChannelConfig, true); // no IRQs on transmit channel
    dma_channel_configure(_dmaTxChannel, &dmaTxChannelConfig,
                        &spi_get_hw(_spi)->dr, // destination, write data to SPI data register
                        &_deviceDataRegister, // source, send the value of the device data register we want to read
                        _deviceReadLength, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                        DONT_START_YET);

    // DMA receive channel, reads from SPI to _deviceReadBuf
    _dmaRxChannel = dma_claim_unused_channel(true);
    dma_channel_config dmaRxChannelConfig = dma_channel_get_default_config(_dmaRxChannel);
    channel_config_set_dreq(&dmaRxChannelConfig, spi_get_dreq(_spi, false));
    channel_config_set_transfer_data_size(&dmaRxChannelConfig, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_write_increment(&dmaRxChannelConfig, true);
    channel_config_set_read_increment(&dmaRxChannelConfig, false); // always read from SPI data register
#if defined(LIBRARY_SENSORS_USE_SPI_DMA_IN_ISR)
    // set it up so that an interrupt goes off when the DMA completes and this interrupt is handled by dmaRxCompleteISR
    channel_config_set_irq_quiet(&dmaRxChannelConfig, false);
    dma_channel_set_irq0_enabled(_dmaRxChannel, true);
    irq_set_exclusive_handler(_dmaInterruptNumber, dmaRxCompleteISR);
    irq_set_enabled(_dmaInterruptNumber, true);
#else
    // no interrupt when DMA completes
    channel_config_set_irq_quiet(&dmaRxChannelConfig, true);
#endif
    dma_channel_configure(_dmaRxChannel, &dmaRxChannelConfig,
                        _deviceReadBuf,
                        &spi_get_hw(_spi)->dr, // source, read data from SPI data register
                        _deviceReadLength, // number of bytes to read (each element is DMA_SIZE_8, ie 8 bits)
                        DONT_START_YET);
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)

#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#endif // FRAMEWORK

#endif // LIBRARY_SENSORS_USE_SPI_DMA
}

/*!
Set this bus so reads are interrupt driven.
When the IMU interrupt pin indicates data ready, the dataReady ISR is called and the data is read in the ISR.

This routine sets the GPIO IRQ pin to input and attaches the dataReady ISR to be triggered by that pin.
*/
void BusSpi::setInterruptDriven(irq_level_e irqLevel) // NOLINT(readability-make-member-function-const)
{
    self = this;
    assert(_pins.irq.pin != IRQ_NOT_SET);

#if defined(FRAMEWORK_RPI_PICO)
    gpio_init(_pins.irq.pin);
    gpio_set_dir(_pins.irq.pin, GPIO_IN);
    enum { IRQ_ENABLED = true };
    gpio_set_irq_enabled_with_callback(_pins.irq.pin, irqLevel, IRQ_ENABLED, &dataReadyISR);
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    const gpio_int_type_t interruptType =
        (irqLevel == IRQ_LEVEL_LOW) ? GPIO_INTR_LOW_LEVEL :
        (irqLevel == IRQ_LEVEL_HIGH) ? GPIO_INTR_HIGH_LEVEL :
        (irqLevel == IRQ_EDGE_FALL) ? GPIO_INTR_NEGEDGE :
        (irqLevel == IRQ_EDGE_RISE) ? GPIO_INTR_POSEDGE : GPIO_INTR_ANYEDGE;
    const gpio_config_t interruptConfig = {
        .pin_bit_mask = (1ULL << _pins.irq.pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = interruptType
    };
    const esp_err_t gpioErr = gpio_config(&interruptConfig);
    (void)gpioErr;
    // now see
    // gpio_intr_enable
    // gpio_isr_register
#elif defined(FRAMEWORK_TEST)
    (void)irqLevel;
#else
    enum { LEVEL_LOW = 0x04, LEVEL_HIGH = 0x05, EDGE_FALL = 0x02, EDGE_RISE = 0x01, EDGE_CHANGE = 0x03 };
    const uint8_t level =
        (irqLevel == IRQ_LEVEL_LOW) ? LEVEL_LOW :
        (irqLevel == IRQ_LEVEL_HIGH) ? LEVEL_HIGH :
        (irqLevel == IRQ_EDGE_FALL) ? EDGE_FALL :
        (irqLevel == IRQ_EDGE_RISE) ? EDGE_RISE : EDGE_CHANGE;
    (void)level;
    //pinMode(_pins.irq.pin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(_pins.irq.pin), &dataReadyISR, level); // esp32-hal-gpio.h
#endif
}

FAST_CODE bool BusSpi::readDeviceData()
{
#if defined(FRAMEWORK_RPI_PICO)
    *_deviceReadBuf = _deviceDataRegister;
    cs_select(*this);
    spi_write_read_blocking(_spi, _deviceReadBuf, _deviceReadBuf, _deviceReadLength);
    cs_deselect(*this);
    return true;
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    // ESPIDF cannot use same memory for tx_buffer and rx_buffer
    _spiTransaction.flags = 0;
    _spiTransaction.addr = _deviceDataRegister,
    _spiTransaction.length = BITS_PER_BYTE*(_deviceReadLength);
    _spiTransaction.rxlength = BITS_PER_BYTE*(_deviceReadLength);
    _spiTransaction.tx_buffer = nullptr;
    _spiTransaction.rx_buffer = _deviceReadBuf;
    spi_device_transmit(_spi, &_spiTransaction);
    return true;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    *_deviceReadBuf = _deviceDataRegister;
    cs_select(*this);
    HAL_SPI_TransmitReceive(&_spi, _deviceReadBuf, _deviceReadBuf, _deviceReadLength, HAL_MAX_DELAY);
    cs_deselect(*this);
    return true;
#else
    return readRegister(_deviceDataRegister, _deviceReadBuf, _deviceReadLength);
#endif
}

#if defined(FRAMEWORK_RPI_PICO)
uint8_t __not_in_flash_func(BusSpi::readRegister)(uint8_t reg) const
#else
FAST_CODE uint8_t BusSpi::readRegister(uint8_t reg) const
#endif
{
    reg |= READ_BIT;
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> outBuf = {{ reg, 0 }};
    std::array<uint8_t, 2> inBuf;
    cs_select(*this);
    spi_write_read_blocking(_spi, &outBuf[0], &inBuf[0], 2);
    cs_deselect(*this);
    return inBuf[1];
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
// see https://github.com/krzychb/esp-lis35de/blob/master/components/lis35de/lis35de.c
    _spiTransaction.flags = SPI_TRANS_USE_RXDATA;
    _spiTransaction.addr = reg,
    _spiTransaction.length = BITS_PER_BYTE;
    _spiTransaction.rxlength = BITS_PER_BYTE;
    _spiTransaction.tx_buffer = nullptr;
    //_spiTransaction.rx_data[0] = 0;
    //spi_device_acquire_bus(_spi, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &_spiTransaction);
#if defined(LIBRARY_SENSORS_SERIAL_DEBUG)
    Serial.printf("read ret:%d(%d)\r\n", ret, _spiTransaction.rx_data[0]);
#endif
    assert(ret == ESP_OK && "SPI readRegister fail");
    //spi_device_release_bus(_spi);
    return _spiTransaction.rx_data[0];
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    std::array<uint8_t, 2> outBuf = {{ reg, 0 }};
    std::array<uint8_t, 2> inBuf;
    cs_select(*this);
    HAL_SPI_TransmitReceive(&_spi, &outBuf[0], &inBuf[0], 2, HAL_MAX_DELAY);
    cs_deselect(*this);
    return inBuf[1];
#elif defined(FRAMEWORK_TEST)
    (void)reg;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequencyHz, MSBFIRST, SPI_MODE0));
    cs_select(*this);
    _spi.transfer(reg);
    const uint8_t ret = _spi.transfer(0); // NOLINT(cppcoreguidelines-init-variables) false positive
    cs_deselect(*this);
    _spi.endTransaction();
    return ret;
#endif // FRAMEWORK
    return 0;
}

FAST_CODE uint8_t BusSpi::readRegisterWithTimeout(uint8_t reg, uint32_t timeoutMs) const
{
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    cs_select(*this);
    std::array<uint8_t, 2> outBuf = {{ static_cast<uint8_t>(reg | READ_BIT), 0 }};
    std::array<uint8_t, 2> inBuf;
    HAL_SPI_TransmitReceive(&_spi, &outBuf[0], &inBuf[0], 2, timeoutMs); // note timeout is in milliseconds
    cs_deselect(*this);
    return inBuf[1];
#else
    (void)timeoutMs;
    return readRegister(reg);
#endif
}

FAST_CODE bool BusSpi::readRegister(uint8_t reg, uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
    reg |= READ_BIT;
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(*this);
    spi_write_blocking(_spi, &reg, 1);
    spi_read_blocking(_spi, 0, data, length); // 0 is the value written as SPI is being read
    cs_deselect(*this);
    return true;
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    _spiTransaction.flags = 0;
    _spiTransaction.addr = reg,
    _spiTransaction.length = BITS_PER_BYTE * length;
    _spiTransaction.rxlength = BITS_PER_BYTE * length;
    _spiTransaction.tx_buffer = nullptr;
    _spiTransaction.rx_buffer = &data[0]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    //spi_device_acquire_bus(_spi, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &_spiTransaction);
    assert(ret == ESP_OK && "SPI readRegister data, len fail");
    //spi_device_release_bus(_spi);
    return ret;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    cs_select(*this);
    HAL_SPI_Transmit(&_spi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&_spi, data, length, HAL_MAX_DELAY);
    cs_deselect(*this);
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    *data = 0;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequencyHz, MSBFIRST, SPI_MODE0));
    cs_select(*this);
    _spi.transfer(reg);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    cs_deselect(*this);
    _spi.endTransaction();
    return true;
#endif // FRAMEWORK
    return false;
}

FAST_CODE bool BusSpi::readBytes(uint8_t* data, size_t length) const // NOLINT(readability-non-const-parameter)
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(*this);
    spi_read_blocking(_spi, 0, data, length);
    cs_deselect(*this);
    return true;
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    _spiTransaction.flags = 0;
    _spiTransaction.addr = *data,
    _spiTransaction.length = BITS_PER_BYTE*length;
    _spiTransaction.rxlength = BITS_PER_BYTE*length;
    _spiTransaction.tx_buffer = nullptr;
    _spiTransaction.rx_buffer = &data[0]; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    //spi_device_acquire_bus(_spi, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &_spiTransaction);
    assert(ret == ESP_OK && "SPI readBytes data, len fail");
    //spi_device_release_bus(_spi);
    return ret;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(&_spi, data, length, HAL_MAX_DELAY);
    return status == HAL_ERROR ? false : true;
#elif defined(FRAMEWORK_TEST)
    *data = 0;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequencyHz, MSBFIRST, SPI_MODE0));
    cs_select(*this);
    for (size_t ii = 0; ii < length; ++ii) {
        data[ii] = _spi.transfer(READ_BIT); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    cs_deselect(*this);
    _spi.endTransaction();
    return true;
#endif // FRAMEWORK
    return false;
}

FAST_CODE bool BusSpi::readBytesWithTimeout(uint8_t* data, size_t length, uint32_t timeoutMs) const
{
#if defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    //  HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(&_spi, data, length, timeoutMs); //!!T
    return status == HAL_ERROR ? false : true;
#else
    (void)timeoutMs;
    return readBytes(data, length);
#endif
}

FAST_CODE uint8_t BusSpi::writeRegister(uint8_t reg, uint8_t data) // NOLINT(readability-make-member-function-const)
{
    reg &= static_cast<uint8_t>(~READ_BIT);
#if defined(FRAMEWORK_RPI_PICO)
    std::array<uint8_t, 2> outBuf = {{ reg, data }}; // remove read bit as this is a write
    std::array<uint8_t, 2> inBuf;
    cs_select(*this);
    spi_write_read_blocking(_spi, &outBuf[0], &inBuf[0], 2);
    cs_deselect(*this);
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    _spiTransaction.flags = SPI_TRANS_USE_TXDATA;
    _spiTransaction.addr = reg,
    _spiTransaction.length = BITS_PER_BYTE;
    _spiTransaction.rxlength = 0;
    _spiTransaction.tx_data[0] = data;
    _spiTransaction.rx_buffer = nullptr;
    //spi_device_acquire_bus(_spi, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &_spiTransaction);
    assert(ret == ESP_OK && "SPI writeRegister data, len fail");
    //spi_device_release_bus(_spi);
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    return writeRegister(reg, &data, 1);
#elif defined(FRAMEWORK_TEST)
    return writeRegister(reg, &data, 1);
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequencyHz, MSBFIRST, SPI_MODE0));
    cs_select(*this);
    _spi.transfer(reg);
    _spi.transfer(data);
    cs_deselect(*this);
    _spi.endTransaction();
#endif // FRAMEWORK
    return 0;
}

FAST_CODE uint8_t BusSpi::writeRegister(uint8_t reg, const uint8_t* data, size_t length) // NOLINT(readability-make-member-function-const)
{
    reg &= static_cast<uint8_t>(~READ_BIT);
#if defined(FRAMEWORK_RPI_PICO)
    reg &= ~READ_BIT; // NOLINT(hicpp-signed-bitwise)
    cs_select(*this);
    spi_write_blocking(_spi, &reg, 1);
    spi_write_blocking(_spi, data, length);
    cs_deselect(*this);
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    _spiTransaction.flags = 0;
    _spiTransaction.addr = reg,
    _spiTransaction.length = BITS_PER_BYTE*length;
    _spiTransaction.rxlength = 0;
    _spiTransaction.tx_buffer = data;
    _spiTransaction.rx_buffer = nullptr;
    //spi_device_acquire_bus(_spi, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &_spiTransaction);
    assert(ret == ESP_OK && "SPI writeRegister data, len fail");
    //spi_device_release_bus(_spi);
    return static_cast<uint8_t>(ret);
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    (void)reg;
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)reg;
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO
    _spi.beginTransaction(SPISettings(_frequencyHz, MSBFIRST, SPI_MODE0));
    cs_select(*this);
    _spi.transfer(reg);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    cs_deselect(*this);
    _spi.endTransaction();

#endif // FRAMEWORK
    return 0;
}

FAST_CODE uint8_t BusSpi::writeBytes(const uint8_t* data, size_t length) // NOLINT(readability-make-member-function-const)
{
#if defined(FRAMEWORK_RPI_PICO)
    cs_select(*this);
    spi_write_blocking(_spi, data, length);
    cs_deselect(*this);
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    _spiTransaction.flags = 0;
    _spiTransaction.addr = *data,
    _spiTransaction.length = BITS_PER_BYTE*(length - 1);
    _spiTransaction.rxlength = 0;
    _spiTransaction.tx_buffer = data + 1;
    _spiTransaction.rx_buffer = nullptr;
    //spi_device_acquire_bus(_spi, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(_spi, &_spiTransaction);
    assert(ret == ESP_OK && "SPI writeBytes data, len fail");
    //spi_device_release_bus(_spi);
    return static_cast<uint8_t>(ret);
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
    (void)data;
    (void)length;
#elif defined(FRAMEWORK_TEST)
    (void)data;
    (void)length;
#else // defaults to FRAMEWORK_ARDUINO

    _spi.beginTransaction(SPISettings(_frequencyHz, MSBFIRST, SPI_MODE0));
    cs_select(*this);
    for (size_t ii = 0; ii < length; ++ii) {
        _spi.transfer(data[ii]); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    cs_deselect(*this);
    _spi.endTransaction();

#endif // FRAMEWORK
    return 0;
}

// Determine the divisor to use for a given bus frequency
uint16_t BusSpi::calculateClockDivider(uint32_t frequencyHz)
{
#if defined(FRAMEWORK_RPI_PICO)
    /*
      SPI clock is set in Betaflight code by calling spiSetClkDivisor, which records a uint16_t value into a .speed field.
      In order to maintain this code (for simplicity), record the prescale and postdiv numbers as calculated in
      pico-sdk/src/rp2_common/hardware_spi.c: spi_set_baudrate()

      prescale and postdiv are in range 1..255 and are packed into the return value.
    */

    uint32_t spiClock = clock_get_hz(clk_peri);
    // Find smallest prescale value which puts output frequency in range of
    // post-divide. Prescale is an even number from 2 to 254 inclusive.
    uint32_t prescale = 2;
    while (prescale <= 254) {
        if (spiClock < prescale * 256 * static_cast<uint64_t>(frequencyHz)) {
            break;
        }
        prescale += 2;
    }
    if (prescale > 254) {
        prescale = 254;
    }

    // Find largest post-divide which makes output <= freq. Post-divide is
    // an integer in the range 1 to 256 inclusive.
    uint32_t postdiv = 256;
    while (postdiv > 1) {
        if (spiClock / (prescale * (postdiv - 1)) > frequencyHz) {
            break;
        }
        --postdiv;
    }

    // Store prescale, (postdiv - 1), both in range 0 to 255.
    return (uint16_t)((prescale << 8) + (postdiv - 1));
#else
    (void)frequencyHz;
    return 1;
#endif
}

// Return the SPI clock based on the given divisor
uint32_t BusSpi::calculateClock(uint16_t clockDivisor)
{
    enum { DEFAULT_CLOCK = 64000000 };
#if defined(FRAMEWORK_RPI_PICO)
    const uint32_t clock = DEFAULT_CLOCK;
#elif defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    const uint32_t clock = DEFAULT_CLOCK;
#elif defined(FRAMEWORK_STM32_CUBE) || defined(FRAMEWORK_ARDUINO_STM32)
#if defined(FRAMEWORK_STM32_CUBE_F1)
    const uint32_t clock = SystemCoreClock / 2;
#elif defined(FRAMEWORK_STM32_CUBE_F3)
    const uint32_t clock = SystemCoreClock / 2;
#elif defined(FRAMEWORK_STM32_CUBE_F4)
    const uint32_t clock = SystemCoreClock / 2;
#elif defined(FRAMEWORK_STM32_CUBE_F7)
    const uint32_t clock = SystemCoreClock / 2;
#else
    const uint32_t clock = DEFAULT_CLOCK;
#endif
    return clock / clockDivisor;
#elif defined(FRAMEWORK_TEST)
    const uint32_t clock = DEFAULT_CLOCK;
#else // defaults to FRAMEWORK_ARDUINO
    const uint32_t clock = DEFAULT_CLOCK;
#endif // FRAMEWORK
    return clock / clockDivisor;
}

// Set the clock divisor to be used for accesses by the given device
void BusSpi::setClockDivisor(uint16_t divisor)
{
    (void)divisor;
}

// Set the clock phase/polarity to be used for accesses by the given device
void BusSpi::setClockPhasePolarity(bool leadingEdge)
{
    (void)leadingEdge;
}

// Enable/disable DMA on a specific device. Enabled by default.
void BusSpi::dmaEnable(bool enable)
{
    (void)enable;
}

// DMA transfer setup and start
void BusSpi::dmaSequence(segment_t* segments)
{
    (void)segments;
}

// Wait for DMA completion
void BusSpi::dmaWait()
{
}

// Negate CS if held asserted after a transfer
void BusSpi::dmaRelease()
{
}

// Return true if DMA engine is busy
bool BusSpi::dmaIsBusy()
{
    return false;
}
