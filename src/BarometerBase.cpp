#include "BarometerBase.h"
#include <cassert>


#if defined(FRAMEWORK_RPI_PICO)
#include <pico/time.h>
#elif defined(FRAMEWORK_ESPIDF)
#elif defined(FRAMEWORK_STM32_CUBE)
#elif defined(FRAMEWORK_TEST)
#else // defaults to FRAMEWORK_ARDUINO
#include <Arduino.h>
#endif


BarometerBase::BarometerBase(BUS_BASE& busBase) :
    _busBase(&busBase)
{
}

void BarometerBase::delayMs(int ms)
{
    BUS_BASE::delayMs(ms);
}
