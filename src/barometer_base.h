#pragma once

#include "bus_base.h"


/*!
Barometer virtual base class.
*/
class BarometerBase {
public:
    static constexpr int32_t NOT_DETECTED = -1;

    // Values for reporting barometer type back to MSP (MultiWii Serial Protocol)
    static constexpr uint8_t MSP_BAROMETER_ID_DEFAULT = 0;
    static constexpr uint8_t MSP_BAROMETER_ID_NONE = 1;
    static constexpr uint8_t MSP_BAROMETER_ID_BMP280 = 4;
    static constexpr uint8_t MSP_BAROMETER_ID_VIRTUAL = 11;

public:
    virtual ~BarometerBase() = default;
    explicit BarometerBase(BusBase& bus_base) : _bus_base(&bus_base) {}
    virtual int init() = 0;

    uint32_t getSample_rate_hz() const { return _sample_rate_hz; }
    float get_temperature_celsius() const { return _temperature_celsius; }
    float get_pressure_pascals() const { return _pressure_pascals; }

    virtual void read_temperature_and_pressure() = 0;
    virtual float calculate_altitude_meters(float pressure_pascals, float temperature_celsius) = 0;

    void set_reference_altitude(float reference_altitude) { _reference_altitude = reference_altitude; }
    float get_reference_altitude() const { return _reference_altitude; };
    void set_pressure_at_reference_altitude(float pressure_at_reference_altitude) { _pressure_at_reference_altitude = pressure_at_reference_altitude; }
    float get_pressure_at_reference_altitude() const { return _pressure_at_reference_altitude; };
protected:
    static void delay_ms(int ms) { BusBase::delay_ms(ms); }
protected:
    BusBase* _bus_base;
    uint32_t _sample_rate_hz {};
    float _reference_altitude {};
    float _pressure_at_reference_altitude {};
    float _pressure_pascals {};
    float _temperature_celsius {};
};
