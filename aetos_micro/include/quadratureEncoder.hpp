#ifndef __QUADRATURE_ENCODER_HPP__
#define __QUADRATURE_ENCODER_HPP__

#include <Arduino.h>
#include "driver/pcnt.h"
#include "timer.hpp"
#include "moving_average.hpp"

// 1398 pulses per revolution

class QuadratureEncoder
{
    static constexpr uint32_t COUNTS_PER_REVOLUTION = 2797 / 2;

public:
    QuadratureEncoder(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit) : _pinA(pinA), _pinB(pinB), _pcntUnit(unit) {}

    void begin()
    {
        // Configure pulse counter
        pcnt_config_t pcntConfig = {
            .pulse_gpio_num = _pinA,
            .ctrl_gpio_num = _pinB,
            .lctrl_mode = PCNT_MODE_REVERSE,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = PCNT_COUNT_INC,
            .neg_mode = PCNT_COUNT_DEC,
            .counter_h_lim = 32767,
            .counter_l_lim = -32768,
            .unit = _pcntUnit,
            .channel = PCNT_CHANNEL_0};

        pcnt_unit_config(&pcntConfig);

        // Enable input filter to remove noise
        pcnt_set_filter_value(_pcntUnit, 100);
        pcnt_filter_enable(_pcntUnit);

        // Initialize counter
        pcnt_counter_pause(_pcntUnit);
        pcnt_counter_clear(_pcntUnit);
        pcnt_counter_resume(_pcntUnit);

        _velocityTimer.init(10);
    }

    int16_t getCount()
    {
        int16_t count = 0;
        pcnt_get_counter_value(_pcntUnit, &count);
        return count;
    }

    float getAngularVelocity()
    {
        update();
        return _angularVelocity;
    };

    void update()
    {
        if (_velocityTimer.isDone())
        {
            _pulses = getCount();
            long pulseDelta = _lastPulses - _pulses;
            float timeDelta = _velocityTimer.getInterval() / 1000.0f;
            float pulsesPerSecond = pulseDelta / timeDelta;

            _angularVelocity = _speedAvg.addValue(pulsesPerSecond * (2.0f * PI) / COUNTS_PER_REVOLUTION);
            _lastPulses = _pulses;
        }
    }

    void reset()
    {
        pcnt_counter_clear(_pcntUnit);
    }

private:
    gpio_num_t _pinA;
    gpio_num_t _pinB;
    pcnt_unit_t _pcntUnit;

    long _pulses;
    long _lastPulses;
    float _angularVelocity = 0.0f;
    Helpers::Timer<unsigned long, millis> _velocityTimer;
    Helpers::MovingAverage<float, 10> _speedAvg = Helpers::MovingAverage<float, 10>(0.0f);
};

#endif