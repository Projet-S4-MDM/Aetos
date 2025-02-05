#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include <Arduino.h>
#include "timer.hpp"
#include "chrono.hpp"

class Encoder
{
public:
    // Timer speed
    static constexpr unsigned long TIME_SPEED_CALC_US = 10'000ul;

    Encoder(gpio_num_t encoder_pin_A_, gpio_num_t encoder_pin_B_)
    {
        _encoder_pin_A = encoder_pin_A_;
        _encoder_pin_B = encoder_pin_B_;
        instance = this;
    };

    void init()
    {
        pinMode(_encoder_pin_A, INPUT);
        pinMode(_encoder_pin_B, INPUT);
        _totalPulses = 0;
        //attachInterrupt(digitalPinToInterrupt(_encoder_pin_A), isrHandler, RISING);
    };

    void reset() { _totalPulses = 0.0f; };
    float getSpeed() { return _currentSpeed; };

    void update()
    {
        if (_timerSpeedCalc.isDone())
        {
            Serial.println("GHEHEHEHEH");
            //_currentSpeed = (_lastPosition - _currentPosition) / ((float)_chronoSpeedCalc.getTime() / 1'000'000.0f);
            //_chronoSpeedCalc.restart();
            //_lastPosition = _currentPosition;
        }
    };

    long getPulses() { return _totalPulses; }

private:
    static void isrHandler()
    {
        if (instance)
        {
            instance->countPulse();
        }
    }

    void countPulse()
    {
        if (digitalRead(_encoder_pin_B))
        {
            _totalPulses++;
        }
        else
        {
            _totalPulses--;
        }
    }

    static Encoder *instance;

    Helpers::Chrono<unsigned long, micros> _chronoSpeedCalc;
    Helpers::Timer<unsigned long, micros> _timerSpeedCalc = Helpers::Timer<unsigned long, micros>(TIME_SPEED_CALC_US);

    int _totalPulses = 0;
    float _currentSpeed = 0.0f;
    float _currentPosition = 0.0f;
    float _lastPosition = 0.0f;

    gpio_num_t _encoder_pin_A;
    gpio_num_t _encoder_pin_B;
};

Encoder *Encoder::instance = nullptr;

#endif
