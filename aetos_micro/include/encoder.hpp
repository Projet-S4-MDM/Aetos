#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include <Arduino.h>
#include "timer.hpp"
#include "moving_average.hpp"

class Encoder
{
public:
    Encoder(gpio_num_t encoder_pin_A_, gpio_num_t encoder_pin_B_)
    {
        _encoder_pin_A = encoder_pin_A_;
        _encoder_pin_B = encoder_pin_B_;
        instance = this;
    }

    void init()
    {
        pinMode(_encoder_pin_A, INPUT);
        pinMode(_encoder_pin_B, INPUT);
        totalPulses = 0;
        lastPulses = 0;
        _angularVelocity = 0.0f;
        _updateTimer.init(20); // 50ms -> 20Hz
        attachInterrupt(digitalPinToInterrupt(_encoder_pin_A), isrHandler, RISING);
    }

    void reset()
    {
        totalPulses = 0;
        lastPulses = 0;
        _angularVelocity = 0.0f;
        _updateTimer.reset();
    }

    long getPulses()
    {
        return totalPulses;
    }

    float getAngle()
    {
        // A quadrature encoder has two channels (A and B) that are 90° out of phase
        return (totalPulses * 8.0f * PI) / countsPerRevolution; // Adjusting for 4x pulse counting
    }

    float getAngularVelocity()
    {
        return _angularVelocity;
    }

    void update()
    {
        if (_updateTimer.isDone())
        {
            long pulseChange = totalPulses - lastPulses;
            float dTheta = ((float)pulseChange * 8.0f * PI) / countsPerRevolution;    // Convert pulses to radians
            _angularVelocity = _speedAvg.addValue(dTheta / (_updateTimer.getInterval() / 1000.0f)); // Convert ms to s
            //Serial.println(_angularVelocity);

            lastPulses = totalPulses;
        }
    }

private:
    static constexpr float countsPerRevolution = 2797.0f;

    static void isrHandler() { instance->countPulse(); }

    void countPulse()
    {
        if (digitalRead(_encoder_pin_B))
        {
            totalPulses++;
        }
        else
        {
            totalPulses--;
        }
    }

    static Encoder *instance;

    long totalPulses;
    long lastPulses;
    float _angularVelocity;
    gpio_num_t _encoder_pin_A;
    gpio_num_t _encoder_pin_B;
    Helpers::Timer<unsigned long, millis> _updateTimer;
    Helpers::MovingAverage<float, 10> _speedAvg = Helpers::MovingAverage<float, 10>(0.0f);
};

Encoder *Encoder::instance = nullptr;

#endif