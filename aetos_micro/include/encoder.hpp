#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include <Arduino.h>
#include "timer.hpp"

class Encoder
{
public:
    Encoder(gpio_num_t encoder_pin_A_, gpio_num_t encoder_pin_B_)
        : updateTimer(50)
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
        angularVelocity = 0.0f;
        updateTimer.init(50); // 50ms -> 20Hz
        attachInterrupt(digitalPinToInterrupt(_encoder_pin_A), isrHandler, RISING);
    }

    void reset()
    {
        totalPulses = 0;
        lastPulses = 0;
        angularVelocity = 0.0f;
        updateTimer.reset();
    }

    long getPulses()
    {
        return totalPulses;
    }

    float getAngle()
    {
        // A quadrature encoder has two channels (A and B) that are 90° out of phase
        return (totalPulses * 8.0f * PI) / 2797.0f; // Adjusting for 4x pulse counting
    }

    float getAngularVelocity()
    {
        return angularVelocity;
    }

    void update()
    {
        if (updateTimer.isDone())
        {
            long pulseChange = totalPulses - lastPulses;
            float dTheta = (pulseChange * 2.0f * PI) / 2797.0f; // Convert pulses to radians
            angularVelocity = dTheta / (updateTimer.getInterval() / 1000.0f);
            lastPulses = totalPulses;
        }
    }

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
    float angularVelocity;
    gpio_num_t _encoder_pin_A;
    gpio_num_t _encoder_pin_B;
    Helpers::Timer<unsigned long, millis> updateTimer;
};

Encoder *Encoder::instance = nullptr;

#endif