#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include <Arduino.h>
#include "timer.hpp"
#include "moving_average.hpp"

class Encoder
{
public:
    static constexpr uint32_t COUNTS_PER_REVOLUTION = 2797;
    static constexpr uint8_t MAX_INSTANCES = 4;
    static Encoder *instances[MAX_INSTANCES];
    static uint8_t instanceCount;

    Encoder(gpio_num_t encoder_pin_A_, gpio_num_t encoder_pin_B_)
    {
        _encoder_pin_A = encoder_pin_A_;
        _encoder_pin_B = encoder_pin_B_;
        _pulses = 0;
        _lastPulses = 0;
        _angularVelocity = 0.0f;
    }

    bool init()
    {
        if (instanceCount >= MAX_INSTANCES)
        {
            return false;
        }
        pinMode(_encoder_pin_A, INPUT);
        pinMode(_encoder_pin_B, INPUT);

        //instances[instanceCount] = this;
        attachInterrupt(digitalPinToInterrupt(_encoder_pin_A), isrHandlerA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_encoder_pin_B), isrHandlerB, CHANGE);

        _velocityTimer.init(10); // 10ms update interval
        instanceCount++;
        return true;
    }

    void reset()
    {
        _pulses = 0;
        _lastPulses = 0;
        _angularVelocity = 0.0f;
        _velocityTimer.reset();
    }

    long getPulses() const
    {
        return _pulses;
    }

    float getAngle() const
    {
        return (_pulses % COUNTS_PER_REVOLUTION) * 360.0f / COUNTS_PER_REVOLUTION;
    }

    float getAngularVelocity()
    {
        update();
        return _angularVelocity;
    }

    void update()
    {
        if (_velocityTimer.isDone())
        {
            long pulseDelta = _lastPulses - _pulses;
            float timeDelta = _velocityTimer.getInterval() / 1000.0f;
            float pulsesPerSecond = pulseDelta / timeDelta;

            _angularVelocity = _speedAvg.addValue(pulsesPerSecond * (2.0f * PI) / COUNTS_PER_REVOLUTION);
            _lastPulses = _pulses;
        }
    }

private:
    gpio_num_t _encoder_pin_A;
    gpio_num_t _encoder_pin_B;
    long _pulses;
    long _lastPulses;
    float _angularVelocity;
    Helpers::Timer<unsigned long, millis> _velocityTimer;
    Helpers::MovingAverage<float, 10> _speedAvg = Helpers::MovingAverage<float, 10>(0.0f);

    static void IRAM_ATTR isrHandlerA()
    {
        for (uint8_t i = 0; i < instanceCount; i++)
        {
            if (instances[i])
            {
                instances[i]->handleInterrupt();
            }
        }
    }

    static void IRAM_ATTR isrHandlerB()
    {
        for (uint8_t i = 0; i < instanceCount; i++)
        {
            if (instances[i])
            {
                instances[i]->handleInterrupt();
            }
        }
    }

    void IRAM_ATTR handleInterrupt()
    {
        static uint8_t oldAB = 0;

        uint8_t currentA = digitalRead(_encoder_pin_A);
        uint8_t currentB = digitalRead(_encoder_pin_B);
        uint8_t currentAB = (currentB << 1) | currentA;
        uint8_t encoded = (oldAB << 2) | currentAB;

        if (encoded == 0b0001 || encoded == 0b0111 || encoded == 0b1110 || encoded == 0b1000)
        {
            _pulses++;
        }
        else if (encoded == 0b0010 || encoded == 0b1011 || encoded == 0b1101 || encoded == 0b0100)
        {
            _pulses--;
        }

        oldAB = currentAB;
    }
};

Encoder *Encoder::instances[Encoder::MAX_INSTANCES] = {nullptr};
uint8_t Encoder::instanceCount = 0;

#endif // __ENCODER_HPP__
