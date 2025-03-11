#ifndef __ENCODER_HPP__
#define __ENCODER_HPP__

#include <Arduino.h>

class Encoder
{
public:
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
        totalPulses = 0;
        attachInterrupt(digitalPinToInterrupt(_encoder_pin_A), isrHandler, RISING);
    };

    void reset() { totalPulses = 0; };
    float getSpeed() { return 0.0; };
    void update() {};
    long getPulses() { return totalPulses; }
    long getAngle() { return 0.0; };

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
        if(digitalRead(_encoder_pin_B))
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
    gpio_num_t _encoder_pin_A;
    gpio_num_t _encoder_pin_B;
};

Encoder *Encoder::instance = nullptr;

#endif
