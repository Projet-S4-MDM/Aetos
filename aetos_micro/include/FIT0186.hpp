#ifndef __FIT0186_HPP__
#define __FIT0186_HPP__

#include <Arduino.h>
#include "driver/ledc.h"

class FIT01876
{
public:
    static constexpr uint32_t PWM_FREQUENCY = 1000;
    static constexpr ledc_timer_bit_t LEDC_TIMER_RESOLUTION = LEDC_TIMER_14_BIT;

    static constexpr uint32_t PERCENT_TO_DUTY(float percent_)
    {
        return (uint32_t)round(abs(percent_) / 100.0f * (float)(1u << LEDC_TIMER_RESOLUTION) - 1.0f);
    }

    FIT01876(gpio_num_t pwm_pin_, gpio_num_t dir_pin_, bool reversed_)
    {
        _pwm_pin = pwm_pin_;
        _dir_pin = dir_pin_;
        _reversed = reversed_;
    }

    ~FIT01876() {}

    void init()
    {
        pinMode(_pwm_pin, OUTPUT);
        pinMode(_dir_pin, OUTPUT);

        _frequencePWM = PWM_FREQUENCY;
        ledc_timer_config_t ledc_timer;
    }

private:
    gpio_num_t _pwm_pin;
    gpio_num_t _dir_pin;
    uint32_t _frequencePWM;
    bool _reversed;
};

#endif