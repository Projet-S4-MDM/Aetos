#ifndef __FIT0186_HPP__
#define __FIT0186_HPP__

#include <Arduino.h>
#include "driver/ledc.h"

class FIT0186
{
public:
    static constexpr uint32_t PWM_FREQUENCY = 1000;
    static constexpr ledc_timer_bit_t LEDC_TIMER_RESOLUTION = LEDC_TIMER_8_BIT;

    static constexpr uint32_t PERCENT_TO_DUTY(float percent_)
    {
        return (uint32_t)round(abs(percent_) / 100.0f * (float)(1u << LEDC_TIMER_RESOLUTION) - 1.0f);
    }

    FIT0186(gpio_num_t pwm_pin_, gpio_num_t dir_pin_, bool reversed_, ledc_timer_t timerNumber_ = LEDC_TIMER_0, ledc_channel_t channelNumber1_ = LEDC_CHANNEL_0)
        : _pwm_pin(pwm_pin_), _dir_pin(dir_pin_), _reversed(reversed_), _currentSpeed(0.0f)
    {
        _ledc_motorTimer = timerNumber_;
        _ledc_motorChannel_1 = channelNumber1_;
    }

    ~FIT0186()
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, 0u);
    }

    void init(void)
    {
        pinMode(_pwm_pin, OUTPUT);
        pinMode(_dir_pin, OUTPUT);

        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_RESOLUTION,
            .timer_num = _ledc_motorTimer,
            .freq_hz = PWM_FREQUENCY,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer_config);

        // Configure LEDC channel
        ledc_channel_config_t channel_config = {
            .gpio_num = _pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = _ledc_motorChannel_1,
            .timer_sel = _ledc_motorTimer,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_config);

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _ledc_motorTimer, _frequencePWM);

        this->setCmd(0.0f);
    }

    void setCmd(float cmd_)
    {

        uint32_t duty = PERCENT_TO_DUTY(abs(cmd_));
        Serial.println(duty);

        if (cmd_ > 0)
        {
            digitalWrite(_dir_pin, _reversed ? LOW : HIGH);
        }
        else
        {
            digitalWrite(_dir_pin, _reversed ? HIGH : LOW);
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1);
    }

private:
    gpio_num_t _pwm_pin;
    gpio_num_t _dir_pin;
    uint32_t _frequencePWM;
    bool _reversed;

    ledc_timer_t _ledc_motorTimer;
    ledc_channel_t _ledc_motorChannel_1;

    float _currentSpeed;
};

#endif