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
        return (uint32_t)round(abs(percent_) / 100.0f * (float)((1u << LEDC_TIMER_RESOLUTION) - 1));
    }

    FIT0186(gpio_num_t pwm_pin_, gpio_num_t dir_pin_, bool reversed_, ledc_timer_t timerNumber_ = LEDC_TIMER_0, ledc_channel_t channelNumber1_ = LEDC_CHANNEL_0)
        : _pwm_pin(pwm_pin_), 
          _dir_pin(dir_pin_), 
          _reversed(reversed_), 
          _currentSpeed(0.0f),
          _frequencePWM(PWM_FREQUENCY)  // Initialize _frequencePWM
    {
        _ledc_motorTimer = timerNumber_;
        _ledc_motorChannel_1 = channelNumber1_;
    }

    ~FIT0186()
    {
        if (ledc_fade_func_install(0) == ESP_OK) {  // Install LEDC fade function first
            ledc_stop(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, 0);
            ledc_fade_func_uninstall();  // Clean up
        }
    }

    void init(void)
    {
        // Configure GPIO pins
        pinMode(_dir_pin, OUTPUT);

        // Configure LEDC timer
        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_RESOLUTION,
            .timer_num = _ledc_motorTimer,
            .freq_hz = _frequencePWM,
            .clk_cfg = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

        // Configure LEDC channel
        ledc_channel_config_t channel_config = {
            .gpio_num = _pwm_pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = _ledc_motorChannel_1,
            .timer_sel = _ledc_motorTimer,
            .duty = 0,
            .hpoint = 0,
            .flags = {
                .output_invert = 0
            }
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

        // Install LEDC fade function
        ESP_ERROR_CHECK(ledc_fade_func_install(0));

        this->setCmd(0.0f);
    }

    void setCmd(float cmd_)
    {
        uint32_t duty = PERCENT_TO_DUTY(abs(cmd_));
        
        if (cmd_ > 0)
        {
            digitalWrite(_dir_pin, _reversed ? LOW : HIGH);
        }
        else
        {
            digitalWrite(_dir_pin, _reversed ? HIGH : LOW);
        }

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _ledc_motorChannel_1));
        
        _currentSpeed = cmd_;
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