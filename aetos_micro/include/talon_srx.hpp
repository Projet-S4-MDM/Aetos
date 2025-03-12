#ifndef __TALON_SRX_HPP__
#define __TALON_SRX_HPP__

#include "Arduino.h"
#include "driver/ledc.h"

#define IN_ERROR(VAR, ERROR, GOAL) ((abs(VAR) < (abs(GOAL) + ERROR) && abs(VAR) > (abs(GOAL) - ERROR)))
#define MAP(x, in_min, in_max, out_min, out_max) \
    (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) + (float)(out_min))

class TalonSrx
{
private:
    static constexpr float SIGNAL_FULL_STOP_MS = 1500.0f; // Middle of deadband
    static constexpr float SIGNAL_FULL_REVERSE_MS = 1000.0f;
    static constexpr float SIGNAL_MIN_REVERSE_MS = 1480.0f;
    static constexpr float SIGNAL_FULL_FORWARD_MS = 2000.0f;
    static constexpr float SIGNAL_MIN_FORWARD_MS = 1520.0f;

    static constexpr uint32_t MS_TO_DUTY(uint32_t freq_, float microSeconds_)
    {
        float period = 1.0f / static_cast<float>(freq_);
        return static_cast<uint32_t>(round(((microSeconds_ / 1'000'000.f) / period) * ((float)(1u << 14))));
    }

public:
    // Try using the same timer for all same frequency signal. Don't use the same channel
    TalonSrx(gpio_num_t pinPWM_,
             ledc_timer_t timerNumber_ = LEDC_TIMER_0,
             ledc_channel_t channelNumber_ = LEDC_CHANNEL_0,
             float signalFrequencyHz_ = 50.0f)
    {
        _pinPWM = pinPWM_;
        _timer = timerNumber_;
        _channel = channelNumber_;

        _freqSignal = (uint32_t)round(signalFrequencyHz_);
    }

    ~TalonSrx(void)
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, _channel, 0u);
    }

    void init(void)
    {
        pinMode(_pinPWM, OUTPUT);

        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = _timer;
        ledc_timer.duty_resolution = LEDC_TIMER_14_BIT;
        ledc_timer.freq_hz = _freqSignal;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_channel;
        ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel = _channel;
        ledc_channel.timer_sel = _timer;
        ledc_channel.intr_type = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num = _pinPWM;
        ledc_channel.duty = MS_TO_DUTY(_freqSignal, SIGNAL_FULL_STOP_MS);
        ledc_channel.hpoint = 0;
        ledc_channel_config(&ledc_channel);

        ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, _freqSignal);
        this->writeMicroseconds(SIGNAL_FULL_STOP_MS);
    }

    // -100.0 to 100.0 for speed
    void setCmd(float cmd_)
    {
        _speed = cmd_;

        if (IN_ERROR(_speed, 0.01f, 0.0f))
        {
            this->stop();
        }
        else if (_speed < 0.0f)
        {
            writeMicroseconds(MAP(_speed, -100.0f, 0.0f, SIGNAL_FULL_REVERSE_MS, SIGNAL_MIN_REVERSE_MS));
        }
        else if (_speed > 0.0f)
        {
            writeMicroseconds(MAP(_speed, 0.0f, 100.0f, SIGNAL_MIN_FORWARD_MS, SIGNAL_FULL_FORWARD_MS));
        }
    }

    float getCmd(void)
    {
        return _speed;
    }

    void disable(void)
    {
        stop();
    }

    // Return true if the motor is moving.
    bool isMoving(void)
    {
        if (_speed != 0.0f)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    void writeMicroseconds(float microseconds_)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, MS_TO_DUTY(_freqSignal, microseconds_));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
    }

    void stop(void)
    {
        writeMicroseconds(SIGNAL_FULL_STOP_MS);
    }

    float _protectionSpeed = 0.0f;
    gpio_num_t _pinPWM;
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    uint32_t _freqSignal;
    float _speed;
};

#endif