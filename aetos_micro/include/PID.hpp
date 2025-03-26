#ifndef __PID_HPP__
#define __PID_HPP__

#include <Arduino.h>

class PID
{
public:
    static constexpr unsigned long PERIOD_SPEED_CALC_US = 10'000ul;

    PID(float kp_, float ki_, float kd_, float intergalLimit_);
    ~PID() {}
    void init(void);
    void setGains(float kp_, float ki_, float kd_);
    void setIntLimit(float limit_);
    /// @brief
    /// @param error_ Error between goal and actual position
    /// @return New command
    float computeCommand(float error_);
    /// @brief Resets integral counter and derivative last value to zero
    void reset(void);

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;

    float _cmdI = 0.0f;
    float _lastcmdD = 0.0f;
    float _integralLimit = 0.0f;
    float _previousError = 0.0f;
    float _lastMeasureTime = 0.0f;
};

PID::PID(float kp_, float ki_, float kd_, float intergalLimit_)
{
    this->setGains(kp_, ki_, kd_);
    this->setIntLimit(intergalLimit_);
    this->reset();
    _lastMeasureTime = micros();
}

void PID::init(void)
{
    this->reset();
}

void PID::setGains(float kp_, float ki_, float kd_)
{
    _kp = kp_;
    _ki = ki_;
    _kd = kd_;
}

void PID::setIntLimit(float limit_)
{
    _integralLimit = abs(limit_);
}

float PID::computeCommand(float error_)
{
    if (isnan(error_))
    {
        error_ = 0.0f;
    }
    if (isinf(error_))
    {
        error_ = 0.0f;
    }

    _cmdI += _ki * error_;

    float cmdP = _kp * error_;
    _cmdI = constrain(_cmdI, -_integralLimit, _integralLimit);

    float cmdD = 0.0f;
    float currentTime = micros();
    if (currentTime - _lastMeasureTime > PERIOD_SPEED_CALC_US)
    {
        float dt = currentTime - _lastMeasureTime;
        _lastMeasureTime = currentTime;

        cmdD = _kd * (error_ - _previousError) / (dt / 1'000'000.0f);
        _lastcmdD = cmdD;
    }
    else
    {
        cmdD = _lastcmdD;
    }

    if (isnan(cmdP))
    {
        cmdP = 0.0f;
    }
    if (isnan(_cmdI))
    {
        _cmdI = 0.0f;
    }
    if (isnan(cmdD))
    {
        cmdD = 0.0f;
    }

    _previousError = error_;

    return cmdP + _cmdI + cmdD;
}

void PID::reset(void)
{
    _cmdI = 0.0f;
    _previousError = 0.0f;
}

#endif // __PID_HPP__
