#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#include <Arduino.h>
#include "QuadratureEncoder.hpp"
#include "PID.hpp"
#include "TalonSrx.hpp"
#include "timer.hpp"

class Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ_HZ = 10000ul;
    static constexpr unsigned long PRINT_FREQ_HZ = 100ul;

    Joint(QuadratureEncoder *encoder,
          PID *pid,
          TalonSrx *talon);

    ~Joint(void) {};

    void init(void);
    void setSpeedRad(float speed_);
    void update(void);
    float getAngleRadians(void);
    void resetEncoders(void);

private:
    QuadratureEncoder *_encoder = nullptr;
    PID *_pid = nullptr;
    TalonSrx *_talon = nullptr;

    Helpers::Timer<unsigned long, micros> _timerPidLoop =
        Helpers::Timer<unsigned long, micros>(1000000.0f / (float)PID_LOOP_FREQ_HZ);

    float _goalSpeed = 0.0f;
};

void Joint::update(void)
{
    _encoder->update();

    if (_timerPidLoop.isDone())
    {
        float cmd = 0.0f;
        float error = _goalSpeed - _encoder->getAngularVelocity();
        cmd = _pid->computeCommand(error);
        _talon->setCmd(cmd);
    }
}

float Joint::getAngleRadians(void)
{
    return _encoder->getAngleRadians();
}

void Joint::setSpeedRad(float speed_)
{
    _goalSpeed = -speed_;
}

void Joint::resetEncoders(void)
{
    _encoder->reset();
}

void Joint::init()
{
    _encoder->begin();
    _pid->init();
    _talon->init();
}

Joint::Joint(QuadratureEncoder *encoder_, PID *pid_, TalonSrx *talon_)
    : _encoder(encoder_), _pid(pid_), _talon(talon_) {}

#endif