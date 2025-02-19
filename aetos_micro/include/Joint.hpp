#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#include <Arduino.h>
#include "encoder.hpp"
#include "PID.hpp"
#include "FIT0186.hpp"
#include "timer.hpp"

class Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ_US = 1000ul;

    Joint(Encoder *encoder,
          PID *pid,
          FIT0186 *fit0186);

    ~Joint(void) {};

    void init(void);
    void setSpeed(float speed_);
    void update(void);
    long getAngle(void);

private:
    Encoder *_encoder = nullptr;
    PID *_pid = nullptr;
    FIT0186 *_fit0186 = nullptr;

    Helpers::Timer<unsigned long, micros> _timerPidLoop =
        Helpers::Timer<unsigned long, micros>(1000000.0f / (float)PID_LOOP_FREQ_US);

    float _goalSpeed = 0.0f;
};

void Joint::update(void)
{
    _encoder->update();
    float cmd = 0.0f;

    if(_timerPidLoop.isDone())
    {
        cmd = _pid->computeCommand(_goalSpeed);
        _fit0186->setCmd(cmd);
    }
}

long Joint::getAngle(void)
{
    // return _encoder->getAngle();
    return _encoder->getPulses();
}

void Joint::setSpeed(float speed_)
{
    _goalSpeed = speed_;
}

void Joint::init()
{
    _encoder->init();
    _pid->init();
    _fit0186->init();
}

Joint::Joint(Encoder *encoder_, PID *pid_, FIT0186 *fit0186_)
    : _encoder(encoder_), _pid(pid_), _fit0186(fit0186_) {}

#endif