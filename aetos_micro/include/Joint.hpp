#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#include <Arduino.h>
#include "encoder.hpp"
#include "PID.hpp"
#include "FIT0186.hpp"

class Joint
{
public:
    Joint(const Encoder encoder, const PID pid, const FIT0186 fit0186);
    ~Joint() {};

    // INIT
    void init(void);

    // ENCODER METHODS
    void resetEncoder(void);
    void updateEncoder(void);
    long getPulses(void);

    // TODO: VERIFY VALIDITY
    // void updateEncoder(void);

    // MOTOR METHODS
    float getSpeed(void);
    void setCmd(float cmd_);

    // PID METHODS
    void setGains(float kp_, float ki_, float kd_);
    void setIntLimit(float limit_);
    float computeCommand(float error_);
    void resetPID(void);

private:
    Encoder _encoder;
    PID _pid;
    FIT0186 _fit0186;
};

long Joint::getPulses(void)
{
    return _encoder.getPulses();
}

float Joint::getSpeed(void)
{
    return _encoder.getSpeed();
}

void Joint::setCmd(float cmd_)
{
     _fit0186.setCmd(cmd_);
}

void Joint::updateEncoder(void)
{
    _encoder.update();
}

void Joint::resetEncoder(void)
{
    _encoder.reset();
}

void Joint::setGains(float kp_, float ki_, float kd_)
{
    _pid.setGains(kp_, ki_, kd_);
}

void Joint::setIntLimit(float limit_)
{
    _pid.setIntLimit(limit_);
}

float Joint::computeCommand(float error_)
{
    return _pid.computeCommand(error_);
}

void Joint::resetPID(void)
{
    _pid.reset();
}

void Joint::init()
{
    _encoder.init();
    _pid.init();
    _fit0186.init();
}

Joint::Joint(const Encoder encoder_, const PID pid_, const FIT0186 fit0186_)
    : _encoder(encoder_), _pid(pid_), _fit0186(fit0186_) {}

#endif