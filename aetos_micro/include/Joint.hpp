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

    void init();

    void resetEncoder();
    void updateEncoder();
    float getSpeed();

private:
    Encoder _encoder;
    PID _pid;
    FIT0186 _fit0186;
};

float Joint::getSpeed()
{
    _encoder.getSpeed();
}

void Joint::updateEncoder()
{
    _encoder.update();
}

void Joint::resetEncoder()
{
    _encoder.reset();
}

void Joint::init()
{
    _encoder.init();
    _pid.init();
    _fit0186.init();
}

Joint::Joint(const Encoder encoder, const PID pid, const FIT0186 fit0186)
    : _encoder(encoder), _pid(pid), _fit0186(fit0186) {}



#endif