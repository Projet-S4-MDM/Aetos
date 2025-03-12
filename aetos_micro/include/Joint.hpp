#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#include <Arduino.h>
#include "quadratureEncoder.hpp"
#include "PID.hpp"
#include "talon_srx.hpp"
#include "timer.hpp"

class Joint
{
public:
    static constexpr unsigned long PID_LOOP_FREQ_HZ = 100ul;

    Joint(QuadratureEncoder *encoder,
          PID *pid,
          TalonSrx *talon);

    ~Joint(void) {};

    void init(void);
    void setSpeedRad(float speed_);
    void update(void);
    long getAngle(void);

private:
    QuadratureEncoder *_encoder = nullptr;
    PID *_pid = nullptr;
    TalonSrx *_talon = nullptr;

    Helpers::Timer<unsigned long, micros> _timerPidLoop =
        Helpers::Timer<unsigned long, micros>(1000000.0f / (float)PID_LOOP_FREQ_HZ);

    Helpers::Timer<unsigned long, micros> _printTimer =
        Helpers::Timer<unsigned long, micros>(1000000.0f / 10ul);

    float _goalSpeed = 0.0f;
};

void Joint::update(void)
{
    _encoder->update();

    if (_printTimer.isDone())
    {
        // Serial.println(_encoder->getAngularVelocity());
    }

    if (_timerPidLoop.isDone())
    {
        float cmd = 0.0f;

        float error = _encoder->getAngularVelocity() - _goalSpeed;

        cmd = _pid->computeCommand(error);

        /*
        Serial.print("Goal speed : ");
        Serial.println(_goalSpeed);
        Serial.print("angular velocity");
        Serial.println(_encoder->getAngularVelocity());
        Serial.print("Error : ");
        Serial.println(error);
        Serial.println();
        */

        _talon->setCmd(cmd);
    }
}

long Joint::getAngle(void)
{
    // return _encoder->getAngle();
    return _encoder->getCount();
}

void Joint::setSpeedRad(float speed_)
{
    _goalSpeed = speed_;
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