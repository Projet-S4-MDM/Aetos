#ifndef __SERIALINTERFACE_HPP__
#define __SERIALINTERFACE_HPP__

#include <Arduino.h>
#include "Joint.hpp"
// #include "joint.hpp"

struct sJoints
{
    Joint joint1;
    Joint joint2;
    Joint joint3;
    Joint joint4;
};

struct sRequestedVelocity
{
    float xVelocity;
    float yVelocity;
    float zVelocity;
};

class SerialCom
{
public:
    SerialCom(Joint joint1, Joint joint2, Joint joint3, Joint joint4);
    ~SerialCom();

private:
    void init();
    void getVelocityData();
    void sendEncoderData();

    sJoints _joints;
    sRequestedVelocity _requestedVelocity;
};

void SerialCom::init()
{
}

void SerialCom::getVelocityData()
{
    if (Serial.available() >= sizeof(sRequestedVelocity))
    {
        size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&_requestedVelocity), sizeof(sRequestedVelocity));
    }
}

void SerialCom::sendEncoderData()
{
}

SerialCom::SerialCom(Joint joint1_, Joint joint2_, Joint joint3_, Joint joint4_)
{
    _joints.joint1 = joint1_;
    _joints.joint2 = joint2_;
    _joints.joint3 = joint3_;
    _joints.joint4 = joint4_;
}

// sInputVelocity getDesiredVelocity()
// {
// if (Serial.available() >= sizeof(sInputVelocity))
// {
// size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&_inputVelocityData), sizeof(sInputVelocity));
// }
// return _inputVelocityData;
// }

#endif