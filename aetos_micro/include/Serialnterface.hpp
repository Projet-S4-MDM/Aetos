#ifndef __SERIALINTERFACE_HPP__
#define __SERIALINTERFACE_HPP__

#include <Arduino.h>
#include "Joint.hpp"

struct sRequestedVelocity
{
    float xVelocity;
    float yVelocity;
    float zVelocity;
};

class SerialCom
{
public:
    SerialCom(Joint *joint1,
              Joint *joint2,
              Joint *joint3,
              Joint *joint4);

    ~SerialCom() {};

    sRequestedVelocity getVelocityData();
    void sendEncoderData(Joint joint1_, Joint joint2_, Joint joint3_, Joint joint4_);

private:
    sRequestedVelocity _requestedVelocity;
     sRequestedVelocity defaultVelocity = {0.0f, 0.0f, 0.0f};

    Joint *_joint1 = nullptr;
    Joint *_joint2 = nullptr;
    Joint *_joint3 = nullptr;
    Joint *_joint4 = nullptr;
};

sRequestedVelocity SerialCom::getVelocityData()
{
    if (Serial.available() >= sizeof(sRequestedVelocity))
    {
        size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&_requestedVelocity), sizeof(sRequestedVelocity));

        return _requestedVelocity;
    }
    return defaultVelocity;
}

void SerialCom::sendEncoderData(Joint joint1_, Joint joint2_, Joint joint3_, Joint joint4_)
{    
    Serial.print(joint1_.getAngle());
    Serial.print(",");
    Serial.print(joint2_.getAngle());
    Serial.print(",");
    Serial.print(joint3_.getAngle());
    Serial.print(",");
    Serial.println(joint4_.getAngle());
}

SerialCom::SerialCom(Joint *joint1_,Joint *joint2_,Joint *joint3_,Joint *joint4_)
    : _joint1(joint1_), _joint2(joint2_), _joint3(joint3_), _joint4(joint4_) {}

#endif