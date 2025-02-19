#ifndef __SERIALINTERFACE_HPP__
#define __SERIALINTERFACE_HPP__

#include <Arduino.h>
#include "Joint.hpp"

struct sRequestedVelocity
{
    float motor1Velocity;
    float motor2Velocity;
    float motor3Velocity;
    float motor4Velocity;
};

struct sEncoderData
{
    float encoder1Data;
    float encoder2Data;
    float encoder3Data;
    float encoder4Data;
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
    sEncoderData _encoderData;

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
    }
    return _requestedVelocity;
}

void SerialCom::sendEncoderData(Joint joint1_, Joint joint2_, Joint joint3_, Joint joint4_)
{
    _encoderData = {
        static_cast<float>(joint1_.getAngle()),
        static_cast<float>(joint2_.getAngle()),
        static_cast<float>(joint3_.getAngle()),
        static_cast<float>(joint4_.getAngle()),
    };

    Serial.write(reinterpret_cast<uint8_t *>(&_encoderData), sizeof(_encoderData));
}

SerialCom::SerialCom(Joint *joint1_, Joint *joint2_, Joint *joint3_, Joint *joint4_)
    : _joint1(joint1_), _joint2(joint2_), _joint3(joint3_), _joint4(joint4_) {}

#endif