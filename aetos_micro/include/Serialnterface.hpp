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

struct sEncoderData
{
    long encoder1;
    long encoder2;
    long encoder3;
    long encoder4;
};

class SerialCom
{
public:
    SerialCom(Joint joint1, Joint joint2, Joint joint3, Joint joint4);
    ~SerialCom() {};

    sRequestedVelocity getVelocityData();
    void sendEncoderData(sEncoderData _encoderData);

private:
    Joint _joint1;
    Joint _joint2;
    Joint _joint3;
    Joint _joint4;

    sRequestedVelocity _requestedVelocity;
    sEncoderData _encoderData;
};

sRequestedVelocity SerialCom::getVelocityData()
{
    if (Serial.available() >= sizeof(sRequestedVelocity))
    {
        size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&_requestedVelocity), sizeof(sRequestedVelocity));

        return _requestedVelocity;
    }
}

void SerialCom::sendEncoderData(sEncoderData encoderData_)
{
    Serial.print(encoderData_.encoder1);
    Serial.print(",");
    Serial.print(encoderData_.encoder2);
    Serial.print(",");
    Serial.print(encoderData_.encoder3);
    Serial.print(",");
    Serial.println(encoderData_.encoder4);
}

SerialCom::SerialCom(const Joint joint1_, const Joint joint2_, const Joint joint3_, const Joint joint4_)
    : _joint1(joint1_), _joint2(joint2_), _joint3(joint3_), _joint4(joint4_) {}

#endif