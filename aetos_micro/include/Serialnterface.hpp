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
    // Serial.print(joint1_.getAngle());
    // Serial.print(",");
    // Serial.print(joint2_.getAngle());
    // Serial.print(",");
    // Serial.print(joint3_.getAngle());
    // Serial.print(",");
    // Serial.println(joint4_.getAngle());
    float value1 = 3.14;
    float value2 = 0.01;
    float value3 = 0.21;
    float value4 = 4.01;
    
    Serial.write(reinterpret_cast<uint8_t *>(&value1), sizeof(value1));
    Serial.write(reinterpret_cast<uint8_t *>(&value2), sizeof(value2));
    Serial.write(reinterpret_cast<uint8_t *>(&value3), sizeof(value3));
    Serial.write(reinterpret_cast<uint8_t *>(&value4), sizeof(value4));

    // Serial.print("1.05");
    // Serial.print(",");
    // Serial.print("-3.41");
    // Serial.print(",");
    // Serial.print("18.2");
    // Serial.print(",");
    // Serial.println("-0.02");
}

SerialCom::SerialCom(Joint *joint1_, Joint *joint2_, Joint *joint3_, Joint *joint4_)
    : _joint1(joint1_), _joint2(joint2_), _joint3(joint3_), _joint4(joint4_) {}

#endif