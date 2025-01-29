#ifndef __SERIALINTERFACE_HPP__
#define __SERIALINTERFACE_HPP__

#include <Arduino.h>

struct sInputVelocity
{
    float _xVelocity;
    float _yVelocity;
    float _zVelocity;
};

struct sOutputEncoderData
{
    uint8_t _motor0EncoderData;
    uint8_t _motor1EncoderData;
    uint8_t _motor2EncoderData;
    uint8_t _motor3EncoderData;
};

class SerialCom
{
public:
    SerialCom(gpio_num_t encoderPin)
    {
        _encoderPin = encoderPin;
    }

    ~SerialCom() {}

    void init()
    {
        pinMode(_encoderPin, OUTPUT);
    }

    void readVelocity()
    {
        if(Serial.available() >= sizeof(sInputVelocity))
        {
            size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&_inputVelocityData), sizeof(sInputVelocity));
        }
    }

    sOutputEncoderData sendEncoderData()
    {

    }

private:
    sInputVelocity _inputVelocityData;
    sOutputEncoderData _outputEncoderData;

    gpio_num_t _encoderPin;
};

#endif