#include <Arduino.h>
struct VelocityData
{
    float vx;
    float vy;
    float vz;
};

void setup()
{
    Serial.begin(460800);
    Serial.println("Serial port ready. Send a message:");
}

void loop()
{
    if (Serial.available() >= sizeof(VelocityData))
    {
        VelocityData velocityData;

        byte *dataPtr = reinterpret_cast<byte *>(&velocityData);
        for (size_t i = 0; i < sizeof(VelocityData); i++)
        {
            dataPtr[i] = Serial.read();
        }

        Serial.println("Received Velocity Data:");
        Serial.print("VX: ");
        Serial.println(velocityData.vx);
        Serial.print("VY: ");
        Serial.println(velocityData.vy);
        Serial.print("VZ: ");
        Serial.println(velocityData.vz);
    }
}
