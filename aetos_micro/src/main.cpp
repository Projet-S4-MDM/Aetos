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

        size_t bytesRead = Serial.readBytes(reinterpret_cast<char *>(&velocityData), sizeof(VelocityData));

        if (bytesRead < sizeof(VelocityData))
        {
            Serial.println("Error: Incomplete data received");
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
