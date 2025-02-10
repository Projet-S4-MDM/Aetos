#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"

// #include "FIT0186.hpp"
// #include "driver/ledc.h"

void setup()
{
    Serial.begin(115200);

    // JOINT 1
    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    PID pid1 = PID(1.0f, 0.0f, 0.0f, 0.0f);
    
    Joint joint1 = Joint(encoder1, pid1, motor1);
    Joint joint2 = Joint(encoder1, pid1, motor1);
    Joint joint3 = Joint(encoder1, pid1, motor1);
    Joint joint4 = Joint(encoder1, pid1, motor1);

    SerialCom serialcom = SerialCom(joint1, joint2, joint3, joint4);
    
    joint1.init();
    

    for (;;)
    {
        joint1.setCmd(-30.0f);

        Serial.println(joint1.getPulses());
        delay(100);
    }
}

void loop()
{
}
=======
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

