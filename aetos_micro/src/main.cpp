#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"

void setup()
{
    Serial.begin(115200);

    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    motor1.init();
    encoder1.init();
    Serial.println("Init done");

    for (;;)
    {
        encoder1.update();
        float currentAngle = encoder1.getAngle();
        float currentVelocity = encoder1.getAngularVelocity();

        if(currentAngle >= 100.0f*(2.0f*PI))
        {
            motor1.setCmd(0.0f);
        }
        else
        {
            motor1.setCmd(30.0f);
        }
    }
}

void loop()
{
}

// void setup()
// {
//     Serial.begin(9600);

//     for (;;)
//     {
//         if (Serial.available() >= sizeof(sRequestedVelocity))
//         {
//             sRequestedVelocity requestedVelocity;

//             byte *dataPtr = reinterpret_cast<byte *>(&requestedVelocity);
//             for (size_t i = 0; i < sizeof(sRequestedVelocity); i++)
//             {
//                 dataPtr[i] = Serial.read();
//             }

//             Serial.println("Received Velocity Data:");
//             Serial.print("VX: ");
//             Serial.println(requestedVelocity.xVelocity);
//             Serial.print("VY: ");
//             Serial.println(requestedVelocity.yVelocity);
//             Serial.print("VZ: ");
//             Serial.println(requestedVelocity.zVelocity);
//         }
//     }
// }

// void loop()
// {
// }
