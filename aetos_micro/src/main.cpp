#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"

void setup()
{
    Serial.begin(115200);

    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, true);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    PID pid1(1.01f, 0.008f, 0.01f, 30.0f);
    Joint joint1 = Joint(&encoder1, &pid1, &motor1);

    joint1.init();

    Serial.println("Init done");

    for (;;)
    {
        joint1.update();
        joint1.setSpeed(2.0f*PI);
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
