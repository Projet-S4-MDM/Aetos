#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"

void setup()
{
    Serial.begin(115200);

    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, true);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    PID pid1(1.0f, 0.0f, 0.0f, 50.0f);
    Joint joint1 = Joint(&encoder1, &pid1, &motor1);

    // joint1.init();
    motor1.init();

    Serial.println("Init done");
    delay(3000);
    for (;;)
    {
        motor1.setCmd(100.0f);
        // joint1.update();+0++++++++++++++++000000000000++0
        // joint1.setSpeedRad(70.0f);
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
