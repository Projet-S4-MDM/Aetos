#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"

void setup()
{
    Serial.begin(115200);

    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    PID pid1 = PID(1.0f, 0.0f, 0.0f, 0.0f);

    Joint joint1 = Joint(&encoder1, &pid1, &motor1);
    Joint joint2 = Joint(&encoder1, &pid1, &motor1);
    Joint joint3 = Joint(&encoder1, &pid1, &motor1);
    Joint joint4 = Joint(&encoder1, &pid1, &motor1);

    joint1.init();
    joint1.setSpeed(30.0f);

    SerialCom serialCom = SerialCom(&joint1, &joint2, &joint3, &joint4);

    sRequestedVelocity requestedVelocity;

    for (;;)
    {
        joint1.updateInternal();
        serialCom.sendEncoderData(joint1, joint2, joint3, joint4);
        // requestedVelocity = serialCom.getVelocityData();
        // serialCom.sendEncoderData(joint1, joint2, joint3, joint4);

        // joint1.
        // motor1.setCmd(30.0f);
        // requestedVelocity = serialCom.getVelocityData();

        // Serial.println(requestedVelocity.xVelocity);
        // Serial.println(requestedVelocity.yVelocity);
        // Serial.println(requestedVelocity.zVelocity);

        // joint1.
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
