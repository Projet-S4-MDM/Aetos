#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"
#include "talon_srx.hpp"
#include "FIT0186.hpp"

void setup()
{
    Serial.begin(115200);

    TalonSrx talon1 = TalonSrx(PIN_PWM_1);
    //FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    // PID pid1(1.0f, 0.0f, 0.0f, 50.0f);
    // Joint joint1 = Joint(&encoder1, &pid1, &talon1);

    // joint1.init();
    encoder1.init();

    talon1.init();

    delay(3000);

    for (;;)
    {
        Serial.println(encoder1.getAngularVelocity());
        // joint1.update();
        // joint1.setSpeedRad(30.0f);
        talon1.setCmd(100.0f);
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
