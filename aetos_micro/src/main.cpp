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
        joint1.update();

        // serialCom.sendEncoderData(joint1, joint2, joint3, joint4);
        requestedVelocity = serialCom.getVelocityData();

        Serial.println(requestedVelocity.xVelocity);
        Serial.println(requestedVelocity.yVelocity);
        Serial.println(requestedVelocity.zVelocity);
    }
}

void loop()
{
}
