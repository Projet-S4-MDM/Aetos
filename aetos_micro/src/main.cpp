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