#include <Arduino.h>
#include "config.hpp"
#include "Joint.hpp"
#include "Serialnterface.hpp"

// #include "FIT0186.hpp"
// #include "driver/ledc.h"

void setup()
{
    Serial.begin(115200);

    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    PID pid1 = PID(1.0f, 0.0f, 0.0f, 0.0f);
    

    

    for (;;)
    {
        delay(100);
    }
}

void loop()
{
}


