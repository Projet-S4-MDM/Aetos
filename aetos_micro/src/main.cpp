#include <Arduino.h>
#include "config.hpp"
#include "FIT0186.hpp"
#include "Joint.hpp"
#include "driver/ledc.h"

void setup()
{
    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);
    Encoder encoder1 = Encoder(PIN_ENCODER_A1, PIN_ENCODER_B1);
    // PID pid1 = PID();

    Serial.begin(115200);

    motor1.init();
    encoder1.init();

    for (;;)
    {
        motor1.setCmd(-30.0f);

        Serial.println(encoder1.getPulses());
        delay(100);
    }
}

void loop()
{
}