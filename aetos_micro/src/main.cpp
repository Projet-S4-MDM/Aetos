#include <Arduino.h>
#include "config.hpp"
#include "FIT0186.hpp"

void setup()
{
    Serial.begin(115200);
    FIT01876 motor1 = FIT01876(PIN_PWM_1, PIN_DIR_1, false);
}

void loop()
{
    Serial.println("test");
    delay(10000);
}

