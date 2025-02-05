#include <Arduino.h>
#include "config.hpp"
#include "FIT0186.hpp"

void setup()
{
    FIT0186 motor1 = FIT0186(PIN_PWM_1, PIN_DIR_1, false);

    Serial.begin(115200);
    pinMode(PIN_DIR_1, OUTPUT);
    pinMode(PIN_PWM_1, OUTPUT);

    motor1.init();

    for (;;)
    {
        float start = -100.0f;
        float end = 100.0f;
        unsigned long startTime = millis();
        unsigned long duration = 5000;

        while (true)
        {
            for (unsigned long startTime = millis(); millis() - startTime < duration;)
            {
                float elapsedTime = float(millis() - startTime) / float(duration);
                float currentCmd = start + (end - start) * elapsedTime;

                motor1.setCmd(currentCmd);

                delay(10);
            }
            float temp = start;
            start = end;
            end = temp;

            delay(100);
        }
    }
}

void loop()
{
}