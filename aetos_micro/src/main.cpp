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
        // Transition from -100.0f to 100.0f and back to -100.0f in 5 seconds each direction
        float start = -100.0f;
        float end = 100.0f;
        unsigned long startTime = millis();
        unsigned long duration = 5000; // 5 seconds

        while (true)
        {
            // Transition from -100.0f to 100.0f
            for (unsigned long startTime = millis(); millis() - startTime < duration;)
            {
                // Calculate the elapsed time
                float elapsedTime = float(millis() - startTime) / float(duration);

                // Interpolate the command value (smooth transition)
                float currentCmd = start + (end - start) * elapsedTime;

                // Set the motor command
                motor1.setCmd(currentCmd);

                // Add a small delay to allow for smooth updates
                delay(10);
            }

            // Switch direction
            float temp = start;
            start = end;
            end = temp;

            // Wait briefly before starting the next transition
            delay(100);
        }
    }
}

void loop()
{
}