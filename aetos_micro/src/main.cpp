#include <Arduino.h>

void setup()
{
    Serial.begin(115200);

    Serial.println("ESP32 has been setup");
}

void loop()
{
    Serial.println("test");
    delay(10000);
}

