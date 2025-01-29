#include <Arduino.h>
#include "config.hpp"
#include "FIT0186.hpp"
#include "Serialnterface.hpp"

void setup()
{
    Serial.begin(460800);

    SerialCom motor0 = SerialCom(PIN_ENCODER_1);
}

void loop()
{
    
}
