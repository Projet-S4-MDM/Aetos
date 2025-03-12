#include <Arduino.h>
#include "config.hpp"
#include "quadratureEncoder.hpp"
#include "talon_srx.hpp"
#include "Joint.hpp"
#include "PID.hpp"

void setup()
{
    Serial.begin(115200);

    QuadratureEncoder encoder1 = QuadratureEncoder(PIN_ENCODER_A1, PIN_ENCODER_B1, PCNT_UNIT_1);
    QuadratureEncoder encoder2 = QuadratureEncoder(PIN_ENCODER_A2, PIN_ENCODER_B2, PCNT_UNIT_3);


    TalonSrx talon1 = TalonSrx(PIN_PWM_1, LEDC_TIMER_0);
    TalonSrx talon2 = TalonSrx(PIN_PWM_2, LEDC_TIMER_1);

    PID pid1 = PID(1.0f, 0.0f, 0.0f, 50.0f);
    PID pid2 = PID(1.0f, 0.0f, 0.0f, 50.0f);

    Joint joint1 = Joint(&encoder1, &pid1, &talon1);
    Joint joint2 = Joint(&encoder2, &pid2, &talon2);

    joint1.init();
    joint2.init();

    Serial.print("Init done!");

    for (;;)
    {
        joint1.update();
        joint2.update();

        joint1.setSpeedRad(30.0f);
        joint2.setSpeedRad(30.0f);

        Serial.println(joint1.getAngle());

    }
}

void loop()
{
}