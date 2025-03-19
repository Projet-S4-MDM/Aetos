#include <Arduino.h>
#include "config.hpp"
#include "quadratureEncoder.hpp"
#include "talon_srx.hpp"
#include "Joint.hpp"
#include "PID.hpp"

void setup()
{
    Serial.begin(115200);

    QuadratureEncoder encoder1 = QuadratureEncoder(PIN_ENCODER_A1, PIN_ENCODER_B1, PCNT_UNIT_0);
    QuadratureEncoder encoder2 = QuadratureEncoder(PIN_ENCODER_A2, PIN_ENCODER_B2, PCNT_UNIT_1);
    QuadratureEncoder encoder3 = QuadratureEncoder(PIN_ENCODER_A3, PIN_ENCODER_B3, PCNT_UNIT_2);
    QuadratureEncoder encoder4 = QuadratureEncoder(PIN_ENCODER_A4, PIN_ENCODER_B4, PCNT_UNIT_3);

    TalonSrx talon1 = TalonSrx(PIN_PWM_1, LEDC_TIMER_0, LEDC_CHANNEL_0);
    TalonSrx talon2 = TalonSrx(PIN_PWM_2, LEDC_TIMER_1, LEDC_CHANNEL_2);
    TalonSrx talon3 = TalonSrx(PIN_PWM_3, LEDC_TIMER_2, LEDC_CHANNEL_3);
    TalonSrx talon4 = TalonSrx(PIN_PWM_4, LEDC_TIMER_3, LEDC_CHANNEL_4);

    PID pid1 = PID(1.0f, 0.0f, 0.0f, 50.0f);
    PID pid2 = PID(1.0f, 0.0f, 0.0f, 50.0f);
    PID pid3 = PID(1.0f, 0.0f, 0.0f, 50.0f);
    PID pid4 = PID(1.0f, 0.0f, 0.0f, 50.0f);

    Joint joint1 = Joint(&encoder1, &pid1, &talon1);
    Joint joint2 = Joint(&encoder2, &pid2, &talon2);
    Joint joint3 = Joint(&encoder3, &pid3, &talon3);
    Joint joint4 = Joint(&encoder4, &pid4, &talon4);

    encoder1.begin();
    encoder2.begin();
    encoder3.begin();
    encoder4.begin();

    joint1.init();
    joint2.init();
    joint3.init();
    joint4.init();

    Serial.print("Init done!");

    for (;;)
    {
        joint1.update();
        joint2.update();
        joint3.update();
        joint4.update();

        joint1.setSpeedRad(10.0f);
        joint2.setSpeedRad(10.0f);
        joint3.setSpeedRad(10.0f);
        joint4.setSpeedRad(10.0f);
    }
}

void loop()
{
}