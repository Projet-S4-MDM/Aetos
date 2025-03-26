#include <Arduino.h>
#include "config.hpp"
#include "quadratureEncoder.hpp"
#include "talon_srx.hpp"
#include "Joint.hpp"
#include "PID.hpp"
#include "Serialnterface.hpp"

static constexpr unsigned long READ_VELOCITY_FREQ_HZ = 100ul;
static constexpr unsigned long SEND_ANGLE_FREQ_HZ = 100ul;

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

    PID pid1 = PID(KP, KI, KD, 50.0f);
    PID pid2 = PID(KP, KI, KD, 50.0f);
    PID pid3 = PID(KP, KI, KD, 50.0f);
    PID pid4 = PID(KP, KI, KD, 50.0f);

    Joint joint1 = Joint(&encoder1, &pid1, &talon1);
    Joint joint2 = Joint(&encoder2, &pid2, &talon2);
    Joint joint3 = Joint(&encoder3, &pid3, &talon3);
    Joint joint4 = Joint(&encoder4, &pid4, &talon4);

    SerialCom serialCom = SerialCom(&joint1, &joint2, &joint3, &joint4);
    sRequestedVelocity requestedVelocity;

    Helpers::Timer<unsigned long, micros> _timerReadVelocity =
        Helpers::Timer<unsigned long, micros>(1000000.0f / (float)READ_VELOCITY_FREQ_HZ);

    Helpers::Timer<unsigned long, micros> _timerSendAngle =
        Helpers::Timer<unsigned long, micros>(1000000.0f / (float)SEND_ANGLE_FREQ_HZ);

    encoder1.begin();
    encoder2.begin();
    encoder3.begin();
    encoder4.begin();

    joint1.init();
    joint2.init();
    joint3.init();
    joint4.init();

    for (;;)
    {
        requestedVelocity = serialCom.getVelocityData();
        // Serial.println(requestedVelocity.motor1Velocity);

        if (_timerReadVelocity.isDone())
        {
            joint1.setSpeedRad(requestedVelocity.motor1Velocity);
            joint2.setSpeedRad(requestedVelocity.motor2Velocity);
            joint3.setSpeedRad(requestedVelocity.motor3Velocity);
            joint4.setSpeedRad(requestedVelocity.motor4Velocity);
        }

        if (_timerSendAngle.isDone())
        {
            serialCom.sendEncoderData();
        }

        joint1.update();
        joint2.update();
        joint3.update();
        joint4.update();
    }
}

void loop()
{
}