#include "PID.h"
#include "DC_motor.h"

//PID-parametre
float PID::Kp = 0.2;
float PID::Ki = 0.1;
float PID::Kd = 0.05;

float PID::integral = 0;

long PID::lastError = 0;

unsigned long PID::lastTime = 0;

PID::PID(){}

// Pid Calculations
long PID::computePID(long targetPulses, long EncoderCount) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    long error = targetPulses - EncoderCount;

    if (abs(DCmotor::PwmValue) < 255) {
        integral += error * dt;
    }

    float derivative = (error - lastError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Sett retning og PWM
    if (output > 0) {
        DCmotor::MotorDirection = true;
        DCmotor::PwmValue = min((int)output, 255);
    } else {
        DCmotor::MotorDirection = false;
        DCmotor::PwmValue = min((int)(-output), 255);
    }

    digitalWrite(DCmotor::directionPin, DCmotor::MotorDirection ? HIGH : LOW);
    analogWrite(DCmotor::PWMpin, DCmotor::PwmValue);

    return output;
}
