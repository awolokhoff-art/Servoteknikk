#include "PID.h"
#include "DC_motor.h"  // For PwmValue, MotorDirection, pins

// --- PID-parametre ---
float Kp = 0.2;
float Ki = 0.1;
float Kd = 0.05;

long lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// --- PID-beregning ---
long computePID(long targetPulses, long EncoderCount) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    long error = targetPulses - EncoderCount;

    if (abs(PwmValue) < 255) {
        integral += error * dt;
    }

    float derivative = (error - lastError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Sett retning og PWM
    if (output > 0) {
        MotorDirection = true;
        PwmValue = min((int)output, 255);
    } else {
        MotorDirection = false;
        PwmValue = min((int)(-output), 255);
    }

    digitalWrite(directionPin, MotorDirection ? HIGH : LOW);
    analogWrite(PWMpin, PwmValue);

    return output;
}
