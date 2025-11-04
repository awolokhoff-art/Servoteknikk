#ifndef PID_H
#define PID_H

#include <Arduino.h>

// --- PID-parametre ---
extern float Kp;  // proporsjonal
extern float Ki;  // integrasjon
extern float Kd;  // derivasjon

extern long lastError;
extern float integral;
extern unsigned long lastTime;

// --- PID-funksjon ---
// Beregn PWM-verdi basert på target pulses og encoder count
long computePID(long targetPulses, long EncoderCount);

#endif
