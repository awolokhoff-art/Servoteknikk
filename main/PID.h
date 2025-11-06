#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID{
  private:
  //PID-parametre
  static float Kp;  // proporsjonal
  static float Ki;  // integrasjon
  static float Kd;  // derivasjon
  
  public:
  static float integral;
  
  static long lastError;

  static unsigned long lastTime;
  
  PID();

  // PID-funksjons:Calculate PWM-values based on target pulses and encoder count
  static long computePID(long targetPulses, long EncoderCount);
  
  
};

#endif
