#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>
#include "Elevator.h"

class DCmotor{
private:
  static const int EncoderA;
  static const int EncoderB;
  static int currentLogicalFloor;
  static float currentEncoderFloor;
  

public:
  static const int PulsesPerRevolution;
  static const int PWMpin;
  static const int directionPin;
  
  static int PwmValue;

  static volatile long EncoderCount;

  static bool MotorDirection;

  static float floorTolerance;
  static float velocity;

  DCmotor();
  static void DC_init();
  static void Enc_init();
  static void encoderAChange();
  static void encoderBChange();
  static void GetEncoderPos();
  static void UpdateVelocity();





};

#endif
