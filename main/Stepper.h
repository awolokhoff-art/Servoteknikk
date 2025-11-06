#ifndef Stepper_h
#define Stepper_h

#include <Arduino.h>
#include "Elevator.h"

class Stepper{

private:
  static const int A_phase ;
  static const int B_phase ;
  static const int A_enable ;
  static const int B_enable ;
  static const int stepsPerRevolution ; 

  static int step;

  static void fullStepForward();
  static void fullStepBackward();
  static void stopMotor();

public:
  Stepper();
  static void OpenDoor();
  static void CloseDoor();
  static void Step_init();

};
#endif // Stepper_h
