#ifndef Stepper_h
#define Stepper_h

#include <Arduino.h>
#include "Elevator.h"

//---- Pins ----
extern const int A_phase ;
extern const int B_phase ;
extern const int A_enable ;
extern const int B_enable ;
extern const int stepsPerRevolution ; 
extern int step;

//---- Functions ----
void fullStepForward();
void fullStepBackward();
void OpenDoor();
void CloseDoor();
void stopMotor();
void Step_init();

#endif // Stepper_h
